#include "channelA.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "calibration.h"
#include "esp_log.h"
#include "esp_check.h"
#include "i2c_bus.h"
#include "probe_config.h"
#include "driver/i2c_master.h"

/*
 * Channel A power output.
 *
 * Hardware:
 *   - controller: TPS55289 buck-boost converter;
 *   - current-sense resistor between ISP and ISN: 10 mOhm;
 *   - user setpoints: U2 and I2.
 *
 * Policy:
 *   - U2 sets auxiliary LDO voltage through MCP4725;
 *   - TPS55289 internal-feedback target is U2 + 1.00 V;
 *   - TPS55289 hardware current limit follows user I2;
 *   - U2 = 0 disables output;
 *   - non-zero U2 below the TPS55289 internal-feedback minimum is clamped to
 *     0.8 V before programming the chip.
 */

/* TPS55289 register addresses.
 * These are kept local to channel A because this module owns the whole power
 * channel behavior, not just the chip access.
 */
#define TPS55289_REG_REF                0x00U
#define TPS55289_REG_IOUT_LIMIT         0x02U
#define TPS55289_REG_VOUT_SR            0x03U
#define TPS55289_REG_VOUT_FS            0x04U
#define TPS55289_REG_MODE               0x06U
#define TPS55289_REG_STATUS             0x07U

#define TPS55289_REF_MIN_MV             800U
#define TPS55289_REF_STEP_MV            10U
#define TPS55289_REF_MAX_CODE           0x0FFFU
#define TPS55289_VOUT_FS_INTFB_20V      0x03U
#define TPS55289_ILIM_ENABLE            0x80U
#define TPS55289_ILIM_STEP_UV           500U
#define TPS55289_ILIM_MAX_CODE          0x7FU
#define TPS55289_MODE_OE                0x80U
#define TPS55289_MODE_FSWDBL            0x40U
#define TPS55289_MODE_HICCUP            0x20U
#define TPS55289_MODE_FPWM              0x02U
#define TPS55289_STATUS_OCP             0x40U
#define TPS55289_VOUT_SR_FAST_OCP       0x01U /* OCP_DELAY=00 (128 us), SR=01 */
#define CHANNEL_A_TRIM_ARM_ERROR_MV     300
#define CHANNEL_A_TRIM_ARM_SAMPLES      3U
#define CHANNEL_A_I2C_TIMEOUT_MS        8

static i2c_master_dev_handle_t s_tps;
static i2c_master_dev_handle_t s_mcp4725;
static const char *TAG = "channelA";
static bool s_output_programmed;
static bool s_last_output_enabled;
static uint16_t s_last_target_mv;
static uint16_t s_last_target_ma;
static uint16_t s_last_command_mv;
static int32_t s_trim_mv;
static int64_t s_trim_hold_until_us;
static uint8_t s_trim_arm_count;
static bool s_trim_armed;

/* write_u8
 * Inputs: register address and byte value.
 * Returns: ESP_OK or I2C error.
 * Does: writes one 8-bit TPS55289 register.
 */
static esp_err_t write_u8(uint8_t reg, uint8_t value)
{
    static uint32_t debug_log_count;
    uint8_t bytes[2] = {reg, value};
    if (false && debug_log_count < 120U) {
        ++debug_log_count;
        ESP_LOGI(TAG, "I2C_TX TPS W %02X %02X %02X", TPS55289_ADDRESS, reg, value);
    }
    return i2c_master_transmit(s_tps, bytes, sizeof(bytes), CHANNEL_A_I2C_TIMEOUT_MS);
}

/* write_ref
 * Inputs: 11-bit REF code.
 * Returns: ESP_OK or I2C error.
 * Does: writes TPS55289 REF_LSB at 00h first, then REF_MSB at 01h as required
 * by the datasheet.
 */
static esp_err_t write_ref(uint16_t value)
{
    if (value > TPS55289_REF_MAX_CODE) value = TPS55289_REF_MAX_CODE;
    esp_err_t err = write_u8(TPS55289_REG_REF, (uint8_t)value);
    if (err != ESP_OK) return err;
    return write_u8((uint8_t)(TPS55289_REG_REF + 1U), (uint8_t)(value >> 8));
}

/* read_u8
 * Inputs: register address.
 * Outputs: *value receives the register byte.
 * Returns: ESP_OK or I2C error.
 * Does: reads one 8-bit TPS55289 register.
 */
static esp_err_t read_u8(uint8_t reg, uint8_t *value)
{
    static uint32_t debug_log_count;
    if (false && debug_log_count < 120U) {
        ++debug_log_count;
        ESP_LOGI(TAG, "I2C_TX TPS R %02X %02X", TPS55289_ADDRESS, reg);
    }
    return i2c_master_transmit_receive(s_tps,
                                       &reg,
                                       1,
                                       value,
                                       1,
                                       CHANNEL_A_I2C_TIMEOUT_MS);
}

/* write_mcp4725
 * Inputs: 12-bit DAC code.
 * Returns: ESP_OK or I2C error.
 * Does: writes the MCP4725 DAC input register in fast mode. EEPROM is not
 * touched, so frequent setpoint changes do not wear non-volatile memory.
 */
static esp_err_t write_mcp4725(uint16_t code)
{
    static uint32_t debug_log_count;
    if (code > 0x0FFFU) code = 0x0FFFU;

    uint8_t bytes[2] = {
        (uint8_t)(code >> 8),
        (uint8_t)code,
    };
    if (false && debug_log_count < 120U) {
        ++debug_log_count;
        ESP_LOGI(TAG, "I2C_TX MCP W %02X %02X %02X", CHANNEL_A_MCP4725_ADDRESS, bytes[0], bytes[1]);
    }
    return i2c_master_transmit(s_mcp4725, bytes, sizeof(bytes), CHANNEL_A_I2C_TIMEOUT_MS);
}

/* clamp_tps_voltage_mv
 * Inputs: requested voltage in millivolts.
 * Returns: TPS-compatible voltage in millivolts.
 * Does: keeps non-zero output requests inside the chip's internal-feedback
 * range. Zero remains zero and means output off.
 */
static uint16_t clamp_tps_voltage_mv(uint16_t target_mv)
{
    if (target_mv == 0U) return 0U;
    if (target_mv < TPS55289_REF_MIN_MV) return TPS55289_REF_MIN_MV;
    return target_mv;
}

/* voltage_to_ref_code
 * Inputs: voltage in millivolts, at least 800 mV.
 * Returns: TPS55289 REF code.
 * Does: converts Vout target to REF code for internal-feedback mode.
 */
static uint16_t voltage_to_ref_code(uint16_t voltage_mv)
{
    if (voltage_mv < TPS55289_REF_MIN_MV) voltage_mv = TPS55289_REF_MIN_MV;
    uint32_t code = (uint32_t)(voltage_mv - TPS55289_REF_MIN_MV) / TPS55289_REF_STEP_MV;
    if (code > TPS55289_REF_MAX_CODE) code = TPS55289_REF_MAX_CODE;
    return (uint16_t)code;
}

/* current_to_limit_code
 * Inputs: current limit in milliamps.
 * Returns: TPS55289 IOUT_LIMIT code.
 * Does: converts current through the 10 mOhm ISP/ISN resistor to shunt-voltage
 * code. At 1.50 A and 10 mOhm, Vsense is 15 mV, code is about 30.
 */
static uint8_t current_to_limit_code(uint16_t target_ma)
{
    uint32_t sense_uv = ((uint32_t)target_ma * CHANNEL_A_CURRENT_SENSE_UOHM + 500U) / 1000U;
    uint32_t code = (sense_uv + TPS55289_ILIM_STEP_UV / 2U) / TPS55289_ILIM_STEP_UV;
    if (code > TPS55289_ILIM_MAX_CODE) code = TPS55289_ILIM_MAX_CODE;
    return (uint8_t)code;
}

typedef struct {
    uint16_t measured_ma;
    uint16_t command_ma;
} channelA_limit_cal_point_t;

static const channelA_limit_cal_point_t s_limit_cal[] = {
    {10U, 100U},
    {44U, 150U},
    {100U, 200U},
    {178U, 300U},
    {255U, 400U},
    {339U, 500U},
    {425U, 600U},
    {512U, 700U},
    {597U, 800U},
    {683U, 900U},
    {767U, 1000U},
    {852U, 1100U},
    {938U, 1200U},
    {966U, 1300U},
};

/* corrected_current_limit_ma
 * Inputs: requested real current limit in milliamps.
 * Returns: TPS55289 current-limit command in milliamps.
 * Does: compensates the measured TPS55289 current-limit error. The table is
 * inverse: measured limit -> command that produced it. Intermediate user
 * values are linearly interpolated; values above the measured region are
 * extrapolated from the stable mid-range gain.
 */
static uint16_t corrected_current_limit_ma(uint16_t target_ma)
{
    if (target_ma == 0U) return 0U;

    const size_t count = sizeof(s_limit_cal) / sizeof(s_limit_cal[0]);
    if (target_ma <= s_limit_cal[0].measured_ma) {
        return s_limit_cal[0].command_ma;
    }

    for (size_t i = 1U; i < count; ++i) {
        const channelA_limit_cal_point_t *low = &s_limit_cal[i - 1U];
        const channelA_limit_cal_point_t *high = &s_limit_cal[i];
        if (target_ma <= high->measured_ma) {
            uint32_t measured_span = (uint32_t)high->measured_ma - low->measured_ma;
            uint32_t command_span = (uint32_t)high->command_ma - low->command_ma;
            uint32_t measured_delta = (uint32_t)target_ma - low->measured_ma;
            return (uint16_t)(low->command_ma +
                              (measured_delta * command_span + measured_span / 2U) /
                              measured_span);
        }
    }

    uint32_t extra_ma = (uint32_t)target_ma - s_limit_cal[count - 1U].measured_ma;
    uint32_t command_ma = (uint32_t)s_limit_cal[count - 1U].command_ma +
                          (extra_ma * 10000U + 4212U) / 8425U;
    if (command_ma > UINT16_MAX) command_ma = UINT16_MAX;
    return (uint16_t)command_ma;
}

/* limit_code_to_current_ma
 * Inputs: TPS55289 IOUT_LIMIT code without enable bit.
 * Returns: quantized current limit represented by the code.
 * Does: exposes the programmed current limit for state/UI.
 */
static uint16_t limit_code_to_current_ma(uint8_t code)
{
    uint32_t sense_uv = (uint32_t)(code & TPS55289_ILIM_MAX_CODE) * TPS55289_ILIM_STEP_UV;
    return (uint16_t)((sense_uv * 1000U + CHANNEL_A_CURRENT_SENSE_UOHM / 2U) /
                      CHANNEL_A_CURRENT_SENSE_UOHM);
}

static int32_t clamp_s32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static int32_t measured_current_ma(const app_ina238_channel_t *measurement)
{
    if (!measurement->valid || measurement->shunt_uohm == 0U) return 0;
    int64_t current_ua = measurement->current_ua;
    if (calibration_current_available(CALIBRATION_CHANNEL_A)) {
        current_ua = calibration_correct_current_ua(CALIBRATION_CHANNEL_A,
                                                    measurement->bus_mv,
                                                    current_ua);
    } else {
        current_ua -= ((int64_t)measurement->bus_mv * CHANNEL_A_DIVIDER_LEAK_UA_PER_MV_NUM +
                       CHANNEL_A_DIVIDER_LEAK_UA_PER_MV_DEN / 2LL) /
                      CHANNEL_A_DIVIDER_LEAK_UA_PER_MV_DEN;
    }
    if (current_ua >= 0) return (int32_t)((current_ua + 500LL) / 1000LL);
    return (int32_t)((current_ua - 500LL) / 1000LL);
}

static uint16_t corrected_target_mv(uint16_t target_mv,
                                    bool output_enabled,
                                    bool current_limit_active,
                                    const app_ina238_channel_t *measurement,
                                    int64_t now_us,
                                    bool setpoint_changed)
{
#if OUTPUT_VOLTAGE_TRIM_ENABLED
    if (calibration_running()) {
        s_trim_mv = 0;
        s_trim_hold_until_us = 0;
        s_trim_arm_count = 0;
        s_trim_armed = false;
        return target_mv;
    }

    if (!output_enabled) {
        s_trim_mv = 0;
        s_trim_hold_until_us = 0;
        s_trim_arm_count = 0;
        s_trim_armed = false;
        return target_mv;
    }

    if (setpoint_changed) {
        s_trim_mv = 0;
        s_trim_hold_until_us = now_us + 500000LL;
        s_trim_arm_count = 0;
        s_trim_armed = false;
    }

    int32_t error_mv = 0;
    bool measurement_ready = now_us >= s_trim_hold_until_us &&
                             measurement->valid &&
                             !current_limit_active;
    if (measurement_ready) {
        error_mv = (int32_t)target_mv - (int32_t)measurement->bus_mv;
        int32_t abs_error_mv = error_mv < 0 ? -error_mv : error_mv;
        if (!s_trim_armed) {
            if (abs_error_mv <= CHANNEL_A_TRIM_ARM_ERROR_MV) {
                if (s_trim_arm_count < CHANNEL_A_TRIM_ARM_SAMPLES) ++s_trim_arm_count;
                if (s_trim_arm_count >= CHANNEL_A_TRIM_ARM_SAMPLES) s_trim_armed = true;
            } else {
                s_trim_arm_count = 0;
            }
        }
    }

    bool trim_active = measurement_ready && s_trim_armed;
    if (trim_active && measurement->valid && !current_limit_active) {
        if (error_mv > OUTPUT_VOLTAGE_TRIM_DEADBAND_MV ||
            error_mv < -OUTPUT_VOLTAGE_TRIM_DEADBAND_MV) {
            int32_t step_mv = error_mv / OUTPUT_VOLTAGE_TRIM_GAIN_DIV;
            if (step_mv == 0) step_mv = error_mv > 0 ? 1 : -1;
            step_mv = clamp_s32(step_mv,
                                -OUTPUT_VOLTAGE_TRIM_STEP_MV,
                                OUTPUT_VOLTAGE_TRIM_STEP_MV);
            s_trim_mv = clamp_s32(s_trim_mv + step_mv,
                                  -OUTPUT_VOLTAGE_TRIM_MAX_MV,
                                  OUTPUT_VOLTAGE_TRIM_MAX_MV);
        }
    }

    int32_t load_comp_mv = 0;
    if (trim_active && !current_limit_active) {
        int32_t current_ma = measured_current_ma(measurement);
        if (current_ma < 0) current_ma = 0;
        load_comp_mv =
            (int32_t)(((int64_t)current_ma * CHANNEL_A_LOAD_COMP_MOHM + 500LL) / 1000LL);
    }
    int32_t command_mv = (int32_t)target_mv +
                         calibration_voltage_correction_mv(CALIBRATION_CHANNEL_A, target_mv) +
                         s_trim_mv +
                         load_comp_mv;
    command_mv = clamp_s32(command_mv,
                           0,
                           (int32_t)UINT16_MAX - (int32_t)CHANNEL_A_LDO_HEADROOM_LOW_MV);
    return (uint16_t)command_mv;
#else
    (void)output_enabled;
    (void)current_limit_active;
    (void)measurement;
    return target_mv;
#endif
}

static uint16_t channelA_ldo_headroom_mv(uint16_t channel_target_mv)
{
    if (channel_target_mv > CHANNEL_A_LDO_HEADROOM_SWITCH_MV) {
        return CHANNEL_A_LDO_HEADROOM_HIGH_MV;
    }
    return CHANNEL_A_LDO_HEADROOM_LOW_MV;
}

typedef struct {
    uint16_t ldo_target_mv;
    uint16_t ldo_programmed_mv;
    uint16_t dac_mv;
    uint16_t dac_code;
    bool saturated;
} channelA_ldo_dac_t;

/* round_div_s64
 * Inputs: signed numerator and positive denominator.
 * Returns: numerator / denominator rounded to nearest integer.
 * Does: keeps the LDO/DAC equation readable without losing precision before
 * division.
 */
static int64_t round_div_s64(int64_t numerator, int64_t denominator)
{
    if (numerator >= 0) {
        return (numerator + denominator / 2) / denominator;
    }
    return -((-numerator + denominator / 2) / denominator);
}

/* ldo_output_from_dac_mv
 * Inputs: DAC output voltage in millivolts.
 * Returns: expected LDO output voltage in millivolts.
 * Does: solves the TPS73801 feedback-node KCL for the actual output voltage
 * that the configured 20k/2k/Rdac network will request.
 */
static uint16_t ldo_output_from_dac_mv(uint16_t dac_mv)
{
    const int64_t vfb = CHANNEL_A_LDO_FB_REF_MV;
    const int64_t r_top = CHANNEL_A_LDO_R_TOP_OHM;
    const int64_t r_bottom = CHANNEL_A_LDO_R_BOTTOM_OHM;
    const int64_t r_dac = CHANNEL_A_LDO_R_DAC_OHM;

    int64_t numerator = (vfb * r_top * r_dac) -
                        (((int64_t)dac_mv - vfb) * r_top * r_bottom);
    int64_t denominator = r_bottom * r_dac;
    int64_t vout = vfb + round_div_s64(numerator, denominator);

    if (vout < 0) return 0U;
    if (vout > UINT16_MAX) return UINT16_MAX;
    return (uint16_t)vout;
}

/* calculate_ldo_dac
 * Inputs: U2 target in millivolts.
 * Returns: DAC code and expected LDO output.
 * Does: commands the auxiliary TPS73801 LDO to U2 using:
 *
 *   (Vout - Vfb) / Rtop + (Vdac - Vfb) / Rdac = Vfb / Rbottom
 *
 * DAC voltage is clamped to the real MCP4725 output range, and the saturated
 * flag tells the UI/telemetry that the requested LDO output is outside what
 * this resistor network can produce from a 0..3.3 V DAC.
 */
static channelA_ldo_dac_t calculate_ldo_dac(uint16_t channel_target_mv)
{
    channelA_ldo_dac_t result = {0};
    result.ldo_target_mv = channel_target_mv;

    const int64_t vfb = CHANNEL_A_LDO_FB_REF_MV;
    const int64_t r_top = CHANNEL_A_LDO_R_TOP_OHM;
    const int64_t r_bottom = CHANNEL_A_LDO_R_BOTTOM_OHM;
    const int64_t r_dac = CHANNEL_A_LDO_R_DAC_OHM;

    int64_t kcl_numerator =
        (vfb * r_top) - (((int64_t)result.ldo_target_mv - vfb) * r_bottom);
    int64_t kcl_denominator = r_bottom * r_top;
    int64_t dac_mv = vfb + round_div_s64(r_dac * kcl_numerator, kcl_denominator);

    result.saturated = false;
    if (dac_mv < 0) {
        dac_mv = 0;
        result.saturated = true;
    } else if (dac_mv > CHANNEL_A_MCP4725_VDD_MV) {
        dac_mv = CHANNEL_A_MCP4725_VDD_MV;
        result.saturated = true;
    }

    result.dac_mv = (uint16_t)dac_mv;
    result.dac_code = (uint16_t)(((uint32_t)result.dac_mv * 4095U +
                                  CHANNEL_A_MCP4725_VDD_MV / 2U) /
                                 CHANNEL_A_MCP4725_VDD_MV);
    if (result.dac_code > 0x0FFFU) result.dac_code = 0x0FFFU;

    result.ldo_programmed_mv = ldo_output_from_dac_mv(result.dac_mv);
    return result;
}

/* publish_state
 * Inputs: state snapshot and calculated channel values.
 * Returns: none.
 * Does: exposes channel A target/programmed values to UI/telemetry.
 */
static void publish_state(app_state_t *state,
                          uint16_t target_mv,
                          uint16_t target_ma,
                          uint16_t programmed_mv,
                          uint16_t programmed_ma,
                          bool output_enabled,
                          const channelA_ldo_dac_t *ldo)
{
    state->tps55289.address = TPS55289_ADDRESS;
    state->tps55289.target_mv = target_mv;
    state->tps55289.target_ma = target_ma;
    state->tps55289.programmed_mv = programmed_mv;
    state->tps55289.programmed_ma = programmed_ma;
    state->tps55289.output_enabled = output_enabled;
    state->tps55289.ldo_target_mv = ldo->ldo_target_mv;
    state->tps55289.ldo_programmed_mv = ldo->ldo_programmed_mv;
    state->tps55289.ldo_dac_mv = ldo->dac_mv;
    state->tps55289.ldo_dac_code = ldo->dac_code;
    state->tps55289.ldo_dac_saturated = ldo->saturated;
}

/* channelA_init
 * Inputs: none.
 * Returns: ESP_OK on success.
 * Does: initializes TPS55289 for channel A when I2C is enabled, leaving output
 * off until U2 becomes non-zero.
 */
esp_err_t channelA_init(void)
{
    i2c_master_bus_handle_t bus = NULL;
    ESP_RETURN_ON_ERROR(i2c_bus_get(&bus), "channelA", "i2c bus");

    if (s_tps == NULL) {
        i2c_device_config_t device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = TPS55289_ADDRESS,
            .scl_speed_hz = TPS55289_I2C_HZ,
        };
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &device_config, &s_tps),
                            "channelA", "i2c device");
    }

    if (s_mcp4725 == NULL) {
        i2c_device_config_t dac_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = CHANNEL_A_MCP4725_ADDRESS,
            .scl_speed_hz = TPS55289_I2C_HZ,
        };
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dac_config, &s_mcp4725),
                            "channelA", "mcp4725 device");
    }

    (void)write_u8(TPS55289_REG_MODE,
                   (uint8_t)(TPS55289_MODE_FSWDBL | TPS55289_MODE_HICCUP | TPS55289_MODE_FPWM));
    (void)write_u8(TPS55289_REG_VOUT_FS, TPS55289_VOUT_FS_INTFB_20V);
    (void)write_ref(voltage_to_ref_code(TPS55289_REF_MIN_MV));
    (void)write_u8(TPS55289_REG_IOUT_LIMIT,
                   (uint8_t)(TPS55289_ILIM_ENABLE |
                             current_to_limit_code(0)));
    (void)write_u8(TPS55289_REG_VOUT_SR, TPS55289_VOUT_SR_FAST_OCP);
    s_output_programmed = false;
    s_last_output_enabled = false;
    s_last_command_mv = 0U;
    s_trim_mv = 0;
    return ESP_OK;
}

/* channelA_i2c_release
 * Inputs: none.
 * Returns: none.
 * Does: removes Channel A I2C device handles before the shared bus is deleted.
 * The handles are recreated by channelA_init() after bus recovery.
 */
void channelA_i2c_release(void)
{
    if (s_tps != NULL) {
        (void)i2c_master_bus_rm_device(s_tps);
        s_tps = NULL;
    }
    if (s_mcp4725 != NULL) {
        (void)i2c_master_bus_rm_device(s_mcp4725);
        s_mcp4725 = NULL;
    }
    s_output_programmed = false;
    s_last_output_enabled = false;
    s_last_command_mv = 0U;
    s_trim_mv = 0;
}

/* channelA_update
 * Inputs:
 *   state  - complete application state. U2/I2 are read from state->control;
 *            channel result is written to state->tps55289.
 *   now_us - current esp_timer_get_time().
 * Returns: none.
 * Does: applies changed channel-A setpoints to TPS55289. In stub mode it only
 * publishes the values that would be programmed.
 */
void channelA_update(app_state_t *state, int64_t now_us, bool channel_enabled)
{
    (void)now_us;

    uint16_t target_mv = state->control.u2_mv;
    uint16_t target_ma = state->control.i2_ma;
    bool output_enabled = channel_enabled && target_mv != 0U;
    bool setpoint_changed = target_mv != s_last_target_mv ||
                            target_ma != s_last_target_ma ||
                            output_enabled != s_last_output_enabled;
    uint16_t command_mv = corrected_target_mv(target_mv,
                                              output_enabled,
                                              state->tps55289.current_limit_active,
                                              &state->ina238.channel[0],
                                              now_us,
                                              setpoint_changed);
    uint16_t tps_target_mv = command_mv == 0U ? 0U :
                             (uint16_t)(command_mv + channelA_ldo_headroom_mv(command_mv));
    uint16_t programmed_mv = clamp_tps_voltage_mv(tps_target_mv);
    channelA_ldo_dac_t ldo = calculate_ldo_dac(command_mv);

    uint16_t limit_command_ma = corrected_current_limit_ma(target_ma);
    uint8_t ilim_code = current_to_limit_code(limit_command_ma);
    uint16_t programmed_ma = limit_code_to_current_ma(ilim_code);

    publish_state(state, target_mv, target_ma, programmed_mv, programmed_ma, output_enabled, &ldo);

    uint8_t ilim = (uint8_t)(TPS55289_ILIM_ENABLE | ilim_code);
    uint8_t mode = (uint8_t)(TPS55289_MODE_FSWDBL | TPS55289_MODE_HICCUP | TPS55289_MODE_FPWM);
    if (output_enabled) mode |= TPS55289_MODE_OE;
    uint16_t ref_code = voltage_to_ref_code(output_enabled ? programmed_mv : TPS55289_REF_MIN_MV);

    if (s_last_output_enabled && !output_enabled) {
        esp_err_t err = write_u8(TPS55289_REG_MODE, mode);
        if (err != ESP_OK) {
            state->tps55289.last_error = err;
            state->tps55289.valid = false;
            state->tps55289.status_valid = false;
            state->tps55289.limit_valid = false;
            state->tps55289.current_limit_active = false;
            return;
        }
        s_last_target_mv = target_mv;
        s_last_target_ma = target_ma;
        s_last_command_mv = command_mv;
        s_last_output_enabled = false;
        s_output_programmed = true;
    } else if (!s_output_programmed ||
               target_mv != s_last_target_mv ||
               target_ma != s_last_target_ma ||
               command_mv != s_last_command_mv ||
               output_enabled != s_last_output_enabled) {
        esp_err_t err = write_mcp4725(ldo.dac_code);
        if (err == ESP_OK) err = write_u8(TPS55289_REG_VOUT_FS, TPS55289_VOUT_FS_INTFB_20V);
        if (err == ESP_OK) err = write_u8(TPS55289_REG_IOUT_LIMIT, ilim);
        if (err == ESP_OK) err = write_ref(ref_code);
        if (err == ESP_OK) err = write_u8(TPS55289_REG_VOUT_SR, TPS55289_VOUT_SR_FAST_OCP);
        if (err == ESP_OK) err = write_u8(TPS55289_REG_MODE, mode);

        if (err != ESP_OK) {
            state->tps55289.last_error = err;
            state->tps55289.valid = false;
            state->tps55289.status_valid = false;
            state->tps55289.limit_valid = false;
            state->tps55289.current_limit_active = false;
            return;
        }

        s_last_target_mv = target_mv;
        s_last_target_ma = target_ma;
        s_last_command_mv = command_mv;
        s_last_output_enabled = output_enabled;
        s_output_programmed = true;
    }

    uint8_t status = 0;
    esp_err_t status_err = read_u8(TPS55289_REG_STATUS, &status);
    state->tps55289.last_error = status_err;
    if (status_err == ESP_OK) {
        state->tps55289.status = status;
        state->tps55289.status_valid = true;
        state->tps55289.current_limit_active = (status & TPS55289_STATUS_OCP) != 0U;
    } else {
        state->tps55289.status_valid = false;
        state->tps55289.current_limit_active = false;
    }

    uint8_t limit = 0;
    esp_err_t limit_err = read_u8(TPS55289_REG_IOUT_LIMIT, &limit);
    if (limit_err == ESP_OK) {
        state->tps55289.limit_reg = limit;
        state->tps55289.limit_valid = true;
    } else {
        state->tps55289.limit_valid = false;
    }

    state->tps55289.valid = status_err == ESP_OK && limit_err == ESP_OK;
    if (status_err == ESP_OK && limit_err != ESP_OK) state->tps55289.last_error = limit_err;
    if (state->tps55289.valid) state->tps55289.last_error = ESP_OK;
}
