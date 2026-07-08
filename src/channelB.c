#include "channelB.h"

#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_check.h"
#include "i2c_bus.h"
#include "probe_config.h"
#include "driver/i2c_master.h"

/*
 * Channel B power output.
 *
 * Hardware:
 *   - controller: LM51772 buck-boost converter;
 *   - I2C address: 0x6A;
 *   - inductor current shunt: 2.5 mOhm;
 *   - output current shunt: 10 mOhm;
 *   - user setpoints: U1 and I1.
 *
 * LM51772 initialization used here:
 *   1. D9.bit5 = 0: current limit is set by internal DAC, not ISET pin.
 *   2. D8.bit7 selects the feedback divider ratio.
 *      For this diagnostic build it is locked to 1 and is not changed in the
 *      normal update loop.
 *   3. Program current limit register 0x0A.
 *   4. Program VOUT register 0x0C/0x0D.
 *   5. Set CONV_EN in 0xD0.bit0 and CONV_EN2 in 0x81.bit0 when output is
 *      enabled.
 */

#define LM51772_REG_CLEAR_FAULTS        0x03U
#define LM51772_REG_CUR_LIM             0x0AU
#define LM51772_REG_VOUT_LSB            0x0CU
#define LM51772_REG_VOUT_MSB            0x0DU
#define LM51772_REG_STATUS_BYTE         0x78U
#define LM51772_REG_CONFIG_D8           0xD8U
#define LM51772_REG_CONFIG_D9           0xD9U
#define LM51772_REG_CTRL_D0             0xD0U
#define LM51772_REG_CTRL_81             0x81U

#define LM51772_D8_SEL_FB_DIV20         0x80U
#define LM51772_D9_SEL_ISET_PIN         0x20U
#define LM51772_CTRL_CONV_EN            0x01U
#define LM51772_CTRL_FORCE_DISCH        0x02U

/* Datasheet formulas used by this module. */
#define LM51772_VOUT_LOW_MIN_MV         1000U
#define LM51772_VOUT_LOW_MAX_MV         24000U
#define LM51772_VOUT_LOW_STEP_MV        10U
#define LM51772_VOUT_HIGH_MIN_MV        3300U
#define LM51772_VOUT_HIGH_MAX_MV        48000U
#define LM51772_VOUT_HIGH_STEP_MV       20U
#define LM51772_CUR_LIMIT_STEP_UV       500U
#define LM51772_CUR_LIMIT_MAX_CODE      0xFFU
#define LM51772_FIXED_DIV20             true
#define LM51772_FIXED_DIV20_BIT         LM51772_D8_SEL_FB_DIV20

static i2c_master_dev_handle_t s_lm51772;
static const char *TAG = "channelB";
static bool s_error_seen;
static bool s_output_programmed;
static uint16_t s_last_target_mv;
static uint16_t s_last_target_ma;

/* write_u8
 * Inputs: register address and byte value.
 * Returns: ESP_OK or I2C error.
 * Does: writes one LM51772 register.
 */
static esp_err_t write_u8(uint8_t reg, uint8_t value)
{
    uint8_t bytes[2] = {reg, value};
    esp_err_t err = i2c_master_transmit(s_lm51772, bytes, sizeof(bytes), 100);
    if (err != ESP_OK) ESP_LOGW(TAG, "write %02X=%02X: %s", reg, value, esp_err_to_name(err));
    return err;
}

/* read_u8
 * Inputs: register address.
 * Outputs: *value receives the register byte.
 * Returns: ESP_OK or I2C error.
 * Does: reads one LM51772 register.
 */
static esp_err_t read_u8(uint8_t reg, uint8_t *value)
{
    esp_err_t err = i2c_master_transmit_receive(s_lm51772, &reg, 1, value, 1, 100);
    if (err != ESP_OK) ESP_LOGW(TAG, "read %02X: %s", reg, esp_err_to_name(err));
    return err;
}

/* update_bits
 * Inputs: register address, mask of bits to change, new masked value.
 * Returns: ESP_OK or I2C error.
 * Does: performs read-modify-write so unrelated configuration bits keep their
 * reset or board-selected values.
 */
static esp_err_t update_bits(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t old_value = 0;
    esp_err_t err = read_u8(reg, &old_value);
    if (err != ESP_OK) return err;

    uint8_t new_value = (uint8_t)((old_value & ~mask) | (value & mask));
    return write_u8(reg, new_value);
}

/* voltage_to_code
 * Inputs: requested output voltage in millivolts and selected divider range.
 * Returns: LM51772 VOUT code.
 * Does: converts U1 to the 10-bit VOUT register value. U1 below 1 V is not a
 * valid user setpoint; if it reaches this layer anyway, output code is zero.
 */
static uint16_t voltage_to_code(uint16_t target_mv, bool div20)
{
    if (target_mv < LM51772_VOUT_LOW_MIN_MV) return 0U;

    uint16_t max_mv = div20 ? LM51772_VOUT_HIGH_MAX_MV : LM51772_VOUT_LOW_MAX_MV;
    uint16_t step_mv = div20 ? LM51772_VOUT_HIGH_STEP_MV : LM51772_VOUT_LOW_STEP_MV;

    if (target_mv > max_mv) target_mv = max_mv;
    uint32_t code = ((uint32_t)target_mv + step_mv / 2U) / step_mv;
    if (code > 0x0FFFU) code = 0x0FFFU;
    return (uint16_t)code;
}

/* code_to_voltage_mv
 * Inputs: LM51772 VOUT code and selected divider range.
 * Returns: voltage represented by that code in millivolts.
 * Does: exposes the actual quantized programmed voltage.
 */
static uint16_t code_to_voltage_mv(uint16_t code, bool div20)
{
    uint16_t step_mv = div20 ? LM51772_VOUT_HIGH_STEP_MV : LM51772_VOUT_LOW_STEP_MV;
    return (uint16_t)((uint32_t)code * step_mv);
}

/* current_to_code
 * Inputs: current limit in milliamps.
 * Returns: LM51772 current-limit DAC code.
 * Does: converts I1 through the 10 mOhm output shunt to the DAC code.
 */
static uint8_t current_to_code(uint16_t target_ma)
{
    if (target_ma > CHANNEL_B_CURRENT_LIMIT_MAX_MA) target_ma = CHANNEL_B_CURRENT_LIMIT_MAX_MA;
    uint32_t sense_uv = ((uint32_t)target_ma * CHANNEL_B_OUTPUT_SHUNT_UOHM + 500U) / 1000U;
    uint32_t code = (sense_uv + LM51772_CUR_LIMIT_STEP_UV / 2U) / LM51772_CUR_LIMIT_STEP_UV;
    if (code > LM51772_CUR_LIMIT_MAX_CODE) code = LM51772_CUR_LIMIT_MAX_CODE;
    return (uint8_t)code;
}

/* code_to_current_ma
 * Inputs: current-limit DAC code.
 * Returns: current represented by that code in milliamps.
 * Does: exposes the quantized programmed current limit.
 */
static uint16_t code_to_current_ma(uint8_t code)
{
    uint32_t sense_uv = (uint32_t)code * LM51772_CUR_LIMIT_STEP_UV;
    return (uint16_t)((sense_uv * 1000U + CHANNEL_B_OUTPUT_SHUNT_UOHM / 2U) /
                      CHANNEL_B_OUTPUT_SHUNT_UOHM);
}

/* publish_state
 * Inputs: state snapshot and calculated channel values.
 * Returns: none.
 * Does: exposes channel B target/programmed values to UI/telemetry.
 */
static void publish_state(app_state_t *state,
                          uint16_t target_mv,
                          uint16_t target_ma,
                          uint16_t programmed_mv,
                          uint16_t programmed_ma,
                          bool output_enabled)
{
    state->lm51772.address = CHANNEL_B_LM51772_ADDRESS;
    state->lm51772.target_mv = target_mv;
    state->lm51772.target_ma = target_ma;
    state->lm51772.programmed_mv = programmed_mv;
    state->lm51772.programmed_ma = programmed_ma;
    state->lm51772.output_enabled = output_enabled;
}

/* channelB_init
 * Inputs: none.
 * Returns: ESP_OK on success.
 * Does: initializes LM51772 for channel B, locks the feedback-divider range,
 * and leaves output off until U1 becomes non-zero.
 */
esp_err_t channelB_init(void)
{
    i2c_master_bus_handle_t bus = NULL;
    ESP_RETURN_ON_ERROR(i2c_bus_get(&bus), "channelB", "i2c bus");

    if (s_lm51772 == NULL) {
        i2c_device_config_t device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = CHANNEL_B_LM51772_ADDRESS,
            .scl_speed_hz = CHANNEL_B_I2C_HZ,
        };
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &device_config, &s_lm51772),
                            "channelB", "i2c device");
    }

    (void)update_bits(LM51772_REG_CTRL_D0, LM51772_CTRL_CONV_EN, 0);
    (void)update_bits(LM51772_REG_CTRL_81,
                      (uint8_t)(LM51772_CTRL_CONV_EN | LM51772_CTRL_FORCE_DISCH),
                      0);
    (void)write_u8(LM51772_REG_CLEAR_FAULTS, 0);
    (void)update_bits(LM51772_REG_CONFIG_D9, LM51772_D9_SEL_ISET_PIN, 0);
    (void)update_bits(LM51772_REG_CONFIG_D8,
                      LM51772_D8_SEL_FB_DIV20,
                      LM51772_FIXED_DIV20_BIT);
    (void)write_u8(LM51772_REG_CUR_LIM, 0);
    (void)write_u8(LM51772_REG_VOUT_LSB, 0);
    (void)write_u8(LM51772_REG_VOUT_MSB, 0);
    s_output_programmed = false;
    return ESP_OK;
}

/* channelB_i2c_release
 * Inputs: none.
 * Returns: none.
 * Does: removes the LM51772 I2C device handle before the shared bus is deleted.
 * The handle is recreated by channelB_init() after bus recovery.
 */
void channelB_i2c_release(void)
{
    if (s_lm51772 != NULL) {
        (void)i2c_master_bus_rm_device(s_lm51772);
        s_lm51772 = NULL;
    }
    s_output_programmed = false;
}

/* channelB_update
 * Inputs:
 *   state  - complete application state. U1/I1 are read from state->control;
 *            channel result is written to state->lm51772.
 *   now_us - current esp_timer_get_time().
 * Returns: none.
 * Does: writes channel-B setpoints to LM51772 and reads back status.
 */
void channelB_update(app_state_t *state, int64_t now_us)
{
    (void)now_us;

    uint16_t target_mv = state->control.u1_mv;
    uint16_t target_ma = state->control.i1_ma;
    if (target_ma > CHANNEL_B_CURRENT_LIMIT_MAX_MA) target_ma = CHANNEL_B_CURRENT_LIMIT_MAX_MA;
    bool output_enabled = target_mv >= LM51772_VOUT_LOW_MIN_MV;
    bool div20 = LM51772_FIXED_DIV20;
    uint16_t vout_code = voltage_to_code(target_mv, div20);
    uint8_t ilim_code = current_to_code(target_ma);
    uint16_t programmed_mv = code_to_voltage_mv(vout_code, div20);
    uint16_t programmed_ma = code_to_current_ma(ilim_code);

    publish_state(state, target_mv, target_ma, programmed_mv, programmed_ma, output_enabled);

    uint8_t vout_lsb = (uint8_t)(vout_code & 0xFFU);
    uint8_t vout_msb = (uint8_t)((vout_code >> 8) & 0x0FU);

    if (!s_output_programmed ||
        target_mv != s_last_target_mv ||
        target_ma != s_last_target_ma) {
        esp_err_t err = ESP_OK;
        if ((err = update_bits(LM51772_REG_CONFIG_D9, LM51772_D9_SEL_ISET_PIN, 0)) != ESP_OK ||
            (err = write_u8(LM51772_REG_CUR_LIM, ilim_code)) != ESP_OK ||
            (err = write_u8(LM51772_REG_VOUT_LSB, vout_lsb)) != ESP_OK ||
            (err = write_u8(LM51772_REG_VOUT_MSB, vout_msb)) != ESP_OK ||
            (output_enabled &&
             (err = update_bits(LM51772_REG_CTRL_81, LM51772_CTRL_CONV_EN, LM51772_CTRL_CONV_EN)) != ESP_OK) ||
            (output_enabled &&
             (err = update_bits(LM51772_REG_CTRL_D0, LM51772_CTRL_CONV_EN, LM51772_CTRL_CONV_EN)) != ESP_OK)) {
            state->lm51772.last_error = err;
            state->lm51772.valid = false;
            state->lm51772.status_valid = false;
            state->lm51772.limit_valid = false;
            s_error_seen = true;
            return;
        }

        s_last_target_mv = target_mv;
        s_last_target_ma = target_ma;
        s_output_programmed = true;
    }

    uint8_t status = 0;
    esp_err_t status_err = read_u8(LM51772_REG_STATUS_BYTE, &status);
    state->lm51772.last_error = status_err;
    if (status_err == ESP_OK) {
        state->lm51772.status = status;
        state->lm51772.status_valid = true;
    } else {
        state->lm51772.status_valid = false;
    }

    uint8_t limit = 0;
    esp_err_t limit_err = read_u8(LM51772_REG_CUR_LIM, &limit);
    if (limit_err == ESP_OK) {
        state->lm51772.limit_reg = limit;
        state->lm51772.limit_valid = true;
    } else {
        state->lm51772.limit_valid = false;
    }

    state->lm51772.valid = status_err == ESP_OK && limit_err == ESP_OK;
    if (status_err == ESP_OK && limit_err != ESP_OK) state->lm51772.last_error = limit_err;
    if (state->lm51772.valid && s_error_seen) {
        ESP_LOGW(TAG, "LM back: status=%02X", status);
        s_error_seen = false;
    }
    if (state->lm51772.valid) state->lm51772.last_error = ESP_OK;
}
