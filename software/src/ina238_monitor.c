#include "ina238_monitor.h"

#include <stdint.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "probe_config.h"
#include "driver/i2c_master.h"

/*
 * INA238 monitor module.
 *
 * Two devices are planned:
 *   channel 0: address 0x41, shunt 100 mOhm;
 *   channel 1: address 0x44, shunt 10 mOhm.
 *
 * Channel A automatically switches shunt-voltage ADC range:
 *   narrow range: +/-40.96 mV, 1.25 uV/LSB;
 *   wide range: +/-163.84 mV, 5 uV/LSB.
 *
 * Channel B uses the same automatic shunt-voltage ADC range switching:
 *   10 mOhm shunt gives 125 uA/LSB in narrow range and 500 uA/LSB in wide.
 *
 */

#define INA238_REG_CONFIG            0x00U
#define INA238_REG_ADC_CONFIG        0x01U
#define INA238_REG_VSHUNT            0x04U
#define INA238_REG_VBUS              0x05U
#define INA238_REG_DIETEMP           0x06U
#define INA238_REG_DIAG_ALRT         0x0BU
#define INA238_REG_SHUNT_OVER_LIMIT  0x0CU
#define INA238_REG_DEVICE_ID         0x3FU
#define INA238_CONFIG_ADCRANGE       0x0010U
#define INA238_CONFIG_DEFAULT        0x0000U
#define INA238_ADC_CONFIG_DEFAULT    0xF954U
#define INA238_DIAG_ALRT_ALATCH      0x8000U
#define INA238_DIAG_ALRT_SLOWALERT   0x2000U
#define INA238_DIAG_ALRT_APOL        0x1000U
#define INA238_A_NARROW_TO_WIDE_NV   40500000LL
#define INA238_A_WIDE_TO_NARROW_NV   39000000LL
#define INA238_B_NARROW_TO_WIDE_NV   40500000LL
#define INA238_B_WIDE_TO_NARROW_NV   39000000LL
#define INA238_WIDE_LIMIT_NV         163840000LL
#define INA238_NARROW_LIMIT_NV       40960000LL
#define INA238_WIDE_SHUNT_LSB_NV     5000U
#define INA238_NARROW_SHUNT_LSB_NV   1250U
#define INA238_I2C_TIMEOUT_MS        8
#define INA238_ERROR_BACKOFF_US      300000LL

typedef struct {
    uint8_t address;
    uint32_t shunt_uohm;
    bool wide_range;
    bool auto_range;
    int64_t narrow_to_wide_nv;
    int64_t wide_to_narrow_nv;
    int64_t next_retry_us;
    i2c_master_dev_handle_t dev;
} ina238_device_t;

static ina238_device_t s_devices[2] = {
    {
        .address = INA238_1_ADDRESS,
        .shunt_uohm = INA238_1_SHUNT_UOHM,
        .wide_range = false,
        .auto_range = true,
        .narrow_to_wide_nv = INA238_A_NARROW_TO_WIDE_NV,
        .wide_to_narrow_nv = INA238_A_WIDE_TO_NARROW_NV,
    },
    {
        .address = INA238_2_ADDRESS,
        .shunt_uohm = INA238_2_SHUNT_UOHM,
        .wide_range = false,
        .auto_range = true,
        .narrow_to_wide_nv = INA238_B_NARROW_TO_WIDE_NV,
        .wide_to_narrow_nv = INA238_B_WIDE_TO_NARROW_NV,
    },
};
static const char *TAG = "ina238";
static bool s_current_graph_mode;

/* write_register
 * Inputs: dev is one INA238 device; reg/value select the register write.
 * Returns: ESP_OK or I2C error.
 * Does: writes one 16-bit big-endian INA238 register.
 */
static esp_err_t write_register(ina238_device_t *dev, uint8_t reg, uint16_t value)
{
    static uint32_t debug_log_count;
    uint8_t bytes[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)value,
    };
    if (false && debug_log_count < 200U) {
        ++debug_log_count;
        ESP_LOGI(TAG, "I2C_TX INA W %02X %02X %02X %02X",
                 dev->address, reg, bytes[1], bytes[2]);
    }
    return i2c_master_transmit(dev->dev, bytes, sizeof(bytes), INA238_I2C_TIMEOUT_MS);
}

/* read_register
 * Inputs: dev is one INA238 device; reg selects the register.
 * Outputs: *value receives the 16-bit big-endian register value.
 * Returns: ESP_OK or I2C error.
 * Does: reads one INA238 register.
 */
static esp_err_t read_register(ina238_device_t *dev, uint8_t reg, uint16_t *value)
{
    static uint32_t debug_log_count;
    uint8_t data[2] = {0};
    if (false && debug_log_count < 200U) {
        ++debug_log_count;
        ESP_LOGI(TAG, "I2C_TX INA R %02X %02X", dev->address, reg);
    }
    esp_err_t err = i2c_master_transmit_receive(dev->dev,
                                                &reg,
                                                1,
                                                data,
                                                sizeof(data),
                                                INA238_I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;
    *value = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

/* configure_range
 * Inputs: dev is one INA238 device; wide selects +/-163.84 mV if true.
 * Returns: ESP_OK or I2C error.
 * Does: writes CONFIG.ADCRANGE and stores the selected range locally.
 */
static esp_err_t configure_range(ina238_device_t *dev, bool wide)
{
    uint16_t config = INA238_CONFIG_DEFAULT;
    if (!wide) config |= INA238_CONFIG_ADCRANGE;
    ESP_RETURN_ON_ERROR(write_register(dev, INA238_REG_CONFIG, config), "ina238", "config");
    dev->wide_range = wide;
    return ESP_OK;
}

/* configure_adc
 * Inputs: dev is one INA238 device.
 * Returns: ESP_OK or I2C error.
 * Does: configures continuous VSHUNT+VBUS+TEMP conversion:
 * VSHUNT 1052 us, VBUS 540 us, TEMP 150 us, averaging 128 samples.
 */
static esp_err_t configure_adc(ina238_device_t *dev)
{
    return write_register(dev, INA238_REG_ADC_CONFIG, INA238_ADC_CONFIG_DEFAULT);
}

/* configure_alert_normal_low
 * Inputs: dev is one INA238 device.
 * Returns: ESP_OK or I2C error.
 * Does: sets DIAG_ALRT.APOL=1. INA238 ALERT is open-drain; with inverted
 * polarity the inactive/normal state is driven low, and an alert releases it
 * high through the external pull-up.
 */
static esp_err_t configure_alert_normal_low(ina238_device_t *dev)
{
    uint16_t diag_alrt = 0;
    ESP_RETURN_ON_ERROR(read_register(dev, INA238_REG_DIAG_ALRT, &diag_alrt),
                        "ina238", "diag read");
    diag_alrt &= (uint16_t)~(INA238_DIAG_ALRT_ALATCH | INA238_DIAG_ALRT_SLOWALERT);
    diag_alrt |= INA238_DIAG_ALRT_APOL;
    ESP_RETURN_ON_ERROR(write_register(dev, INA238_REG_SHUNT_OVER_LIMIT, 0x7FFFU),
                        "ina238", "clear stale shunt limit");
    ESP_RETURN_ON_ERROR(write_register(dev, INA238_REG_DIAG_ALRT, diag_alrt),
                        "ina238", "diag write");
    return ESP_OK;
}

/* signed_register
 * Inputs: raw 16-bit register.
 * Returns: signed value with two's-complement interpretation.
 * Does: keeps conversion code readable.
 */
static int16_t signed_register(uint16_t raw)
{
    return (int16_t)raw;
}

/* convert_shunt_nv
 * Inputs: raw VSHUNT register and selected range.
 * Returns: shunt voltage in nanovolts.
 * Does: uses INA238 LSB of 5000 nV in wide range and 1250 nV in narrow
 * range, preserving the 1.25 uV narrow-range resolution.
 */
static int64_t convert_shunt_nv(uint16_t raw, bool wide_range)
{
    int32_t signed_raw = signed_register(raw);
    uint32_t lsb_nv = wide_range ? INA238_WIDE_SHUNT_LSB_NV : INA238_NARROW_SHUNT_LSB_NV;
    return (int64_t)signed_raw * (int64_t)lsb_nv;
}

static int32_t convert_shunt_uv_from_nv(int64_t shunt_nv)
{
    if (shunt_nv >= 0) return (int32_t)((shunt_nv + 500LL) / 1000LL);
    return (int32_t)((shunt_nv - 500LL) / 1000LL);
}

/* convert_bus_mv
 * Inputs: raw VBUS register.
 * Returns: bus voltage in millivolts.
 * Does: uses INA238 bus voltage LSB of 3.125 mV.
 */
static uint32_t convert_bus_mv(uint16_t raw)
{
    return ((uint32_t)raw * 3125U + 500U) / 1000U;
}

/* convert_temperature_mc
 * Inputs: raw DIETEMP register.
 * Returns: die temperature in milli-degrees Celsius.
 * Does: uses INA238 temperature LSB of 7.8125 mC.
 */
static int32_t convert_temperature_mc(uint16_t raw)
{
    return ((int32_t)signed_register(raw) * 78125) / 10000;
}

/* convert_current_ua
 * Inputs: shunt voltage in nanovolts and shunt resistance in micro-ohms.
 * Returns: current in microamps.
 * Does: I[uA] = Vshunt[nV] * 1000 / R[uOhm].
 */
static int64_t convert_current_ua(int64_t shunt_nv, uint32_t shunt_uohm)
{
    int64_t numerator = shunt_nv * 1000LL;
    int64_t denominator = (int64_t)shunt_uohm;
    if (numerator >= 0) return (numerator + denominator / 2LL) / denominator;
    return (numerator - denominator / 2LL) / denominator;
}

static int64_t shunt_limit_nv(bool wide_range)
{
    return wide_range ? INA238_WIDE_LIMIT_NV : INA238_NARROW_LIMIT_NV;
}

static void update_auto_range(ina238_device_t *dev, int64_t abs_shunt_nv)
{
    if (!dev->auto_range) return;

    if (!dev->wide_range && abs_shunt_nv >= dev->narrow_to_wide_nv) {
        (void)configure_range(dev, true);
    } else if (dev->wide_range && abs_shunt_nv <= dev->wide_to_narrow_nv) {
        (void)configure_range(dev, false);
    }
}

/* fill_static_channel_info
 * Inputs: channel state and corresponding device config.
 * Returns: none.
 * Does: keeps address/shunt/range visible even when I2C is not connected.
 */
static void fill_static_channel_info(app_ina238_channel_t *out, const ina238_device_t *dev)
{
    out->address = dev->address;
    out->shunt_uohm = dev->shunt_uohm;
    out->wide_range = dev->wide_range;
}

/* ina238_monitor_init
 * Inputs: none.
 * Returns: ESP_OK on success.
 * Does: initializes I2C and both INA238 devices when the bus is enabled.
 */
esp_err_t ina238_monitor_init(void)
{
    i2c_master_bus_handle_t bus = NULL;
    ESP_RETURN_ON_ERROR(i2c_bus_get(&bus), "ina238", "i2c bus");

    for (size_t i = 0; i < 2U; ++i) {
        if (s_devices[i].dev == NULL) {
            i2c_device_config_t device_config = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = s_devices[i].address,
                .scl_speed_hz = INA238_I2C_HZ,
            };
            ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &device_config, &s_devices[i].dev),
                                "ina238", "i2c device");
        }
        (void)configure_adc(&s_devices[i]);
        s_devices[i].auto_range = !s_current_graph_mode;
        (void)configure_range(&s_devices[i], s_current_graph_mode ? true : s_devices[i].wide_range);
        if (s_devices[i].address == INA238_1_ADDRESS) {
            (void)configure_alert_normal_low(&s_devices[i]);
        }
    }
    return ESP_OK;
}

/* ina238_monitor_i2c_release
 * Inputs: none.
 * Returns: none.
 * Does: removes both INA238 I2C device handles before the shared bus is
 * deleted. The handles are recreated by ina238_monitor_init().
 */
void ina238_monitor_i2c_release(void)
{
    for (size_t i = 0; i < 2U; ++i) {
        if (s_devices[i].dev != NULL) {
            (void)i2c_master_bus_rm_device(s_devices[i].dev);
            s_devices[i].dev = NULL;
        }
        s_devices[i].next_retry_us = 0;
    }
}

void ina238_monitor_set_current_graph_mode(bool enabled)
{
    if (s_current_graph_mode == enabled) return;
    s_current_graph_mode = enabled;

    for (size_t i = 0; i < 2U; ++i) {
        s_devices[i].auto_range = !enabled;
        if (enabled && s_devices[i].dev != NULL) {
            (void)configure_range(&s_devices[i], true);
        }
    }
}

static void update_device(app_state_t *state,
                          ina238_device_t *dev,
                          app_ina238_channel_t *out,
                          int64_t now_us)
{
    (void)state;
    fill_static_channel_info(out, dev);
    if (now_us < dev->next_retry_us) {
        out->valid = false;
        return;
    }

    uint16_t raw_shunt = 0;
    uint16_t raw_bus = 0;
    uint16_t raw_temp = 0;
    if (read_register(dev, INA238_REG_VSHUNT, &raw_shunt) != ESP_OK ||
        read_register(dev, INA238_REG_VBUS, &raw_bus) != ESP_OK ||
        read_register(dev, INA238_REG_DIETEMP, &raw_temp) != ESP_OK) {
        out->valid = false;
        dev->next_retry_us = now_us + INA238_ERROR_BACKOFF_US;
        return;
    }
    dev->next_retry_us = 0;

    int64_t shunt_nv = convert_shunt_nv(raw_shunt, dev->wide_range);
    int64_t abs_shunt_nv = shunt_nv < 0 ? -shunt_nv : shunt_nv;
    int64_t current_ua = convert_current_ua(shunt_nv, dev->shunt_uohm);

    out->valid = true;
    out->shunt_uv = convert_shunt_uv_from_nv(shunt_nv);
    out->current_ua = current_ua;
    if (current_ua >= 0) {
        out->current_ma = (int32_t)((current_ua + 500LL) / 1000LL);
    } else {
        out->current_ma = (int32_t)((current_ua - 500LL) / 1000LL);
    }
    out->bus_mv = convert_bus_mv(raw_bus);
    out->temperature_mc = convert_temperature_mc(raw_temp);
    out->saturated = abs_shunt_nv >= shunt_limit_nv(dev->wide_range);
    update_auto_range(dev, abs_shunt_nv);
}

/* ina238_monitor_update
 * Inputs:
 *   state  - app state to receive INA238 measurements.
 *   now_us - current esp_timer_get_time().
 * Returns: none.
 * Does: reads both INA238 devices and updates state.
 */
void ina238_monitor_update(app_state_t *state, int64_t now_us)
{
    for (size_t i = 0; i < 2U; ++i) {
        update_device(state, &s_devices[i], &state->ina238.channel[i], now_us);
    }
}

void ina238_monitor_update_address(app_state_t *state, int64_t now_us, uint8_t address)
{
    for (size_t i = 0; i < 2U; ++i) {
        if (s_devices[i].address == address) {
            update_device(state, &s_devices[i], &state->ina238.channel[i], now_us);
            return;
        }
    }
}

void ina238_monitor_update_bus_voltage_address(app_state_t *state, int64_t now_us, uint8_t address)
{
    for (size_t i = 0; i < 2U; ++i) {
        ina238_device_t *dev = &s_devices[i];
        if (dev->address != address) continue;

        app_ina238_channel_t *out = &state->ina238.channel[i];
        fill_static_channel_info(out, dev);
        if (now_us < dev->next_retry_us) {
            out->valid = false;
            return;
        }

        uint16_t raw_bus = 0;
        if (read_register(dev, INA238_REG_VBUS, &raw_bus) != ESP_OK) {
            out->valid = false;
            dev->next_retry_us = now_us + INA238_ERROR_BACKOFF_US;
            return;
        }

        dev->next_retry_us = 0;
        out->valid = true;
        out->bus_mv = convert_bus_mv(raw_bus);
        return;
    }
}

void ina238_monitor_read_id_address(app_state_t *state, int64_t now_us, uint8_t address)
{
    for (size_t i = 0; i < 2U; ++i) {
        ina238_device_t *dev = &s_devices[i];
        if (dev->address != address) continue;

        app_ina238_channel_t *out = &state->ina238.channel[i];
        fill_static_channel_info(out, dev);
        if (now_us < dev->next_retry_us) {
            out->valid = false;
            return;
        }

        uint16_t raw_id = 0;
        if (read_register(dev, INA238_REG_DEVICE_ID, &raw_id) != ESP_OK) {
            out->valid = false;
            dev->next_retry_us = now_us + INA238_ERROR_BACKOFF_US;
            return;
        }

        dev->next_retry_us = 0;
        out->valid = true;
        return;
    }
}
