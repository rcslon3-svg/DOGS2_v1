#include "i2c_worker.h"

#include "board_io.h"
#include "channelA.h"
#include "channelB.h"
#include "control.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "ina238_monitor.h"

#define I2C_WORKER_FAST_PERIOD_MS 5U
#define I2C_WORKER_SLOW_PERIOD_MS 100U
#define I2C_RECOVERY_MIN_INTERVAL_US 1000000LL
#define ADS1110_CONFIG_240SPS_SINGLE_GAIN1 0x10U
#define ADS1110_CONFIG_START_SINGLE_240SPS_GAIN1 0x90U
#define ADS1110_240SPS_LSB_NV 1000000LL
#define ADS_CONVERSION_DELAY_MS 5U
#define ADS_SYNC_STATS_WINDOW_US 1000000LL

static const char *TAG = "i2c_worker";
static app_state_t *s_state;
static int64_t s_last_recovery_us;
static i2c_master_dev_handle_t s_ads1110;
static uint8_t s_channel_a_error_cycles;
static uint8_t s_channel_b_error_cycles;
static int64_t s_ads_read_due_us;
static int64_t s_ads_stats_start_us;
static uint64_t s_ads_sum_low_mv;
static uint64_t s_ads_sum_high_mv;
static uint32_t s_ads_count_low;
static uint32_t s_ads_count_high;
static uint32_t s_ads_ready_count;
static uint32_t s_ads_busy_count;
static uint32_t s_ads_error_count;
static uint32_t s_ads_min_mv = UINT32_MAX;
static uint32_t s_ads_max_mv;
static bool s_ads_conversion_pending;
static bool s_ads_conversion_level;

static void ads1110_i2c_release(void);

static bool i2c_errors_visible(const app_state_t *state)
{
    (void)state;
    return false;
}

static void recover_i2c_if_needed(app_state_t *state, int64_t now_us)
{
    if (!i2c_errors_visible(state)) return;
    bool lm_error = state->lm51772.last_error != ESP_OK;
    if (!lm_error && now_us - s_last_recovery_us < I2C_RECOVERY_MIN_INTERVAL_US) return;
    s_last_recovery_us = now_us;

    ++state->i2c.recovery_count;
    state->i2c.last_recovery_error = ESP_OK;

    ESP_LOGW(TAG, "I2C recovery #%lu: release devices, release controller, recover lines, recreate bus, chip init",
             (unsigned long)state->i2c.recovery_count);

    ina238_monitor_i2c_release();
    channelA_i2c_release();
    channelB_i2c_release();
    board_io_i2c_release();
    ads1110_i2c_release();

    esp_err_t err = i2c_bus_release();
    if (err != ESP_OK) state->i2c.last_recovery_error = err;

    err = i2c_bus_recover_lines();
    if (err != ESP_OK) state->i2c.last_recovery_error = err;

    err = ina238_monitor_init();
    if (err != ESP_OK) state->i2c.last_recovery_error = err;

    err = channelA_init();
    if (err != ESP_OK) state->i2c.last_recovery_error = err;

    err = channelB_init();
    if (err != ESP_OK) state->i2c.last_recovery_error = err;

    err = board_io_init();
    if (err != ESP_OK) state->i2c.last_recovery_error = err;
}

static esp_err_t ads1110_init(void)
{
    if (s_ads1110 != NULL) return ESP_OK;

    i2c_master_bus_handle_t bus = NULL;
    ESP_RETURN_ON_ERROR(i2c_bus_get(&bus), TAG, "ads bus");

    i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAIN_BUS_ADS1110_ADDRESS,
        .scl_speed_hz = INA238_I2C_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &config, &s_ads1110),
                        TAG, "ads add");

    uint8_t ads_config = ADS1110_CONFIG_START_SINGLE_240SPS_GAIN1;
    return i2c_master_transmit(s_ads1110, &ads_config, sizeof(ads_config), 100);
}

static void ads1110_i2c_release(void)
{
    if (s_ads1110 != NULL) {
        (void)i2c_master_bus_rm_device(s_ads1110);
        s_ads1110 = NULL;
    }
}

static esp_err_t ads1110_start_conversion(void)
{
    if (s_ads1110 == NULL && ads1110_init() != ESP_OK) return ESP_FAIL;

    uint8_t ads_config = ADS1110_CONFIG_START_SINGLE_240SPS_GAIN1;
    return i2c_master_transmit(s_ads1110, &ads_config, sizeof(ads_config), 100);
}

static bool ads1110_read_result(app_state_t *state, uint32_t *voltage_mv, bool *ready)
{
    if (state == NULL) return false;
    if (voltage_mv != NULL) *voltage_mv = 0;
    if (ready != NULL) *ready = false;
    if (s_ads1110 == NULL && ads1110_init() != ESP_OK) return false;

    uint8_t bytes[3] = {0};
    esp_err_t err = i2c_master_receive(s_ads1110, bytes, sizeof(bytes), 100);
    if (err != ESP_OK) {
        state->analog.logic_state = PROBE_UNDEFINED;
        return false;
    }
    if ((bytes[2] & 0x70U) != ADS1110_CONFIG_240SPS_SINGLE_GAIN1) {
        uint8_t ads_config = ADS1110_CONFIG_START_SINGLE_240SPS_GAIN1;
        (void)i2c_master_transmit(s_ads1110, &ads_config, sizeof(ads_config), 100);
        return false;
    }
    if (ready != NULL) *ready = (bytes[2] & 0x80U) == 0U;

    int16_t raw = (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
    int64_t adc_nv = (int64_t)raw * ADS1110_240SPS_LSB_NV;
    if (adc_nv < 0) adc_nv = 0;

    uint32_t adc_mv = (uint32_t)((adc_nv + 500000LL) / 1000000LL);
    uint32_t input_mv = (uint32_t)(((uint64_t)adc_mv * MAIN_BUS_ADS1110_DIVIDER_NUM +
                                    MAIN_BUS_ADS1110_DIVIDER_DEN / 2U) /
                                   MAIN_BUS_ADS1110_DIVIDER_DEN);

    state->analog.voltage_mv = input_mv;
    state->analog.vpp_mv = 0;
    state->analog.bias_current_na = 0;
    state->analog.test_visible = false;
    state->analog.logic_state = input_mv > TIP_OVERVOLTAGE_MV ? PROBE_OVERVOLTAGE : PROBE_UNDEFINED;
    if (voltage_mv != NULL) *voltage_mv = input_mv;
    return true;
}

static void ads_delay_stats_reset(int64_t now_us)
{
    s_ads_sum_low_mv = 0;
    s_ads_sum_high_mv = 0;
    s_ads_count_low = 0;
    s_ads_count_high = 0;
    s_ads_ready_count = 0;
    s_ads_busy_count = 0;
    s_ads_error_count = 0;
    s_ads_min_mv = UINT32_MAX;
    s_ads_max_mv = 0;
    s_ads_stats_start_us = now_us;
}

static void ads_delay_stats_ready(app_state_t *state, uint32_t voltage_mv, bool test_high)
{
    if (state == NULL) return;

    if (voltage_mv < s_ads_min_mv) s_ads_min_mv = voltage_mv;
    if (voltage_mv > s_ads_max_mv) s_ads_max_mv = voltage_mv;
    ++s_ads_ready_count;

    if (test_high) {
        s_ads_sum_high_mv += voltage_mv;
        ++s_ads_count_high;
    } else {
        s_ads_sum_low_mv += voltage_mv;
        ++s_ads_count_low;
    }
}

static void ads_delay_stats_maybe_log(app_state_t *state, int64_t now_us)
{
    if (state == NULL) return;
    if (s_ads_stats_start_us == 0) {
        ads_delay_stats_reset(now_us);
        return;
    }
    if (now_us - s_ads_stats_start_us < ADS_SYNC_STATS_WINDOW_US) return;

    uint32_t avg_low = s_ads_count_low == 0U ? 0U :
                       (uint32_t)((s_ads_sum_low_mv + s_ads_count_low / 2U) /
                                  s_ads_count_low);
    uint32_t avg_high = s_ads_count_high == 0U ? 0U :
                        (uint32_t)((s_ads_sum_high_mv + s_ads_count_high / 2U) /
                                   s_ads_count_high);
    int32_t signed_delta = (int32_t)avg_high - (int32_t)avg_low;
    uint32_t amplitude = signed_delta < 0 ? (uint32_t)-signed_delta : (uint32_t)signed_delta;
    state->analog.test_span_mv = amplitude;

    ESP_LOGI(TAG,
             "ADS 5ms: ready=%lu busy=%lu err=%lu low=%lu high=%lu delta=%ld amp=%lu span=%lu..%lu cnt=%lu/%lu",
             (unsigned long)s_ads_ready_count,
             (unsigned long)s_ads_busy_count,
             (unsigned long)s_ads_error_count,
             (unsigned long)avg_low,
             (unsigned long)avg_high,
             (long)signed_delta,
             (unsigned long)amplitude,
             (unsigned long)(s_ads_min_mv == UINT32_MAX ? 0U : s_ads_min_mv),
             (unsigned long)s_ads_max_mv,
             (unsigned long)s_ads_count_low,
             (unsigned long)s_ads_count_high);

    ads_delay_stats_reset(now_us);
}

static void i2c_worker_task(void *argument)
{
    app_state_t *state = (app_state_t *)argument;
    int64_t last_slow_us = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        int64_t now = esp_timer_get_time();

        if (!s_ads_conversion_pending) {
            (void)board_io_set_test_100hz(state, s_ads_conversion_level);
            if (ads1110_start_conversion() == ESP_OK) {
                s_ads_conversion_pending = true;
                s_ads_read_due_us = esp_timer_get_time() +
                                    ADS_CONVERSION_DELAY_MS * 1000LL;
            } else {
                ++s_ads_error_count;
            }
        } else if (now < s_ads_read_due_us) {
            /* Conversion is intentionally left alone until the selected delay expires. */
        } else {
            uint32_t ads_voltage_mv = 0;
            bool ads_ready = false;
            bool ads_valid = ads1110_read_result(state, &ads_voltage_mv, &ads_ready);
            if (ads_valid) {
                if (ads_ready) {
                    ads_delay_stats_ready(state, ads_voltage_mv, s_ads_conversion_level);
                } else {
                    ++s_ads_busy_count;
                }
                s_ads_conversion_level = !s_ads_conversion_level;
                (void)board_io_set_test_100hz(state, s_ads_conversion_level);
                if (ads1110_start_conversion() == ESP_OK) {
                    s_ads_conversion_pending = true;
                    s_ads_read_due_us = esp_timer_get_time() +
                                        ADS_CONVERSION_DELAY_MS * 1000LL;
                } else {
                    s_ads_conversion_pending = false;
                    ++s_ads_error_count;
                }
            } else {
                s_ads_conversion_pending = false;
                ++s_ads_error_count;
            }
        }
        ads_delay_stats_maybe_log(state, now);
        if (last_slow_us != 0 &&
            now - last_slow_us < I2C_WORKER_SLOW_PERIOD_MS * 1000LL) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(I2C_WORKER_FAST_PERIOD_MS));
            continue;
        }
        last_slow_us = now;

        ina238_monitor_update(state, now);

        if (state->control.channel_a_enabled || state->tps55289.output_enabled) {
            channelA_update(state, now, state->control.channel_a_enabled);
            if (state->control.channel_a_enabled && state->tps55289.last_error != ESP_OK) {
                if (s_channel_a_error_cycles < 3U) ++s_channel_a_error_cycles;
                if (s_channel_a_error_cycles >= 3U) {
                    ESP_LOGW(TAG, "Channel A disabled after 3 I2C error cycles");
                    control_force_channel_enabled('A', false);
                    state->control.channel_a_enabled = false;
                }
            } else {
                s_channel_a_error_cycles = 0U;
            }
        } else {
            s_channel_a_error_cycles = 0U;
            state->tps55289.output_enabled = false;
            state->tps55289.valid = false;
            state->tps55289.last_error = ESP_OK;
        }

        if (state->control.channel_b_enabled || state->lm51772.output_enabled) {
            channelB_update(state, now, state->control.channel_b_enabled);
            if (state->control.channel_b_enabled && state->lm51772.last_error != ESP_OK) {
                if (s_channel_b_error_cycles < 3U) ++s_channel_b_error_cycles;
                if (s_channel_b_error_cycles >= 3U) {
                    ESP_LOGW(TAG, "Channel B disabled after 3 I2C error cycles");
                    control_force_channel_enabled('B', false);
                    state->control.channel_b_enabled = false;
                }
            } else {
                s_channel_b_error_cycles = 0U;
            }
        } else {
            s_channel_b_error_cycles = 0U;
            state->lm51772.output_enabled = false;
            state->lm51772.valid = false;
            state->lm51772.last_error = ESP_OK;
        }

        recover_i2c_if_needed(state, now);
        board_io_update(state);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(I2C_WORKER_FAST_PERIOD_MS));
    }
}

esp_err_t i2c_worker_start(app_state_t *state)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;
    s_state = state;
    ESP_RETURN_ON_ERROR(board_io_init(), "i2c_worker", "board io init");

    BaseType_t ok = xTaskCreate(i2c_worker_task, "i2c_worker", 4096, s_state, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
