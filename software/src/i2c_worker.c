#include "i2c_worker.h"

#include <string.h>

#include "board_io.h"
#include "calibration.h"
#include "channelA.h"
#include "channelB.h"
#include "control.h"
#include "current_graph.h"
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
#define I2C_WORKER_SLOW_LOG_US 20000LL
#define I2C_WORKER_ADS_TIMEOUT_MS 8
#define I2C_WORKER_ADS_ERROR_BACKOFF_US 200000LL
#define ADS1110_CONFIG_240SPS_SINGLE_GAIN1 0x10U
#define ADS1110_CONFIG_START_SINGLE_240SPS_GAIN1 0x90U
#define ADS1110_240SPS_LSB_NV 1000000LL
#define ADS_CONVERSION_DELAY_MS 5U
#define ADS_SYNC_STATS_WINDOW_US 1000000LL
#define CALIBRATION_SETTLE_MS 4000U
#define CALIBRATION_SAMPLE_COUNT 32U

static const char *TAG = "i2c_worker";
static const uint16_t s_cal_a_points[] = {5000U, 10000U, 15000U, 20000U};
static const uint16_t s_cal_b_points[] = {5000U, 10000U, 15000U, 20000U, 30000U, 40000U};
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
static int64_t s_ads_next_retry_us;
static bool s_ads_conversion_pending;
static bool s_ads_conversion_level;

typedef enum {
    CAL_STATE_IDLE,
    CAL_STATE_SET_POINT,
    CAL_STATE_SETTLE,
    CAL_STATE_SAMPLE,
    CAL_STATE_FINISH,
} calibration_worker_state_t;

static calibration_worker_state_t s_cal_state = CAL_STATE_IDLE;
static calibration_data_t s_cal_data;
static uint8_t s_cal_channel;
static uint8_t s_cal_index;
static int64_t s_cal_step_started_us;
static uint32_t s_cal_sample_count;
static uint64_t s_cal_sum_mv;
static int64_t s_cal_sum_current_ua;
static uint16_t s_cal_restore_u1_mv;
static uint16_t s_cal_restore_i1_ma;
static uint16_t s_cal_restore_u2_mv;
static uint16_t s_cal_restore_i2_ma;
static bool s_cal_restore_a_enabled;
static bool s_cal_restore_b_enabled;

static void ads1110_i2c_release(void);

static void i2c_worker_delay_until(TickType_t *last_wake)
{
    TickType_t before = xTaskGetTickCount();
    (void)xTaskDelayUntil(last_wake, pdMS_TO_TICKS(I2C_WORKER_FAST_PERIOD_MS));
    if (xTaskGetTickCount() == before) {
        vTaskDelay(1);
        *last_wake = xTaskGetTickCount();
    }
}

static bool i2c_errors_visible(const app_state_t *state)
{
    if (state == NULL) return false;
    return state->tps55289.last_error == ESP_ERR_TIMEOUT ||
           state->lm51772.last_error == ESP_ERR_TIMEOUT;
}

static void recover_i2c_if_needed(app_state_t *state, int64_t now_us)
{
    if (!i2c_errors_visible(state)) return;
    if (now_us - s_last_recovery_us < I2C_RECOVERY_MIN_INTERVAL_US) return;
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
    return i2c_master_transmit(s_ads1110,
                               &ads_config,
                               sizeof(ads_config),
                               I2C_WORKER_ADS_TIMEOUT_MS);
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
    return i2c_master_transmit(s_ads1110,
                               &ads_config,
                               sizeof(ads_config),
                               I2C_WORKER_ADS_TIMEOUT_MS);
}

static bool ads1110_read_result(app_state_t *state, uint32_t *voltage_mv, bool *ready)
{
    if (state == NULL) return false;
    if (voltage_mv != NULL) *voltage_mv = 0;
    if (ready != NULL) *ready = false;
    if (s_ads1110 == NULL && ads1110_init() != ESP_OK) return false;

    uint8_t bytes[3] = {0};
    esp_err_t err = i2c_master_receive(s_ads1110,
                                       bytes,
                                       sizeof(bytes),
                                       I2C_WORKER_ADS_TIMEOUT_MS);
    if (err != ESP_OK) {
        state->analog.logic_state = PROBE_UNDEFINED;
        return false;
    }
    if ((bytes[2] & 0x70U) != ADS1110_CONFIG_240SPS_SINGLE_GAIN1) {
        uint8_t ads_config = ADS1110_CONFIG_START_SINGLE_240SPS_GAIN1;
        (void)i2c_master_transmit(s_ads1110,
                                  &ads_config,
                                  sizeof(ads_config),
                                  I2C_WORKER_ADS_TIMEOUT_MS);
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

static uint16_t calibration_point_target_mv(void)
{
    if (s_cal_channel == CALIBRATION_CHANNEL_A) return s_cal_a_points[s_cal_index];
    return s_cal_b_points[s_cal_index];
}

static uint8_t calibration_point_count(void)
{
    if (s_cal_channel == CALIBRATION_CHANNEL_A) {
        return (uint8_t)(sizeof(s_cal_a_points) / sizeof(s_cal_a_points[0]));
    }
    return (uint8_t)(sizeof(s_cal_b_points) / sizeof(s_cal_b_points[0]));
}

static void calibration_apply_override(app_state_t *state)
{
    uint16_t target_mv = calibration_point_target_mv();
    bool channel_a = s_cal_channel == CALIBRATION_CHANNEL_A;
    bool channel_b = s_cal_channel == CALIBRATION_CHANNEL_B;

    control_calibration_override(true,
                                 channel_a ? target_mv : 0U,
                                 2000U,
                                 channel_a,
                                 channel_b ? target_mv : 0U,
                                 5000U,
                                 channel_b);
    state->control.u2_mv = channel_a ? target_mv : 0U;
    state->control.i2_ma = 2000U;
    state->control.channel_a_enabled = channel_a;
    state->control.u1_mv = channel_b ? target_mv : 0U;
    state->control.i1_ma = 5000U;
    state->control.channel_b_enabled = channel_b;
}

static bool calibration_sample_valid(const app_state_t *state)
{
    if (s_cal_channel == CALIBRATION_CHANNEL_A) {
        return state->ina238.channel[0].valid &&
               state->tps55289.last_error == ESP_OK &&
               state->tps55289.valid;
    }
    return state->ina238.channel[1].valid &&
           state->lm51772.last_error == ESP_OK &&
           state->lm51772.valid;
}

static const app_ina238_channel_t *calibration_measurement(const app_state_t *state)
{
    return s_cal_channel == CALIBRATION_CHANNEL_A ? &state->ina238.channel[0] :
                                                    &state->ina238.channel[1];
}

static void calibration_store_point(void)
{
    calibration_point_t *point = s_cal_channel == CALIBRATION_CHANNEL_A ?
                                 &s_cal_data.a[s_cal_index] :
                                 &s_cal_data.b[s_cal_index];
    point->target_mv = calibration_point_target_mv();
    point->measured_mv = (uint32_t)((s_cal_sum_mv + s_cal_sample_count / 2U) /
                                    s_cal_sample_count);
    point->measured_current_ua = s_cal_sum_current_ua / (int64_t)s_cal_sample_count;
    point->valid = true;
    calibration_set_progress(s_cal_channel == CALIBRATION_CHANNEL_A ? 'A' : 'B',
                             point->target_mv,
                             point->measured_mv,
                             point->measured_current_ua,
                             s_cal_sample_count);

    ESP_LOGI(TAG,
             "cal %c %u mV: measured=%lu mV current=%lld uA samples=%lu",
             s_cal_channel == CALIBRATION_CHANNEL_A ? 'A' : 'B',
             (unsigned)point->target_mv,
             (unsigned long)point->measured_mv,
             (long long)point->measured_current_ua,
             (unsigned long)s_cal_sample_count);
}

static void calibration_log_table(const calibration_data_t *data)
{
    if (data == NULL) return;
    ESP_LOGI(TAG, "calibration table:");
    ESP_LOGI(TAG, "CH SET_mV MEAS_mV ERR_mV I_uA VALID");
    for (size_t i = 0U; i < sizeof(data->a) / sizeof(data->a[0]); ++i) {
        const calibration_point_t *p = &data->a[i];
        ESP_LOGI(TAG,
                 "A %u %lu %ld %lld %u",
                 (unsigned)p->target_mv,
                 (unsigned long)p->measured_mv,
                 (long)((int32_t)p->target_mv - (int32_t)p->measured_mv),
                 (long long)p->measured_current_ua,
                 p->valid ? 1U : 0U);
    }
    for (size_t i = 0U; i < sizeof(data->b) / sizeof(data->b[0]); ++i) {
        const calibration_point_t *p = &data->b[i];
        ESP_LOGI(TAG,
                 "B %u %lu %ld %lld %u",
                 (unsigned)p->target_mv,
                 (unsigned long)p->measured_mv,
                 (long)((int32_t)p->target_mv - (int32_t)p->measured_mv),
                 (long long)p->measured_current_ua,
                 p->valid ? 1U : 0U);
    }
}

static void calibration_next_point(void)
{
    ++s_cal_index;
    if (s_cal_index < calibration_point_count()) {
        s_cal_state = CAL_STATE_SET_POINT;
        return;
    }
    if (s_cal_channel == CALIBRATION_CHANNEL_A) {
        s_cal_channel = CALIBRATION_CHANNEL_B;
        s_cal_index = 0U;
        s_cal_state = CAL_STATE_SET_POINT;
        return;
    }
    s_cal_state = CAL_STATE_FINISH;
}

static void calibration_finish(app_state_t *state)
{
    esp_err_t err = calibration_save(&s_cal_data);
    if (err != ESP_OK) ESP_LOGW(TAG, "cal save: %s", esp_err_to_name(err));
    calibration_log_table(&s_cal_data);

    control_calibration_override(false, 0U, 0U, false, 0U, 0U, false);
    calibration_set_running(false);
    calibration_mark_done();
    state->control.u1_mv = s_cal_restore_u1_mv;
    state->control.i1_ma = s_cal_restore_i1_ma;
    state->control.u2_mv = s_cal_restore_u2_mv;
    state->control.i2_ma = s_cal_restore_i2_ma;
    state->control.channel_a_enabled = s_cal_restore_a_enabled;
    state->control.channel_b_enabled = s_cal_restore_b_enabled;
    s_cal_state = CAL_STATE_IDLE;
    ESP_LOGI(TAG, "calibration finished: %s", esp_err_to_name(err));
}

static void calibration_worker_update(app_state_t *state, int64_t now_us)
{
    if (s_cal_state == CAL_STATE_IDLE) {
        if (!calibration_take_start_request()) return;

        s_cal_restore_u1_mv = state->control.u1_mv;
        s_cal_restore_i1_ma = state->control.i1_ma;
        s_cal_restore_u2_mv = state->control.u2_mv;
        s_cal_restore_i2_ma = state->control.i2_ma;
        s_cal_restore_a_enabled = state->control.channel_a_enabled;
        s_cal_restore_b_enabled = state->control.channel_b_enabled;
        memset(&s_cal_data, 0, sizeof(s_cal_data));
        s_cal_channel = CALIBRATION_CHANNEL_A;
        s_cal_index = 0U;
        calibration_set_running(true);
        calibration_clear_done();
        calibration_set_progress('A', s_cal_a_points[0], 0U, 0, 0U);
        s_cal_state = CAL_STATE_SET_POINT;
        ESP_LOGI(TAG, "calibration started");
    }

    if (s_cal_state == CAL_STATE_SET_POINT) {
        calibration_apply_override(state);
        board_io_update(state);
        s_cal_step_started_us = now_us;
        s_cal_sample_count = 0U;
        s_cal_sum_mv = 0U;
        s_cal_sum_current_ua = 0;
        calibration_set_progress(s_cal_channel == CALIBRATION_CHANNEL_A ? 'A' : 'B',
                                 calibration_point_target_mv(),
                                 0U,
                                 0,
                                 0U);
        s_cal_state = CAL_STATE_SETTLE;
        ESP_LOGI(TAG, "cal %c set %u mV",
                 s_cal_channel == CALIBRATION_CHANNEL_A ? 'A' : 'B',
                 (unsigned)calibration_point_target_mv());
        return;
    }

    calibration_apply_override(state);

    if (s_cal_state == CAL_STATE_SETTLE) {
        const app_ina238_channel_t *measurement = calibration_measurement(state);
        if (calibration_sample_valid(state)) {
            calibration_set_progress(s_cal_channel == CALIBRATION_CHANNEL_A ? 'A' : 'B',
                                     calibration_point_target_mv(),
                                     measurement->bus_mv,
                                     measurement->current_ua,
                                     0U);
        }

        if (now_us - s_cal_step_started_us >= CALIBRATION_SETTLE_MS * 1000LL) {
            s_cal_state = CAL_STATE_SAMPLE;
        }
        return;
    }

    if (s_cal_state == CAL_STATE_SAMPLE) {
        if (calibration_sample_valid(state)) {
            const app_ina238_channel_t *measurement = calibration_measurement(state);
            s_cal_sum_mv += measurement->bus_mv;
            s_cal_sum_current_ua += measurement->current_ua;
            ++s_cal_sample_count;
            calibration_set_progress(s_cal_channel == CALIBRATION_CHANNEL_A ? 'A' : 'B',
                                     calibration_point_target_mv(),
                                     (uint32_t)((s_cal_sum_mv + s_cal_sample_count / 2U) /
                                                s_cal_sample_count),
                                     s_cal_sum_current_ua / (int64_t)s_cal_sample_count,
                                     s_cal_sample_count);
        }

        if (s_cal_sample_count >= CALIBRATION_SAMPLE_COUNT) {
            calibration_store_point();
            calibration_next_point();
        } else if (now_us - s_cal_step_started_us >=
                   (CALIBRATION_SETTLE_MS + 5000U) * 1000LL) {
            ESP_LOGW(TAG,
                     "cal %c %u mV skipped: samples=%lu",
                     s_cal_channel == CALIBRATION_CHANNEL_A ? 'A' : 'B',
                     (unsigned)calibration_point_target_mv(),
                     (unsigned long)s_cal_sample_count);
            calibration_next_point();
        }
        return;
    }

    if (s_cal_state == CAL_STATE_FINISH) {
        calibration_finish(state);
    }
}

static void i2c_worker_task(void *argument)
{
    app_state_t *state = (app_state_t *)argument;
    int64_t last_slow_us = 0;
    TickType_t last_wake = xTaskGetTickCount();
    bool graph_mode_active = false;

    while (true) {
        int64_t now = esp_timer_get_time();
        int64_t cycle_start_us = now;
        int64_t ads_time_us = 0;
        int64_t buttons_time_us = 0;
        int64_t ina_time_us = 0;
        int64_t cal_time_us = 0;
        int64_t ch_a_time_us = 0;
        int64_t ch_b_time_us = 0;
        int64_t recovery_time_us = 0;
        int64_t io_time_us = 0;

        int64_t section_start_us = esp_timer_get_time();
        if (!s_ads_conversion_pending && now < s_ads_next_retry_us) {
            /* ADS retry is intentionally delayed after an I2C error. */
        } else if (!s_ads_conversion_pending) {
            (void)board_io_set_test_100hz(state, s_ads_conversion_level);
            if (ads1110_start_conversion() == ESP_OK) {
                s_ads_conversion_pending = true;
                s_ads_read_due_us = esp_timer_get_time() +
                                    ADS_CONVERSION_DELAY_MS * 1000LL;
            } else {
                ++s_ads_error_count;
                s_ads_next_retry_us = now + I2C_WORKER_ADS_ERROR_BACKOFF_US;
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
                    s_ads_next_retry_us = now + I2C_WORKER_ADS_ERROR_BACKOFF_US;
                }
            } else {
                s_ads_conversion_pending = false;
                ++s_ads_error_count;
                s_ads_next_retry_us = now + I2C_WORKER_ADS_ERROR_BACKOFF_US;
            }
        }
        ads_time_us = esp_timer_get_time() - section_start_us;
        ads_delay_stats_maybe_log(state, now);

        section_start_us = esp_timer_get_time();
        board_io_poll_ui_buttons();
        buttons_time_us = esp_timer_get_time() - section_start_us;
        if (last_slow_us != 0 &&
            now - last_slow_us < I2C_WORKER_SLOW_PERIOD_MS * 1000LL) {
            i2c_worker_delay_until(&last_wake);
            continue;
        }
        last_slow_us = now;

        bool graph_mode = state->control.mode == APP_MODE_1WIRE;
        if (graph_mode != graph_mode_active) {
            graph_mode_active = graph_mode;
            ina238_monitor_set_current_graph_mode(graph_mode);
            if (graph_mode) current_graph_reset();
        }

        section_start_us = esp_timer_get_time();
        ina238_monitor_update(state, now);
        ina_time_us = esp_timer_get_time() - section_start_us;

        current_graph_configure(state->control.current_graph_decimation);
        if (graph_mode_active) current_graph_add_ina_sample(state);

        section_start_us = esp_timer_get_time();
        calibration_worker_update(state, now);
        cal_time_us = esp_timer_get_time() - section_start_us;

        if (state->control.channel_a_enabled || state->tps55289.output_enabled) {
            section_start_us = esp_timer_get_time();
            channelA_update(state, now, state->control.channel_a_enabled);
            ch_a_time_us = esp_timer_get_time() - section_start_us;
            if (!state->control.overcurrent_cc &&
                state->control.channel_a_enabled &&
                state->tps55289.current_limit_active) {
                ESP_LOGW(TAG, "Channel A disabled by overcurrent trigger");
                control_force_channel_enabled('A', false);
                state->control.channel_a_enabled = false;
                s_channel_a_error_cycles = 0U;
            } else if (state->control.channel_a_enabled && state->tps55289.last_error != ESP_OK) {
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
            section_start_us = esp_timer_get_time();
            channelB_update(state, now, state->control.channel_b_enabled);
            ch_b_time_us = esp_timer_get_time() - section_start_us;
            if (!state->control.overcurrent_cc &&
                state->control.channel_b_enabled &&
                state->lm51772.current_limit_active) {
                ESP_LOGW(TAG, "Channel B disabled by overcurrent trigger");
                control_force_channel_enabled('B', false);
                state->control.channel_b_enabled = false;
                s_channel_b_error_cycles = 0U;
            } else if (state->control.channel_b_enabled && state->lm51772.last_error != ESP_OK) {
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

        section_start_us = esp_timer_get_time();
        recover_i2c_if_needed(state, now);
        recovery_time_us = esp_timer_get_time() - section_start_us;

        section_start_us = esp_timer_get_time();
        board_io_update(state);
        io_time_us = esp_timer_get_time() - section_start_us;

        int64_t total_time_us = esp_timer_get_time() - cycle_start_us;
        if (total_time_us > I2C_WORKER_SLOW_LOG_US) {
            ESP_LOGW(TAG,
                     "slow cycle %lld us: ads=%lld btn=%lld ina=%lld cal=%lld cha=%lld chb=%lld rec=%lld io=%lld",
                     (long long)total_time_us,
                     (long long)ads_time_us,
                     (long long)buttons_time_us,
                     (long long)ina_time_us,
                     (long long)cal_time_us,
                     (long long)ch_a_time_us,
                     (long long)ch_b_time_us,
                     (long long)recovery_time_us,
                     (long long)io_time_us);
        }

        i2c_worker_delay_until(&last_wake);
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
