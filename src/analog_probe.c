#include "analog_probe.h"

#include <limits.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "probe_config.h"

/*
 * Analog probe.
 *
 * Init:
 *   PROBE_BIAS_GPIO = LOW.
 *
 * Measurement step, every ANALOG_SAMPLE_PERIOD_MS:
 *   1. read ADC;
 *   2. store the sample according to current PROBE_BIAS_GPIO state;
 *   3. toggle PROBE_BIAS_GPIO.
 *
 * Result is published after ANALOG_WINDOW_SAMPLES measurement steps.
 */

static adc_oneshot_unit_handle_t adc_unit;
static adc_cali_handle_t adc_cal;

static bool bias_high;
static uint32_t last_sample_us;

static uint64_t sum_probe_mv;
static uint64_t sum_probe_low_mv;
static uint64_t sum_probe_high_mv;
static uint32_t sum_adc_mv;
static uint32_t sample_count;
static uint32_t low_count;
static uint32_t high_count;
static uint32_t low_min_probe_mv;
static uint32_t low_max_probe_mv;
static uint32_t high_min_probe_mv;
static uint32_t high_max_probe_mv;
static uint32_t vpp_history[ANALOG_VPP_AVG_WINDOWS];
static uint32_t vpp_history_index;
static uint32_t vpp_history_count;

static uint32_t read_adc_mv(void)
{
    uint32_t values[ANALOG_ADC_READS];

    for (uint32_t i = 0; i < ANALOG_ADC_READS; ++i) {
        int raw = 0;
        int mv = 0;

        ESP_ERROR_CHECK(adc_oneshot_read(adc_unit, PROBE_ADC_CHANNEL, &raw));

        if (adc_cal != NULL && adc_cali_raw_to_voltage(adc_cal, raw, &mv) == ESP_OK) {
            /* calibrated value is ready */
        } else {
            mv = raw * 3100 / 4095;
        }

        values[i] = (uint32_t)mv;
    }

    for (uint32_t i = 1; i < ANALOG_ADC_READS; ++i) {
        uint32_t value = values[i];
        uint32_t j = i;
        while (j > 0 && values[j - 1U] > value) {
            values[j] = values[j - 1U];
            --j;
        }
        values[j] = value;
    }

    if ((ANALOG_ADC_READS & 1U) != 0U) {
        return values[ANALOG_ADC_READS / 2U];
    }

    return (values[ANALOG_ADC_READS / 2U - 1U] + values[ANALOG_ADC_READS / 2U] + 1U) / 2U;
}

static uint32_t adc_mv_to_probe_mv(uint32_t adc_mv)
{
    return (uint32_t)((float)adc_mv * ADC_INPUT_SCALE + 0.5f);
}

static void clear_window(void)
{
    sum_probe_mv = 0;
    sum_probe_low_mv = 0;
    sum_probe_high_mv = 0;
    sum_adc_mv = 0;
    sample_count = 0;
    low_count = 0;
    high_count = 0;
    low_min_probe_mv = UINT32_MAX;
    low_max_probe_mv = 0;
    high_min_probe_mv = UINT32_MAX;
    high_max_probe_mv = 0;
}

esp_err_t analog_probe_init(void)
{
    adc_oneshot_unit_init_cfg_t adc_unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_unit_cfg, &adc_unit), "analog", "adc unit");

    adc_oneshot_chan_cfg_t adc_channel_cfg = {
        .atten = PROBE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_unit, PROBE_ADC_CHANNEL, &adc_channel_cfg),
                        "analog", "adc channel");

    adc_cali_line_fitting_config_t adc_cal_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = PROBE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .default_vref = 1100U,
    };
    if (adc_cali_create_scheme_line_fitting(&adc_cal_cfg, &adc_cal) != ESP_OK) {
        adc_cal = NULL;
    }

    bias_high = false;
    gpio_set_direction(PROBE_BIAS_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(PROBE_BIAS_GPIO, 0);
    clear_window();

    return ESP_OK;
}

void analog_probe_update(app_state_t *app, bool timing_quiet)
{
    (void)timing_quiet;

    uint32_t now_us = (uint32_t)esp_timer_get_time();
    if (last_sample_us != 0 &&
        now_us - last_sample_us < ANALOG_SAMPLE_PERIOD_MS * 1000U) {
        return;
    }
    last_sample_us = now_us;

    uint32_t adc_mv = read_adc_mv();
    uint32_t probe_mv = adc_mv_to_probe_mv(adc_mv);

    sum_probe_mv += probe_mv;
    sum_adc_mv += adc_mv;
    ++sample_count;

    if (bias_high) {
        sum_probe_high_mv += probe_mv;
        ++high_count;

        if (probe_mv < high_min_probe_mv) {
            high_min_probe_mv = probe_mv;
        }
        if (probe_mv > high_max_probe_mv) {
            high_max_probe_mv = probe_mv;
        }
    } else {
        sum_probe_low_mv += probe_mv;
        ++low_count;

        if (probe_mv < low_min_probe_mv) {
            low_min_probe_mv = probe_mv;
        }
        if (probe_mv > low_max_probe_mv) {
            low_max_probe_mv = probe_mv;
        }
    }

    bias_high = !bias_high;
    gpio_set_level(PROBE_BIAS_GPIO, bias_high ? 1 : 0);

    if (sample_count < ANALOG_WINDOW_SAMPLES || low_count == 0 || high_count == 0) {
        return;
    }

    uint32_t avg_mv = (uint32_t)((sum_probe_mv + sample_count / 2U) / sample_count);
    uint32_t adc_avg_mv = (sum_adc_mv + sample_count / 2U) / sample_count;
    uint32_t low_avg_mv = (uint32_t)((sum_probe_low_mv + low_count / 2U) / low_count);
    uint32_t high_avg_mv = (uint32_t)((sum_probe_high_mv + high_count / 2U) / high_count);
    uint32_t test_span_mv = high_avg_mv > low_avg_mv
        ? high_avg_mv - low_avg_mv
        : low_avg_mv - high_avg_mv;
    uint32_t low_vpp_mv = low_max_probe_mv > low_min_probe_mv
        ? low_max_probe_mv - low_min_probe_mv
        : 0;
    uint32_t high_vpp_mv = high_max_probe_mv > high_min_probe_mv
        ? high_max_probe_mv - high_min_probe_mv
        : 0;
    uint32_t current_vpp_mv = high_vpp_mv > low_vpp_mv ? high_vpp_mv : low_vpp_mv;

    vpp_history[vpp_history_index] = current_vpp_mv;
    vpp_history_index = (vpp_history_index + 1U) % ANALOG_VPP_AVG_WINDOWS;
    if (vpp_history_count < ANALOG_VPP_AVG_WINDOWS) {
        ++vpp_history_count;
    }

    uint32_t vpp_sum_mv = 0;
    for (uint32_t i = 0; i < vpp_history_count; ++i) {
        vpp_sum_mv += vpp_history[i];
    }
    uint32_t vpp_mv = (vpp_sum_mv + vpp_history_count / 2U) / vpp_history_count;

    bool test_signal_visible = test_span_mv >= OPEN_TEST_DELTA_MV;
    probe_logic_state_t logic_state = PROBE_UNDEFINED;

    if (test_signal_visible) {
        logic_state = PROBE_OPEN;
    } else if (avg_mv > TIP_OVERVOLTAGE_MV) {
        logic_state = PROBE_OVERVOLTAGE;
    } else if (avg_mv <= LOGIC_LOW_MAX_MV) {
        logic_state = PROBE_LOW;
    } else if (avg_mv >= LOGIC_HIGH_MIN_MV) {
        logic_state = PROBE_HIGH;
    }

    app->analog.adc_mv = adc_avg_mv;
    app->analog.voltage_mv = avg_mv;
    app->analog.vpp_mv = vpp_mv;
    app->analog.test_span_mv = test_span_mv;
    app->analog.bias_current_na =
        (int32_t)(((int64_t)(bias_high ? 3300 : 0) - avg_mv) * 1000000LL /
                  BIAS_RESISTOR_OHM);
    app->analog.test_visible = test_signal_visible;
    app->analog.logic_state = logic_state;

    clear_window();
}

void analog_probe_reset(void)
{
    clear_window();
    for (uint32_t i = 0; i < ANALOG_VPP_AVG_WINDOWS; ++i) {
        vpp_history[i] = 0;
    }
    vpp_history_index = 0;
    vpp_history_count = 0;
}
