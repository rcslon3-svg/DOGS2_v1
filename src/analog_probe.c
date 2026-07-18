#include "analog_probe.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "probe_config.h"

/*
 * Input-voltage ADC monitor.
 *
 * GPIO36 measures VIN through the board divider defined in probe_config.h.
 * The old bias/test-probe GPIO logic is intentionally not used here because
 * GPIO27 is the encoder B input on this board.
 */

static adc_oneshot_unit_handle_t adc_unit;
static adc_cali_handle_t adc_cal;

static uint32_t last_sample_us;

static uint64_t sum_probe_mv;
static uint32_t sum_adc_mv;
static uint32_t sample_count;

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
    sum_adc_mv = 0;
    sample_count = 0;
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

    if (sample_count < ANALOG_WINDOW_SAMPLES) {
        return;
    }

    uint32_t avg_mv = (uint32_t)((sum_probe_mv + sample_count / 2U) / sample_count);
    uint32_t adc_avg_mv = (sum_adc_mv + sample_count / 2U) / sample_count;

    app->analog.adc_mv = adc_avg_mv;
    app->analog.voltage_mv = avg_mv;
    app->analog.vpp_mv = 0;
    app->analog.test_span_mv = 0;
    app->analog.bias_current_na = 0;
    app->analog.test_visible = false;
    app->analog.logic_state = PROBE_UNDEFINED;

    clear_window();
}

void analog_probe_reset(void)
{
    clear_window();
}
