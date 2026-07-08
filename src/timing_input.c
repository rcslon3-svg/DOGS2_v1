#include "timing_input.h"

#include <string.h>

#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "probe_config.h"

/*
 * Frequency input.
 *
 * IO22 is connected to the ESP32 PCNT peripheral.
 * PCNT counts rising edges in hardware. The CPU does not receive an interrupt
 * on every edge.
 *
 * timing_input_update() periodically reads the counter and calculates:
 *
 *   frequency_hz = delta_edges / delta_time
 *
 * No duty, no pulse polarity, no edge event detection here.
 */

static pcnt_unit_handle_t pcnt_unit;
static uint32_t last_time_us;
static int last_count;

static float last_frequency_hz;
static uint32_t last_activity_us;

esp_err_t timing_input_init(void)
{
    pcnt_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767,
        .flags.accum_count = true,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, &pcnt_unit), "timing", "pcnt unit");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(pcnt_unit, unit_config.low_limit),
                        "timing", "pcnt low watch");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(pcnt_unit, unit_config.high_limit),
                        "timing", "pcnt high watch");

    pcnt_chan_config_t channel_config = {
        .edge_gpio_num = PROBE_COMPARATOR_GPIO,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t channel = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(pcnt_unit, &channel_config, &channel),
                        "timing", "pcnt channel");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(channel,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_HOLD),
                        "timing", "pcnt edge action");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(channel,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP),
                        "timing", "pcnt level action");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(pcnt_unit), "timing", "pcnt enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(pcnt_unit), "timing", "pcnt clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(pcnt_unit), "timing", "pcnt start");

    return ESP_OK;
}

void timing_input_update(app_state_t *state, uint32_t now_us)
{
    app_timing_state_t *out = &state->timing;
    memset(out, 0, sizeof(*out));

    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &count));

    out->edge_count = (uint32_t)count;

    if (last_time_us == 0U) {
        last_time_us = now_us;
        last_count = count;
        return;
    }

    uint32_t elapsed_us = now_us - last_time_us;
    int delta_edges = count - last_count;

    if (delta_edges > 0) {
        last_activity_us = now_us;
    }

    out->last_edge_age_us = last_activity_us == 0U ? UINT32_MAX : now_us - last_activity_us;

    bool publish = elapsed_us >= 1000000U;

    if (publish) {
        if (delta_edges >= 2) {
            last_frequency_hz = (float)delta_edges * 1000000.0f / (float)elapsed_us;
        } else {
            last_frequency_hz = 0.0f;
        }

        last_time_us = now_us;
        last_count = count;
    }

    out->frequency_hz = last_frequency_hz;
    out->signal_missing = out->last_edge_age_us > SIGNAL_MISSING_MS * 1000U;
    if (out->signal_missing) {
        out->frequency_hz = 0.0f;
        last_frequency_hz = 0.0f;
    }
}

void timing_input_reset(void)
{
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    last_time_us = 0;
    last_count = 0;
    last_frequency_hz = 0.0f;
    last_activity_us = 0;
}
