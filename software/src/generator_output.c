#include "generator_output.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "nvs.h"
#include "probe_config.h"

#define GENERATOR_NVS_NAMESPACE "generator"
#define GENERATOR_SAVE_DELAY_US 1000000LL
#define GENERATOR_LEDC_SOURCE_HZ 80000000ULL
#define GENERATOR_LEDC_MODE LEDC_HIGH_SPEED_MODE
#define GENERATOR_LEDC_TIMER LEDC_TIMER_1
#define GENERATOR_LEDC_CHANNEL LEDC_CHANNEL_1

static generator_output_state_t s_state = {
    .frequency_hz = 1000U,
    .duty_percent = 50U,
    .enabled = false,
};
static bool s_dirty;
static int64_t s_last_change_us;

static uint32_t clamp_u32(uint32_t value, uint32_t minimum, uint32_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static uint8_t clamp_u8(uint8_t value, uint8_t minimum, uint8_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static ledc_timer_bit_t select_ledc_resolution(uint32_t frequency_hz)
{
    uint32_t bits = 10U;
    while (bits > 1U && ((uint64_t)frequency_hz << bits) > GENERATOR_LEDC_SOURCE_HZ) {
        --bits;
    }
    return (ledc_timer_bit_t)bits;
}

static esp_err_t drive_low(void)
{
    (void)ledc_stop(GENERATOR_LEDC_MODE, GENERATOR_LEDC_CHANNEL, 0);
    gpio_config_t output = {
        .pin_bit_mask = (1ULL << GENERATOR_OUTPUT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output), "generator", "gpio config");
    gpio_set_level(GENERATOR_OUTPUT_GPIO, 0);
    return ESP_OK;
}

static esp_err_t apply_output(void)
{
    if (!s_state.enabled) return drive_low();

    gpio_reset_pin(GENERATOR_OUTPUT_GPIO);

    ledc_timer_bit_t resolution = select_ledc_resolution(s_state.frequency_hz);
    ledc_timer_config_t timer = {
        .speed_mode = GENERATOR_LEDC_MODE,
        .timer_num = GENERATOR_LEDC_TIMER,
        .duty_resolution = resolution,
        .freq_hz = s_state.frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), "generator", "ledc timer");

    uint32_t duty_steps = 1UL << (uint32_t)resolution;
    uint32_t ledc_duty = (duty_steps * (uint32_t)s_state.duty_percent + 50U) / 100U;
    if (ledc_duty >= duty_steps) ledc_duty = duty_steps - 1U;

    ledc_channel_config_t channel = {
        .gpio_num = GENERATOR_OUTPUT_GPIO,
        .speed_mode = GENERATOR_LEDC_MODE,
        .channel = GENERATOR_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = GENERATOR_LEDC_TIMER,
        .duty = ledc_duty,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), "generator", "ledc channel");
    ESP_RETURN_ON_ERROR(ledc_set_duty(GENERATOR_LEDC_MODE,
                                      GENERATOR_LEDC_CHANNEL,
                                      ledc_duty),
                        "generator", "ledc duty");
    return ledc_update_duty(GENERATOR_LEDC_MODE, GENERATOR_LEDC_CHANNEL);
}

static void load_state(void)
{
    nvs_handle_t nvs;
    if (nvs_open(GENERATOR_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;

    uint32_t frequency = 0;
    uint8_t duty = 0;
    uint8_t enabled = 0;
    if (nvs_get_u32(nvs, "freq", &frequency) == ESP_OK) {
        s_state.frequency_hz = clamp_u32(frequency, GENERATOR_FREQ_MIN_HZ, GENERATOR_FREQ_MAX_HZ);
    }
    if (nvs_get_u8(nvs, "duty", &duty) == ESP_OK) {
        s_state.duty_percent = clamp_u8(duty, GENERATOR_DUTY_MIN_PERCENT, GENERATOR_DUTY_MAX_PERCENT);
    }
    if (nvs_get_u8(nvs, "on", &enabled) == ESP_OK) {
        s_state.enabled = enabled != 0U;
    }

    nvs_close(nvs);
}

static esp_err_t save_state(void)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(GENERATOR_NVS_NAMESPACE, NVS_READWRITE, &nvs),
                        "generator", "nvs open");

    esp_err_t err = ESP_OK;
    if ((err = nvs_set_u32(nvs, "freq", s_state.frequency_hz)) != ESP_OK ||
        (err = nvs_set_u8(nvs, "duty", s_state.duty_percent)) != ESP_OK ||
        (err = nvs_set_u8(nvs, "on", s_state.enabled ? 1U : 0U)) != ESP_OK ||
        (err = nvs_commit(nvs)) != ESP_OK) {
        nvs_close(nvs);
        return err;
    }

    nvs_close(nvs);
    return ESP_OK;
}

static void mark_dirty(void)
{
    s_dirty = true;
    s_last_change_us = esp_timer_get_time();
}

esp_err_t generator_output_init(void)
{
    load_state();
    return apply_output();
}

generator_output_state_t generator_output_get_state(void)
{
    return s_state;
}

void generator_output_set_frequency(uint32_t frequency_hz)
{
    frequency_hz = clamp_u32(frequency_hz, GENERATOR_FREQ_MIN_HZ, GENERATOR_FREQ_MAX_HZ);
    if (frequency_hz == s_state.frequency_hz) return;
    s_state.frequency_hz = frequency_hz;
    (void)apply_output();
    mark_dirty();
}

void generator_output_set_duty(uint8_t duty_percent)
{
    duty_percent = clamp_u8(duty_percent,
                            GENERATOR_DUTY_MIN_PERCENT,
                            GENERATOR_DUTY_MAX_PERCENT);
    if (duty_percent == s_state.duty_percent) return;
    s_state.duty_percent = duty_percent;
    (void)apply_output();
    mark_dirty();
}

void generator_output_set_enabled(bool enabled)
{
    if (enabled == s_state.enabled) return;
    s_state.enabled = enabled;
    (void)apply_output();
    mark_dirty();
}

void generator_output_persistence_update(int64_t now_us)
{
    if (!s_dirty) return;
    if (now_us - s_last_change_us < GENERATOR_SAVE_DELAY_US) return;

    if (save_state() == ESP_OK) {
        s_dirty = false;
    } else {
        s_last_change_us = now_us;
    }
}
