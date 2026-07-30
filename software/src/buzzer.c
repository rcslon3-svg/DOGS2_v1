#include "buzzer.h"

#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "probe_config.h"

#define BUZZER_LEDC_MODE LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_TIMER LEDC_TIMER_2
#define BUZZER_LEDC_CHANNEL LEDC_CHANNEL_2
#define BUZZER_LEDC_RESOLUTION LEDC_TIMER_10_BIT
#define BUZZER_FREQUENCY_HZ 1000U
#define BUZZER_DURATION_US 50000LL
#define BUZZER_DUTY 512U

static bool s_initialized;
static bool s_active;
static int64_t s_stop_at_us;

esp_err_t buzzer_init(void)
{
    if (BUZZER_OUTPUT_GPIO == GPIO_NUM_NC) return ESP_OK;

    ledc_timer_config_t timer = {
        .speed_mode = BUZZER_LEDC_MODE,
        .timer_num = BUZZER_LEDC_TIMER,
        .duty_resolution = BUZZER_LEDC_RESOLUTION,
        .freq_hz = BUZZER_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), "buzzer", "timer");

    ledc_channel_config_t channel = {
        .gpio_num = BUZZER_OUTPUT_GPIO,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel = BUZZER_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), "buzzer", "channel");
    (void)ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);

    s_initialized = true;
    s_active = false;
    return ESP_OK;
}

void buzzer_beep_1khz_50ms(void)
{
    if (!s_initialized || BUZZER_OUTPUT_GPIO == GPIO_NUM_NC) return;

    (void)ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, BUZZER_FREQUENCY_HZ);
    (void)ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY);
    (void)ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    s_active = true;
    s_stop_at_us = esp_timer_get_time() + BUZZER_DURATION_US;
}

void buzzer_update(void)
{
    if (!s_active) return;
    if (esp_timer_get_time() < s_stop_at_us) return;

    (void)ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    s_active = false;
}
