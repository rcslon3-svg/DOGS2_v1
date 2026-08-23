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
#define BUZZER_MAX_DUTY 512U

typedef struct {
    uint16_t frequency_hz;
    uint16_t duration_ms;
} buzzer_note_t;

static const buzzer_note_t s_overheat_warning[] = {
    {262U, 90U},
    {330U, 90U},
    {294U, 90U},
    {349U, 90U},
    {392U, 180U},
};

static const buzzer_note_t s_overpower_warning[] = {
    {880U, 70U},
    {0U, 50U},
    {880U, 70U},
    {0U, 50U},
    {1175U, 140U},
};

static bool s_initialized;
static bool s_active;
static int64_t s_stop_at_us;
static const buzzer_note_t *s_sequence;
static uint8_t s_sequence_count;
static uint8_t s_sequence_index;
static uint8_t s_volume_percent = 50U;

static uint32_t volume_to_duty(uint8_t volume_percent)
{
    if (volume_percent > 100U) volume_percent = 100U;
    return ((uint32_t)BUZZER_MAX_DUTY * (uint32_t)volume_percent + 50U) / 100U;
}

void buzzer_set_volume(uint8_t volume_percent)
{
    s_volume_percent = volume_percent > 100U ? 100U : volume_percent;
}

static void buzzer_stop_output(void)
{
    (void)ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    s_active = false;
}

static void buzzer_start_note(uint16_t frequency_hz, uint16_t duration_ms)
{
    if (frequency_hz == 0U || s_volume_percent == 0U) {
        buzzer_stop_output();
    } else {
        (void)ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, frequency_hz);
        (void)ledc_set_duty(BUZZER_LEDC_MODE,
                            BUZZER_LEDC_CHANNEL,
                            volume_to_duty(s_volume_percent));
        (void)ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
        s_active = true;
    }
    s_stop_at_us = esp_timer_get_time() + (int64_t)duration_ms * 1000LL;
}

static void buzzer_play_sequence(const buzzer_note_t *sequence,
                                 uint8_t count,
                                 uint8_t volume_percent)
{
    if (!s_initialized || BUZZER_OUTPUT_GPIO == GPIO_NUM_NC || sequence == NULL || count == 0U) {
        return;
    }

    s_sequence = sequence;
    s_sequence_count = count;
    s_sequence_index = 0U;
    buzzer_set_volume(volume_percent);
    buzzer_start_note(s_sequence[0].frequency_hz, s_sequence[0].duration_ms);
}

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
    (void)ledc_set_duty(BUZZER_LEDC_MODE,
                        BUZZER_LEDC_CHANNEL,
                        volume_to_duty(s_volume_percent));
    (void)ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    s_sequence = NULL;
    s_sequence_count = 0U;
    s_sequence_index = 0U;
    s_active = true;
    s_stop_at_us = esp_timer_get_time() + BUZZER_DURATION_US;
}

void buzzer_play_overheat_warning(uint8_t volume_percent)
{
    buzzer_play_sequence(s_overheat_warning,
                         (uint8_t)(sizeof(s_overheat_warning) / sizeof(s_overheat_warning[0])),
                         volume_percent);
}

void buzzer_play_overpower_warning(uint8_t volume_percent)
{
    buzzer_play_sequence(s_overpower_warning,
                         (uint8_t)(sizeof(s_overpower_warning) / sizeof(s_overpower_warning[0])),
                         volume_percent);
}

void buzzer_update(void)
{
    if (!s_active && s_sequence == NULL) return;
    if (esp_timer_get_time() < s_stop_at_us) return;

    if (s_sequence != NULL && ++s_sequence_index < s_sequence_count) {
        buzzer_start_note(s_sequence[s_sequence_index].frequency_hz,
                          s_sequence[s_sequence_index].duration_ms);
        return;
    }

    s_sequence = NULL;
    s_sequence_count = 0U;
    s_sequence_index = 0U;
    buzzer_stop_output();
}
