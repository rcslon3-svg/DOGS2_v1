#include "encoder_input.h"

#include <stdint.h>
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "probe_config.h"

/*
 * Incremental encoder input module.
 *
 * Hardware:
 *   ENCODER_A_GPIO and ENCODER_B_GPIO are assigned in probe_config.h.
 *
 * Algorithm requested for comparison:
 *   1. Poll both encoder channels every ENCODER_POLL_US.
 *   2. Decode only valid Gray-code transitions:
 *      00 -> 01 -> 11 -> 10 -> 00 counts up.
 *      00 -> 10 -> 11 -> 01 -> 00 counts down.
 *   3. Accumulate four valid quarter-steps before reporting one detent.
 *      Invalid transitions are ignored.
 *
 * This module does not know anything about U1/I1/U2/I2, selected digit, limits
 * or decimal carry. It only produces signed steps.
 */

#define ENCODER_POLL_US 1000U
#define ENCODER_TRANSITIONS_PER_DETENT 4

static portMUX_TYPE s_encoder_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t s_encoder_steps;
static uint8_t s_encoder_previous_state;
static int8_t s_encoder_transition_accum;
static esp_timer_handle_t s_poll_timer;

/* read_level
 * Inputs: gpio is the input pin number.
 * Returns: true for high level, false for low level.
 * Does: reads one GPIO input without interpretation.
 */
static bool read_level(gpio_num_t gpio)
{
    return gpio_get_level(gpio) != 0;
}

/* encoder_add_step
 * Inputs: direction is +1 or -1.
 * Returns: none.
 * Does: adds one encoder step. The accumulator is later consumed by
 * encoder_input_take_steps() in the main/control context.
 */
static void encoder_add_step(int direction)
{
    portENTER_CRITICAL(&s_encoder_lock);
    s_encoder_steps += direction;
    portEXIT_CRITICAL(&s_encoder_lock);
}

/* encoder_read_state
 * Inputs: none.
 * Returns: two-bit encoder state: bit 1 is A, bit 0 is B.
 * Does: reads both encoder phases as one logical quadrature state.
 */
static uint8_t encoder_read_state(void)
{
    uint8_t a = gpio_get_level(ENCODER_A_GPIO) != 0 ? 2U : 0U;
    uint8_t b = gpio_get_level(ENCODER_B_GPIO) != 0 ? 1U : 0U;
    return (uint8_t)(a | b);
}

/* encoder_poll_timer_callback
 * Inputs: arg is unused.
 * Returns: none.
 * Does: samples A/B, decodes valid quadrature transitions, and converts four
 * quarter-steps into one signed detent.
 */
static void encoder_poll_timer_callback(void *arg)
{
    (void)arg;

    static const int8_t transition_table[16] = {
        0,  1, -1,  0,
       -1,  0,  0,  1,
        1,  0,  0, -1,
        0, -1,  1,  0,
    };
    uint8_t current_state = encoder_read_state();
    uint8_t transition = (uint8_t)((s_encoder_previous_state << 2) | current_state);
    int8_t delta = transition_table[transition];

    s_encoder_previous_state = current_state;
    if (delta == 0) return;

    s_encoder_transition_accum += delta;
    if (s_encoder_transition_accum >= ENCODER_TRANSITIONS_PER_DETENT) {
        s_encoder_transition_accum = 0;
        encoder_add_step(1);
    } else if (s_encoder_transition_accum <= -ENCODER_TRANSITIONS_PER_DETENT) {
        s_encoder_transition_accum = 0;
        encoder_add_step(-1);
    }
}

esp_err_t encoder_input_init(void)
{
    gpio_config_t input = {
        .pin_bit_mask = (1ULL << ENCODER_A_GPIO) | (1ULL << ENCODER_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&input), "encoder", "gpio config");

    s_encoder_steps = 0;
    s_encoder_transition_accum = 0;
    s_encoder_previous_state = encoder_read_state();

    const esp_timer_create_args_t timer_args = {
        .callback = encoder_poll_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "enc_poll",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_poll_timer),
                        "encoder", "poll timer create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_poll_timer, ENCODER_POLL_US),
                        "encoder", "poll timer start");

    return ESP_OK;
}

int32_t encoder_input_take_steps(void)
{
    portENTER_CRITICAL(&s_encoder_lock);
    int32_t steps = s_encoder_steps;
    s_encoder_steps = 0;
    portEXIT_CRITICAL(&s_encoder_lock);
    return steps;
}

bool encoder_input_get_a(void)
{
    return read_level(ENCODER_A_GPIO);
}

bool encoder_input_get_b(void)
{
    return read_level(ENCODER_B_GPIO);
}
