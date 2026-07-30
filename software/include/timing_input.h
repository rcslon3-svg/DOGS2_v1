#pragma once

#include <stdbool.h>
#include "app_state.h"
#include "esp_err.h"

/* timing_input_init
 * Inputs: none.
 * Returns: ESP_OK on success, or an ESP-IDF error from MCPWM/PCNT setup.
 * Does: configures IO22 as the timing input for capture and edge counting.
 */
esp_err_t timing_input_init(void);

/* timing_input_update
 * Inputs:
 *   state  - application state to update; must not be NULL.
 *   now_us - current esp_timer_get_time() value truncated to 32 bits.
 * Returns: none.
 * Does: derives frequency, duty and pulse/event state, writing only
 * state->timing.
 */
void timing_input_update(app_state_t *state, uint32_t now_us);

/* timing_input_reset
 * Inputs: none.
 * Returns: none.
 * Does: clears captured timing statistics and event memory.
 */
void timing_input_reset(void);
