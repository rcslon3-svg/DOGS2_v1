#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* encoder_input_init
 * Inputs: none.
 * Returns: ESP_OK on success or an ESP-IDF error from GPIO/timer setup.
 * Does: initializes the two GPIO inputs of the panel incremental encoder and
 * starts the periodic sampler that decodes encoder movement.
 */
esp_err_t encoder_input_init(void);

/* encoder_input_take_steps
 * Inputs: none.
 * Returns: signed number of encoder detents accumulated since the previous
 * call. Positive and negative values are rotation directions as decoded by the
 * current A/B transition table.
 * Does: atomically reads and clears the movement accumulator. The UI layer
 * consumes these steps and applies them to the selected setpoint.
 */
int32_t encoder_input_take_steps(void);

/* encoder_input_get_a
 * Inputs: none.
 * Returns: current raw logic level of encoder phase A.
 * Does: exposes the raw input for diagnostics/display only. It is not used by
 * the setpoint arithmetic.
 */
bool encoder_input_get_a(void);

/* encoder_input_get_b
 * Inputs: none.
 * Returns: current raw logic level of encoder phase B.
 * Does: exposes the raw input for diagnostics/display only. It is not used by
 * the setpoint arithmetic.
 */
bool encoder_input_get_b(void);
