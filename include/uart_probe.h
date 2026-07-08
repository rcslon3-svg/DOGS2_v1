#pragma once

#include "app_state.h"
#include "esp_err.h"

/* uart_probe_init
 * Inputs: none.
 * Returns: ESP_OK on success, or an ESP-IDF error from UART/task setup.
 * Does: configures UART A RX on IO22 and starts the receive task.
 */
esp_err_t uart_probe_init(void);

/* uart_probe_update
 * Inputs:
 *   state - application state to update; must not be NULL.
 * Returns: none.
 * Does: copies the last received UART text and error counter into state->uart.
 */
void uart_probe_update(app_state_t *state);
