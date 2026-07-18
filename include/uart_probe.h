#pragma once

#include <stddef.h>
#include "app_state.h"
#include "esp_err.h"

/* uart_probe_init
 * Inputs: none.
 * Returns: ESP_OK on success, or an ESP-IDF error from UART/task setup.
 * Does: configures UART A TX/RX and starts the test TX plus receive tasks.
 */
esp_err_t uart_probe_init(void);

/* uart_probe_update
 * Inputs:
 *   state - application state to update; must not be NULL.
 * Returns: none.
 * Does: copies the last received UART text and error counter into state->uart.
 */
void uart_probe_update(app_state_t *state);

/* uart_probe_configure
 * Inputs: active application mode and selected UART/RS485 baud rates.
 * Returns: none.
 * Does: applies the real UART baud and enables/disables Bluetooth forwarding
 * according to the active screen.
 */
void uart_probe_configure(app_mode_t mode, uint32_t uart_baud, uint32_t rs485_baud);

/* uart_probe_write_bytes
 * Inputs: data points to bytes to transmit on UART TX.
 * Returns: none.
 * Does: sends Bluetooth-originated bytes out through the UART TX pin.
 */
void uart_probe_write_bytes(const char *data, size_t length);
