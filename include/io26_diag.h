#pragma once

#include <stdbool.h>
#include "esp_err.h"

/* io26_diag_init
 * Inputs: none.
 * Returns: ESP_OK on success, or an ESP-IDF timer setup error.
 * Does: creates internal test timers/tasks and leaves IO26 in High-Z.
 */
esp_err_t io26_diag_init(void);

/* io26_diag_handle_command
 * Inputs:
 *   command - zero-terminated command line: h, r, v<mV>, f<Hz>-<duty>, u<baud>.
 * Returns: true when the command belonged to IO26 diagnostics.
 * Does: executes all modes that may drive IO26. This module is the sole owner
 * of IO26 output peripherals.
 */
bool io26_diag_handle_command(const char *command);
