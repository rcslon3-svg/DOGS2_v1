#pragma once

#include <stdbool.h>
#include "app_state.h"
#include "esp_err.h"

esp_err_t board_io_init(void);
void board_io_i2c_release(void);
void board_io_poll_ui_buttons(void);
void board_io_update(const app_state_t *state);
esp_err_t board_io_toggle_test_100hz(const app_state_t *state);
esp_err_t board_io_set_test_100hz(const app_state_t *state, bool high);
bool board_io_test_100hz_level(void);
esp_err_t board_io_read_ctrl_debug(uint8_t *input, uint8_t *output);
void board_io_set_rs485_tx_enabled(bool enabled);
esp_err_t board_io_read_ui_buttons(bool *encoder_button_high,
                                   bool *menu_button_high,
                                   bool *channel_a_button_high,
                                   bool *channel_b_button_high);
