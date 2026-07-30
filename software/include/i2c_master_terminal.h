#pragma once

#include "app_state.h"
#include "esp_err.h"

esp_err_t i2c_master_terminal_init(void);
void i2c_master_terminal_configure(app_mode_t mode);
bool i2c_master_terminal_input_char(char ch);
void i2c_master_terminal_update(app_state_t *state);
