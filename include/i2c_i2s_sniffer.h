#pragma once

#include "app_state.h"
#include "esp_err.h"

esp_err_t i2c_i2s_sniffer_init(void);
void i2c_i2s_sniffer_configure(app_mode_t mode);
void i2c_i2s_sniffer_update(app_state_t *state);
