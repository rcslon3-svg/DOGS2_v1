#pragma once

#include "app_state.h"
#include "esp_err.h"

esp_err_t i2c_sniffer_init(void);
void i2c_sniffer_configure(app_mode_t mode, uint16_t mask_value, uint8_t mask_care);
void i2c_sniffer_update(app_state_t *state);
