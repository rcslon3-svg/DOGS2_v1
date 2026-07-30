#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "app_state.h"
#include "esp_err.h"

esp_err_t can_probe_init(void);
void can_probe_configure(app_mode_t mode,
                         uint32_t bitrate,
                         uint16_t mask_value,
                         uint8_t mask_care);
void can_probe_update(app_state_t *state, int64_t now_us);
bool can_probe_input_char(char ch);

