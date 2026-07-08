#pragma once

#include "app_state.h"
#include "esp_err.h"

esp_err_t channelB_init(void);
void channelB_i2c_release(void);
void channelB_update(app_state_t *state, int64_t now_us);
