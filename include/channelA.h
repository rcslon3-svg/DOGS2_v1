#pragma once

#include "app_state.h"
#include "esp_err.h"

esp_err_t channelA_init(void);
void channelA_i2c_release(void);
void channelA_update(app_state_t *state, int64_t now_us);
