#pragma once

#include "app_state.h"
#include "esp_err.h"

esp_err_t control_init(void);
void control_update(app_state_t *state);
void control_persistence_update(int64_t now_us);
void control_force_channel_enabled(char channel, bool enabled);
