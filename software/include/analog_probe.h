#pragma once

#include "app_state.h"
#include "esp_err.h"

esp_err_t analog_probe_init(void);
void analog_probe_update(app_state_t *app, bool timing_quiet);
void analog_probe_reset(void);