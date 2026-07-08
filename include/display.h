#pragma once

#include "app_state.h"
#include "esp_err.h"

esp_err_t display_init(void);
void display_render(const app_state_t *snapshot);
void display_set_rgb(probe_logic_state_t state);
void display_black(void);
