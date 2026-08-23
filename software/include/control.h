#pragma once

#include "app_state.h"
#include "esp_err.h"

esp_err_t control_init(void);
void control_update(app_state_t *state);
void control_persistence_update(int64_t now_us);
void control_force_channel_enabled(char channel, bool enabled);
void control_calibration_override(bool active,
                                  uint16_t u2_mv,
                                  uint16_t i2_ma,
                                  bool channel_a_enabled,
                                  uint16_t u1_mv,
                                  uint16_t i1_ma,
                                  bool channel_b_enabled);
