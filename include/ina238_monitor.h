#pragma once

#include "app_state.h"
#include "esp_err.h"

esp_err_t ina238_monitor_init(void);
void ina238_monitor_i2c_release(void);
void ina238_monitor_update(app_state_t *state, int64_t now_us);
void ina238_monitor_update_address(app_state_t *state, int64_t now_us, uint8_t address);
void ina238_monitor_update_bus_voltage_address(app_state_t *state, int64_t now_us, uint8_t address);
void ina238_monitor_read_id_address(app_state_t *state, int64_t now_us, uint8_t address);
