#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t i2c_bus_get(i2c_master_bus_handle_t *bus);
esp_err_t i2c_bus_probe(uint8_t address);
esp_err_t i2c_bus_recover_lines(void);
esp_err_t i2c_bus_reset_controller(void);
esp_err_t i2c_bus_release(void);
