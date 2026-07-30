#pragma once

#include <stddef.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t i2c_aux_bus_init(void);
esp_err_t i2c_aux_bus_start(void);
esp_err_t i2c_aux_bus_stop(void);
esp_err_t i2c_aux_bus_scan(uint8_t *addresses,
                           size_t capacity,
                           size_t *address_count);
esp_err_t i2c_aux_bus_transmit_receive(uint8_t address,
                                       const uint8_t *write_data,
                                       size_t write_length,
                                       uint8_t *read_data,
                                       size_t read_length,
                                       int timeout_ms);
esp_err_t i2c_aux_bus_receive(uint8_t address,
                              uint8_t *data,
                              size_t length,
                              int timeout_ms);
esp_err_t i2c_aux_bus_transmit(uint8_t address,
                               const uint8_t *data,
                               size_t length,
                               int timeout_ms);
esp_err_t i2c_aux_bus_recover_lines(void);
esp_err_t i2c_aux_bus_reset_controller(void);
