#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef void (*bluetooth_command_cb_t)(char command);

esp_err_t bluetooth_spp_init(bluetooth_command_cb_t callback);
esp_err_t bluetooth_spp_deinit(void);
bool bluetooth_spp_initialized(void);
bool bluetooth_spp_connected(void);
void bluetooth_spp_write(const char *data, size_t length);
const char *bluetooth_spp_device_name(void);
uint16_t bluetooth_spp_board_number(void);
