#pragma once

#include "esp_err.h"

esp_err_t buzzer_init(void);
void buzzer_beep_1khz_50ms(void);
void buzzer_update(void);
