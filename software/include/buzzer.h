#pragma once

#include "esp_err.h"

esp_err_t buzzer_init(void);
void buzzer_set_volume(uint8_t volume_percent);
void buzzer_beep_1khz_50ms(void);
void buzzer_play_overheat_warning(uint8_t volume_percent);
void buzzer_play_overpower_warning(uint8_t volume_percent);
void buzzer_update(void);
