#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define GENERATOR_FREQ_MIN_HZ 1U
#define GENERATOR_FREQ_MAX_HZ 100000U
#define GENERATOR_DUTY_MIN_PERCENT 1U
#define GENERATOR_DUTY_MAX_PERCENT 99U

typedef struct {
    uint32_t frequency_hz;
    uint8_t duty_percent;
    bool enabled;
} generator_output_state_t;

esp_err_t generator_output_init(void);
generator_output_state_t generator_output_get_state(void);
void generator_output_set_frequency(uint32_t frequency_hz);
void generator_output_set_duty(uint8_t duty_percent);
void generator_output_set_enabled(bool enabled);
void generator_output_persistence_update(int64_t now_us);
