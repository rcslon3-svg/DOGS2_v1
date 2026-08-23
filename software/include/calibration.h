#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_state.h"
#include "esp_err.h"

#define CALIBRATION_CHANNEL_A 0U
#define CALIBRATION_CHANNEL_B 1U
#define CALIBRATION_VOLTAGE_LIMIT_MV 300

typedef struct {
    uint16_t target_mv;
    uint32_t measured_mv;
    int64_t measured_current_ua;
    bool valid;
} calibration_point_t;

typedef struct {
    calibration_point_t a[4];
    calibration_point_t b[6];
} calibration_data_t;

void calibration_load(void);
esp_err_t calibration_save(const calibration_data_t *data);
bool calibration_current_available(uint8_t channel);
int64_t calibration_correct_current_ua(uint8_t channel,
                                       uint32_t measured_mv,
                                       int64_t current_ua);
int32_t calibration_voltage_correction_mv(uint8_t channel, uint16_t target_mv);
void calibration_request_start(void);
bool calibration_take_start_request(void);
bool calibration_running(void);
void calibration_set_running(bool running);
void calibration_set_progress(char channel,
                              uint16_t target_mv,
                              uint32_t measured_mv,
                              int64_t measured_current_ua,
                              uint32_t sample_count);
void calibration_get_progress(char *channel,
                              uint16_t *target_mv,
                              uint32_t *measured_mv,
                              int64_t *measured_current_ua,
                              uint32_t *sample_count);
void calibration_mark_done(void);
bool calibration_done(void);
void calibration_clear_done(void);
