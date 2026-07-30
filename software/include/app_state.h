#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "probe_config.h"

#define CONTROL_DIGIT_WHOLE 0xFFU

typedef enum {
    APP_MODE_POWER_SUPPLY = 0,
    APP_MODE_GENERATOR,
    APP_MODE_UART,
    APP_MODE_LIN,
    APP_MODE_1WIRE,
    APP_MODE_RS485,
    APP_MODE_CAN,
    APP_MODE_I2C,
    APP_MODE_I2C_MASTER,
    APP_MODE_SETTING,
    APP_MODE_COUNT
} app_mode_t;

typedef enum {
    CONTROL_SELECT_U1 = 0,
    CONTROL_SELECT_I1 = 1,
    CONTROL_SELECT_U2 = 2,
    CONTROL_SELECT_I2 = 3,
    CONTROL_SELECT_NONE = 4,
    CONTROL_SELECT_GEN_FREQ = 5,
    CONTROL_SELECT_GEN_DUTY = 6,
    CONTROL_SELECT_GEN_ON = 7,
    CONTROL_SELECT_UART_BAUD = 8,
    CONTROL_SELECT_RS485_BAUD = 9,
    CONTROL_SELECT_CAN_BITRATE = 10,
    CONTROL_SELECT_LIN_BAUD = 11,
    CONTROL_SELECT_OVERCURRENT = 12,
    CONTROL_SELECT_OVERHEAT = 13,
    CONTROL_SELECT_OVERPOWER = 14,
    CONTROL_SELECT_VOLUME = 15,
    CONTROL_SELECT_LIN_MASK = 16,
    CONTROL_SELECT_CAN_MASK = 17,
    CONTROL_SELECT_I2C_MASK = 18,
} control_select_t;

typedef enum {
    PROBE_LOW,
    PROBE_HIGH,
    PROBE_UNDEFINED,
    PROBE_OPEN,
    PROBE_OVERVOLTAGE
} probe_logic_state_t;

typedef enum {
    PROBE_EVENT_NONE,
    PROBE_EVENT_RISE,
    PROBE_EVENT_FALL,
    PROBE_EVENT_HIGH_PULSE,
    PROBE_EVENT_LOW_PULSE
} probe_event_t;

typedef struct {
    uint32_t edge_count;
    uint32_t last_edge_age_us;
    uint32_t high_us;
    uint32_t low_us;
    uint32_t period_us;
    float frequency_hz;
    float duty_percent;
    bool signal_missing;
    bool event_visible;
    probe_event_t event;
} app_timing_state_t;

typedef struct {
    bool valid;
    uint32_t base_mv;
    uint32_t pulled_low_mv;
    uint32_t pulled_high_mv;
    uint32_t span_mv;
    probe_logic_state_t classification;
} app_floating_state_t;

typedef struct {
    uint32_t adc_mv;
    uint32_t input_mv;
    uint32_t voltage_mv;
    uint32_t vpp_mv;
    uint32_t test_span_mv;
    int32_t bias_current_na;
    bool test_visible;
    probe_logic_state_t logic_state;
    app_floating_state_t floating;
} app_analog_state_t;

typedef struct {
    char text[UART_DISPLAY_CHARS + 1U];
    char lines[3][UART_DISPLAY_CHARS + 1U];
    uint32_t errors;
} app_uart_state_t;

typedef struct {
    char text[I2C_DISPLAY_CHARS + 1U];
    char lines[4][I2C_DISPLAY_CHARS + 1U];
    uint32_t packets;
    uint32_t errors;
} app_i2c_sniffer_state_t;

typedef struct {
    char text[I2C_DISPLAY_CHARS + 1U];
    char lines[3][I2C_DISPLAY_CHARS + 1U];
    uint32_t packets;
    uint32_t errors;
    uint32_t tx_error_counter;
    uint32_t tx_queue_msgs;
    uint8_t tx_result;
    bool last_tx_ok;
    bool bus_off;
} app_can_state_t;

typedef struct {
    bool valid;
    bool wide_range;
    bool saturated;
    uint8_t address;
    uint32_t shunt_uohm;
    int32_t shunt_uv;
    int32_t current_ma;
    uint32_t bus_mv;
    int32_t temperature_mc;
} app_ina238_channel_t;

typedef struct {
    app_ina238_channel_t channel[2];
} app_ina238_state_t;

typedef struct {
    bool valid;
    bool output_enabled;
    bool ldo_dac_saturated;
    bool current_limit_active;
    bool status_valid;
    bool limit_valid;
    uint8_t address;
    uint8_t status;
    uint8_t limit_reg;
    int32_t last_error;
    uint16_t target_mv;
    uint16_t target_ma;
    uint16_t programmed_mv;
    uint16_t programmed_ma;
    uint16_t ldo_target_mv;
    uint16_t ldo_programmed_mv;
    uint16_t ldo_dac_mv;
    uint16_t ldo_dac_code;
} app_tps55289_state_t;

typedef struct {
    bool valid;
    bool output_enabled;
    bool current_limit_active;
    bool status_valid;
    bool limit_valid;
    uint8_t address;
    uint8_t status;
    uint8_t limit_reg;
    int32_t last_error;
    uint16_t target_mv;
    uint16_t target_ma;
    uint16_t programmed_mv;
    uint16_t programmed_ma;
} app_lm51772_state_t;

typedef struct {
    bool encoder_a;
    bool encoder_b;
    bool ui_button;
    bool s_button;
    bool mode_button;
    app_mode_t mode;
    bool menu_open;
    uint8_t menu_index;
    uint16_t u1_mv;
    uint16_t i1_ma;
    uint16_t u2_mv;
    uint16_t i2_ma;
    uint32_t generator_freq_hz;
    uint8_t generator_duty_percent;
    bool generator_on;
    uint32_t uart_baud;
    uint32_t lin_baud;
    uint32_t rs485_baud;
    uint32_t can_bitrate;
    char lin_mask[3];
    char can_mask[4];
    char i2c_mask[3];
    uint16_t lin_mask_value;
    uint8_t lin_mask_care;
    uint16_t can_mask_value;
    uint8_t can_mask_care;
    uint16_t i2c_mask_value;
    uint8_t i2c_mask_care;
    bool overcurrent_cc;
    uint8_t overheat_c;
    uint16_t overpower_w;
    uint8_t volume_percent;
    bool channel_a_enabled;
    bool channel_b_enabled;
    uint8_t selected_value;
    uint8_t selected_digit;
    int32_t last_encoder_steps;
} app_control_state_t;

typedef struct {
    uint32_t recovery_count;
    int32_t last_recovery_error;
} app_i2c_state_t;

typedef struct {
    app_analog_state_t analog;
    app_timing_state_t timing;
    app_uart_state_t uart;
    app_i2c_sniffer_state_t i2c_sniffer;
    app_can_state_t can;
    app_ina238_state_t ina238;
    app_tps55289_state_t tps55289;
    app_lm51772_state_t lm51772;
    app_control_state_t control;
    app_i2c_state_t i2c;
} app_state_t;

const char *app_logic_state_name(probe_logic_state_t state);
const char *app_event_name(probe_event_t event);
