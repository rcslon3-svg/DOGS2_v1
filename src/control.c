#include "control.h"

#include <stddef.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "encoder_input.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "generator_output.h"
#include "nvs.h"
#include "probe_config.h"

/*
 * Control input module.
 *
 * GPIOs:
 *   ENCODER_A_GPIO - encoder A
 *   ENCODER_B_GPIO - encoder B
 *   UI_BUTTON_GPIO - optional external UI button
 *   ENCODER_BUTTON_GPIO - encoder push button, same action as UI_BUTTON_GPIO
 *   MODE_BUTTON_GPIO - mode button, active low
 * A press on either UI_BUTTON_GPIO or ENCODER_BUTTON_GPIO selects channel A
 * voltage first, then enters digit editing for the selected value. There is no
 * separate S button in Logic_v2.
 *
 * This module owns the user controls:
 *   - debounces external and encoder buttons;
 *   - consumes already-decoded encoder steps from encoder_input.c;
 *   - changes U1/I1/U2/I2 set values.
 *
 * Values are stored as integers:
 *   U1/U2 - millivolts, step 10 mV;
 *   I1/I2 - milliamps, step 10 mA.
 */

#define BUTTON_DEBOUNCE_US 25000LL
#define SETTINGS_SAVE_DELAY_US 1000000LL
#define EDIT_TIMEOUT_US 30000000LL
#define SETTINGS_NVS_NAMESPACE "setpoints"
#define CONTROL_U1_MIN_MV 1000U
#define CONTROL_U1_MAX_MV 48000U
#define CONTROL_U2_MAX_MV 19000U
#define CONTROL_I1_MAX_MA CHANNEL_B_CURRENT_LIMIT_MAX_MA
#define CONTROL_I2_MAX_MA 2000U
#define CONTROL_CURRENT_LIMIT_STEP_MA 50U
#define CONTROL_OVERHEAT_MIN_C 40U
#define CONTROL_OVERHEAT_MAX_C 70U
#define CONTROL_OVERPOWER_MIN_W 50U
#define CONTROL_OVERPOWER_MAX_W 150U
#define CONTROL_VOLUME_MAX_PERCENT 100U

static uint16_t s_u1_mv = 12000;
static uint16_t s_i1_ma = 500;
static uint16_t s_u2_mv = 12000;
static uint16_t s_i2_ma = 500;
static bool s_overcurrent_cc = true;
static uint8_t s_overheat_c = 50U;
static uint16_t s_overpower_w = 100U;
static uint8_t s_volume_percent = 50U;
static app_mode_t s_mode = APP_MODE_POWER_SUPPLY;
static bool s_menu_open;
static uint8_t s_menu_index = APP_MODE_POWER_SUPPLY;
static size_t s_uart_baud_index = 6U;
static size_t s_rs485_baud_index = 6U;
static size_t s_can_bitrate_index = 2U;
static uint8_t s_selected_value = CONTROL_SELECT_NONE;
static uint8_t s_selected_digit = CONTROL_DIGIT_WHOLE;
static int32_t s_last_encoder_steps;
static bool s_settings_dirty;
static int64_t s_settings_last_change_us;
static int64_t s_last_control_activity_us;

static bool s_ui_raw;
static bool s_ui_stable;
static bool s_ui_pressed_previous;
static int64_t s_ui_last_change_us;

static bool s_encoder_button_raw;
static bool s_encoder_button_stable;
static bool s_encoder_button_pressed_previous;
static int64_t s_encoder_button_last_change_us;

static bool s_mode_button_raw;
static bool s_mode_button_stable;
static bool s_mode_button_pressed_previous;
static int64_t s_mode_button_last_change_us;

static const uint32_t s_serial_baud_rates[] = {
    2400U, 4800U, 9600U, 19200U, 38400U, 57600U, 115200U
};

static const uint32_t s_can_bitrates[] = {
    100000U, 125000U, 250000U, 500000U, 1000000U
};

/* read_level
 * Inputs: gpio is the input pin number.
 * Returns: true for high level, false for low level.
 * Does: reads one GPIO input with no interpretation.
 */
static bool read_level(gpio_num_t gpio)
{
    if (gpio == GPIO_NUM_NC) return true;
    return gpio_get_level(gpio) != 0;
}

static uint64_t optional_gpio_mask(gpio_num_t gpio)
{
    if (gpio == GPIO_NUM_NC) return 0ULL;
    return 1ULL << gpio;
}

/* debounce_pressed
 * Inputs:
 *   raw_high - raw GPIO level; buttons are active low.
 *   now_us - current monotonic time.
 *   last_raw/stable/last_change_us/pressed_previous - debouncer storage.
 * Returns: true once, on a new confirmed press.
 * Does: accepts a changed raw level after it has been stable for a fixed time.
 * This keeps button response independent from display redraw time.
 */
static bool debounce_pressed(bool raw_high,
                             int64_t now_us,
                             bool *last_raw,
                             bool *stable,
                             int64_t *last_change_us,
                             bool *pressed_previous)
{
    if (raw_high != *last_raw) {
        *last_raw = raw_high;
        *last_change_us = now_us;
    }

    if (raw_high != *stable &&
        now_us - *last_change_us >= BUTTON_DEBOUNCE_US) {
        *stable = raw_high;
    }

    bool pressed = !*stable;
    bool event = pressed && !*pressed_previous;
    *pressed_previous = pressed;
    return event;
}

/* selected_digit_count
 * Inputs: selected value index.
 * Returns: number of editable decimal digits for that value.
 * Does: voltage has four digits xx.xx. Current limits are selected from the
 * real converter DAC-code values, so they have one non-digit edit state.
 */
static uint8_t selected_digit_count(uint8_t selected)
{
    if (selected == CONTROL_SELECT_U1 || selected == CONTROL_SELECT_U2) return 4U;
    if (selected == CONTROL_SELECT_I1 || selected == CONTROL_SELECT_I2) return 1U;
    if (selected == CONTROL_SELECT_GEN_FREQ) return 6U;
    if (selected == CONTROL_SELECT_GEN_DUTY) return 2U;
    if (selected == CONTROL_SELECT_OVERCURRENT) return 1U;
    if (selected == CONTROL_SELECT_OVERHEAT) return 2U;
    if (selected == CONTROL_SELECT_OVERPOWER || selected == CONTROL_SELECT_VOLUME) return 3U;
    return 0U;
}

/* selected_step
 * Inputs: selected value index and selected digit.
 * Returns: one encoder step in mV or mA.
 * Does: maps the highlighted display digit to its natural decimal weight.
 * Voltage digits are xx.xx -> 10 V, 1 V, 0.1 V, 0.01 V.
 * Current limits step through adjacent 500 uV / 10 mOhm DAC codes: 50 mA.
 */
static uint16_t selected_step(uint8_t selected, uint8_t digit)
{
    static const uint16_t voltage_steps[4] = {10000U, 1000U, 100U, 10U};

    if ((selected == CONTROL_SELECT_U1 || selected == CONTROL_SELECT_U2) && digit < 4U) {
        return voltage_steps[digit];
    }
    if ((selected == CONTROL_SELECT_I1 || selected == CONTROL_SELECT_I2) && digit == 0U) {
        return CONTROL_CURRENT_LIMIT_STEP_MA;
    }
    if (selected == CONTROL_SELECT_OVERHEAT && digit < 2U) {
        static const uint16_t overheat_steps[2] = {10U, 1U};
        return overheat_steps[digit];
    }
    if ((selected == CONTROL_SELECT_OVERPOWER || selected == CONTROL_SELECT_VOLUME) && digit < 3U) {
        static const uint16_t percent_steps[3] = {100U, 10U, 1U};
        return percent_steps[digit];
    }
    return 0U;
}

static uint32_t selected_step_u32(uint8_t selected, uint8_t digit)
{
    static const uint32_t frequency_steps[6] = {
        100000U, 10000U, 1000U, 100U, 10U, 1U
    };
    static const uint32_t duty_steps[2] = {10U, 1U};

    if (selected == CONTROL_SELECT_GEN_FREQ && digit < 6U) return frequency_steps[digit];
    if (selected == CONTROL_SELECT_GEN_DUTY && digit < 2U) return duty_steps[digit];
    return 0U;
}

/* apply_signed_step
 * Inputs: current value, signed delta and maximum allowed value.
 * Returns: value + delta clipped to 0..maximum.
 * Does: performs the actual setpoint arithmetic. It is intentionally simple:
 * the highlighted digit only selects the step size; decimal carry/borrow is
 * the normal integer carry/borrow of the complete value.
 */
static uint16_t apply_signed_step(uint16_t value, int32_t delta, uint16_t maximum)
{
    int32_t next = (int32_t)value + delta;
    if (next < 0) return 0U;
    if (next > (int32_t)maximum) return maximum;
    return (uint16_t)next;
}

static uint16_t apply_signed_step_range(uint16_t value,
                                        int32_t delta,
                                        uint16_t minimum,
                                        uint16_t maximum)
{
    int32_t next = (int32_t)value + delta;
    if (next < (int32_t)minimum) return minimum;
    if (next > (int32_t)maximum) return maximum;
    return (uint16_t)next;
}

static uint32_t apply_signed_step_range_u32(uint32_t value,
                                            int32_t delta,
                                            uint32_t minimum,
                                            uint32_t maximum)
{
    int64_t next = (int64_t)value + (int64_t)delta;
    if (next < (int64_t)minimum) return minimum;
    if (next > (int64_t)maximum) return maximum;
    return (uint32_t)next;
}

static uint16_t clamp_loaded_value(uint16_t value, uint16_t maximum)
{
    return value > maximum ? maximum : value;
}

static uint16_t quantize_current_limit(uint16_t value, uint16_t maximum)
{
    if (value > maximum) value = maximum;
    uint16_t quantized = (uint16_t)(((uint32_t)value + CONTROL_CURRENT_LIMIT_STEP_MA / 2U) /
                                   CONTROL_CURRENT_LIMIT_STEP_MA *
                                   CONTROL_CURRENT_LIMIT_STEP_MA);
    return quantized > maximum ? maximum : quantized;
}

static uint16_t clamp_loaded_value_range(uint16_t value, uint16_t minimum, uint16_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void load_setpoints(void)
{
    nvs_handle_t nvs;
    if (nvs_open(SETTINGS_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;

    uint16_t value = 0;
    if (nvs_get_u16(nvs, "u1", &value) == ESP_OK) {
        s_u1_mv = clamp_loaded_value_range(value, CONTROL_U1_MIN_MV, CONTROL_U1_MAX_MV);
    }
    if (nvs_get_u16(nvs, "i1", &value) == ESP_OK) s_i1_ma = quantize_current_limit(value, CONTROL_I1_MAX_MA);
    if (nvs_get_u16(nvs, "u2", &value) == ESP_OK) s_u2_mv = clamp_loaded_value(value, CONTROL_U2_MAX_MV);
    if (nvs_get_u16(nvs, "i2", &value) == ESP_OK) s_i2_ma = quantize_current_limit(value, CONTROL_I2_MAX_MA);
    if (nvs_get_u16(nvs, "oc", &value) == ESP_OK) s_overcurrent_cc = value != 0U;
    if (nvs_get_u16(nvs, "oh", &value) == ESP_OK) {
        s_overheat_c = (uint8_t)clamp_loaded_value_range(value, CONTROL_OVERHEAT_MIN_C, CONTROL_OVERHEAT_MAX_C);
    }
    if (nvs_get_u16(nvs, "op", &value) == ESP_OK) {
        s_overpower_w = clamp_loaded_value_range(value, CONTROL_OVERPOWER_MIN_W, CONTROL_OVERPOWER_MAX_W);
    }
    if (nvs_get_u16(nvs, "vol", &value) == ESP_OK) {
        s_volume_percent = (uint8_t)clamp_loaded_value(value, CONTROL_VOLUME_MAX_PERCENT);
    }

    nvs_close(nvs);
}

static esp_err_t save_setpoints(void)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(SETTINGS_NVS_NAMESPACE, NVS_READWRITE, &nvs),
                        "control", "nvs open");

    esp_err_t err = ESP_OK;
    if ((err = nvs_set_u16(nvs, "u1", s_u1_mv)) != ESP_OK ||
        (err = nvs_set_u16(nvs, "i1", s_i1_ma)) != ESP_OK ||
        (err = nvs_set_u16(nvs, "u2", s_u2_mv)) != ESP_OK ||
        (err = nvs_set_u16(nvs, "i2", s_i2_ma)) != ESP_OK ||
        (err = nvs_set_u16(nvs, "oc", s_overcurrent_cc ? 1U : 0U)) != ESP_OK ||
        (err = nvs_set_u16(nvs, "oh", s_overheat_c)) != ESP_OK ||
        (err = nvs_set_u16(nvs, "op", s_overpower_w)) != ESP_OK ||
        (err = nvs_set_u16(nvs, "vol", s_volume_percent)) != ESP_OK ||
        (err = nvs_commit(nvs)) != ESP_OK) {
        nvs_close(nvs);
        return err;
    }

    nvs_close(nvs);
    return ESP_OK;
}

static void mark_setpoints_changed(void)
{
    s_settings_dirty = true;
    s_settings_last_change_us = esp_timer_get_time();
}

static void mark_control_activity(int64_t now_us)
{
    s_last_control_activity_us = now_us;
}

static void clear_selection(void)
{
    s_selected_value = CONTROL_SELECT_NONE;
    s_selected_digit = CONTROL_DIGIT_WHOLE;
}

static void open_mode_menu(void)
{
    s_menu_open = true;
    s_menu_index = (uint8_t)s_mode;
    clear_selection();
}

static void close_mode_menu(void)
{
    s_menu_open = false;
}

static void move_mode_menu(int direction)
{
    if (direction >= 0) {
        s_menu_index = (uint8_t)((s_menu_index + 1U) % (uint8_t)APP_MODE_COUNT);
    } else {
        s_menu_index = (s_menu_index == 0U) ? ((uint8_t)APP_MODE_COUNT - 1U) : (uint8_t)(s_menu_index - 1U);
    }
}

static void select_mode_menu_item(void)
{
    s_mode = (app_mode_t)s_menu_index;
    close_mode_menu();
    clear_selection();
}

static void select_whole_value(uint8_t selected)
{
    s_selected_value = selected;
    s_selected_digit = CONTROL_DIGIT_WHOLE;
}

static uint8_t first_selection_for_mode(app_mode_t mode)
{
    switch (mode) {
        case APP_MODE_POWER_SUPPLY: return CONTROL_SELECT_U2;
        case APP_MODE_GENERATOR:    return CONTROL_SELECT_GEN_FREQ;
        case APP_MODE_UART:         return CONTROL_SELECT_UART_BAUD;
        case APP_MODE_RS485:        return CONTROL_SELECT_RS485_BAUD;
        case APP_MODE_CAN:          return CONTROL_SELECT_CAN_BITRATE;
        case APP_MODE_SETTING:      return CONTROL_SELECT_OVERCURRENT;
        default:                    return CONTROL_SELECT_NONE;
    }
}

/* change_selected_value
 * Inputs: direction is +1 or -1 from the encoder.
 * Returns: none.
 * Does: changes the currently selected setpoint by the selected digit step.
 */
static void change_selected_value(int direction)
{
    if (s_selected_value == CONTROL_SELECT_NONE) return;
    if (s_selected_digit == CONTROL_DIGIT_WHOLE) return;
    uint16_t step = selected_step(s_selected_value, s_selected_digit);
    if (step == 0U) return;
    int32_t delta = (int32_t)direction * (int32_t)step;

    switch (s_selected_value) {
        case CONTROL_SELECT_U1:
        {
            uint16_t old = s_u1_mv;
            s_u1_mv = apply_signed_step_range(s_u1_mv,
                                              delta,
                                              CONTROL_U1_MIN_MV,
                                              CONTROL_U1_MAX_MV);
            if (s_u1_mv != old) mark_setpoints_changed();
            break;
        }
        case CONTROL_SELECT_I1:
        {
            uint16_t old = s_i1_ma;
            s_i1_ma = apply_signed_step(s_i1_ma, delta, CONTROL_I1_MAX_MA);
            if (s_i1_ma != old) mark_setpoints_changed();
            break;
        }
        case CONTROL_SELECT_U2:
        {
            uint16_t old = s_u2_mv;
            s_u2_mv = apply_signed_step(s_u2_mv, delta, CONTROL_U2_MAX_MV);
            if (s_u2_mv != old) mark_setpoints_changed();
            break;
        }
        case CONTROL_SELECT_I2:
        {
            uint16_t old = s_i2_ma;
            s_i2_ma = apply_signed_step(s_i2_ma, delta, CONTROL_I2_MAX_MA);
            if (s_i2_ma != old) mark_setpoints_changed();
            break;
        }
        default:
            break;
    }
}

static void change_setting_value(int direction)
{
    if (s_selected_value == CONTROL_SELECT_OVERCURRENT) {
        if (s_selected_digit != CONTROL_DIGIT_WHOLE) {
            s_overcurrent_cc = !s_overcurrent_cc;
            mark_setpoints_changed();
        }
        return;
    }

    if (s_selected_digit == CONTROL_DIGIT_WHOLE) return;
    uint16_t step = selected_step(s_selected_value, s_selected_digit);
    if (step == 0U) return;
    int32_t delta = (int32_t)direction * (int32_t)step;

    switch (s_selected_value) {
        case CONTROL_SELECT_OVERHEAT:
        {
            uint8_t old = s_overheat_c;
            s_overheat_c = (uint8_t)apply_signed_step_range(s_overheat_c,
                                                            delta,
                                                            CONTROL_OVERHEAT_MIN_C,
                                                            CONTROL_OVERHEAT_MAX_C);
            if (s_overheat_c != old) mark_setpoints_changed();
            break;
        }
        case CONTROL_SELECT_OVERPOWER:
        {
            uint16_t old = s_overpower_w;
            s_overpower_w = apply_signed_step_range(s_overpower_w,
                                                    delta,
                                                    CONTROL_OVERPOWER_MIN_W,
                                                    CONTROL_OVERPOWER_MAX_W);
            if (s_overpower_w != old) mark_setpoints_changed();
            break;
        }
        case CONTROL_SELECT_VOLUME:
        {
            uint8_t old = s_volume_percent;
            s_volume_percent = (uint8_t)apply_signed_step(s_volume_percent,
                                                          delta,
                                                          CONTROL_VOLUME_MAX_PERCENT);
            if (s_volume_percent != old) mark_setpoints_changed();
            break;
        }
        default:
            break;
    }
}

static void change_generator_value(int direction)
{
    generator_output_state_t generator = generator_output_get_state();

    if (s_selected_digit == CONTROL_DIGIT_WHOLE) {
        if (s_selected_value == CONTROL_SELECT_GEN_ON) {
            generator_output_set_enabled(!generator.enabled);
        }
        return;
    }

    uint32_t step = selected_step_u32(s_selected_value, s_selected_digit);
    if (step == 0U) return;
    int32_t delta = (int32_t)((int64_t)direction * (int64_t)step);

    switch (s_selected_value) {
        case CONTROL_SELECT_GEN_FREQ:
            generator_output_set_frequency(apply_signed_step_range_u32(generator.frequency_hz,
                                                                       delta,
                                                                       GENERATOR_FREQ_MIN_HZ,
                                                                       GENERATOR_FREQ_MAX_HZ));
            break;
        case CONTROL_SELECT_GEN_DUTY:
            generator_output_set_duty((uint8_t)apply_signed_step_range_u32(generator.duty_percent,
                                                                           delta,
                                                                           GENERATOR_DUTY_MIN_PERCENT,
                                                                           GENERATOR_DUTY_MAX_PERCENT));
            break;
        default:
            break;
    }
}

static void move_index(size_t *index, size_t count, int direction)
{
    if (count == 0U) return;
    if (direction >= 0) {
        *index = (*index + 1U) % count;
    } else {
        *index = (*index == 0U) ? (count - 1U) : (*index - 1U);
    }
}

static void change_protocol_value(int direction)
{
    switch (s_selected_value) {
        case CONTROL_SELECT_UART_BAUD:
            move_index(&s_uart_baud_index,
                       sizeof(s_serial_baud_rates) / sizeof(s_serial_baud_rates[0]),
                       direction);
            break;
        case CONTROL_SELECT_RS485_BAUD:
            move_index(&s_rs485_baud_index,
                       sizeof(s_serial_baud_rates) / sizeof(s_serial_baud_rates[0]),
                       direction);
            break;
        case CONTROL_SELECT_CAN_BITRATE:
            move_index(&s_can_bitrate_index,
                       sizeof(s_can_bitrates) / sizeof(s_can_bitrates[0]),
                       direction);
            break;
        default:
            break;
    }
}

static void move_setting_selection(int direction)
{
    if (direction >= 0) {
        switch (s_selected_value) {
            case CONTROL_SELECT_OVERCURRENT: select_whole_value(CONTROL_SELECT_OVERHEAT); break;
            case CONTROL_SELECT_OVERHEAT:    select_whole_value(CONTROL_SELECT_OVERPOWER); break;
            case CONTROL_SELECT_OVERPOWER:   select_whole_value(CONTROL_SELECT_VOLUME); break;
            case CONTROL_SELECT_VOLUME:      select_whole_value(CONTROL_SELECT_NONE); break;
            default:                         select_whole_value(CONTROL_SELECT_OVERCURRENT); break;
        }
    } else {
        switch (s_selected_value) {
            case CONTROL_SELECT_OVERCURRENT: select_whole_value(CONTROL_SELECT_NONE); break;
            case CONTROL_SELECT_OVERHEAT:    select_whole_value(CONTROL_SELECT_OVERCURRENT); break;
            case CONTROL_SELECT_OVERPOWER:   select_whole_value(CONTROL_SELECT_OVERHEAT); break;
            case CONTROL_SELECT_VOLUME:      select_whole_value(CONTROL_SELECT_OVERPOWER); break;
            default:                         select_whole_value(CONTROL_SELECT_VOLUME); break;
        }
    }
}

/* move_whole_selection
 * Inputs: direction is +1 or -1 from the encoder.
 * Returns: none.
 * Does: cycles whole-value selection:
 *   +: voltage A, voltage B, current A, current B, none.
 *   -: the reverse order.
 */
static void move_whole_selection(int direction)
{
    if (s_mode == APP_MODE_GENERATOR) {
        if (s_selected_value == CONTROL_SELECT_GEN_ON) {
            generator_output_state_t generator = generator_output_get_state();
            generator_output_set_enabled(!generator.enabled);
            select_whole_value(direction >= 0 ? CONTROL_SELECT_NONE : CONTROL_SELECT_GEN_DUTY);
            return;
        }
        if (direction >= 0) {
            switch (s_selected_value) {
                case CONTROL_SELECT_GEN_FREQ: select_whole_value(CONTROL_SELECT_GEN_DUTY); break;
                case CONTROL_SELECT_GEN_DUTY: select_whole_value(CONTROL_SELECT_GEN_ON); break;
                case CONTROL_SELECT_GEN_ON:   select_whole_value(CONTROL_SELECT_NONE); break;
                default:                      select_whole_value(CONTROL_SELECT_GEN_FREQ); break;
            }
        } else {
            switch (s_selected_value) {
                case CONTROL_SELECT_GEN_FREQ: select_whole_value(CONTROL_SELECT_NONE); break;
                case CONTROL_SELECT_GEN_DUTY: select_whole_value(CONTROL_SELECT_GEN_FREQ); break;
                case CONTROL_SELECT_GEN_ON:   select_whole_value(CONTROL_SELECT_GEN_DUTY); break;
                default:                      select_whole_value(CONTROL_SELECT_GEN_ON); break;
            }
        }
    } else if (s_mode == APP_MODE_SETTING) {
        move_setting_selection(direction);
    } else if (s_mode == APP_MODE_UART || s_mode == APP_MODE_RS485 || s_mode == APP_MODE_CAN) {
        if (s_selected_value == CONTROL_SELECT_NONE) select_whole_value(first_selection_for_mode(s_mode));
        else select_whole_value(CONTROL_SELECT_NONE);
    } else if (direction >= 0) {
        switch (s_selected_value) {
            case CONTROL_SELECT_U2:   select_whole_value(CONTROL_SELECT_U1); break;
            case CONTROL_SELECT_U1:   select_whole_value(CONTROL_SELECT_I2); break;
            case CONTROL_SELECT_I2:   select_whole_value(CONTROL_SELECT_I1); break;
            case CONTROL_SELECT_I1:   select_whole_value(CONTROL_SELECT_NONE); break;
            default:                  select_whole_value(CONTROL_SELECT_U2); break;
        }
    } else {
        switch (s_selected_value) {
            case CONTROL_SELECT_U2:   select_whole_value(CONTROL_SELECT_NONE); break;
            case CONTROL_SELECT_U1:   select_whole_value(CONTROL_SELECT_U2); break;
            case CONTROL_SELECT_I2:   select_whole_value(CONTROL_SELECT_U1); break;
            case CONTROL_SELECT_I1:   select_whole_value(CONTROL_SELECT_I2); break;
            default:                  select_whole_value(CONTROL_SELECT_I1); break;
        }
    }
}

/* button_advance_selection
 * Inputs: none.
 * Returns: none.
 * Does: one button selects channel A voltage from idle, enters digit editing
 * from a whole-value selection at the least significant digit, then moves
 * toward more significant digits and cycles back to whole-value selection.
 */
static void button_advance_selection(void)
{
    if (s_selected_value == CONTROL_SELECT_NONE) {
        select_whole_value(first_selection_for_mode(s_mode));
        return;
    }

    uint8_t count = selected_digit_count(s_selected_value);
    if (count == 0U) {
        if (s_selected_value == CONTROL_SELECT_GEN_ON) {
            generator_output_state_t generator = generator_output_get_state();
            generator_output_set_enabled(!generator.enabled);
        }
        return;
    }

    if (s_selected_digit == CONTROL_DIGIT_WHOLE) {
        s_selected_digit = (uint8_t)(count - 1U);
    } else if (s_selected_digit == 0U) {
        s_selected_digit = CONTROL_DIGIT_WHOLE;
    } else {
        --s_selected_digit;
    }
}

esp_err_t control_init(void)
{
    gpio_config_t input = {
        .pin_bit_mask = optional_gpio_mask(UI_BUTTON_GPIO) |
                        optional_gpio_mask(ENCODER_BUTTON_GPIO) |
                        optional_gpio_mask(MODE_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&input), "control", "gpio config");
    ESP_RETURN_ON_ERROR(encoder_input_init(), "control", "encoder init");
    ESP_RETURN_ON_ERROR(generator_output_init(), "control", "generator init");
    load_setpoints();

    s_ui_raw = read_level(UI_BUTTON_GPIO);
    s_ui_stable = s_ui_raw;
    s_encoder_button_raw = read_level(ENCODER_BUTTON_GPIO);
    s_encoder_button_stable = s_encoder_button_raw;
    s_mode_button_raw = read_level(MODE_BUTTON_GPIO);
    s_mode_button_stable = s_mode_button_raw;
    s_last_control_activity_us = esp_timer_get_time();
    s_ui_last_change_us = s_last_control_activity_us;
    s_encoder_button_last_change_us = s_last_control_activity_us;
    s_mode_button_last_change_us = s_last_control_activity_us;

    return ESP_OK;
}

void control_persistence_update(int64_t now_us)
{
    generator_output_persistence_update(now_us);

    if (!s_settings_dirty) return;
    if (now_us - s_settings_last_change_us < SETTINGS_SAVE_DELAY_US) return;

    if (save_setpoints() == ESP_OK) {
        s_settings_dirty = false;
    } else {
        s_settings_last_change_us = now_us;
    }
}

void control_update(app_state_t *state)
{
    int64_t now_us = esp_timer_get_time();
    bool encoder_a = encoder_input_get_a();
    bool encoder_b = encoder_input_get_b();
    bool ui_button = read_level(UI_BUTTON_GPIO);
    bool encoder_button = read_level(ENCODER_BUTTON_GPIO);
    bool mode_button = read_level(MODE_BUTTON_GPIO);

    bool ui_pressed = debounce_pressed(ui_button,
                                       now_us,
                                       &s_ui_raw,
                                       &s_ui_stable,
                                       &s_ui_last_change_us,
                                       &s_ui_pressed_previous);
    bool encoder_button_pressed = debounce_pressed(encoder_button,
                                                   now_us,
                                                   &s_encoder_button_raw,
                                                   &s_encoder_button_stable,
                                                   &s_encoder_button_last_change_us,
                                                   &s_encoder_button_pressed_previous);
    bool mode_button_pressed = debounce_pressed(mode_button,
                                                now_us,
                                                &s_mode_button_raw,
                                                &s_mode_button_stable,
                                                &s_mode_button_last_change_us,
                                                &s_mode_button_pressed_previous);
    if (mode_button_pressed) {
        if (s_menu_open) close_mode_menu();
        else open_mode_menu();
        mark_control_activity(now_us);
    }

    if ((ui_pressed || encoder_button_pressed) && s_menu_open) {
        select_mode_menu_item();
        mark_control_activity(now_us);
    } else if (ui_pressed || encoder_button_pressed) {
        button_advance_selection();
        mark_control_activity(now_us);
    }

    int32_t encoder_steps = encoder_input_take_steps();
    s_last_encoder_steps = encoder_steps;
    if (encoder_steps != 0) {
        mark_control_activity(now_us);
    }
    while (encoder_steps > 0) {
        if (s_menu_open) {
            move_mode_menu(1);
        } else if (s_selected_digit == CONTROL_DIGIT_WHOLE) {
            if (s_mode == APP_MODE_UART || s_mode == APP_MODE_RS485 || s_mode == APP_MODE_CAN) {
                change_protocol_value(1);
            } else {
                move_whole_selection(1);
            }
        } else if (s_mode == APP_MODE_GENERATOR) {
            change_generator_value(1);
        } else if (s_mode == APP_MODE_SETTING) {
            change_setting_value(1);
        } else {
            change_selected_value(1);
        }
        --encoder_steps;
    }
    while (encoder_steps < 0) {
        if (s_menu_open) {
            move_mode_menu(-1);
        } else if (s_selected_digit == CONTROL_DIGIT_WHOLE) {
            if (s_mode == APP_MODE_UART || s_mode == APP_MODE_RS485 || s_mode == APP_MODE_CAN) {
                change_protocol_value(-1);
            } else {
                move_whole_selection(-1);
            }
        } else if (s_mode == APP_MODE_GENERATOR) {
            change_generator_value(-1);
        } else if (s_mode == APP_MODE_SETTING) {
            change_setting_value(-1);
        } else {
            change_selected_value(-1);
        }
        ++encoder_steps;
    }

    if (s_menu_open && now_us - s_last_control_activity_us >= EDIT_TIMEOUT_US) {
        close_mode_menu();
    }

    if (!s_menu_open &&
        s_selected_value != CONTROL_SELECT_NONE &&
        now_us - s_last_control_activity_us >= EDIT_TIMEOUT_US) {
        s_selected_value = CONTROL_SELECT_NONE;
        s_selected_digit = CONTROL_DIGIT_WHOLE;
    }

    state->control.encoder_a = encoder_a;
    state->control.encoder_b = encoder_b;
    state->control.ui_button = ui_button;
    state->control.s_button = true;
    state->control.mode_button = mode_button;
    state->control.mode = s_mode;
    state->control.menu_open = s_menu_open;
    state->control.menu_index = s_menu_index;
    state->control.u1_mv = s_u1_mv;
    state->control.i1_ma = s_i1_ma;
    state->control.u2_mv = s_u2_mv;
    state->control.i2_ma = s_i2_ma;
    generator_output_state_t generator = generator_output_get_state();
    state->control.generator_freq_hz = generator.frequency_hz;
    state->control.generator_duty_percent = generator.duty_percent;
    state->control.generator_on = generator.enabled;
    state->control.uart_baud = s_serial_baud_rates[s_uart_baud_index];
    state->control.rs485_baud = s_serial_baud_rates[s_rs485_baud_index];
    state->control.can_bitrate = s_can_bitrates[s_can_bitrate_index];
    state->control.overcurrent_cc = s_overcurrent_cc;
    state->control.overheat_c = s_overheat_c;
    state->control.overpower_w = s_overpower_w;
    state->control.volume_percent = s_volume_percent;
    state->control.selected_value = s_selected_value;
    state->control.selected_digit = s_selected_digit;
    state->control.last_encoder_steps = s_last_encoder_steps;
}
