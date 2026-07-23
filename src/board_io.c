#include "board_io.h"

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "probe_config.h"

#define PCA9557_REG_INPUT      0x00U
#define PCA9557_REG_OUTPUT     0x01U
#define PCA9557_REG_POLARITY   0x02U
#define PCA9557_REG_CONFIG     0x03U

#define BIT_U8(bit) ((uint8_t)(1U << (bit)))

#if LOGIC_V2_BOARD_REV == LOGIC_V2_BOARD_ENGINEERING_SAMPLE

static const char *TAG = "board_io";
static i2c_master_dev_handle_t s_ctrl_expander;
static i2c_master_dev_handle_t s_ui_expander;
static uint8_t s_last_ctrl_output;
static uint8_t s_last_ui_output;
static bool s_ctrl_output_valid;
static bool s_ui_output_valid;
static bool s_rs485_tx_enabled;
static bool s_test_100hz_level;
static app_mode_t s_last_mode = APP_MODE_POWER_SUPPLY;
static uint8_t s_last_ui_input = 0xFFU;
static bool s_ui_input_valid;

static esp_err_t add_expander(uint8_t address, i2c_master_dev_handle_t *handle)
{
    i2c_master_bus_handle_t bus = NULL;
    ESP_RETURN_ON_ERROR(i2c_bus_get(&bus), TAG, "get bus");

    i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = INA238_I2C_HZ,
    };
    return i2c_master_bus_add_device(bus, &config, handle);
}

static esp_err_t write_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t value)
{
    uint8_t bytes[2] = {reg, value};
    return i2c_master_transmit(handle, bytes, sizeof(bytes), 100);
}

static esp_err_t read_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *value)
{
    if (value == NULL) return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit_receive(handle, &reg, 1, value, 1, 100);
}

static uint8_t ctrl_output_for_mode(app_mode_t mode)
{
    uint8_t output = BIT_U8(CTRL_EXP_CAN_RS_BIT);

    if (mode == APP_MODE_CAN || mode == APP_MODE_I2C ||
        mode == APP_MODE_I2C_MASTER || mode == APP_MODE_RS485) {
        output |= BIT_U8(CTRL_EXP_RELAY1_BIT);
    }

    if (mode == APP_MODE_I2C || mode == APP_MODE_I2C_MASTER) {
        output |= BIT_U8(CTRL_EXP_RELAY2_BIT);
    }

    if (mode == APP_MODE_CAN) {
        output &= (uint8_t)~BIT_U8(CTRL_EXP_CAN_RS_BIT);
    }

    if (mode == APP_MODE_POWER_SUPPLY) {
        output |= BIT_U8(CTRL_EXP_UART_RX_FREQ_EN_BIT);
    }

    if (mode == APP_MODE_UART || mode == APP_MODE_LIN) {
        output |= BIT_U8(CTRL_EXP_UART_RX_FREQ_EN_BIT);
        output |= BIT_U8(CTRL_EXP_UART_TX_ENABLE_BIT);
    }

    if (mode == APP_MODE_RS485) {
        output &= (uint8_t)~BIT_U8(CTRL_EXP_UART_RX_FREQ_EN_BIT);
        if (s_rs485_tx_enabled) {
            output |= BIT_U8(CTRL_EXP_RS485_TX_ENABLE_BIT);
        } else {
            output &= (uint8_t)~BIT_U8(CTRL_EXP_RS485_TX_ENABLE_BIT);
        }
    }

    if (s_test_100hz_level) {
        output |= BIT_U8(CTRL_EXP_TEST_100HZ_BIT);
    }

    return output;
}

static esp_err_t write_ctrl_output(uint8_t output)
{
    if (s_ctrl_expander == NULL) return ESP_OK;
    if (s_ctrl_output_valid && output == s_last_ctrl_output) return ESP_OK;

    esp_err_t err = write_reg(s_ctrl_expander, PCA9557_REG_OUTPUT, output);
    if (err == ESP_OK) {
        s_last_ctrl_output = output;
        s_ctrl_output_valid = true;
    } else {
        ESP_LOGW(TAG, "ctrl output %02X: %s", output, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t write_ui_output(uint8_t output)
{
    if (s_ui_expander == NULL) return ESP_OK;
    if (s_ui_output_valid && output == s_last_ui_output) return ESP_OK;

    esp_err_t err = write_reg(s_ui_expander, PCA9557_REG_OUTPUT, output);
    if (err == ESP_OK) {
        s_last_ui_output = output;
        s_ui_output_valid = true;
    } else {
        ESP_LOGW(TAG, "ui output %02X: %s", output, esp_err_to_name(err));
    }
    return err;
}

static uint8_t ui_output_for_state(const app_state_t *state)
{
    uint8_t output = 0U;
    if (state == NULL) return output;

    bool channel_a_power = state->control.channel_a_enabled ||
                           state->tps55289.output_enabled;
    bool channel_b_power = state->control.channel_b_enabled ||
                           state->lm51772.output_enabled;
    if (channel_a_power) output |= BIT_U8(UI_EXP_CHANNEL_A_ENABLE_BIT);
    if (channel_b_power) output |= BIT_U8(UI_EXP_CHANNEL_B_ENABLE_BIT);
    return output;
}

esp_err_t board_io_init(void)
{
    if (s_ctrl_expander == NULL) {
        ESP_RETURN_ON_ERROR(add_expander(IO_EXPANDER_CTRL_ADDRESS, &s_ctrl_expander),
                            TAG, "add ctrl expander");
    }
    if (s_ui_expander == NULL) {
        ESP_RETURN_ON_ERROR(add_expander(IO_EXPANDER_UI_ADDRESS, &s_ui_expander),
                            TAG, "add ui expander");
    }

    s_ctrl_output_valid = false;
    s_ui_output_valid = false;
    s_rs485_tx_enabled = false;
    s_test_100hz_level = false;
    s_last_mode = APP_MODE_POWER_SUPPLY;

    ESP_RETURN_ON_ERROR(write_reg(s_ctrl_expander,
                                  PCA9557_REG_OUTPUT,
                                  BIT_U8(CTRL_EXP_CAN_RS_BIT)),
                        TAG, "ctrl safe output");
    ESP_RETURN_ON_ERROR(write_reg(s_ctrl_expander, PCA9557_REG_POLARITY, 0x00U),
                        TAG, "ctrl polarity");
    ESP_RETURN_ON_ERROR(write_reg(s_ctrl_expander, PCA9557_REG_CONFIG, 0x00U),
                        TAG, "ctrl config");
    s_last_ctrl_output = BIT_U8(CTRL_EXP_CAN_RS_BIT);
    s_ctrl_output_valid = true;

    ESP_RETURN_ON_ERROR(write_reg(s_ui_expander, PCA9557_REG_OUTPUT, 0x00U),
                        TAG, "ui safe output");
    ESP_RETURN_ON_ERROR(write_reg(s_ui_expander, PCA9557_REG_POLARITY, 0x00U),
                        TAG, "ui polarity");
    ESP_RETURN_ON_ERROR(write_reg(s_ui_expander, PCA9557_REG_CONFIG,
                                  BIT_U8(UI_EXP_ENCODER_BUTTON_BIT) |
                                  BIT_U8(UI_EXP_CHANNEL_A_BUTTON_BIT) |
                                  BIT_U8(UI_EXP_CHANNEL_B_BUTTON_BIT) |
                                  BIT_U8(UI_EXP_MENU_BUTTON_BIT) |
                                  BIT_U8(UI_EXP_BUTTON_C_BIT) |
                                  BIT_U8(UI_EXP_UNUSED_BIT)),
                        TAG, "ui config");
    s_last_ui_output = 0x00U;
    s_ui_output_valid = true;

    return ESP_OK;
}

void board_io_i2c_release(void)
{
    if (s_ctrl_expander != NULL) {
        (void)i2c_master_bus_rm_device(s_ctrl_expander);
        s_ctrl_expander = NULL;
    }
    if (s_ui_expander != NULL) {
        (void)i2c_master_bus_rm_device(s_ui_expander);
        s_ui_expander = NULL;
    }
    s_ctrl_output_valid = false;
    s_ui_output_valid = false;
}

esp_err_t board_io_read_ui_buttons(bool *encoder_button_high,
                                   bool *menu_button_high,
                                   bool *channel_a_button_high,
                                   bool *channel_b_button_high)
{
    if (encoder_button_high == NULL || menu_button_high == NULL ||
        channel_a_button_high == NULL || channel_b_button_high == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *encoder_button_high = (s_last_ui_input & BIT_U8(UI_EXP_ENCODER_BUTTON_BIT)) != 0U;
    *menu_button_high = (s_last_ui_input & BIT_U8(UI_EXP_MENU_BUTTON_BIT)) != 0U;
    *channel_a_button_high = (s_last_ui_input & BIT_U8(UI_EXP_CHANNEL_A_BUTTON_BIT)) != 0U;
    *channel_b_button_high = (s_last_ui_input & BIT_U8(UI_EXP_CHANNEL_B_BUTTON_BIT)) != 0U;
    return s_ui_input_valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}

void board_io_update(const app_state_t *state)
{
    if (state == NULL) return;

    if (s_ctrl_expander == NULL || s_ui_expander == NULL) {
        if (board_io_init() != ESP_OK) return;
    }

    s_last_mode = state->control.mode;
    if (s_last_mode != APP_MODE_RS485) {
        s_rs485_tx_enabled = false;
    }
    (void)write_ctrl_output(ctrl_output_for_mode(state->control.mode));
    (void)write_ui_output(ui_output_for_state(state));

    uint8_t input = 0xFFU;
    esp_err_t err = read_reg(s_ui_expander, PCA9557_REG_INPUT, &input);
    if (err == ESP_OK) {
        s_last_ui_input = input;
        s_ui_input_valid = true;
    } else {
        ESP_LOGW(TAG, "ui input: %s", esp_err_to_name(err));
    }
}

bool board_io_test_100hz_level(void)
{
    return s_test_100hz_level;
}

esp_err_t board_io_toggle_test_100hz(const app_state_t *state)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;

    if (s_ctrl_expander == NULL || s_ui_expander == NULL) {
        ESP_RETURN_ON_ERROR(board_io_init(), TAG, "init for test toggle");
    }

    s_last_mode = state->control.mode;
    if (s_last_mode != APP_MODE_RS485) {
        s_rs485_tx_enabled = false;
    }
    s_test_100hz_level = !s_test_100hz_level;
    return write_ctrl_output(ctrl_output_for_mode(state->control.mode));
}

esp_err_t board_io_set_test_100hz(const app_state_t *state, bool high)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;

    if (s_ctrl_expander == NULL || s_ui_expander == NULL) {
        ESP_RETURN_ON_ERROR(board_io_init(), TAG, "init for test set");
    }

    s_last_mode = state->control.mode;
    if (s_last_mode != APP_MODE_RS485) {
        s_rs485_tx_enabled = false;
    }
    s_test_100hz_level = high;
    return write_ctrl_output(ctrl_output_for_mode(state->control.mode));
}

esp_err_t board_io_read_ctrl_debug(uint8_t *input, uint8_t *output)
{
    if (input == NULL || output == NULL) return ESP_ERR_INVALID_ARG;
    if (s_ctrl_expander == NULL) {
        ESP_RETURN_ON_ERROR(board_io_init(), TAG, "init for ctrl debug");
    }

    ESP_RETURN_ON_ERROR(read_reg(s_ctrl_expander, PCA9557_REG_INPUT, input),
                        TAG, "ctrl input debug");
    ESP_RETURN_ON_ERROR(read_reg(s_ctrl_expander, PCA9557_REG_OUTPUT, output),
                        TAG, "ctrl output debug");
    return ESP_OK;
}

void board_io_set_rs485_tx_enabled(bool enabled)
{
    s_rs485_tx_enabled = enabled;
    if (s_last_mode != APP_MODE_RS485) return;

    if (s_ctrl_expander == NULL || s_ui_expander == NULL) {
        if (board_io_init() != ESP_OK) return;
    }

    (void)write_ctrl_output(ctrl_output_for_mode(APP_MODE_RS485));
}

#else

esp_err_t board_io_init(void)
{
    return ESP_OK;
}

void board_io_i2c_release(void)
{
}

void board_io_update(const app_state_t *state)
{
    (void)state;
}

bool board_io_test_100hz_level(void)
{
    return false;
}

esp_err_t board_io_toggle_test_100hz(const app_state_t *state)
{
    (void)state;
    return ESP_OK;
}

esp_err_t board_io_set_test_100hz(const app_state_t *state, bool high)
{
    (void)state;
    (void)high;
    return ESP_OK;
}

esp_err_t board_io_read_ctrl_debug(uint8_t *input, uint8_t *output)
{
    if (input == NULL || output == NULL) return ESP_ERR_INVALID_ARG;
    *input = 0;
    *output = 0;
    return ESP_OK;
}

void board_io_set_rs485_tx_enabled(bool enabled)
{
    (void)enabled;
}

esp_err_t board_io_read_ui_buttons(bool *encoder_button_high,
                                   bool *menu_button_high,
                                   bool *channel_a_button_high,
                                   bool *channel_b_button_high)
{
    if (encoder_button_high == NULL || menu_button_high == NULL ||
        channel_a_button_high == NULL || channel_b_button_high == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *encoder_button_high = true;
    *menu_button_high = true;
    *channel_a_button_high = true;
    *channel_b_button_high = true;
    return ESP_OK;
}

#endif
