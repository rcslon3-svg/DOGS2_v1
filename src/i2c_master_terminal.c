#include "i2c_master_terminal.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "bluetooth_spp.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "i2c_aux_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "probe_config.h"

#define I2C_MASTER_LINE_CHARS    128U
#define I2C_MASTER_DATA_MAX      32U
#define I2C_MASTER_TIMEOUT_MS    100
#define I2C_MASTER_COMMAND_IDLE_US 200000LL

static QueueHandle_t s_char_queue;
static portMUX_TYPE s_text_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_text[I2C_DISPLAY_CHARS + 1U] = "";
static char s_lines[4][I2C_DISPLAY_CHARS + 1U] = {{0}};
static uint32_t s_packets;
static uint32_t s_errors;
static volatile app_mode_t s_active_mode = APP_MODE_POWER_SUPPLY;
static app_mode_t s_requested_mode = APP_MODE_POWER_SUPPLY;
static bool s_bt_was_connected;
static bool s_hint_sent;

static void send_line(const char *line)
{
    if (line == NULL) return;
    printf("%s", line);
    if (bluetooth_spp_connected()) bluetooth_spp_write(line, strlen(line));
}

static void store_preview(const char *line)
{
    if (line == NULL) return;
    size_t length = strlen(line);
    if (length > I2C_DISPLAY_CHARS) length = I2C_DISPLAY_CHARS;

    portENTER_CRITICAL(&s_text_lock);
    memcpy(s_lines[0], s_lines[1], sizeof(s_lines[0]));
    memcpy(s_lines[1], s_lines[2], sizeof(s_lines[1]));
    memcpy(s_lines[2], s_lines[3], sizeof(s_lines[2]));
    memcpy(s_lines[3], line, length);
    s_lines[3][length] = '\0';
    memcpy(s_text, line, length + 1U);
    portEXIT_CRITICAL(&s_text_lock);
}

static void reply(const char *line)
{
    char out[192];
    int written = snprintf(out, sizeof(out), "[I2C] %s\r\n", line);
    if (written <= 0) return;
    if ((size_t)written >= sizeof(out)) {
        memcpy(out + sizeof(out) - 4U, "\r\n", 3U);
        written = (int)sizeof(out) - 1;
    }
    store_preview(line);
    send_line(out);
}

static int hex_nibble(char ch)
{
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

static bool parse_hex_u32(const char *text, uint32_t max_value, uint32_t *value)
{
    if (text == NULL || text[0] == '\0' || value == NULL) return false;
    uint32_t result = 0U;
    for (size_t i = 0U; text[i] != '\0'; ++i) {
        int nibble = hex_nibble(text[i]);
        if (nibble < 0) return false;
        result = (result << 4) | (uint32_t)nibble;
        if (result > max_value) return false;
    }
    *value = result;
    return true;
}

static bool parse_dec_u32(const char *text, uint32_t min_value, uint32_t max_value, uint32_t *value)
{
    if (text == NULL || text[0] == '\0' || value == NULL) return false;
    uint32_t result = 0U;
    for (size_t i = 0U; text[i] != '\0'; ++i) {
        if (text[i] < '0' || text[i] > '9') return false;
        result = result * 10U + (uint32_t)(text[i] - '0');
        if (result > max_value) return false;
    }
    if (result < min_value) return false;
    *value = result;
    return true;
}

static char *next_token(char **cursor)
{
    if (cursor == NULL || *cursor == NULL) return NULL;
    char *p = *cursor;
    while (*p == ' ' || *p == '\t' || *p == ',') ++p;
    if (*p == '\0') {
        *cursor = p;
        return NULL;
    }
    char *start = p;
    while (*p != '\0' && *p != ' ' && *p != '\t' && *p != ',') ++p;
    if (*p != '\0') {
        *p = '\0';
        ++p;
    }
    *cursor = p;
    return start;
}

static void append_bytes(char *out, size_t out_size, size_t *used, const uint8_t *data, size_t length)
{
    for (size_t i = 0U; i < length && *used + 4U < out_size; ++i) {
        *used += (size_t)snprintf(out + *used, out_size - *used, " %02X", data[i]);
    }
}

static const char *i2c_error_name(esp_err_t err)
{
    if (err == ESP_ERR_TIMEOUT) return "TIMEOUT";
    if (err == ESP_ERR_NOT_FOUND) return "NACK_ADDR";
    return esp_err_to_name(err);
}

static void recover_after_error(esp_err_t err)
{
    if (err == ESP_ERR_TIMEOUT) {
        (void)i2c_aux_bus_recover_lines();
    } else if (err == ESP_ERR_INVALID_STATE) {
        (void)i2c_aux_bus_reset_controller();
    }
}

static void command_scan(void)
{
    char out[192];
    size_t used = (size_t)snprintf(out, sizeof(out), "SCAN:");
    if (s_active_mode != APP_MODE_I2C_MASTER) return;
    uint8_t addresses[0x75U];
    size_t address_count = 0U;
    esp_err_t err = i2c_aux_bus_scan(addresses,
                                     sizeof(addresses),
                                     &address_count);
    if (err == ESP_OK) {
        for (size_t i = 0U; i < address_count && used + 4U < sizeof(out); ++i) {
            used += (size_t)snprintf(out + used,
                                     sizeof(out) - used,
                                     " %02X",
                                     addresses[i]);
        }
    }

    if (err == ESP_OK) {
        if (used == strlen("SCAN:")) {
            (void)snprintf(out + used, sizeof(out) - used, " NONE");
        }
        ++s_packets;
    } else {
        if (s_active_mode != APP_MODE_I2C_MASTER) return;
        recover_after_error(err);
        ++s_errors;
        snprintf(out, sizeof(out), "SCAN ERR %s", i2c_error_name(err));
    }
    reply(out);
}

static void command_read(uint8_t address, uint8_t reg, size_t length)
{
    uint8_t data[I2C_MASTER_DATA_MAX];
    esp_err_t err = i2c_aux_bus_transmit_receive(address,
                                                 &reg,
                                                 1U,
                                                 data,
                                                 length,
                                                 I2C_MASTER_TIMEOUT_MS);
    recover_after_error(err);

    char out[192];
    size_t used = 0U;
    if (err == ESP_OK) {
        used = (size_t)snprintf(out, sizeof(out), "R %02X %02X:", address, reg);
        append_bytes(out, sizeof(out), &used, data, length);
        snprintf(out + used, sizeof(out) - used, " OK");
        ++s_packets;
    } else {
        snprintf(out, sizeof(out), "R %02X %02X ERR %s", address, reg, i2c_error_name(err));
        ++s_errors;
    }
    reply(out);
}

static void command_raw_read(uint8_t address, size_t length)
{
    uint8_t data[I2C_MASTER_DATA_MAX];
    esp_err_t err =
        i2c_aux_bus_receive(address, data, length, I2C_MASTER_TIMEOUT_MS);
    recover_after_error(err);

    char out[192];
    size_t used = 0U;
    if (err == ESP_OK) {
        used = (size_t)snprintf(out, sizeof(out), "RR %02X:", address);
        append_bytes(out, sizeof(out), &used, data, length);
        snprintf(out + used, sizeof(out) - used, " OK");
        ++s_packets;
    } else {
        snprintf(out, sizeof(out), "RR %02X ERR %s", address, i2c_error_name(err));
        ++s_errors;
    }
    reply(out);
}

static void command_write(uint8_t address, const uint8_t *data, size_t length, bool raw)
{
    esp_err_t err =
        i2c_aux_bus_transmit(address, data, length, I2C_MASTER_TIMEOUT_MS);
    recover_after_error(err);

    char out[192];
    size_t used = 0U;
    used = (size_t)snprintf(out, sizeof(out), "%s %02X", raw ? "WW" : "W", address);
    append_bytes(out, sizeof(out), &used, data, length);
    snprintf(out + used, sizeof(out) - used, err == ESP_OK ? " OK" : " ERR %s", i2c_error_name(err));
    if (err == ESP_OK) ++s_packets;
    else ++s_errors;
    reply(out);
}

static void command_syntax_error(void)
{
    ++s_errors;
    reply("ERR syntax: S | R <addr> <reg> <len> | W <addr> <reg> <data...> | RR <addr> <len> | WW <addr> <data...>");
}

static void process_line(char *line)
{
    char *cursor = line;
    char *command = next_token(&cursor);
    uint32_t address;
    uint32_t reg;
    uint32_t length;
    uint8_t data[I2C_MASTER_DATA_MAX + 1U];
    size_t data_length = 0U;

    if (command == NULL) return;
    for (size_t i = 0U; command[i] != '\0'; ++i) command[i] = (char)toupper((unsigned char)command[i]);

    if (strcmp(command, "S") == 0) {
        if (next_token(&cursor) != NULL) command_syntax_error();
        else command_scan();
        return;
    }

    char *addr_token = next_token(&cursor);
    if (!parse_hex_u32(addr_token, 0x7FU, &address)) {
        command_syntax_error();
        return;
    }

    if (strcmp(command, "R") == 0) {
        if (!parse_hex_u32(next_token(&cursor), 0xFFU, &reg) ||
            !parse_dec_u32(next_token(&cursor), 1U, I2C_MASTER_DATA_MAX, &length) ||
            next_token(&cursor) != NULL) {
            command_syntax_error();
            return;
        }
        command_read((uint8_t)address, (uint8_t)reg, (size_t)length);
        return;
    }

    if (strcmp(command, "RR") == 0) {
        if (!parse_dec_u32(next_token(&cursor), 1U, I2C_MASTER_DATA_MAX, &length) ||
            next_token(&cursor) != NULL) {
            command_syntax_error();
            return;
        }
        command_raw_read((uint8_t)address, (size_t)length);
        return;
    }

    if (strcmp(command, "W") == 0 || strcmp(command, "WW") == 0) {
        bool raw = strcmp(command, "WW") == 0;
        if (!raw) {
            if (!parse_hex_u32(next_token(&cursor), 0xFFU, &reg)) {
                command_syntax_error();
                return;
            }
            data[data_length++] = (uint8_t)reg;
        }
        char *token;
        while ((token = next_token(&cursor)) != NULL) {
            uint32_t byte_value;
            if (data_length >= I2C_MASTER_DATA_MAX + (raw ? 0U : 1U) ||
                !parse_hex_u32(token, 0xFFU, &byte_value)) {
                command_syntax_error();
                return;
            }
            data[data_length++] = (uint8_t)byte_value;
        }
        if (data_length == 0U || (!raw && data_length == 1U)) {
            command_syntax_error();
            return;
        }
        command_write((uint8_t)address, data, data_length, raw);
        return;
    }

    command_syntax_error();
}

static void input_task(void *argument)
{
    (void)argument;
    char line[I2C_MASTER_LINE_CHARS];
    size_t used = 0U;
    int64_t last_input_us = 0;
    char ch;

    while (true) {
        if (s_active_mode != APP_MODE_I2C_MASTER) {
            used = 0U;
            (void)xQueueReset(s_char_queue);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (xQueueReceive(s_char_queue, &ch, pdMS_TO_TICKS(20)) != pdTRUE) {
            if (used != 0U && esp_timer_get_time() - last_input_us >= I2C_MASTER_COMMAND_IDLE_US) {
                line[used] = '\0';
                process_line(line);
                used = 0U;
            }
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            if (used != 0U) {
                line[used] = '\0';
                process_line(line);
                used = 0U;
            }
            continue;
        }
        if ((unsigned char)ch < 0x20U) continue;
        if (used + 1U < sizeof(line)) {
            line[used++] = ch;
            last_input_us = esp_timer_get_time();
        } else {
            used = 0U;
            command_syntax_error();
        }
    }
}

static void hint_update(app_mode_t mode)
{
    bool bt_connected = bluetooth_spp_connected();
    bool bt_connected_now = bt_connected && !s_bt_was_connected;
    bool entering = mode == APP_MODE_I2C_MASTER && s_active_mode != APP_MODE_I2C_MASTER;

    if (mode != APP_MODE_I2C_MASTER || !bt_connected) {
        s_hint_sent = false;
    }

    if (mode == APP_MODE_I2C_MASTER && bt_connected && !s_hint_sent &&
        (entering || bt_connected_now)) {
        static const char hint[] =
            "[I2C HINT] S | R <addr> <reg> <len> | W <addr> <reg> <data...> | RR <addr> <len> | WW <addr> <data...>\r\n";
        bluetooth_spp_write(hint, sizeof(hint) - 1U);
        s_hint_sent = true;
    }

    s_bt_was_connected = bt_connected;
}

esp_err_t i2c_master_terminal_init(void)
{
    esp_err_t err = i2c_aux_bus_init();
    if (err != ESP_OK) return err;
    s_char_queue = xQueueCreate(128, sizeof(char));
    if (s_char_queue == NULL) return ESP_ERR_NO_MEM;
    BaseType_t ok = xTaskCreate(input_task, "i2c_master_term", 4096, NULL, 4, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void i2c_master_terminal_configure(app_mode_t mode)
{
    hint_update(mode);
    if (mode != s_requested_mode) {
        bool was_master = s_active_mode == APP_MODE_I2C_MASTER;
        s_requested_mode = mode;
        if (mode != APP_MODE_I2C_MASTER) {
            /*
             * Stop routing new bytes before waiting for an in-flight bus
             * transaction and deleting the controller.
             */
            s_active_mode = mode;
            if (s_char_queue != NULL) xQueueReset(s_char_queue);
            (void)i2c_aux_bus_stop();
        } else {
            esp_err_t err = i2c_aux_bus_start();
            if (err == ESP_OK) s_active_mode = mode;
            else {
                ++s_errors;
                reply("MASTER START ERR");
            }
        }
        if (mode == APP_MODE_I2C_MASTER || was_master) {
            portENTER_CRITICAL(&s_text_lock);
            memset(s_lines, 0, sizeof(s_lines));
            s_text[0] = '\0';
            portEXIT_CRITICAL(&s_text_lock);
        }
    }
}

bool i2c_master_terminal_input_char(char ch)
{
    if (s_active_mode != APP_MODE_I2C_MASTER) return false;
    if (s_char_queue == NULL) return true;
    (void)xQueueSend(s_char_queue, &ch, pdMS_TO_TICKS(10));
    return true;
}

void i2c_master_terminal_update(app_state_t *state)
{
    if (state == NULL) return;
    if (s_active_mode != APP_MODE_I2C_MASTER) return;
    portENTER_CRITICAL(&s_text_lock);
    memcpy(state->i2c_sniffer.text, s_text, sizeof(state->i2c_sniffer.text));
    memcpy(state->i2c_sniffer.lines, s_lines, sizeof(state->i2c_sniffer.lines));
    state->i2c_sniffer.packets = s_packets;
    state->i2c_sniffer.errors = s_errors;
    portEXIT_CRITICAL(&s_text_lock);
}
