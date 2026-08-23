#include "can_probe.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "bluetooth_spp.h"
#include "driver/twai.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "probe_config.h"

#define CAN_COMMAND_CHARS 64U
#define CAN_COMMAND_IDLE_US 200000LL
#define CAN_RECOVERY_RETRY_US 1000000LL

#define CAN_TX_RESULT_NONE 0U
#define CAN_TX_RESULT_OK 1U
#define CAN_TX_RESULT_BUSY 2U
#define CAN_TX_RESULT_ERR 3U

static const char *TAG = "can_probe";
static portMUX_TYPE s_text_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_text[I2C_DISPLAY_CHARS + 1U] = "";
static char s_lines[3][I2C_DISPLAY_CHARS + 1U] = {{0}};
static uint32_t s_packets;
static uint32_t s_errors;
static uint32_t s_tx_error_counter;
static uint32_t s_tx_queue_msgs;
static uint8_t s_tx_result = CAN_TX_RESULT_NONE;
static bool s_last_tx_ok;
static bool s_bus_off;
static bool s_installed;
static bool s_running;
static bool s_recovering;
static app_mode_t s_active_mode = APP_MODE_POWER_SUPPLY;
static uint32_t s_active_bitrate;
static uint16_t s_mask_value;
static uint8_t s_mask_care;
static char s_command[CAN_COMMAND_CHARS];
static size_t s_command_used;
static int64_t s_command_last_us;
static int64_t s_recovery_started_us;
static esp_err_t s_last_config_error;
static uint32_t s_last_config_error_bitrate;

static void send_line(const char *line)
{
    if (line == NULL) return;
    printf("%s", line);
    if (bluetooth_spp_connected()) bluetooth_spp_write(line, strlen(line));
}

static void store_line(const char *line)
{
    if (line == NULL) return;
    size_t length = strlen(line);
    if (length > I2C_DISPLAY_CHARS) length = I2C_DISPLAY_CHARS;

    portENTER_CRITICAL(&s_text_lock);
    memcpy(s_lines[0], s_lines[1], sizeof(s_lines[0]));
    memcpy(s_lines[1], s_lines[2], sizeof(s_lines[1]));
    memcpy(s_lines[2], line, length);
    s_lines[2][length] = '\0';
    portEXIT_CRITICAL(&s_text_lock);
}

static void store_status(const char *line)
{
    if (line == NULL) return;
    size_t length = strlen(line);
    if (length > I2C_DISPLAY_CHARS) length = I2C_DISPLAY_CHARS;

    portENTER_CRITICAL(&s_text_lock);
    memcpy(s_text, line, length);
    s_text[length] = '\0';
    portEXIT_CRITICAL(&s_text_lock);
}

static bool mask_allows(uint32_t identifier)
{
    for (uint8_t i = 0U; i < 3U; ++i) {
        if ((s_mask_care & (uint8_t)(1U << i)) == 0U) continue;
        uint8_t shift = (uint8_t)((2U - i) * 4U);
        uint8_t want = (uint8_t)((s_mask_value >> shift) & 0x0FU);
        uint8_t have = (uint8_t)((identifier >> shift) & 0x0FU);
        if (want != have) return false;
    }
    return true;
}

static void format_frame(char *out, size_t out_size, const twai_message_t *message)
{
    size_t used = (size_t)snprintf(out, out_size, "%03lX:",
                                   (unsigned long)(message->identifier & 0x7FFU));
    for (uint8_t i = 0U; i < message->data_length_code && i < 8U && used + 4U < out_size; ++i) {
        used += (size_t)snprintf(out + used, out_size - used, "%s%02X",
                                 i == 0U ? "" : " ",
                                 message->data[i]);
    }
}

static void publish_frame(const twai_message_t *message)
{
    char payload[64];
    char line[96];
    format_frame(payload, sizeof(payload), message);
    store_line(payload);
    int length = snprintf(line, sizeof(line), "[CAN] %s\r\n", payload);
    if (length > 0) send_line(line);
    ++s_packets;
}

static void publish_tx_result(const twai_message_t *message, bool ok)
{
    char payload[64];
    char line[128];
    if (message == NULL) return;
    format_frame(payload, sizeof(payload), message);
    size_t used = (size_t)snprintf(line, sizeof(line), "[CAN TX] %s %s",
                           payload,
                           ok ? "OK" : "ERR");
    if (used + 3U < sizeof(line)) {
        memcpy(line + used, "\r\n", 3U);
    } else {
        memcpy(line + sizeof(line) - 3U, "\r\n", 3U);
    }
    send_line(line);
    s_last_tx_ok = ok;
    if (!ok) ++s_errors;
}

static twai_timing_config_t timing_for_bitrate(uint32_t bitrate)
{
    switch (bitrate) {
        case 100000U: return (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS();
        case 125000U: return (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
        case 250000U: return (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
        case 500000U: return (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
        case 1000000U: return (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
        default: return (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
    }
}

static void reset_driver_flags(void)
{
    s_installed = false;
    s_running = false;
    s_bus_off = false;
    s_recovering = false;
    s_active_bitrate = 0U;
    s_tx_error_counter = 0U;
    s_tx_queue_msgs = 0U;
    s_tx_result = CAN_TX_RESULT_NONE;
    s_recovery_started_us = 0LL;
}

static void begin_recovery(void)
{
    if (!s_installed || s_recovering) return;
    s_last_tx_ok = false;
    s_bus_off = true;
    s_running = false;
    ++s_errors;
    store_status("BUSOFF");
    if (twai_initiate_recovery() == ESP_OK) {
        s_recovering = true;
        s_recovery_started_us = esp_timer_get_time();
        store_status("RECOVERY");
    }
}

static esp_err_t stop_driver(void)
{
    if (!s_installed) {
        reset_driver_flags();
        return ESP_OK;
    }

    twai_status_info_t status = {0};
    esp_err_t status_err = twai_get_status_info(&status);
    if (status_err == ESP_OK && status.state == TWAI_STATE_RUNNING) {
        esp_err_t stop_err = twai_stop();
        if (stop_err != ESP_OK) {
            ESP_LOGW(TAG, "stop: %s", esp_err_to_name(stop_err));
            return stop_err;
        }
        s_running = false;
        status.state = TWAI_STATE_STOPPED;
    }

    if (status_err == ESP_OK && status.state == TWAI_STATE_RECOVERING) {
        for (uint8_t i = 0U; i < 25U; ++i) {
            vTaskDelay(pdMS_TO_TICKS(1));
            status_err = twai_get_status_info(&status);
            if (status_err != ESP_OK ||
                status.state == TWAI_STATE_STOPPED ||
                status.state == TWAI_STATE_BUS_OFF) {
                break;
            }
        }
        if (status_err == ESP_OK && status.state == TWAI_STATE_RECOVERING) {
            s_running = false;
            s_recovering = true;
            s_bus_off = false;
            return ESP_ERR_TIMEOUT;
        }
    }

    esp_err_t uninstall_err = twai_driver_uninstall();
    if (uninstall_err != ESP_OK) {
        ESP_LOGW(TAG, "uninstall: %s", esp_err_to_name(uninstall_err));
        return uninstall_err;
    }

    reset_driver_flags();
    return ESP_OK;
}

static esp_err_t start_driver(uint32_t bitrate)
{
    if (CAN_RX_GPIO == GPIO_NUM_NC || CAN_TX_GPIO == GPIO_NUM_NC) return ESP_ERR_NOT_SUPPORTED;
    if (s_installed && s_active_bitrate == bitrate) return ESP_OK;
    esp_err_t stop_err = stop_driver();
    if (stop_err != ESP_OK) return stop_err;

    twai_general_config_t general =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    general.tx_queue_len = 0U;
    general.rx_queue_len = 16U;
    general.alerts_enabled = TWAI_ALERT_BUS_OFF |
                             TWAI_ALERT_RECOVERY_IN_PROGRESS |
                             TWAI_ALERT_BUS_RECOVERED |
                             TWAI_ALERT_TX_FAILED |
                             TWAI_ALERT_TX_SUCCESS |
                             TWAI_ALERT_RX_QUEUE_FULL |
                             TWAI_ALERT_RX_FIFO_OVERRUN;
    twai_timing_config_t timing = timing_for_bitrate(bitrate);
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t install_err = twai_driver_install(&general, &timing, &filter);
    if (install_err != ESP_OK) {
        reset_driver_flags();
        return install_err;
    }
    s_installed = true;
    esp_err_t start_err = twai_start();
    if (start_err != ESP_OK) {
        (void)twai_driver_uninstall();
        reset_driver_flags();
        return start_err;
    }
    s_running = true;
    s_active_bitrate = bitrate;
    s_bus_off = false;
    s_recovering = false;
    s_recovery_started_us = 0LL;
    s_last_config_error = ESP_OK;
    s_last_config_error_bitrate = 0U;
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        s_tx_error_counter = status.tx_error_counter;
        s_tx_queue_msgs = status.msgs_to_tx;
    }
    ESP_LOGI(TAG, "started bitrate=%lu tx=%d rx=%d",
             (unsigned long)bitrate,
             (int)CAN_TX_GPIO,
             (int)CAN_RX_GPIO);
    return ESP_OK;
}

static int hex_nibble(char ch)
{
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

static bool parse_can_command(const char *command, uint32_t *identifier, uint8_t *data, uint8_t *length)
{
    uint8_t nibbles[19];
    uint8_t count = 0U;
    if (command == NULL || identifier == NULL || data == NULL || length == NULL) return false;

    for (size_t i = 0U; command[i] != '\0'; ++i) {
        if (command[i] == ' ' || command[i] == '\t' || command[i] == ',' || command[i] == ':') continue;
        int nibble = hex_nibble(command[i]);
        if (nibble < 0 || count >= sizeof(nibbles)) return false;
        nibbles[count++] = (uint8_t)nibble;
    }
    if (count < 4U) return false;

    uint32_t id = 0U;
    uint8_t data_start = 4U;
    id = ((uint32_t)nibbles[0] << 12U) |
         ((uint32_t)nibbles[1] << 8U) |
         ((uint32_t)nibbles[2] << 4U) |
         (uint32_t)nibbles[3];
    if (id > 0x7FFU || ((count - data_start) & 1U) != 0U) return false;

    uint8_t dlc = (uint8_t)((count - data_start) / 2U);
    if (dlc > 8U) return false;
    for (uint8_t i = 0U; i < dlc; ++i) {
        data[i] = (uint8_t)((nibbles[data_start + i * 2U] << 4U) |
                            nibbles[data_start + 1U + i * 2U]);
    }
    *identifier = id;
    *length = dlc;
    return true;
}

static esp_err_t transmit_frame(uint32_t identifier, const uint8_t *data, uint8_t length)
{
    if (!s_running || s_bus_off) return ESP_ERR_INVALID_STATE;
    if (length > 8U) return ESP_ERR_INVALID_ARG;
    twai_message_t message = {
        .identifier = identifier & 0x7FFU,
        .data_length_code = length,
        .ss = 1,
    };
    if (data != NULL && length != 0U) memcpy(message.data, data, length);
    esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(20));
    if (err == ESP_OK) {
        s_last_tx_ok = true;
        s_tx_result = CAN_TX_RESULT_OK;
    } else {
        publish_tx_result(&message, false);
        s_tx_result = (err == ESP_FAIL || err == ESP_ERR_TIMEOUT) ?
                      CAN_TX_RESULT_BUSY : CAN_TX_RESULT_ERR;
    }
    return err;
}

static void run_command(void)
{
    uint32_t identifier = 0U;
    uint8_t data[8] = {0};
    uint8_t length = 0U;

    s_command[s_command_used] = '\0';
    if (!parse_can_command(s_command, &identifier, data, &length)) {
        const char *error = "[CAN TX] ERR syntax\r\n";
        send_line(error);
        ++s_errors;
    } else {
        (void)transmit_frame(identifier, data, length);
    }
    s_command_used = 0U;
}

static void process_command_idle(int64_t now_us)
{
    if (s_command_used == 0U) return;
    if (now_us - s_command_last_us < CAN_COMMAND_IDLE_US) return;
    run_command();
}

static void poll_alerts(void)
{
    if (!s_installed) return;
    uint32_t alerts = 0U;
    if (twai_read_alerts(&alerts, 0) != ESP_OK) return;
    if ((alerts & TWAI_ALERT_BUS_OFF) != 0U) {
        begin_recovery();
    }
    if ((alerts & TWAI_ALERT_RECOVERY_IN_PROGRESS) != 0U) {
        s_recovering = true;
        s_running = false;
        if (s_recovery_started_us == 0LL) s_recovery_started_us = esp_timer_get_time();
        store_status("RECOVERY");
    }
    if ((alerts & TWAI_ALERT_BUS_RECOVERED) != 0U) {
        s_recovering = false;
        s_bus_off = false;
        s_recovery_started_us = 0LL;
        if (twai_start() == ESP_OK) {
            s_running = true;
            store_status("RUN");
        }
    }
    if ((alerts & TWAI_ALERT_TX_SUCCESS) != 0U) {
        s_last_tx_ok = true;
        s_tx_result = CAN_TX_RESULT_OK;
    }
    if ((alerts & TWAI_ALERT_TX_FAILED) != 0U) {
        s_last_tx_ok = false;
        s_tx_result = CAN_TX_RESULT_ERR;
        ++s_errors;
    }
    if ((alerts & (TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_RX_FIFO_OVERRUN)) != 0U) {
        ++s_errors;
    }
}

static void sync_driver_status(void)
{
    if (!s_installed) return;
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) return;

    s_tx_error_counter = status.tx_error_counter;
    s_tx_queue_msgs = status.msgs_to_tx;

    if (status.state == TWAI_STATE_STOPPED && s_recovering) {
        s_recovering = false;
        s_bus_off = false;
        s_recovery_started_us = 0LL;
        if (twai_start() == ESP_OK) {
            s_running = true;
            store_status("RUN");
        }
    } else if (status.state == TWAI_STATE_RUNNING) {
        s_running = true;
        s_bus_off = false;
        s_recovering = false;
        s_recovery_started_us = 0LL;
    } else if (status.state == TWAI_STATE_BUS_OFF) {
        s_running = false;
        s_bus_off = true;
    } else if (status.state == TWAI_STATE_RECOVERING) {
        s_running = false;
        s_recovering = true;
    }
}

static void recovery_timer_update(int64_t now_us)
{
    if (!s_installed || (!s_bus_off && !s_recovering)) return;
    if (s_recovery_started_us == 0LL) {
        s_recovery_started_us = now_us;
        return;
    }
    if (now_us - s_recovery_started_us < CAN_RECOVERY_RETRY_US) return;

    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) return;
    s_recovery_started_us = now_us;

    if (status.state == TWAI_STATE_STOPPED) {
        s_recovering = false;
        s_bus_off = false;
        if (twai_start() == ESP_OK) {
            s_running = true;
            store_status("RUN");
        }
    } else if (status.state == TWAI_STATE_RECOVERING) {
        s_recovery_started_us = now_us;
    }
}

esp_err_t can_probe_init(void)
{
    return ESP_OK;
}

void can_probe_configure(app_mode_t mode,
                         uint32_t bitrate,
                         uint16_t mask_value,
                         uint8_t mask_care)
{
    s_mask_value = mask_value;
    s_mask_care = mask_care;
    if (mode != APP_MODE_CAN) {
        if (s_active_mode == APP_MODE_CAN) {
            (void)stop_driver();
            portENTER_CRITICAL(&s_text_lock);
            memset(s_lines, 0, sizeof(s_lines));
            s_text[0] = '\0';
            portEXIT_CRITICAL(&s_text_lock);
        }
        s_active_mode = mode;
        s_command_used = 0U;
        return;
    }

    s_active_mode = mode;
    esp_err_t err = start_driver(bitrate);
    if (err != ESP_OK) {
        if (err != s_last_config_error || bitrate != s_last_config_error_bitrate) {
            ESP_LOGW(TAG, "start %lu: %s", (unsigned long)bitrate, esp_err_to_name(err));
            if (err != ESP_ERR_TIMEOUT && err != ESP_ERR_INVALID_STATE) ++s_errors;
            s_last_config_error = err;
            s_last_config_error_bitrate = bitrate;
        }
        return;
    }
    s_last_config_error = ESP_OK;
    s_last_config_error_bitrate = 0U;
}

bool can_probe_input_char(char ch)
{
    if (s_active_mode != APP_MODE_CAN) return false;
    if (ch == '\r' || ch == '\n') {
        if (s_command_used != 0U) run_command();
        return true;
    }
    if (s_command_used + 1U < sizeof(s_command)) {
        s_command[s_command_used++] = ch;
        s_command_last_us = esp_timer_get_time();
    } else {
        s_command_used = 0U;
        const char *error = "[CAN TX] ERR command too long\r\n";
        send_line(error);
        ++s_errors;
    }
    return true;
}

void can_probe_update(app_state_t *state, int64_t now_us)
{
    poll_alerts();
    sync_driver_status();
    recovery_timer_update(now_us);
    process_command_idle(now_us);

    if (s_running && !s_bus_off) {
        twai_message_t message;
        uint8_t limit = 0U;
        while (limit++ < 16U && twai_receive(&message, 0) == ESP_OK) {
            if (message.extd || message.rtr || message.data_length_code > 8U) continue;
            if (!mask_allows(message.identifier)) continue;
            publish_frame(&message);
        }
    }
    if (state == NULL) return;
    portENTER_CRITICAL(&s_text_lock);
    memcpy(state->can.text, s_text, sizeof(state->can.text));
    memcpy(state->can.lines, s_lines, sizeof(state->can.lines));
    state->can.packets = s_packets;
    state->can.errors = s_errors;
    state->can.tx_error_counter = s_tx_error_counter;
    state->can.tx_queue_msgs = s_tx_queue_msgs;
    state->can.tx_result = s_tx_result;
    state->can.last_tx_ok = s_last_tx_ok;
    state->can.bus_off = s_bus_off;
    portEXIT_CRITICAL(&s_text_lock);
}
