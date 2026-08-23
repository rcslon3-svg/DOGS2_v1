#include "uart_probe.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "board_io.h"
#include "bluetooth_spp.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lin_sniffer.h"
#include "probe_config.h"

#define UART_A_PORT UART_NUM_1
#define UART_RX_LINE_CHARS 128U
#define RS485_DISPLAY_BYTES_WITH_ELLIPSIS 4U
#define RS485_PACKET_TIMEOUT_MIN_US 2000LL
#define UART_LINE_TIMEOUT_MIN_US 20000LL

static portMUX_TYPE s_uart_text_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_uart_a_text[UART_DISPLAY_CHARS + 1U] = "";
static char s_uart_a_lines[3][UART_DISPLAY_CHARS + 1U] = {{0}};
static char s_uart_bt_out[UART_RX_LINE_CHARS * 3U + 16U];
static uint32_t s_uart_a_errors;
static QueueHandle_t s_uart_a_event_queue;
static bool s_uart_ready;
static bool s_uart_pins_active;
static volatile app_mode_t s_active_mode = APP_MODE_POWER_SUPPLY;
static volatile uint32_t s_active_baud = UART_TEST_BAUD;

static int64_t rs485_packet_timeout_us(void)
{
    uint32_t baud = s_active_baud == 0U ? UART_TEST_BAUD : s_active_baud;
    int64_t timeout = (int64_t)((40ULL * 1000000ULL + baud - 1ULL) / baud);
    return timeout < RS485_PACKET_TIMEOUT_MIN_US ? RS485_PACKET_TIMEOUT_MIN_US : timeout;
}

static int64_t uart_line_timeout_us(void)
{
    uint32_t baud = s_active_baud == 0U ? UART_TEST_BAUD : s_active_baud;
    int64_t timeout = (int64_t)((40ULL * 1000000ULL + baud - 1ULL) / baud);
    return timeout < UART_LINE_TIMEOUT_MIN_US ? UART_LINE_TIMEOUT_MIN_US : timeout;
}

static bool uart_protocol_mode(app_mode_t mode)
{
    return mode == APP_MODE_UART || mode == APP_MODE_LIN || mode == APP_MODE_RS485;
}

static void uart_probe_set_pins_active(bool active)
{
    if (active == s_uart_pins_active) return;

    if (active) {
        if (uart_set_pin(UART_A_PORT, UART_A_TX_GPIO, UART_A_RX_GPIO,
                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK) {
            s_uart_pins_active = true;
        }
        return;
    }

    (void)uart_wait_tx_done(UART_A_PORT, pdMS_TO_TICKS(20));
    (void)gpio_set_direction(UART_A_TX_GPIO, GPIO_MODE_INPUT);
    (void)gpio_set_direction(UART_A_RX_GPIO, GPIO_MODE_INPUT);
    s_uart_pins_active = false;
}

/* uart_preview_store
 * Inputs:
 *   line - zero-terminated text to show on the display; must not be NULL.
 * Returns: none.
 * Does: stores the last completed UART A line in the shared preview buffer.
 */
static void uart_preview_store(const char *line)
{
    size_t length = strlen(line);
    if (length > UART_DISPLAY_CHARS) length = UART_DISPLAY_CHARS;

    portENTER_CRITICAL(&s_uart_text_lock);
    memcpy(s_uart_a_lines[0], s_uart_a_lines[1], sizeof(s_uart_a_lines[0]));
    memcpy(s_uart_a_lines[1], s_uart_a_lines[2], sizeof(s_uart_a_lines[1]));
    memcpy(s_uart_a_lines[2], line, length);
    s_uart_a_lines[2][length] = '\0';
    memcpy(s_uart_a_text, line, length + 1U);
    portEXIT_CRITICAL(&s_uart_text_lock);
}

static void uart_line_store_and_forward(const char *line, bool truncated)
{
    char preview[UART_DISPLAY_CHARS + 1U];
    size_t length = strlen(line);

    if (s_active_mode == APP_MODE_LIN) {
        const char *status = strstr(line, " CRC");
        const char *parity = strstr(line, " PAR:");
        const char *end = status != NULL ? status : parity;
        length = end != NULL ? (size_t)(end - line) : length;
        if (length > UART_DISPLAY_CHARS) length = UART_DISPLAY_CHARS;
        memcpy(preview, line, length);
        preview[length] = '\0';
    } else if (length > UART_PREVIEW_CHARS || truncated) {
        memcpy(preview, line, UART_PREVIEW_CHARS);
        memcpy(preview + UART_PREVIEW_CHARS, "...", 4U);
    } else {
        memcpy(preview, line, length + 1U);
    }
    uart_preview_store(preview);

    if ((s_active_mode == APP_MODE_UART || s_active_mode == APP_MODE_LIN) &&
        bluetooth_spp_connected()) {
        char out[UART_RX_LINE_CHARS + 16U];
        const char *prefix = s_active_mode == APP_MODE_LIN ? "[LIN]" : "[UART]";
        int written = snprintf(out, sizeof(out), "%s%s\r\n", prefix, line);
        if (written > 0) {
            size_t length = (size_t)written;
            if (length >= sizeof(out)) length = sizeof(out) - 1U;
            bluetooth_spp_write(out, length);
        }
    }
}

static void rs485_format_display(const uint8_t *data, size_t length, bool truncated, char *out, size_t out_size)
{
    size_t bytes_to_show = length;
    size_t used = 0U;
    bool ellipsis = truncated || length > 5U;

    if (ellipsis && bytes_to_show > RS485_DISPLAY_BYTES_WITH_ELLIPSIS) {
        bytes_to_show = RS485_DISPLAY_BYTES_WITH_ELLIPSIS;
    }

    if (out_size == 0U) return;
    out[0] = '\0';
    for (size_t i = 0U; i < bytes_to_show && used + 3U < out_size; ++i) {
        int written = snprintf(out + used,
                               out_size - used,
                               i == 0U ? "%02X" : " %02X",
                               data[i]);
        if (written <= 0) break;
        used += (size_t)written;
    }
    if (ellipsis && used + 5U <= out_size) {
        memcpy(out + used, " ...", 5U);
    }
}

static void rs485_packet_store_and_forward(const uint8_t *data, size_t length, bool truncated)
{
    char preview[UART_DISPLAY_CHARS + 1U];
    size_t used;

    if (length == 0U) return;
    rs485_format_display(data, length, truncated, preview, sizeof(preview));
    uart_preview_store(preview);

    if (s_active_mode != APP_MODE_RS485 || !bluetooth_spp_connected()) return;

    used = (size_t)snprintf(s_uart_bt_out, sizeof(s_uart_bt_out), "[485]");
    for (size_t i = 0U; i < length && used + 4U < sizeof(s_uart_bt_out); ++i) {
        int written = snprintf(s_uart_bt_out + used,
                               sizeof(s_uart_bt_out) - used,
                               "%s%02X",
                               i == 0U ? "" : " ",
                               data[i]);
        if (written <= 0) break;
        used += (size_t)written;
    }
    if (truncated && used + 4U < sizeof(s_uart_bt_out)) {
        memcpy(s_uart_bt_out + used, " ...", 4U);
        used += 4U;
    }
    if (used + 2U < sizeof(s_uart_bt_out)) {
        memcpy(s_uart_bt_out + used, "\r\n", 2U);
        used += 2U;
    }
    bluetooth_spp_write(s_uart_bt_out, used);
}

/* uart_error_add
 * Inputs:
 *   count - number of receive-side errors to add.
 * Returns: none.
 * Does: increments the shared UART A diagnostic error counter.
 */
static void uart_error_add(uint32_t count)
{
    portENTER_CRITICAL(&s_uart_text_lock);
    s_uart_a_errors += count;
    portEXIT_CRITICAL(&s_uart_text_lock);
}

/* uart_error_is_static_low_artifact
 * Inputs:
 *   type - UART driver event type reported for UART A.
 * Returns: true when the event is caused by a normal probe LOW state.
 * Does: suppresses BREAK/framing artifacts that appear when UART RX is shorted
 * to ground and therefore should be interpreted as logic LOW, not UART fault.
 */
static bool uart_error_is_static_low_artifact(uart_event_type_t type)
{
    if (type == UART_BREAK) return true;
    if (type == UART_FRAME_ERR && gpio_get_level(UART_A_RX_GPIO) == 0) return true;
    return false;
}

/* uart_a_rx_task
 * Inputs:
 *   argument - unused FreeRTOS task argument.
 * Returns: never returns.
 * Does: receives UART driver events from UART A, assembles one display line,
 * and updates the error counter for meaningful UART receive problems.
 */
static void uart_a_rx_task(void *argument)
{
    (void)argument;
    char line[UART_RX_LINE_CHARS + 1U] = "";
    uint8_t packet[UART_RX_LINE_CHARS] = {0};
    size_t used = 0U;
    size_t packet_used = 0U;
    bool truncated = false;
    bool packet_truncated = false;
    int64_t last_rs485_byte_us = 0;
    int64_t last_uart_byte_us = 0;
    app_mode_t observed_mode = s_active_mode;
    uart_event_t event;
    uint8_t bytes[64];

    while (true) {
        if (observed_mode != s_active_mode) {
            observed_mode = s_active_mode;
            used = 0U;
            packet_used = 0U;
            truncated = false;
            packet_truncated = false;
        }

        if (xQueueReceive(s_uart_a_event_queue, &event, pdMS_TO_TICKS(1)) != pdTRUE) {
            if (s_active_mode == APP_MODE_RS485 &&
                packet_used != 0U &&
                esp_timer_get_time() - last_rs485_byte_us >= rs485_packet_timeout_us()) {
                rs485_packet_store_and_forward(packet, packet_used, packet_truncated);
                packet_used = 0U;
                packet_truncated = false;
            } else if (s_active_mode == APP_MODE_LIN) {
                lin_sniffer_poll(esp_timer_get_time(), uart_line_store_and_forward);
            } else if (s_active_mode == APP_MODE_UART &&
                       used != 0U &&
                       esp_timer_get_time() - last_uart_byte_us >= uart_line_timeout_us()) {
                line[used] = '\0';
                uart_line_store_and_forward(line, truncated);
                used = 0U;
                truncated = false;
            }
            continue;
        }

        switch (event.type) {
            case UART_DATA: {
                size_t remaining = event.size;
                while (remaining != 0U) {
                    int request = remaining > sizeof(bytes) ? sizeof(bytes) : remaining;
                    int count = uart_read_bytes(UART_A_PORT, bytes, request, 0);
                    if (count <= 0) break;
                    remaining -= (size_t)count;

                    for (int i = 0; i < count; ++i) {
                        uint8_t byte = bytes[i];
                        if (s_active_mode == APP_MODE_RS485) {
                            if (packet_used < sizeof(packet)) {
                                packet[packet_used++] = byte;
                            } else {
                                packet_truncated = true;
                            }
                            last_rs485_byte_us = esp_timer_get_time();
                            continue;
                        }
                        if (s_active_mode == APP_MODE_LIN) {
                            lin_sniffer_on_byte(byte, esp_timer_get_time(), uart_line_store_and_forward);
                            continue;
                        }

                        if (byte == '\r' || byte == '\n') {
                            if (used != 0U) {
                                line[used] = '\0';
                                uart_line_store_and_forward(line, truncated);
                                used = 0U;
                                truncated = false;
                            }
                            continue;
                        }

                        if (used < UART_RX_LINE_CHARS) {
                            line[used++] = (char)byte;
                            line[used] = '\0';
                        } else {
                            truncated = true;
                        }
                        last_uart_byte_us = esp_timer_get_time();
                    }
                }
                break;
            }

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_error_add(1U);
                uart_flush_input(UART_A_PORT);
                xQueueReset(s_uart_a_event_queue);
                used = 0U;
                packet_used = 0U;
                truncated = false;
                packet_truncated = false;
                break;

            case UART_PARITY_ERR:
            case UART_FRAME_ERR:
            case UART_BREAK:
                if (s_active_mode == APP_MODE_LIN &&
                    (event.type == UART_BREAK || event.type == UART_FRAME_ERR)) {
                    lin_sniffer_on_break();
                    break;
                }
                if (!uart_error_is_static_low_artifact(event.type)) uart_error_add(1U);
                break;

            default:
                break;
        }
    }
}

/* uart_probe_init
 * Inputs: none.
 * Returns: ESP_OK on success, or an ESP-IDF error from UART setup.
 * Does: initializes UART A and starts RX preview plus test TX. Pins are only
 * assigned while a UART/LIN/RS485 mode is active, because Engineering Sample
 * shares IO13 with the power-screen frequency counter.
 */
esp_err_t uart_probe_init(void)
{
    const uart_config_t config = {
        .baud_rate = UART_TEST_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(UART_A_PORT, 1024, 256, 20, &s_uart_a_event_queue, 0),
                        "uart_probe", "install UART A");
    ESP_RETURN_ON_ERROR(uart_param_config(UART_A_PORT, &config), "uart_probe", "config UART A");
    xTaskCreate(uart_a_rx_task, "uart_a_rx", 4096, NULL, 5, NULL);
    s_uart_ready = true;
    return ESP_OK;
}

void uart_probe_configure(app_mode_t mode,
                          uint32_t uart_baud,
                          uint32_t lin_baud,
                          uint32_t rs485_baud,
                          uint16_t lin_mask_value,
                          uint8_t lin_mask_care)
{
    uint32_t baud = mode == APP_MODE_RS485 ? rs485_baud :
                    mode == APP_MODE_LIN ? lin_baud : uart_baud;
    lin_sniffer_configure(lin_baud, lin_mask_value, lin_mask_care);

    if (!uart_protocol_mode(mode)) {
        uart_probe_set_pins_active(false);
        if (mode != s_active_mode) {
            portENTER_CRITICAL(&s_uart_text_lock);
            memset(s_uart_a_lines, 0, sizeof(s_uart_a_lines));
            s_uart_a_text[0] = '\0';
            portEXIT_CRITICAL(&s_uart_text_lock);
            lin_sniffer_reset();
            s_active_mode = mode;
        }
        return;
    }

    uart_probe_set_pins_active(true);

    if (baud == 0U) baud = UART_TEST_BAUD;
    if (s_uart_ready && baud != s_active_baud) {
        if (uart_wait_tx_done(UART_A_PORT, pdMS_TO_TICKS(20)) == ESP_OK &&
            uart_set_baudrate(UART_A_PORT, baud) == ESP_OK) {
            s_active_baud = baud;
        }
    } else if (!s_uart_ready) {
        s_active_baud = baud;
    }

    if (mode != s_active_mode) {
        portENTER_CRITICAL(&s_uart_text_lock);
        memset(s_uart_a_lines, 0, sizeof(s_uart_a_lines));
        s_uart_a_text[0] = '\0';
        portEXIT_CRITICAL(&s_uart_text_lock);
        lin_sniffer_reset();
        s_active_mode = mode;
    }
}

/* uart_probe_update
 * Inputs:
 *   state - application state to update; ignored when NULL.
 * Returns: none.
 * Does: copies UART display state into state->uart under a short lock.
 */
void uart_probe_update(app_state_t *state)
{
    if (state == NULL) return;
    portENTER_CRITICAL(&s_uart_text_lock);
    memcpy(state->uart.text, s_uart_a_text, sizeof(state->uart.text));
    memcpy(state->uart.lines, s_uart_a_lines, sizeof(state->uart.lines));
    state->uart.errors = s_uart_a_errors;
    portEXIT_CRITICAL(&s_uart_text_lock);
}

void uart_probe_write_bytes(const char *data, size_t length)
{
    if (!s_uart_ready || data == NULL || length == 0U) return;

    if (s_active_mode == APP_MODE_RS485) {
        board_io_set_rs485_tx_enabled(true);
        esp_rom_delay_us(20U);
    }

    uart_write_bytes(UART_A_PORT, data, length);

    if (s_active_mode == APP_MODE_RS485) {
        uint32_t baud = s_active_baud == 0U ? UART_TEST_BAUD : s_active_baud;
        uint32_t timeout_ms = (uint32_t)((length * 12ULL * 1000ULL + baud - 1ULL) / baud) + 10U;
        (void)uart_wait_tx_done(UART_A_PORT, pdMS_TO_TICKS(timeout_ms));
        board_io_set_rs485_tx_enabled(false);
    }
}
