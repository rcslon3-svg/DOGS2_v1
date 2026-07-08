#include "uart_probe.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "probe_config.h"

#define UART_A_PORT UART_NUM_1

static portMUX_TYPE s_uart_text_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_uart_a_text[UART_DISPLAY_CHARS + 1U] = "";
static uint32_t s_uart_a_errors;
static QueueHandle_t s_uart_a_event_queue;

/* uart_preview_store
 * Inputs:
 *   line - zero-terminated text to show on the display; must not be NULL.
 * Returns: none.
 * Does: stores the last completed UART A line in the shared preview buffer.
 */
static void uart_preview_store(const char *line)
{
    portENTER_CRITICAL(&s_uart_text_lock);
    snprintf(s_uart_a_text, sizeof(s_uart_a_text), "%s", line);
    portEXIT_CRITICAL(&s_uart_text_lock);
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
 * Does: suppresses BREAK/framing artifacts that appear when IO22 is shorted
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
    char line[UART_DISPLAY_CHARS + 1U] = "";
    size_t used = 0U;
    uart_event_t event;
    uint8_t bytes[64];

    while (true) {
        if (xQueueReceive(s_uart_a_event_queue, &event, pdMS_TO_TICKS(250)) != pdTRUE)
            continue;

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
                        if (byte == '\r' || byte == '\n') {
                            if (used != 0U) {
                                line[used] = '\0';
                                uart_preview_store(line);
                                used = 0U;
                            }
                            continue;
                        }

                        if (used < UART_DISPLAY_CHARS) {
                            line[used++] = (char)byte;
                            line[used] = '\0';
                        }
                        if (used == UART_DISPLAY_CHARS) uart_preview_store(line);
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
                break;

            case UART_PARITY_ERR:
            case UART_FRAME_ERR:
            case UART_BREAK:
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
 * Does: initializes UART A RX on IO22 and starts the RX preview task.
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

    ESP_RETURN_ON_ERROR(uart_driver_install(UART_A_PORT, 1024, 0, 20, &s_uart_a_event_queue, 0),
                        "uart_probe", "install UART A");
    ESP_RETURN_ON_ERROR(uart_param_config(UART_A_PORT, &config), "uart_probe", "config UART A");
    ESP_RETURN_ON_ERROR(uart_set_pin(UART_A_PORT, UART_PIN_NO_CHANGE, UART_A_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        "uart_probe", "pins UART A");

    xTaskCreate(uart_a_rx_task, "uart_a_rx", 3072, NULL, 5, NULL);
    return ESP_OK;
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
    snprintf(state->uart.text, sizeof(state->uart.text), "%s", s_uart_a_text);
    state->uart.errors = s_uart_a_errors;
    portEXIT_CRITICAL(&s_uart_text_lock);
}
