#include "i2c_sniffer.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bluetooth_spp.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "probe_config.h"
#include "soc/gpio_struct.h"

#define I2C_SNIFFER_EVENT_BUFFER_COUNT 8U
#define I2C_SNIFFER_EVENT_BUFFER_LEN 160U
#define I2C_SNIFFER_PACKET_TEXT_LEN 256U
#define I2C_SNIFFER_TOKEN_BYTE_ACK_BASE 0x000U
#define I2C_SNIFFER_TOKEN_BYTE_NACK_BASE 0x100U
#define I2C_SNIFFER_TOKEN_START 0x200U
#define I2C_SNIFFER_TOKEN_STOP 0x201U
#define I2C_SNIFFER_DEBUG_ALWAYS 1
#define I2C_SNIFFER_DEBUG_LOG 1

typedef struct {
    uint16_t events[I2C_SNIFFER_EVENT_BUFFER_LEN];
    uint16_t length;
    bool overflow;
} i2c_sniffer_event_buffer_t;

typedef struct {
    uint8_t buffer_index;
    uint16_t length;
    bool overflow;
} i2c_sniffer_ready_buffer_t;

static i2c_sniffer_event_buffer_t s_event_buffers[I2C_SNIFFER_EVENT_BUFFER_COUNT];
static QueueHandle_t s_ready_queue;
static const char *TAG = "i2c_sniff";
static portMUX_TYPE s_text_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_text[UART_DISPLAY_CHARS + 1U] = "";
static char s_lines[3][UART_DISPLAY_CHARS + 1U] = {{0}};
static uint32_t s_packets;
static uint32_t s_errors;
static volatile uint32_t s_isr_errors;
static volatile app_mode_t s_active_mode = APP_MODE_POWER_SUPPLY;
static volatile uint8_t s_active_buffer;
static volatile bool s_recording;
static volatile uint8_t s_last_sda = 1U;
static volatile uint8_t s_last_scl = 1U;
static volatile uint8_t s_current_byte;
static volatile uint8_t s_bit_count;

static void store_preview(const char *line)
{
    size_t length = strlen(line);
    if (length > UART_DISPLAY_CHARS) length = UART_DISPLAY_CHARS;

    portENTER_CRITICAL(&s_text_lock);
    memcpy(s_lines[0], s_lines[1], sizeof(s_lines[0]));
    memcpy(s_lines[1], s_lines[2], sizeof(s_lines[1]));
    memcpy(s_lines[2], line, length);
    s_lines[2][length] = '\0';
    memcpy(s_text, line, length + 1U);
    ++s_packets;
    portEXIT_CRITICAL(&s_text_lock);
}

static void clear_preview(void)
{
    portENTER_CRITICAL(&s_text_lock);
    memset(s_lines, 0, sizeof(s_lines));
    s_text[0] = '\0';
    portEXIT_CRITICAL(&s_text_lock);
}

static void add_error(uint32_t count)
{
    portENTER_CRITICAL(&s_text_lock);
    s_errors += count;
    portEXIT_CRITICAL(&s_text_lock);
}

static void IRAM_ATTR event_buffer_reset(uint8_t index)
{
    s_event_buffers[index].length = 0U;
    s_event_buffers[index].overflow = false;
}

static inline uint8_t IRAM_ATTR read_gpio_fast(gpio_num_t gpio)
{
    uint32_t number = (uint32_t)gpio;
    if (number < 32U) return (uint8_t)((GPIO.in >> number) & 0x1U);
    return (uint8_t)((GPIO.in1.val >> (number - 32U)) & 0x1U);
}

static void IRAM_ATTR event_buffer_push(uint16_t event)
{
    uint8_t index = s_active_buffer;
    uint16_t length = s_event_buffers[index].length;
    if (length < I2C_SNIFFER_EVENT_BUFFER_LEN) {
        s_event_buffers[index].events[length] = event;
        s_event_buffers[index].length = (uint16_t)(length + 1U);
    } else {
        s_event_buffers[index].overflow = true;
    }
}

static void IRAM_ATTR publish_active_buffer_from_isr(void)
{
    uint8_t ready_index = s_active_buffer;
    i2c_sniffer_ready_buffer_t ready = {
        .buffer_index = ready_index,
        .length = s_event_buffers[ready_index].length,
        .overflow = s_event_buffers[ready_index].overflow,
    };

    uint8_t next_index = (uint8_t)((ready_index + 1U) % I2C_SNIFFER_EVENT_BUFFER_COUNT);
    event_buffer_reset(next_index);
    s_active_buffer = next_index;
    s_recording = false;
    s_current_byte = 0U;
    s_bit_count = 0U;

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (xQueueSendFromISR(s_ready_queue, &ready, &higher_priority_task_woken) != pdTRUE) {
        ++s_isr_errors;
    }
    if (higher_priority_task_woken == pdTRUE) portYIELD_FROM_ISR();
}

static void IRAM_ATTR i2c_sniffer_sda_isr(void *argument)
{
    (void)argument;
    uint8_t sda = read_gpio_fast(I2C_SNIFFER_SDA_GPIO);
    uint8_t scl = read_gpio_fast(I2C_SNIFFER_SCL_GPIO);

    if (sda == s_last_sda) return;
    uint8_t previous_sda = s_last_sda;
    s_last_sda = sda;

    if (!I2C_SNIFFER_DEBUG_ALWAYS && s_active_mode != APP_MODE_I2C) return;
    if (scl == 0U) return;

    if (previous_sda != 0U && sda == 0U) {
        if (!s_recording) {
            event_buffer_reset(s_active_buffer);
            s_recording = true;
        }
        s_current_byte = 0U;
        s_bit_count = 0U;
        event_buffer_push(I2C_SNIFFER_TOKEN_START);
    } else if (previous_sda == 0U && sda != 0U && s_recording) {
        event_buffer_push(I2C_SNIFFER_TOKEN_STOP);
        publish_active_buffer_from_isr();
    }
}

static void IRAM_ATTR i2c_sniffer_scl_isr(void *argument)
{
    (void)argument;
    uint8_t scl = read_gpio_fast(I2C_SNIFFER_SCL_GPIO);
    if (scl == s_last_scl) return;
    uint8_t previous_scl = s_last_scl;
    s_last_scl = scl;

    if (previous_scl != 0U || scl == 0U) return;
    if (!s_recording) return;
    if (!I2C_SNIFFER_DEBUG_ALWAYS && s_active_mode != APP_MODE_I2C) return;

    uint8_t sda = read_gpio_fast(I2C_SNIFFER_SDA_GPIO);
    if (s_bit_count < 8U) {
        s_current_byte = (uint8_t)((s_current_byte << 1) | sda);
        ++s_bit_count;
        return;
    }

    uint16_t token = sda == 0U ? I2C_SNIFFER_TOKEN_BYTE_ACK_BASE : I2C_SNIFFER_TOKEN_BYTE_NACK_BASE;
    event_buffer_push((uint16_t)(token | s_current_byte));
    s_current_byte = 0U;
    s_bit_count = 0U;
}

static bool append_text(char *out, size_t out_size, size_t *used, const char *format, ...)
{
    if (*used >= out_size) return false;

    va_list args;
    va_start(args, format);
    int written = vsnprintf(out + *used, out_size - *used, format, args);
    va_end(args);
    if (written <= 0) return false;

    if ((size_t)written >= out_size - *used) {
        *used = out_size - 1U;
        out[*used] = '\0';
        return false;
    }

    *used += (size_t)written;
    return true;
}

static void store_line(const char *line)
{
    static uint32_t debug_log_count;
    size_t length = strlen(line);
    const char *preview = line;
    if (length >= 5U && memcmp(line, "[I2C]", 5U) == 0) {
        preview = line + 5U;
    }

    store_preview(preview);

#if I2C_SNIFFER_DEBUG_LOG
    if (debug_log_count < 200U) {
        ++debug_log_count;
        ESP_LOGI(TAG, "I2C_SNIFF %s", line);
    }
#endif
    if (s_active_mode != APP_MODE_I2C || !bluetooth_spp_connected()) return;
    bluetooth_spp_write(line, length);
    bluetooth_spp_write("\r\n", 2U);
}

static void publish_line(char *text, size_t length, bool truncated)
{
    if (length == 0U) return;

    if (truncated) {
        (void)append_text(text, I2C_SNIFFER_PACKET_TEXT_LEN, &length, " ...");
    }
    store_line(text);
}

static void parse_ready_buffer(const i2c_sniffer_ready_buffer_t *ready)
{
    char line[I2C_SNIFFER_PACKET_TEXT_LEN] = "[I2C]";
    size_t used = 5U;
    bool truncated = ready->overflow;
    bool have_data = false;
    bool have_byte = false;
    bool expect_address = true;
    const i2c_sniffer_event_buffer_t *buffer = &s_event_buffers[ready->buffer_index];

    for (uint16_t i = 0U; i < ready->length; ++i) {
        uint16_t event = buffer->events[i];
        if (event == I2C_SNIFFER_TOKEN_START) {
            if (have_data) {
                if (!append_text(line, sizeof(line), &used, " S")) truncated = true;
            }
            have_data = true;
            have_byte = false;
            expect_address = true;
            continue;
        }

        if (event == I2C_SNIFFER_TOKEN_STOP) {
            if (have_data && have_byte) {
                if (!append_text(line, sizeof(line), &used, " P")) truncated = true;
                publish_line(line, used, truncated);
            }
            have_data = false;
            have_byte = false;
            expect_address = true;
            continue;
        }

        if (!have_data) continue;
        if ((event & ~(uint16_t)0x1FFU) != 0U) continue;

        uint8_t byte_value = (uint8_t)(event & 0xFFU);
        char ack = (event & I2C_SNIFFER_TOKEN_BYTE_NACK_BASE) != 0U ? 'N' : 'A';
        have_byte = true;
        if (expect_address) {
            char rw = (byte_value & 0x01U) != 0U ? 'R' : 'W';
            uint8_t address = (uint8_t)(byte_value >> 1);
            const char *format = used == 5U ? "%c %02X %c" : " %c %02X %c";
            if (!append_text(line, sizeof(line), &used, format, rw, address, ack)) {
                truncated = true;
            }
            expect_address = false;
        } else {
            if (!append_text(line, sizeof(line), &used, " %02X %c", byte_value, ack)) {
                truncated = true;
            }
        }
    }
}

static void i2c_sniffer_decode_task(void *argument)
{
    (void)argument;
    i2c_sniffer_ready_buffer_t ready;
    uint32_t last_isr_errors = 0U;

    while (true) {
        if (s_isr_errors != last_isr_errors) {
            add_error(s_isr_errors - last_isr_errors);
            last_isr_errors = s_isr_errors;
        }
        if (xQueueReceive(s_ready_queue, &ready, portMAX_DELAY) != pdTRUE) continue;
        parse_ready_buffer(&ready);
    }
}

esp_err_t i2c_sniffer_init(void)
{
#if I2C_SNIFFER_ENABLED
    s_ready_queue = xQueueCreate(I2C_SNIFFER_EVENT_BUFFER_COUNT, sizeof(i2c_sniffer_ready_buffer_t));
    if (s_ready_queue == NULL) return ESP_ERR_NO_MEM;

    for (uint8_t i = 0U; i < I2C_SNIFFER_EVENT_BUFFER_COUNT; ++i) {
        event_buffer_reset(i);
    }
    s_active_buffer = 0U;
    s_recording = false;
    s_current_byte = 0U;
    s_bit_count = 0U;

    gpio_config_t input = {
        .pin_bit_mask = (1ULL << I2C_SNIFFER_SCL_GPIO) | (1ULL << I2C_SNIFFER_SDA_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&input), "i2c_sniffer", "gpio config");
    s_last_sda = read_gpio_fast(I2C_SNIFFER_SDA_GPIO);
    s_last_scl = read_gpio_fast(I2C_SNIFFER_SCL_GPIO);

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(I2C_SNIFFER_SDA_GPIO, GPIO_INTR_ANYEDGE),
                        "i2c_sniffer",
                        "sda intr");
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(I2C_SNIFFER_SCL_GPIO, GPIO_INTR_ANYEDGE),
                        "i2c_sniffer",
                        "scl intr");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(I2C_SNIFFER_SDA_GPIO,
                                             i2c_sniffer_sda_isr,
                                             NULL),
                        "i2c_sniffer",
                        "sda isr");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(I2C_SNIFFER_SCL_GPIO,
                                             i2c_sniffer_scl_isr,
                                             NULL),
                        "i2c_sniffer",
                        "scl isr");

    BaseType_t ok = xTaskCreate(i2c_sniffer_decode_task, "i2c_decode", 4096, NULL, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
#else
    return ESP_OK;
#endif
}

void i2c_sniffer_configure(app_mode_t mode)
{
    if (mode != s_active_mode) {
        if (mode == APP_MODE_I2C || s_active_mode == APP_MODE_I2C) {
            if (s_ready_queue != NULL) xQueueReset(s_ready_queue);
            s_recording = false;
            s_current_byte = 0U;
            s_bit_count = 0U;
            s_last_sda = read_gpio_fast(I2C_SNIFFER_SDA_GPIO);
            s_last_scl = read_gpio_fast(I2C_SNIFFER_SCL_GPIO);
            clear_preview();
        }
        s_active_mode = mode;
    }
}

void i2c_sniffer_update(app_state_t *state)
{
    if (state == NULL) return;
    portENTER_CRITICAL(&s_text_lock);
    memcpy(state->i2c_sniffer.text, s_text, sizeof(state->i2c_sniffer.text));
    memcpy(state->i2c_sniffer.lines, s_lines, sizeof(state->i2c_sniffer.lines));
    state->i2c_sniffer.packets = s_packets;
    state->i2c_sniffer.errors = s_errors;
    portEXIT_CRITICAL(&s_text_lock);
}
