#include "i2c_dma_sniffer.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bluetooth_spp.h"
#include "driver/gpio.h"
#include "driver/rmt_common.h"
#include "driver/rmt_rx.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "probe_config.h"

#define I2C_DMA_RMT_RESOLUTION_HZ 1000000U
#define I2C_DMA_RMT_SYMBOLS 512U
#define I2C_DMA_IDLE_STOP_NS 200000U
#define I2C_DMA_GLITCH_NS 500U
#define I2C_DMA_LINE_LEN 256U
#define I2C_DMA_NOTIFY_SCL BIT0
#define I2C_DMA_NOTIFY_SDA BIT1
#define I2C_DMA_DEBUG_LOG 1

typedef struct {
    rmt_channel_handle_t channel;
    rmt_symbol_word_t *symbols;
    size_t received;
    uint32_t notify_bit;
} i2c_dma_rmt_line_t;

static const char *TAG = "i2c_dma";
static i2c_dma_rmt_line_t s_scl;
static i2c_dma_rmt_line_t s_sda;
static TaskHandle_t s_task;
static volatile app_mode_t s_active_mode = APP_MODE_POWER_SUPPLY;
static portMUX_TYPE s_text_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_text[UART_DISPLAY_CHARS + 1U] = "";
static char s_lines[3][UART_DISPLAY_CHARS + 1U] = {{0}};
static uint32_t s_packets;
static uint32_t s_errors;

static void store_preview(const char *line)
{
    const char *preview = line;
    if (strlen(line) >= 5U && memcmp(line, "[I2C]", 5U) == 0) {
        preview = line + 5U;
    }

    size_t length = strlen(preview);
    if (length > UART_DISPLAY_CHARS) length = UART_DISPLAY_CHARS;

    portENTER_CRITICAL(&s_text_lock);
    memcpy(s_lines[0], s_lines[1], sizeof(s_lines[0]));
    memcpy(s_lines[1], s_lines[2], sizeof(s_lines[1]));
    memcpy(s_lines[2], preview, length);
    s_lines[2][length] = '\0';
    memcpy(s_text, preview, length + 1U);
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

static void publish_line(const char *line)
{
    static uint32_t debug_log_count;
    size_t length = strlen(line);
    store_preview(line);
#if I2C_DMA_DEBUG_LOG
    if (debug_log_count < 200U) {
        ++debug_log_count;
        ESP_LOGI(TAG, "I2C_SNIFF %s", line);
    }
#endif
    if (s_active_mode != APP_MODE_I2C || !bluetooth_spp_connected()) return;
    bluetooth_spp_write(line, length);
    bluetooth_spp_write("\r\n", 2U);
}

static bool IRAM_ATTR rmt_done_callback(rmt_channel_handle_t channel,
                                        const rmt_rx_done_event_data_t *edata,
                                        void *user_data)
{
    (void)channel;
    BaseType_t wake = pdFALSE;
    i2c_dma_rmt_line_t *line = (i2c_dma_rmt_line_t *)user_data;
    line->received = edata->num_symbols;
    if (s_task != NULL) xTaskNotifyFromISR(s_task, line->notify_bit, eSetBits, &wake);
    return wake == pdTRUE;
}

static esp_err_t start_receive(i2c_dma_rmt_line_t *line)
{
    line->received = 0U;
    rmt_receive_config_t config = {
        .signal_range_min_ns = I2C_DMA_GLITCH_NS,
        .signal_range_max_ns = I2C_DMA_IDLE_STOP_NS,
    };
    return rmt_receive(line->channel,
                       line->symbols,
                       I2C_DMA_RMT_SYMBOLS * sizeof(rmt_symbol_word_t),
                       &config);
}

static uint32_t symbol_duration(const rmt_symbol_word_t *symbol)
{
    return (uint32_t)symbol->duration0 + (uint32_t)symbol->duration1;
}

static uint32_t total_duration(const i2c_dma_rmt_line_t *line)
{
    uint32_t total = 0U;
    for (size_t i = 0U; i < line->received; ++i) total += symbol_duration(&line->symbols[i]);
    return total;
}

static int line_level_at(const i2c_dma_rmt_line_t *line, uint32_t time_us)
{
    uint32_t t = 0U;
    for (size_t i = 0U; i < line->received; ++i) {
        const rmt_symbol_word_t *sym = &line->symbols[i];
        if (time_us < t + sym->duration0) return sym->level0 != 0;
        t += sym->duration0;
        if (time_us < t + sym->duration1) return sym->level1 != 0;
        t += sym->duration1;
    }
    if (line->received == 0U) return gpio_get_level(line == &s_scl ? I2C_SNIFFER_SCL_GPIO : I2C_SNIFFER_SDA_GPIO);
    const rmt_symbol_word_t *last = &line->symbols[line->received - 1U];
    return last->level1 != 0;
}

static bool append_byte(char *line,
                        size_t line_size,
                        size_t *used,
                        bool *truncated,
                        bool *expect_address,
                        uint8_t byte_value,
                        int ack_level)
{
    char ack = ack_level == 0 ? 'A' : 'N';
    if (*expect_address) {
        char rw = (byte_value & 0x01U) != 0U ? 'R' : 'W';
        uint8_t address = (uint8_t)(byte_value >> 1);
        if (!append_text(line, line_size, used, "%c %02X %c", rw, address, ack)) {
            *truncated = true;
            return false;
        }
        *expect_address = false;
    } else if (!append_text(line, line_size, used, " %02X %c", byte_value, ack)) {
        *truncated = true;
        return false;
    }
    return true;
}

static void decode_capture(void)
{
    if (s_scl.received == 0U || s_sda.received == 0U) return;

    char line[I2C_DMA_LINE_LEN] = "[I2C]";
    size_t used = 5U;
    bool have_data = false;
    bool expect_address = true;
    bool truncated = false;
    uint8_t bit_count = 0U;
    uint8_t byte_value = 0U;
    uint32_t sda_symbol_index = 0U;
    uint32_t sda_time = 0U;
    int previous_sda = line_level_at(&s_sda, 0U);
    uint32_t total_us = total_duration(&s_scl);

    for (size_t si = 0U, t = 0U; si < s_scl.received; ++si) {
        const rmt_symbol_word_t *sym = &s_scl.symbols[si];
        uint32_t edge_times[2] = {t + sym->duration0, t + sym->duration0 + sym->duration1};
        int edge_from[2] = {sym->level0, sym->level1};
        int edge_to[2] = {sym->level1, si + 1U < s_scl.received ? s_scl.symbols[si + 1U].level0 : sym->level1};

        for (size_t e = 0U; e < 2U; ++e) {
            uint32_t edge_time = edge_times[e];
            while (sda_symbol_index < s_sda.received) {
                const rmt_symbol_word_t *sd = &s_sda.symbols[sda_symbol_index];
                uint32_t sda_edges[2] = {sda_time + sd->duration0, sda_time + sd->duration0 + sd->duration1};
                int sda_to[2] = {sd->level1,
                                 sda_symbol_index + 1U < s_sda.received
                                     ? s_sda.symbols[sda_symbol_index + 1U].level0
                                     : sd->level1};
                bool advanced = false;
                for (size_t se = 0U; se < 2U; ++se) {
                    if (sda_edges[se] > edge_time || sda_edges[se] > total_us) break;
                    int scl_at_sda = line_level_at(&s_scl, sda_edges[se]);
                    if (scl_at_sda != 0 && sda_to[se] != previous_sda) {
                        if (previous_sda != 0 && sda_to[se] == 0) {
                            if (have_data) {
                                if (!append_text(line, sizeof(line), &used, " S")) truncated = true;
                            }
                            have_data = true;
                            expect_address = true;
                            bit_count = 0U;
                            byte_value = 0U;
                        } else if (previous_sda == 0 && sda_to[se] != 0 && have_data) {
                            if (!append_text(line, sizeof(line), &used, " P")) truncated = true;
                            if (truncated) (void)append_text(line, sizeof(line), &used, " ...");
                            publish_line(line);
                            return;
                        }
                    }
                    previous_sda = sda_to[se];
                    advanced = true;
                }
                if (!advanced || sda_edges[1] > edge_time) break;
                sda_time += symbol_duration(sd);
                ++sda_symbol_index;
            }

            if (edge_from[e] == 0 && edge_to[e] != 0 && have_data) {
                int sda_level = line_level_at(&s_sda, edge_time);
                if (bit_count < 8U) {
                    byte_value = (uint8_t)((byte_value << 1) | (sda_level != 0 ? 1U : 0U));
                    ++bit_count;
                } else {
                    (void)append_byte(line,
                                      sizeof(line),
                                      &used,
                                      &truncated,
                                      &expect_address,
                                      byte_value,
                                      sda_level);
                    bit_count = 0U;
                    byte_value = 0U;
                }
            }
        }
        t += symbol_duration(sym);
    }
}

static void i2c_dma_sniffer_task(void *argument)
{
    (void)argument;
    while (true) {
        if (s_active_mode != APP_MODE_I2C) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        uint32_t bits = 0U;
        esp_err_t err = start_receive(&s_scl);
        if (err == ESP_OK) err = start_receive(&s_sda);
        if (err != ESP_OK) {
            add_error(1U);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (xTaskNotifyWait(0, UINT32_MAX, &bits, pdMS_TO_TICKS(50)) != pdTRUE) continue;
        while ((bits & (I2C_DMA_NOTIFY_SCL | I2C_DMA_NOTIFY_SDA)) !=
               (I2C_DMA_NOTIFY_SCL | I2C_DMA_NOTIFY_SDA)) {
            uint32_t more = 0U;
            if (xTaskNotifyWait(0, UINT32_MAX, &more, pdMS_TO_TICKS(10)) != pdTRUE) break;
            bits |= more;
        }
        if ((bits & (I2C_DMA_NOTIFY_SCL | I2C_DMA_NOTIFY_SDA)) ==
            (I2C_DMA_NOTIFY_SCL | I2C_DMA_NOTIFY_SDA)) {
            decode_capture();
        }
    }
}

static esp_err_t init_line(i2c_dma_rmt_line_t *line, gpio_num_t gpio, uint32_t notify_bit)
{
    line->symbols = heap_caps_aligned_calloc(64,
                                             I2C_DMA_RMT_SYMBOLS,
                                             sizeof(rmt_symbol_word_t),
                                             MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (line->symbols == NULL) return ESP_ERR_NO_MEM;
    line->notify_bit = notify_bit;

    rmt_rx_channel_config_t config = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = I2C_DMA_RMT_RESOLUTION_HZ,
        .mem_block_symbols = I2C_DMA_RMT_SYMBOLS,
    };
    esp_err_t err = rmt_new_rx_channel(&config, &line->channel);
    if (err != ESP_OK) {
        heap_caps_free(line->symbols);
        line->symbols = NULL;
        return err;
    }

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_done_callback,
    };
    err = rmt_rx_register_event_callbacks(line->channel, &cbs, line);
    if (err == ESP_OK) err = rmt_enable(line->channel);
    if (err != ESP_OK) {
        if (line->channel != NULL) {
            (void)rmt_disable(line->channel);
            (void)rmt_del_channel(line->channel);
            line->channel = NULL;
        }
        heap_caps_free(line->symbols);
        line->symbols = NULL;
    }
    return err;
}

esp_err_t i2c_dma_sniffer_init(void)
{
#if I2C_SNIFFER_ENABLED
    esp_err_t err = init_line(&s_scl, I2C_SNIFFER_SCL_GPIO, I2C_DMA_NOTIFY_SCL);
    if (err == ESP_OK) err = init_line(&s_sda, I2C_SNIFFER_SDA_GPIO, I2C_DMA_NOTIFY_SDA);
    if (err != ESP_OK) {
        if (s_scl.channel != NULL) {
            (void)rmt_disable(s_scl.channel);
            (void)rmt_del_channel(s_scl.channel);
            s_scl.channel = NULL;
        }
        if (s_sda.channel != NULL) {
            (void)rmt_disable(s_sda.channel);
            (void)rmt_del_channel(s_sda.channel);
            s_sda.channel = NULL;
        }
        heap_caps_free(s_scl.symbols);
        heap_caps_free(s_sda.symbols);
        s_scl.symbols = NULL;
        s_sda.symbols = NULL;
        return err;
    }
    BaseType_t ok = xTaskCreate(i2c_dma_sniffer_task, "i2c_dma", 4096, NULL, 5, &s_task);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
#else
    return ESP_OK;
#endif
}

void i2c_dma_sniffer_configure(app_mode_t mode)
{
    if (mode != s_active_mode) {
        if (mode == APP_MODE_I2C || s_active_mode == APP_MODE_I2C) clear_preview();
        s_active_mode = mode;
    }
}

void i2c_dma_sniffer_update(app_state_t *state)
{
    if (state == NULL) return;
    portENTER_CRITICAL(&s_text_lock);
    memcpy(state->i2c_sniffer.text, s_text, sizeof(state->i2c_sniffer.text));
    memcpy(state->i2c_sniffer.lines, s_lines, sizeof(state->i2c_sniffer.lines));
    state->i2c_sniffer.packets = s_packets;
    state->i2c_sniffer.errors = s_errors;
    portEXIT_CRITICAL(&s_text_lock);
}
