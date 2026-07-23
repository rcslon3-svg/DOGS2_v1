#include "i2c_i2s_sniffer.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bluetooth_spp.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "probe_config.h"
#include "rom/lldesc.h"
#include "soc/gpio_pins.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_struct.h"

#define I2C_I2S_SAMPLE_HZ 2000000U
#define I2C_I2S_DMA_DESC_BYTES 4000U
#define I2C_I2S_DMA_DESC_COUNT 16U
#define I2C_I2S_BUFFER_BYTES (I2C_I2S_DMA_DESC_BYTES * I2C_I2S_DMA_DESC_COUNT)
#define I2C_I2S_TX_CLOCK_BUFFER_BYTES 256U
#define I2C_I2S_LINE_LEN 256U
#define I2C_I2S_ENABLE_EXPERIMENT 1
#define I2C_I2S_CLOCK_OUT_GPIO I2C_SNIFFER_I2S_CLOCK_OUT_GPIO
#define I2C_I2S_CLOCK_IN_GPIO I2C_SNIFFER_I2S_CLOCK_IN_GPIO
#define I2C_I2S_DEBUG_ALWAYS 0
#define I2C_I2S_DEBUG_LOG 0
#define I2C_I2S_LAYOUT_DEBUG_CAPTURES 0U
#define I2C_I2S_RAW_DEBUG_ACTIVE_CAPTURES 0U
#define I2C_I2S_MIN_STABLE_SAMPLES 3U
#define I2C_I2S_CLKM_DIV_NUM (80000000U / I2C_I2S_SAMPLE_HZ)
#define I2C_I2S_PUBLISH_MIN_US 50000LL

typedef struct {
    bool active;
    bool line_has_byte;
    bool expect_address;
    uint8_t previous_scl;
    uint8_t previous_sda;
    uint8_t bit_count;
    uint8_t byte_value;
    char line[I2C_I2S_LINE_LEN];
    size_t used;
} i2c_decode_context_t;

typedef struct {
    bool initialized;
    uint8_t accepted;
    uint8_t candidate;
    uint8_t count;
} i2c_sample_filter_t;

static const char *TAG = "i2c_i2s";
static lldesc_t s_dma_desc[I2C_I2S_DMA_DESC_COUNT];
static lldesc_t s_tx_clock_dma_desc;
static uint8_t *s_dma_buffer;
static uint8_t *s_tx_clock_buffer;
static QueueHandle_t s_dma_ready_queue;
static intr_handle_t s_i2s0_intr;
static TaskHandle_t s_task;
static volatile app_mode_t s_active_mode = APP_MODE_POWER_SUPPLY;
static portMUX_TYPE s_text_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_text[I2C_DISPLAY_CHARS + 1U] = "";
static char s_lines[4][I2C_DISPLAY_CHARS + 1U] = {{0}};
static uint32_t s_packets;
static uint32_t s_errors;
static uint32_t s_captures;
static uint32_t s_timeouts;
static uint32_t s_valid_lines;
static uint32_t s_bad_lines;
static uint32_t s_unknown_addr_lines;
static volatile uint32_t s_overruns;
static volatile bool s_overrun_pending;
static bool s_rx_running;
static uint16_t s_mask_value;
static uint8_t s_mask_care;
static int64_t s_last_publish_us;
static i2c_decode_context_t s_decoder;
static i2c_sample_filter_t s_sample_filter;
#if I2C_I2S_LAYOUT_DEBUG_CAPTURES > 0U
static uint32_t s_layout_debug_count;
#endif
#if I2C_I2S_RAW_DEBUG_ACTIVE_CAPTURES > 0U
static uint32_t s_raw_debug_count;
#endif

static void store_preview(const char *line)
{
    const char *preview = line;
    if (strlen(line) >= 5U && memcmp(line, "[I2C]", 5U) == 0) preview = line + 5U;

    size_t length = strlen(preview);
    if (length > I2C_DISPLAY_CHARS) length = I2C_DISPLAY_CHARS;

    portENTER_CRITICAL(&s_text_lock);
    memcpy(s_lines[0], s_lines[1], sizeof(s_lines[0]));
    memcpy(s_lines[1], s_lines[2], sizeof(s_lines[1]));
    memcpy(s_lines[2], s_lines[3], sizeof(s_lines[2]));
    memcpy(s_lines[3], preview, length);
    s_lines[3][length] = '\0';
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

static bool parse_hex_byte_token(const char **cursor, uint8_t *value)
{
    while (**cursor == ' ') ++(*cursor);
    char c0 = (*cursor)[0];
    char c1 = (*cursor)[1];
    uint8_t n0;
    uint8_t n1;
    if (c0 >= '0' && c0 <= '9') n0 = (uint8_t)(c0 - '0');
    else if (c0 >= 'A' && c0 <= 'F') n0 = (uint8_t)(c0 - 'A' + 10);
    else return false;
    if (c1 >= '0' && c1 <= '9') n1 = (uint8_t)(c1 - '0');
    else if (c1 >= 'A' && c1 <= 'F') n1 = (uint8_t)(c1 - 'A' + 10);
    else return false;
    if ((*cursor)[2] != '\0' && (*cursor)[2] != ' ') return false;
    *value = (uint8_t)((n0 << 4) | n1);
    *cursor += 2;
    return true;
}

static bool parse_ack_token(const char **cursor)
{
    while (**cursor == ' ') ++(*cursor);
    char c = **cursor;
    if (c != 'A' && c != 'N') return false;
    ++(*cursor);
    return **cursor == '\0' || **cursor == ' ';
}

static bool address_matches_mask(uint16_t address, uint8_t digits, uint16_t mask_value, uint8_t mask_care)
{
    if (mask_care == 0U) return true;
    for (uint8_t i = 0U; i < digits; ++i) {
        if ((mask_care & (uint8_t)(1U << i)) == 0U) continue;
        uint8_t shift = (uint8_t)((digits - 1U - i) * 4U);
        if (((address >> shift) & 0x0FU) != ((mask_value >> shift) & 0x0FU)) return false;
    }
    return true;
}

static bool line_matches_i2c_mask(const char *line)
{
    const char *cursor = line;
    if (memcmp(cursor, "[I2C]", 5U) == 0) cursor += 5;
    if (s_mask_care == 0U) return true;

    while (*cursor != '\0') {
        while (*cursor == ' ') ++cursor;
        if (*cursor == 'S') {
            ++cursor;
            continue;
        }
        if (*cursor == 'W' || *cursor == 'R') {
            ++cursor;
            uint8_t address = 0U;
            if (!parse_hex_byte_token(&cursor, &address)) return false;
            if (address_matches_mask(address, 2U, s_mask_value, s_mask_care)) return true;
            (void)parse_ack_token(&cursor);
            continue;
        }
        uint8_t data = 0U;
        if (!parse_hex_byte_token(&cursor, &data) || !parse_ack_token(&cursor)) return false;
    }
    return false;
}

static bool publish_rate_allowed(void)
{
    int64_t now = esp_timer_get_time();
    if (now - s_last_publish_us < I2C_I2S_PUBLISH_MIN_US) return false;
    s_last_publish_us = now;
    return true;
}

static bool known_i2c_address(uint8_t address)
{
    return address == 0x41U || address == 0x44U || address == 0x75U ||
           address == 0x60U || address == 0x6AU;
}

static bool validate_line(const char *line)
{
    const char *cursor = line;
    if (memcmp(cursor, "[I2C]", 5U) == 0) cursor += 5;

    bool expect_address = true;
    bool saw_address = false;
    bool saw_stop = false;
    bool malformed = false;
    bool unknown = false;

    while (*cursor != '\0') {
        while (*cursor == ' ') ++cursor;
        if (*cursor == '\0') break;

        if (*cursor == 'S') {
            ++cursor;
            expect_address = true;
            continue;
        }
        if (*cursor == 'P') {
            ++cursor;
            saw_stop = true;
            while (*cursor == ' ') ++cursor;
            if (*cursor != '\0') malformed = true;
            break;
        }
        if (*cursor == 'W' || *cursor == 'R') {
            ++cursor;
            uint8_t address = 0;
            if (!expect_address || !parse_hex_byte_token(&cursor, &address) || !parse_ack_token(&cursor)) {
                malformed = true;
                break;
            }
            saw_address = true;
            expect_address = false;
            if (!known_i2c_address(address)) unknown = true;
            continue;
        }

        uint8_t data = 0;
        if (expect_address || !parse_hex_byte_token(&cursor, &data) || !parse_ack_token(&cursor)) {
            malformed = true;
            break;
        }
    }

    portENTER_CRITICAL(&s_text_lock);
    if (malformed || !saw_address || !saw_stop) ++s_bad_lines;
    else if (unknown) ++s_unknown_addr_lines;
    else ++s_valid_lines;
    portEXIT_CRITICAL(&s_text_lock);

    return !malformed && saw_address && saw_stop;
}

static void publish_line(const char *line)
{
#if I2C_I2S_DEBUG_LOG
    static uint32_t debug_log_count;
#endif
    size_t length = strlen(line);
    if (!validate_line(line)) return;
    if (!line_matches_i2c_mask(line)) return;
    if (s_mask_care == 0U && !publish_rate_allowed()) return;
    store_preview(line);
#if I2C_I2S_DEBUG_LOG
    if (s_captures > 8000U && debug_log_count < 20U) {
        ++debug_log_count;
        ESP_LOGI(TAG, "I2C_SNIFF %s", line);
    }
#endif
    if (s_active_mode != APP_MODE_I2C || !bluetooth_spp_connected()) return;
    char bt_line[I2C_I2S_LINE_LEN + 2U];
    if (length > I2C_I2S_LINE_LEN) length = I2C_I2S_LINE_LEN;
    memcpy(bt_line, line, length);
    memcpy(bt_line + length, "\r\n", 2U);
    bluetooth_spp_write(bt_line, length + 2U);
}

static void decoder_reset_line(i2c_decode_context_t *ctx)
{
    memcpy(ctx->line, "[I2C]", 6U);
    ctx->used = 5U;
    ctx->line_has_byte = false;
    ctx->expect_address = true;
    ctx->bit_count = 0U;
    ctx->byte_value = 0U;
}

static void decoder_start(i2c_decode_context_t *ctx)
{
    if (!ctx->active) {
        decoder_reset_line(ctx);
        ctx->active = true;
    } else if (!append_text(ctx->line, sizeof(ctx->line), &ctx->used, " S")) {
        add_error(1U);
    }
    ctx->expect_address = true;
    ctx->bit_count = 0U;
    ctx->byte_value = 0U;
}

static void decoder_stop(i2c_decode_context_t *ctx)
{
    if (ctx->active && ctx->line_has_byte) {
        if (!append_text(ctx->line, sizeof(ctx->line), &ctx->used, " P")) add_error(1U);
        publish_line(ctx->line);
    }
    decoder_reset_line(ctx);
    ctx->active = false;
}

static void decoder_byte(i2c_decode_context_t *ctx, uint8_t byte_value, uint8_t ack_level)
{
    char ack = ack_level == 0U ? 'A' : 'N';
    ctx->line_has_byte = true;
    if (ctx->expect_address) {
        char rw = (byte_value & 0x01U) != 0U ? 'R' : 'W';
        uint8_t address = (uint8_t)(byte_value >> 1);
        const char *format = ctx->used == 5U ? "%c %02X %c" : " %c %02X %c";
        if (!append_text(ctx->line, sizeof(ctx->line), &ctx->used, format, rw, address, ack)) add_error(1U);
        ctx->expect_address = false;
    } else {
        if (!append_text(ctx->line, sizeof(ctx->line), &ctx->used, " %02X %c", byte_value, ack)) add_error(1U);
    }
}

static void decoder_sample(i2c_decode_context_t *ctx, uint8_t sample)
{
    uint8_t scl = sample & 0x01U;
    uint8_t sda = (sample >> 1U) & 0x01U;

    if (sda != ctx->previous_sda && scl != 0U) {
        if (ctx->previous_sda != 0U && sda == 0U) decoder_start(ctx);
        else if (ctx->previous_sda == 0U && sda != 0U) decoder_stop(ctx);
    }

    if (ctx->previous_scl == 0U && scl != 0U && ctx->active) {
        if (ctx->bit_count < 8U) {
            ctx->byte_value = (uint8_t)((ctx->byte_value << 1) | sda);
            ++ctx->bit_count;
        } else {
            decoder_byte(ctx, ctx->byte_value, sda);
            ctx->bit_count = 0U;
            ctx->byte_value = 0U;
        }
    }

    ctx->previous_scl = scl;
    ctx->previous_sda = sda;
}

static void decoder_filtered_sample(uint8_t sample)
{
    if (!s_sample_filter.initialized) {
        s_sample_filter.initialized = true;
        s_sample_filter.accepted = sample;
        s_sample_filter.candidate = sample;
        s_sample_filter.count = 1U;
        decoder_sample(&s_decoder, sample);
        return;
    }

    if (sample == s_sample_filter.candidate) {
        if (s_sample_filter.count < 255U) ++s_sample_filter.count;
    } else {
        s_sample_filter.candidate = sample;
        s_sample_filter.count = 1U;
    }

    if (s_sample_filter.candidate != s_sample_filter.accepted &&
        s_sample_filter.count >= I2C_I2S_MIN_STABLE_SAMPLES) {
        s_sample_filter.accepted = s_sample_filter.candidate;
        decoder_sample(&s_decoder, s_sample_filter.accepted);
    }
}

static void log_dma_layout(const uint8_t *buffer, size_t length)
{
    uint32_t counts[4][4] = {{0}};
    for (size_t i = 0U; i < length; ++i) {
        ++counts[i & 3U][buffer[i] & 0x03U];
    }

#if I2C_I2S_LAYOUT_DEBUG_CAPTURES > 0U
    if (s_layout_debug_count < I2C_I2S_LAYOUT_DEBUG_CAPTURES) {
        ESP_LOGI(TAG,
                 "layout cap=%lu len=%lu "
                 "m0=%lu/%lu/%lu/%lu m1=%lu/%lu/%lu/%lu "
                 "m2=%lu/%lu/%lu/%lu m3=%lu/%lu/%lu/%lu",
                 (unsigned long)s_layout_debug_count,
                 (unsigned long)length,
                 (unsigned long)counts[0][0],
                 (unsigned long)counts[0][1],
                 (unsigned long)counts[0][2],
                 (unsigned long)counts[0][3],
                 (unsigned long)counts[1][0],
                 (unsigned long)counts[1][1],
                 (unsigned long)counts[1][2],
                 (unsigned long)counts[1][3],
                 (unsigned long)counts[2][0],
                 (unsigned long)counts[2][1],
                 (unsigned long)counts[2][2],
                 (unsigned long)counts[2][3],
                 (unsigned long)counts[3][0],
                 (unsigned long)counts[3][1],
                 (unsigned long)counts[3][2],
                 (unsigned long)counts[3][3]);

        char low_word_order[97];
        char swapped_word_order[97];
        size_t low_used = 0U;
        size_t swapped_used = 0U;
        for (size_t i = 0U; i < length && low_used + 2U < sizeof(low_word_order) && i < 96U; i += 2U) {
            low_word_order[low_used++] = (char)('0' + (buffer[i] & 0x03U));
        }
        for (size_t i = 0U; i + 2U < length && swapped_used + 2U < sizeof(swapped_word_order) && i < 96U; i += 4U) {
            swapped_word_order[swapped_used++] = (char)('0' + (buffer[i + 2U] & 0x03U));
            swapped_word_order[swapped_used++] = (char)('0' + (buffer[i] & 0x03U));
        }
        low_word_order[low_used] = '\0';
        swapped_word_order[swapped_used] = '\0';
        ESP_LOGI(TAG, "layout low-word-order=%s", low_word_order);
        ESP_LOGI(TAG, "layout swapped-word-order=%s", swapped_word_order);
        ++s_layout_debug_count;
    }
#else
    (void)counts;
#endif

#if I2C_I2S_RAW_DEBUG_ACTIVE_CAPTURES > 0U
    uint32_t valid_counts[4] = {0};
    for (size_t i = 0U; i < length; i += 2U) {
        ++valid_counts[buffer[i] & 0x03U];
    }
    uint32_t active_samples = valid_counts[0] + valid_counts[1] + valid_counts[2];
    if (s_raw_debug_count >= I2C_I2S_RAW_DEBUG_ACTIVE_CAPTURES || active_samples < 80U) return;

    ESP_LOGI(TAG,
             "raw-active cap=%lu active=%lu valid=%lu/%lu/%lu/%lu",
             (unsigned long)s_raw_debug_count,
             (unsigned long)active_samples,
             (unsigned long)valid_counts[0],
             (unsigned long)valid_counts[1],
             (unsigned long)valid_counts[2],
             (unsigned long)valid_counts[3]);

    char line[101];
    size_t used = 0U;
    size_t offset = 0U;
    for (size_t i = 0U; i < length; i += 2U) {
        line[used++] = (char)('0' + (buffer[i] & 0x03U));
        if (used == 100U) {
            line[used] = '\0';
            ESP_LOGI(TAG, "raw%lu %s", (unsigned long)offset, line);
            offset += used;
            used = 0U;
        }
    }
    if (used != 0U) {
        line[used] = '\0';
        ESP_LOGI(TAG, "raw%lu %s", (unsigned long)offset, line);
    }
    ++s_raw_debug_count;
#endif
}

static void decoder_reset_stream(void)
{
    memset(&s_decoder, 0, sizeof(s_decoder));
    memset(&s_sample_filter, 0, sizeof(s_sample_filter));
    decoder_reset_line(&s_decoder);
    s_decoder.previous_scl = (uint8_t)gpio_get_level(I2C_SNIFFER_SCL_GPIO);
    s_decoder.previous_sda = (uint8_t)gpio_get_level(I2C_SNIFFER_SDA_GPIO);
}

static void parse_dma_buffer(const uint8_t *buffer, size_t length)
{
    log_dma_layout(buffer, length);
    for (size_t i = 0U; i < length; i += 2U) {
        decoder_filtered_sample((uint8_t)(buffer[i] & 0x03U));
    }
}

static void i2s_dma_reset(void)
{
    I2S0.lc_conf.in_rst = 1;
    I2S0.lc_conf.in_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;
    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;

    I2S0.conf.rx_reset = 1;
    I2S0.conf.rx_reset = 0;
    I2S0.conf.rx_fifo_reset = 1;
    I2S0.conf.rx_fifo_reset = 0;
    I2S0.conf.tx_reset = 1;
    I2S0.conf.tx_reset = 0;
    I2S0.conf.tx_fifo_reset = 1;
    I2S0.conf.tx_fifo_reset = 0;
    while (I2S0.state.rx_fifo_reset_back) {}
}

static void i2s_rx_ring_prepare(void)
{
    for (size_t i = 0U; i < I2C_I2S_DMA_DESC_COUNT; ++i) {
        s_dma_desc[i].length = I2C_I2S_DMA_DESC_BYTES;
        s_dma_desc[i].size = I2C_I2S_DMA_DESC_BYTES;
        s_dma_desc[i].owner = 1;
        s_dma_desc[i].sosf = 0;
        s_dma_desc[i].buf = s_dma_buffer + i * I2C_I2S_DMA_DESC_BYTES;
        s_dma_desc[i].offset = 0;
        s_dma_desc[i].empty = 0;
        s_dma_desc[i].eof = 1;
        s_dma_desc[i].qe.stqe_next =
            i == I2C_I2S_DMA_DESC_COUNT - 1U ? &s_dma_desc[0] : &s_dma_desc[i + 1U];
    }
}

static void i2s_rx_ring_start(void)
{
    if (s_rx_running) return;
    I2S0.conf.rx_start = 0;
    I2S0.in_link.stop = 1;
    I2S0.in_link.start = 0;
    i2s_dma_reset();
    memset(s_dma_buffer, 0, I2C_I2S_BUFFER_BYTES);
    i2s_rx_ring_prepare();
    xQueueReset(s_dma_ready_queue);
    s_overrun_pending = false;
    decoder_reset_stream();

    I2S0.rx_eof_num = I2C_I2S_DMA_DESC_BYTES / 4U;
    I2S0.in_link.addr = (uint32_t)&s_dma_desc[0];
    I2S0.in_link.stop = 0;
    I2S0.int_clr.val = I2S0.int_raw.val;
    I2S0.int_ena.val = 0;
    I2S0.int_ena.in_suc_eof = 1;
    I2S0.in_link.start = 1;
    I2S0.conf.rx_start = 1;
    s_rx_running = true;
}

static bool dma_buffer_is_valid(const uint8_t *buffer)
{
    return buffer >= s_dma_buffer && buffer < s_dma_buffer + I2C_I2S_BUFFER_BYTES &&
           (((uintptr_t)(buffer - s_dma_buffer) % I2C_I2S_DMA_DESC_BYTES) == 0U);
}

static void IRAM_ATTR i2s0_rx_isr(void *argument)
{
    (void)argument;
    uint32_t status = I2S0.int_st.val;
    I2S0.int_clr.val = status;
    if ((status & BIT(9)) == 0U) return; /* in_suc_eof */

    lldesc_t *finished = (lldesc_t *)I2S0.in_eof_des_addr;
    if (finished == NULL) return;

    const uint8_t *buffer = (const uint8_t *)finished->buf;

    BaseType_t need_yield = pdFALSE;
    if (xQueueSendFromISR(s_dma_ready_queue, &buffer, &need_yield) != pdTRUE) {
        const uint8_t *dropped = NULL;
        (void)xQueueReceiveFromISR(s_dma_ready_queue, &dropped, &need_yield);
        s_overrun_pending = true;
        ++s_overruns;
        (void)xQueueSendFromISR(s_dma_ready_queue, &buffer, &need_yield);
    }
    if (need_yield == pdTRUE) portYIELD_FROM_ISR();
}

static void i2s_rx_ring_stop(void)
{
    if (!s_rx_running) return;
    I2S0.conf.rx_start = 0;
    I2S0.in_link.stop = 1;
    I2S0.int_ena.in_suc_eof = 0;
    I2S0.int_clr.val = I2S0.int_raw.val;
    xQueueReset(s_dma_ready_queue);
    s_overrun_pending = false;
    s_rx_running = false;
}

static void i2c_i2s_task(void *argument)
{
    (void)argument;
    int64_t last_log_us = 0;
    while (true) {
        if (!I2C_I2S_DEBUG_ALWAYS && s_active_mode != APP_MODE_I2C) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        const uint8_t *buffer = NULL;
        if (xQueueReceive(s_dma_ready_queue, &buffer, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (s_overrun_pending) {
                s_overrun_pending = false;
                decoder_reset_stream();
                add_error(1U);
            }
            if (dma_buffer_is_valid(buffer)) {
                ++s_captures;
                parse_dma_buffer(buffer, I2C_I2S_DMA_DESC_BYTES);
            } else {
                add_error(1U);
            }
        } else {
            ++s_timeouts;
        }
        int64_t now = esp_timer_get_time();
        if (now - last_log_us >= 1000000LL) {
            last_log_us = now;
            if (s_active_mode == APP_MODE_I2C) {
                ESP_LOGI(TAG,
                         "stats captures=%lu timeouts=%lu overruns=%lu packets=%lu valid=%lu bad=%lu unknown=%lu errors=%lu mode=%d",
                         (unsigned long)s_captures,
                         (unsigned long)s_timeouts,
                         (unsigned long)s_overruns,
                         (unsigned long)s_packets,
                         (unsigned long)s_valid_lines,
                         (unsigned long)s_bad_lines,
                         (unsigned long)s_unknown_addr_lines,
                         (unsigned long)s_errors,
                         (int)s_active_mode);
            }
        }
    }
    i2s_rx_ring_stop();
}

static void gpio_matrix_input(gpio_num_t gpio, uint32_t signal)
{
    if (gpio == I2C_I2S_CLOCK_OUT_GPIO) {
        (void)gpio_input_enable(gpio);
        esp_rom_gpio_connect_in_signal(gpio, signal, false);
        return;
    }

    gpio_config_t config = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&config);
    esp_rom_gpio_connect_in_signal(gpio, signal, false);
}

static void gpio_matrix_output(gpio_num_t gpio, uint32_t signal)
{
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&config);
    esp_rom_gpio_connect_out_signal(gpio, signal, false, false);
    if (gpio == I2C_I2S_CLOCK_IN_GPIO) {
        (void)gpio_input_enable(gpio);
    }
}

static void i2s1_clock_dma_reset(void)
{
    I2S1.lc_conf.out_rst = 1;
    I2S1.lc_conf.out_rst = 0;
    I2S1.lc_conf.ahbm_rst = 1;
    I2S1.lc_conf.ahbm_rst = 0;
    I2S1.lc_conf.ahbm_fifo_rst = 1;
    I2S1.lc_conf.ahbm_fifo_rst = 0;

    I2S1.conf.tx_reset = 1;
    I2S1.conf.tx_reset = 0;
    I2S1.conf.tx_fifo_reset = 1;
    I2S1.conf.tx_fifo_reset = 0;
    while (I2S1.state.tx_fifo_reset_back) {}
}

static void i2s1_clock_output_setup(void)
{
    periph_module_enable(PERIPH_I2S1_MODULE);
    i2s1_clock_dma_reset();
    gpio_matrix_output(I2C_I2S_CLOCK_OUT_GPIO, I2S1O_BCK_OUT_IDX);

    memset(s_tx_clock_buffer, 0, I2C_I2S_TX_CLOCK_BUFFER_BYTES);
    s_tx_clock_dma_desc.length = I2C_I2S_TX_CLOCK_BUFFER_BYTES;
    s_tx_clock_dma_desc.size = I2C_I2S_TX_CLOCK_BUFFER_BYTES;
    s_tx_clock_dma_desc.owner = 1;
    s_tx_clock_dma_desc.sosf = 0;
    s_tx_clock_dma_desc.buf = s_tx_clock_buffer;
    s_tx_clock_dma_desc.offset = 0;
    s_tx_clock_dma_desc.empty = 0;
    s_tx_clock_dma_desc.eof = 1;
    s_tx_clock_dma_desc.qe.stqe_next = &s_tx_clock_dma_desc;

    I2S1.conf.val = 0;
    I2S1.conf.tx_slave_mod = 0;
    I2S1.conf.tx_mono = 1;
    I2S1.conf.tx_msb_shift = 0;
    I2S1.conf.tx_short_sync = 1;
    I2S1.conf2.val = 0;

    I2S1.clkm_conf.val = 0;
    I2S1.clkm_conf.clk_en = 1;
    I2S1.clkm_conf.clka_en = 0;
    I2S1.clkm_conf.clkm_div_a = 1;
    I2S1.clkm_conf.clkm_div_b = 0;
    I2S1.clkm_conf.clkm_div_num = I2C_I2S_CLKM_DIV_NUM;

    I2S1.sample_rate_conf.val = 0;
    I2S1.sample_rate_conf.tx_bits_mod = 16;
    I2S1.sample_rate_conf.tx_bck_div_num = 2;
    I2S1.fifo_conf.val = 0;
    I2S1.fifo_conf.dscr_en = 1;
    I2S1.fifo_conf.tx_fifo_mod = 1;
    I2S1.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S1.conf_chan.val = 0;
    I2S1.conf_chan.tx_chan_mod = 1;
    I2S1.timing.val = 0;

    I2S1.out_link.addr = (uint32_t)&s_tx_clock_dma_desc;
    I2S1.out_link.start = 1;
    I2S1.int_clr.val = I2S1.int_raw.val;
    I2S1.conf.tx_start = 1;
    ESP_LOGI(TAG, "I2S1 BCK clock on GPIO%d at %u Hz", I2C_I2S_CLOCK_OUT_GPIO, (unsigned)I2C_I2S_SAMPLE_HZ);
}

static void i2s0_parallel_rx_setup(void)
{
    periph_module_enable(PERIPH_I2S0_MODULE);
    i2s_dma_reset();

    gpio_matrix_input(I2C_I2S_CLOCK_IN_GPIO, I2S0I_BCK_IN_IDX);
    gpio_matrix_input(I2C_I2S_CLOCK_IN_GPIO, I2S0I_WS_IN_IDX);
    gpio_matrix_input(I2C_SNIFFER_SCL_GPIO, I2S0I_DATA_IN0_IDX);
    gpio_matrix_input(I2C_SNIFFER_SDA_GPIO, I2S0I_DATA_IN1_IDX);
    for (uint32_t signal = I2S0I_DATA_IN2_IDX; signal <= I2S0I_DATA_IN15_IDX; ++signal) {
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, signal, false);
    }
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, I2S0I_V_SYNC_IDX, false);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, I2S0I_H_SYNC_IDX, false);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, I2S0I_H_ENABLE_IDX, false);

    I2S0.conf.val = 0;
    I2S0.conf.rx_slave_mod = 1;
    I2S0.conf2.val = 0;
    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;

    I2S0.clkm_conf.val = 0;
    I2S0.clkm_conf.clk_en = 1;
    I2S0.clkm_conf.clka_en = 0;
    I2S0.clkm_conf.clkm_div_a = 1;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = I2C_I2S_CLKM_DIV_NUM;

    I2S0.fifo_conf.val = 0;
    I2S0.fifo_conf.dscr_en = 1;
    I2S0.fifo_conf.rx_fifo_mod = 1;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.conf_chan.val = 0;
    I2S0.conf_chan.rx_chan_mod = 1;

    I2S0.sample_rate_conf.val = 0;
    I2S0.sample_rate_conf.rx_bits_mod = 8;
    I2S0.sample_rate_conf.rx_bck_div_num = 1;
    I2S0.conf.rx_right_first = 1;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 1;
    I2S0.conf.rx_short_sync = 1;
    I2S0.timing.val = 0;
}

esp_err_t i2c_i2s_sniffer_init(void)
{
#if I2C_SNIFFER_ENABLED
#if !I2C_I2S_ENABLE_EXPERIMENT
    return ESP_ERR_NOT_SUPPORTED;
#endif
    s_dma_buffer = heap_caps_aligned_calloc(4, I2C_I2S_BUFFER_BYTES, 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (s_dma_buffer == NULL) return ESP_ERR_NO_MEM;
    s_tx_clock_buffer = heap_caps_aligned_calloc(4, I2C_I2S_TX_CLOCK_BUFFER_BYTES, 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (s_tx_clock_buffer == NULL) return ESP_ERR_NO_MEM;

    s_dma_ready_queue = xQueueCreate(I2C_I2S_DMA_DESC_COUNT, sizeof(const uint8_t *));
    if (s_dma_ready_queue == NULL) return ESP_ERR_NO_MEM;

    i2s1_clock_output_setup();
    i2s0_parallel_rx_setup();
    ESP_RETURN_ON_ERROR(esp_intr_alloc(ETS_I2S0_INTR_SOURCE,
                                       ESP_INTR_FLAG_LEVEL1,
                                       i2s0_rx_isr,
                                       NULL,
                                       &s_i2s0_intr),
                        TAG,
                        "alloc I2S0 interrupt");
    BaseType_t ok = xTaskCreate(i2c_i2s_task, "i2c_i2s", 4096, NULL, 5, &s_task);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
#else
    return ESP_OK;
#endif
}

void i2c_i2s_sniffer_configure(app_mode_t mode, uint16_t mask_value, uint8_t mask_care)
{
    s_mask_value = mask_value;
    s_mask_care = mask_care;
    if (mode != s_active_mode) {
        bool was_i2c = s_active_mode == APP_MODE_I2C;
        bool now_i2c = mode == APP_MODE_I2C;
        if (was_i2c && !now_i2c) {
            i2s_rx_ring_stop();
            clear_preview();
        } else if (!was_i2c && now_i2c) {
            decoder_reset_stream();
            clear_preview();
            i2s_rx_ring_start();
        }
        s_active_mode = mode;
    }
}

void i2c_i2s_sniffer_update(app_state_t *state)
{
    if (state == NULL) return;
    portENTER_CRITICAL(&s_text_lock);
    memcpy(state->i2c_sniffer.text, s_text, sizeof(state->i2c_sniffer.text));
    memcpy(state->i2c_sniffer.lines, s_lines, sizeof(state->i2c_sniffer.lines));
    state->i2c_sniffer.packets = s_packets;
    state->i2c_sniffer.errors = s_errors;
    portEXIT_CRITICAL(&s_text_lock);
}
