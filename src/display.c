#include "display.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "probe_config.h"
#include "smooth_font.h"
#include "splash_bitmap.h"

#define RGB565(r, g, b) (uint16_t)((((r) & 0xF8U) << 8) | (((g) & 0xFCU) << 3) | ((b) >> 3))
#define COLOR_BLACK RGB565(0, 0, 0)
#define COLOR_WHITE RGB565(255, 255, 255)
#define COLOR_GREEN RGB565(40, 230, 90)
#define COLOR_RED RGB565(255, 45, 35)
#define COLOR_BLUE RGB565(30, 80, 255)
#define COLOR_YELLOW RGB565(255, 210, 0)
#define COLOR_CYAN RGB565(0, 210, 255)
#define COLOR_ORANGE RGB565(255, 120, 0)
#define COLOR_GRAY RGB565(70, 75, 85)
#define COLOR_BADGE_CV_FG COLOR_WHITE
#define COLOR_BADGE_CV_BG RGB565(18, 18, 18)
#define COLOR_BADGE_CC_FG COLOR_BLACK
#define COLOR_BADGE_CC_BG RGB565(0, 210, 220)
#define COLOR_ALERT_RED_FG COLOR_BLACK
#define COLOR_ALERT_RED_BG RGB565(0, 210, 220)
#define TFT_BACKLIGHT_LEDC_MODE LEDC_HIGH_SPEED_MODE
#define TFT_BACKLIGHT_LEDC_TIMER LEDC_TIMER_3
#define TFT_BACKLIGHT_LEDC_CHANNEL LEDC_CHANNEL_7
#define TFT_BACKLIGHT_LEDC_RESOLUTION LEDC_TIMER_10_BIT
#define TFT_BACKLIGHT_LEDC_HZ 20000U
#define TFT_BACKLIGHT_DUTY_MAX ((1U << 10U) - 1U)
#define TFT_BACKLIGHT_FADE_MS 800U
#define TFT_BACKLIGHT_FADE_STEPS 32U
#define TFT_SPLASH_VISIBLE_MS 2000U

static spi_device_handle_t s_tft;
static const char *TAG = "display";

/*
 * Text is composed in RAM before it is sent to the TFT.  Besides allowing
 * four-bit antialiasing, this is important for a measurement instrument:
 * clearing and drawing a line in separate SPI operations made the old 5x7
 * font visibly blink.  A 320 x 40 buffer costs 25.6 kB but each completed
 * line now reaches the display as one continuous image.
 */
static uint16_t s_text_buffer[TFT_WIDTH * 40];
static uint8_t s_fill_block[4096];

static uint32_t backlight_physical_duty(uint32_t logical_duty)
{
    if (logical_duty > TFT_BACKLIGHT_DUTY_MAX) logical_duty = TFT_BACKLIGHT_DUTY_MAX;
    return TFT_BACKLIGHT_ACTIVE_LEVEL ? logical_duty : (TFT_BACKLIGHT_DUTY_MAX - logical_duty);
}

static esp_err_t backlight_pwm_set(uint32_t logical_duty)
{
    esp_err_t err = ledc_set_duty(TFT_BACKLIGHT_LEDC_MODE,
                                  TFT_BACKLIGHT_LEDC_CHANNEL,
                                  backlight_physical_duty(logical_duty));
    if (err != ESP_OK) return err;
    return ledc_update_duty(TFT_BACKLIGHT_LEDC_MODE, TFT_BACKLIGHT_LEDC_CHANNEL);
}

static esp_err_t backlight_pwm_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = TFT_BACKLIGHT_LEDC_MODE,
        .timer_num = TFT_BACKLIGHT_LEDC_TIMER,
        .duty_resolution = TFT_BACKLIGHT_LEDC_RESOLUTION,
        .freq_hz = TFT_BACKLIGHT_LEDC_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "backlight timer");

    ledc_channel_config_t channel = {
        .gpio_num = TFT_BACKLIGHT_GPIO,
        .speed_mode = TFT_BACKLIGHT_LEDC_MODE,
        .channel = TFT_BACKLIGHT_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = TFT_BACKLIGHT_LEDC_TIMER,
        .duty = backlight_physical_duty(0U),
        .hpoint = 0,
        .flags = {
            .output_invert = 0,
        },
    };
    return ledc_channel_config(&channel);
}

static void backlight_fade_in(void)
{
    const TickType_t step_delay = pdMS_TO_TICKS(TFT_BACKLIGHT_FADE_MS / TFT_BACKLIGHT_FADE_STEPS);
    for (uint32_t step = 0U; step <= TFT_BACKLIGHT_FADE_STEPS; ++step) {
        uint32_t duty = (TFT_BACKLIGHT_DUTY_MAX * step) / TFT_BACKLIGHT_FADE_STEPS;
        esp_err_t err = backlight_pwm_set(duty);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "backlight duty: %s", esp_err_to_name(err));
            return;
        }
        if (step < TFT_BACKLIGHT_FADE_STEPS) vTaskDelay(step_delay);
    }
}

static void log_pin_levels(const char *stage)
{
    ESP_LOGI(TAG, "%s pins: SCL=%d SDA=%d RES=%d DC=%d CS=%d BLK=%d",
             stage,
             gpio_get_level(TFT_SCLK_GPIO),
             gpio_get_level(TFT_MOSI_GPIO),
             gpio_get_level(TFT_RESET_GPIO),
             gpio_get_level(TFT_DC_GPIO),
             gpio_get_level(TFT_CS_GPIO),
             gpio_get_level(TFT_BACKLIGHT_GPIO));
}

/* send
 * Inputs: data selects command/data mode; bytes/length point to SPI payload.
 * Returns: none.
 * Does: sends one blocking SPI transaction to the TFT.
 */
static void send(bool data, const void *bytes, size_t length)
{
    if (s_tft == NULL) return;
    gpio_set_level(TFT_DC_GPIO, data ? 1 : 0);
    spi_transaction_t transaction = {
        .length = length * 8U,
        .tx_buffer = bytes
    };
    esp_err_t err = spi_device_polling_transmit(s_tft, &transaction);
    if (err != ESP_OK) ESP_LOGE(TAG, "spi tx: %s", esp_err_to_name(err));
}

/* command
 * Inputs: value is one ST7789 command byte.
 * Returns: none.
 * Does: sends a display command byte without parameters.
 */
static void command(uint8_t value)
{
    send(false, &value, 1U);
}

/* command_data
 * Inputs: cmd is the command byte; data/length are optional parameters.
 * Returns: none.
 * Does: sends one ST7789 command followed by its data bytes.
 */
static void command_data(uint8_t cmd, const uint8_t *data, size_t length)
{
    command(cmd);
    if (length != 0U) send(true, data, length);
}

/* set_window
 * Inputs: x/y/width/height describe the rectangular TFT write area.
 * Returns: none.
 * Does: programs the ST7789 column/page address window.
 */
static void set_window(int x, int y, int width, int height)
{
    x += TFT_X_OFFSET;
    y += TFT_Y_OFFSET;
    uint16_t x2 = (uint16_t)(x + width - 1);
    uint16_t y2 = (uint16_t)(y + height - 1);
    uint8_t cols[] = {(uint8_t)(x >> 8), (uint8_t)x, (uint8_t)(x2 >> 8), (uint8_t)x2};
    uint8_t rows[] = {(uint8_t)(y >> 8), (uint8_t)y, (uint8_t)(y2 >> 8), (uint8_t)y2};
    command_data(0x2A, cols, sizeof(cols));
    command_data(0x2B, rows, sizeof(rows));
    command(0x2C);
}

/* fill_rect
 * Inputs: rectangle coordinates and RGB565 color.
 * Returns: none.
 * Does: fills a rectangular TFT area with one solid color.
 */
static void fill_rect(int x, int y, int width, int height, uint16_t color)
{
    if (s_tft == NULL || width <= 0 || height <= 0) return;
    for (size_t i = 0; i < sizeof(s_fill_block); i += 2U) {
        s_fill_block[i] = (uint8_t)(color >> 8);
        s_fill_block[i + 1U] = (uint8_t)color;
    }
    set_window(x, y, width, height);
    size_t remaining = (size_t)width * (size_t)height * 2U;
    gpio_set_level(TFT_DC_GPIO, 1);
    while (remaining != 0U) {
        size_t amount = remaining > sizeof(s_fill_block) ? sizeof(s_fill_block) : remaining;
        spi_transaction_t transaction = {.length = amount * 8U, .tx_buffer = s_fill_block};
        esp_err_t err = spi_device_polling_transmit(s_tft, &transaction);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "fill tx: %s", esp_err_to_name(err));
            return;
        }
        remaining -= amount;
    }
}

/* draw_splash_bitmap
 * Inputs: none.
 * Returns: none.
 * Does: sends the pre-rendered 320x170 RGB565 startup splash to the TFT.
 */
static void draw_splash_bitmap(void)
{
    if (s_tft == NULL) return;
    if (SPLASH_BITMAP_WIDTH != TFT_WIDTH || SPLASH_BITMAP_HEIGHT != TFT_HEIGHT) {
        ESP_LOGE(TAG, "splash size mismatch: %ux%u for TFT %dx%d",
                 (unsigned)SPLASH_BITMAP_WIDTH,
                 (unsigned)SPLASH_BITMAP_HEIGHT,
                 TFT_WIDTH,
                 TFT_HEIGHT);
        return;
    }

    set_window(0, 0, TFT_WIDTH, TFT_HEIGHT);
    gpio_set_level(TFT_DC_GPIO, 1);
    const uint8_t *bytes = splash_bitmap_rgb565;
    size_t remaining = splash_bitmap_rgb565_size;
    while (remaining != 0U) {
        size_t amount = remaining > sizeof(s_fill_block) ? sizeof(s_fill_block) : remaining;
        for (size_t i = 0U; i < amount; i += 2U) {
            uint16_t pixel = (uint16_t)((uint16_t)bytes[i] << 8) | bytes[i + 1U];
            pixel ^= 0xFFFFU;
            s_fill_block[i] = (uint8_t)(pixel >> 8);
            s_fill_block[i + 1U] = (uint8_t)pixel;
        }
        spi_transaction_t transaction = {.length = amount * 8U, .tx_buffer = s_fill_block};
        esp_err_t err = spi_device_polling_transmit(s_tft, &transaction);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "splash tx: %s", esp_err_to_name(err));
            return;
        }
        bytes += amount;
        remaining -= amount;
    }
}

/* blend_on_black
 * Inputs: RGB565 color and 4-bit alpha value.
 * Returns: byte-swapped RGB565 pixel blended over black.
 * Does: prepares antialiased font pixels for direct TFT transfer.
 */
static uint16_t blend_on_black(uint16_t color, uint8_t alpha)
{
    uint16_t red = (uint16_t)(((color >> 11) & 0x1FU) * alpha / 15U);
    uint16_t green = (uint16_t)(((color >> 5) & 0x3FU) * alpha / 15U);
    uint16_t blue = (uint16_t)((color & 0x1FU) * alpha / 15U);
    uint16_t pixel = (uint16_t)((red << 11) | (green << 5) | blue);
    /* ST7789 receives the high byte first, while ESP32 RAM is little-endian. */
    return (uint16_t)((pixel << 8) | (pixel >> 8));
}

/* blend_rgb565
 * Inputs: foreground and background RGB565 colors, and 4-bit alpha.
 * Returns: byte-swapped RGB565 pixel blended over the requested background.
 * Does: prepares antialiased font pixels when a highlighted background is used.
 */
static uint16_t blend_rgb565(uint16_t foreground, uint16_t background, uint8_t alpha)
{
    uint16_t fr = (foreground >> 11) & 0x1FU;
    uint16_t fg = (foreground >> 5) & 0x3FU;
    uint16_t fb = foreground & 0x1FU;
    uint16_t br = (background >> 11) & 0x1FU;
    uint16_t bg = (background >> 5) & 0x3FU;
    uint16_t bb = background & 0x1FU;

    uint16_t red = (uint16_t)((fr * alpha + br * (15U - alpha)) / 15U);
    uint16_t green = (uint16_t)((fg * alpha + bg * (15U - alpha)) / 15U);
    uint16_t blue = (uint16_t)((fb * alpha + bb * (15U - alpha)) / 15U);
    uint16_t pixel = (uint16_t)((red << 11) | (green << 5) | blue);
    return (uint16_t)((pixel << 8) | (pixel >> 8));
}

/* send_text_buffer
 * Inputs: y is the top screen coordinate; height is the buffered line height.
 * Returns: none.
 * Does: sends the prepared full-width text buffer to the TFT.
 */
static void send_text_buffer(int y, int height)
{
    if (s_tft == NULL) return;
    set_window(0, y, TFT_WIDTH, height);
    gpio_set_level(TFT_DC_GPIO, 1);
    const uint8_t *bytes = (const uint8_t *)s_text_buffer;
    size_t remaining = (size_t)TFT_WIDTH * (size_t)height * 2U;
    while (remaining != 0U) {
        size_t amount = remaining > 4096U ? 4096U : remaining;
        spi_transaction_t transaction = {.length = amount * 8U, .tx_buffer = bytes};
        esp_err_t err = spi_device_polling_transmit(s_tft, &transaction);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "text tx: %s", esp_err_to_name(err));
            return;
        }
        bytes += amount;
        remaining -= amount;
    }
}

/* draw_text
 * Inputs: x/y position, zero-terminated text, font pointer and RGB565 color.
 * Returns: none.
 * Does: renders one antialiased text line into RAM and transfers it as one
 * image to avoid flicker.
 */
static void draw_text(int x, int y, const char *text, const smooth_font_t *font, uint16_t color)
{
    int height = font->line_height;
    if (height > 40) height = 40;
    memset(s_text_buffer, 0, (size_t)TFT_WIDTH * (size_t)height * sizeof(uint16_t));

    int cursor = x;
    while (*text != '\0' && cursor < TFT_WIDTH) {
        uint8_t code = (uint8_t)*text++;
        if (code < font->first_char || code > font->last_char) code = (uint8_t)'?';
        const smooth_glyph_t *g = &font->glyphs[code - font->first_char];
        unsigned pixel_index = 0U;
        for (int row = 0; row < g->height; ++row) {
            int target_y = row + g->y_offset;
            for (int column = 0; column < g->width; ++column, ++pixel_index) {
                int target_x = cursor + g->x_offset + column;
                uint8_t packed = font->bitmap[g->bitmap_offset + pixel_index / 2U];
                uint8_t alpha = (pixel_index & 1U) == 0U ? packed >> 4 : packed & 0x0FU;
                if (alpha != 0U && target_x >= 0 && target_x < TFT_WIDTH &&
                    target_y >= 0 && target_y < height)
                    s_text_buffer[target_y * TFT_WIDTH + target_x] = blend_on_black(color, alpha);
            }
        }
        cursor += g->advance;
    }
    send_text_buffer(y, height);
}

/* draw_rich_text
 * Inputs:
 *   x/y   - text position;
 *   text  - zero-terminated ASCII string;
 *   font  - font used for all characters;
 *   fg/bg - per-character foreground and background colors.
 * Returns: none.
 * Does: draws one full-width line with optional per-character highlighting.
 */
static void draw_rich_text(int x,
                           int y,
                           const char *text,
                           const smooth_font_t *font,
                           const uint16_t *fg,
                           const uint16_t *bg)
{
    int height = font->line_height;
    if (height > 40) height = 40;
    memset(s_text_buffer, 0, (size_t)TFT_WIDTH * (size_t)height * sizeof(uint16_t));

    int cursor = x;
    size_t char_index = 0U;
    while (text[char_index] != '\0' && cursor < TFT_WIDTH) {
        uint8_t code = (uint8_t)text[char_index];
        if (code < font->first_char || code > font->last_char) code = (uint8_t)'?';
        const smooth_glyph_t *g = &font->glyphs[code - font->first_char];
        uint16_t foreground = fg[char_index];
        uint16_t background = bg[char_index];

        if (background != COLOR_BLACK) {
            int left = cursor;
            int right = cursor + g->advance;
            if (left < 0) left = 0;
            if (right > TFT_WIDTH) right = TFT_WIDTH;
            for (int row = 0; row < height; ++row) {
                for (int column = left; column < right; ++column) {
                    s_text_buffer[row * TFT_WIDTH + column] =
                        (uint16_t)((background << 8) | (background >> 8));
                }
            }
        }

        unsigned pixel_index = 0U;
        for (int row = 0; row < g->height; ++row) {
            int target_y = row + g->y_offset;
            for (int column = 0; column < g->width; ++column, ++pixel_index) {
                int target_x = cursor + g->x_offset + column;
                uint8_t packed = font->bitmap[g->bitmap_offset + pixel_index / 2U];
                uint8_t alpha = (pixel_index & 1U) == 0U ? packed >> 4 : packed & 0x0FU;
                if (alpha != 0U && target_x >= 0 && target_x < TFT_WIDTH &&
                    target_y >= 0 && target_y < height) {
                    s_text_buffer[target_y * TFT_WIDTH + target_x] =
                        blend_rgb565(foreground, background, alpha);
                }
            }
        }

        cursor += g->advance;
        ++char_index;
    }

    send_text_buffer(y, height);
}

/* mark_range
 * Inputs: color arrays, start/length and colors to apply.
 * Returns: none.
 * Does: marks a substring as active on the settings display line.
 */
static void mark_range(uint16_t *fg,
                       uint16_t *bg,
                       size_t start,
                       size_t length,
                       uint16_t foreground,
                       uint16_t background)
{
    for (size_t i = start; i < start + length; ++i) {
        fg[i] = foreground;
        bg[i] = background;
    }
}

/* fill_text_colors
 * Inputs: per-character foreground/background arrays.
 * Returns: none.
 * Does: initializes a whole rich-text line to the default UI colors.
 */
static void fill_text_colors(uint16_t *fg, uint16_t *bg, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        fg[i] = COLOR_WHITE;
        bg[i] = COLOR_BLACK;
    }
}

/* append_colored
 * Inputs:
 *   line/fg/bg/pos - rich text buffer and current write position;
 *   text           - zero-terminated text to append;
 *   color          - foreground color for appended characters.
 * Returns: none.
 * Does: appends one colored text fragment to a rich-text line.
 */
static void append_colored(char *line,
                           uint16_t *fg,
                           uint16_t *bg,
                           size_t *pos,
                           const char *text,
                           uint16_t color)
{
    while (*text != '\0' && *pos < 63U) {
        line[*pos] = *text++;
        fg[*pos] = color;
        bg[*pos] = COLOR_BLACK;
        ++(*pos);
    }
    line[*pos] = '\0';
}

/* append_format_colored
 * Inputs: rich text buffer, color and printf-style format.
 * Returns: none.
 * Does: formats a short fragment and appends it with one color.
 */
static void append_format_colored(char *line,
                                  uint16_t *fg,
                                  uint16_t *bg,
                                  size_t *pos,
                                  uint16_t color,
                                  const char *format,
                                  ...)
{
    char fragment[24];
    va_list args;
    va_start(args, format);
    vsnprintf(fragment, sizeof(fragment), format, args);
    va_end(args);
    append_colored(line, fg, bg, pos, fragment, color);
}

/* draw_channel_actual_line
 * Inputs: screen row, channel letter and measured INA channel.
 * Returns: none.
 * Does: draws the large "actual output" line: channel, voltage and current.
 */
static uint8_t measured_current_decimals(char channel)
{
    return channel == 'A' ? 5U : 4U;
}

static int64_t measured_current_ua(char channel, const app_ina238_channel_t *measurement)
{
    if (!measurement->valid || measurement->shunt_uohm == 0U) return 0;
    int64_t current_ua = ((int64_t)measurement->shunt_uv * 1000000LL) /
                         (int64_t)measurement->shunt_uohm;
    if (channel == 'A') {
        current_ua -= ((int64_t)measurement->bus_mv * 420LL + 5000LL) / 10000LL;
    }
    return current_ua;
}

static void format_measured_current(char *out,
                                    size_t size,
                                    char channel,
                                    const app_ina238_channel_t *measurement)
{
    int64_t current_ua = measured_current_ua(channel, measurement);
    uint8_t decimals = measured_current_decimals(channel);
    int64_t scale = 1;
    for (uint8_t i = 0; i < decimals; ++i) scale *= 10LL;

    int sign = current_ua < 0 ? -1 : 1;
    int64_t abs_ua = current_ua < 0 ? -current_ua : current_ua;
    int64_t scaled = (abs_ua * scale + 500000LL) / 1000000LL;
    int64_t whole = scaled / scale;
    int64_t frac = scaled % scale;

    if (decimals == 4U) {
        snprintf(out, size, "%s%lld.%04lld",
                 sign < 0 ? "-" : "", (long long)whole, (long long)frac);
    } else if (decimals == 6U) {
        snprintf(out, size, "%s%lld.%06lld",
                 sign < 0 ? "-" : "", (long long)whole, (long long)frac);
    } else {
        snprintf(out, size, "%s%lld.%05lld",
                 sign < 0 ? "-" : "", (long long)whole, (long long)frac);
    }
}

static int64_t channel_power_mw(char channel, const app_ina238_channel_t *measurement)
{
    if (!measurement->valid) return 0;
    int64_t current_ua = measured_current_ua(channel, measurement);
    int64_t power_nw = (int64_t)measurement->bus_mv * current_ua;
    if (power_nw < 0) return -((-power_nw + 500000LL) / 1000000LL);
    return (power_nw + 500000LL) / 1000000LL;
}

static void append_cv_cc_badge(char *line, uint16_t *fg, uint16_t *bg, size_t *pos, bool cc_active)
{
    size_t badge_start;
    uint16_t badge_fg = cc_active ? COLOR_BADGE_CC_FG : COLOR_BADGE_CV_FG;
    uint16_t badge_bg = cc_active ? COLOR_BADGE_CC_BG : COLOR_BADGE_CV_BG;

    append_colored(line, fg, bg, pos, " ", COLOR_GRAY);
    badge_start = *pos;
    append_colored(line, fg, bg, pos, cc_active ? "CC" : "CV", badge_fg);
    mark_range(fg,
               bg,
               badge_start,
               2U,
               badge_fg,
               badge_bg);
}

static void draw_channel_actual_line(int y,
                                     char channel,
                                     const app_ina238_channel_t *measurement,
                                     bool cc_active)
{
    char line[64] = "";
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;

    fill_text_colors(fg, bg, 64U);
    append_format_colored(line, fg, bg, &pos, COLOR_ORANGE, "%c ", channel);

    if (measurement->valid) {
        append_format_colored(line, fg, bg, &pos, COLOR_WHITE,
                              "%.2f", (double)measurement->bus_mv / 1000.0);
    } else {
        append_colored(line, fg, bg, &pos, "--.--", COLOR_GRAY);
    }
    append_colored(line, fg, bg, &pos, "V ", COLOR_GREEN);

    if (measurement->valid) {
        char current[32];
        format_measured_current(current, sizeof(current), channel, measurement);
        append_colored(line, fg, bg, &pos, current, COLOR_WHITE);
    } else {
        append_colored(line, fg, bg, &pos, "--.--", COLOR_GRAY);
    }
    append_colored(line, fg, bg, &pos, "A", COLOR_CYAN);
    append_cv_cc_badge(line, fg, bg, &pos, cc_active);

    draw_rich_text(2, y, line, &roboto_30, fg, bg);
}

static void format_voltage(char *out, size_t size, uint16_t mv);
static void format_current(char *out, size_t size, uint16_t ma);
static size_t editable_digit_offset(const char *value, uint8_t digit);

/* draw_channel_set_line
 * Inputs:
 *   y - screen row;
 *   target_mv/target_ma - channel setpoints;
 *   measurement - INA data for temperature;
 *   voltage_select/current_select - control indices owned by this channel;
 *   selected/digit - active editor position.
 * Returns: none.
 * Does: draws requested values directly under the channel. The selected
 * voltage/current and selected decimal digit are highlighted here, not in a
 * separate bottom editor.
 */
static void draw_channel_set_line(int y,
                                  uint16_t target_mv,
                                  uint16_t target_ma,
                                  const app_ina238_channel_t *measurement,
                                  uint8_t voltage_select,
                                  uint8_t current_select,
                                  uint8_t selected,
                                  uint8_t digit)
{
    char line[64] = "";
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;
    char voltage[8];
    char current[8];
    char temperature[8];
    size_t voltage_start;
    size_t current_start;

    fill_text_colors(fg, bg, 64U);
    append_colored(line, fg, bg, &pos, "SET ", COLOR_GRAY);
    voltage_start = pos;
    format_voltage(voltage, sizeof(voltage), target_mv);
    append_colored(line, fg, bg, &pos, voltage, COLOR_YELLOW);
    append_colored(line, fg, bg, &pos, "V  ", COLOR_GREEN);

    append_colored(line, fg, bg, &pos, "LIM ", COLOR_GRAY);
    current_start = pos;
    format_current(current, sizeof(current), target_ma);
    append_colored(line, fg, bg, &pos, current, COLOR_YELLOW);
    append_colored(line, fg, bg, &pos, "A  ", COLOR_CYAN);

    if (measurement->valid) {
        size_t temperature_start = pos;
        snprintf(temperature,
                 sizeof(temperature),
                 "%.0fC",
                 (double)measurement->temperature_mc / 1000.0);
        append_colored(line, fg, bg, &pos, temperature, COLOR_GRAY);
        if (measurement->temperature_mc > 45000) {
            size_t digit_count = strlen(temperature);
            if (digit_count != 0U && temperature[digit_count - 1U] == 'C') --digit_count;
            mark_range(fg,
                       bg,
                       temperature_start,
                       digit_count,
                       COLOR_ALERT_RED_FG,
                       COLOR_ALERT_RED_BG);
        }
    } else {
        append_colored(line, fg, bg, &pos, "--C", COLOR_GRAY);
    }

    if (selected == voltage_select) {
        if (digit == CONTROL_DIGIT_WHOLE) {
            mark_range(fg, bg, voltage_start, strlen(voltage), COLOR_BLACK, COLOR_YELLOW);
        } else {
            mark_range(fg, bg, voltage_start, strlen(voltage), COLOR_YELLOW, COLOR_BLACK);
            mark_range(fg, bg, voltage_start + editable_digit_offset(voltage, digit),
                       1U, COLOR_BLACK, COLOR_YELLOW);
        }
    } else if (selected == current_select) {
        (void)digit;
        mark_range(fg, bg, current_start, strlen(current), COLOR_BLACK, COLOR_YELLOW);
    }

    draw_rich_text(2, y, line, &roboto_18, fg, bg);
}

static void draw_compact_actual_line(int y, char channel, const app_ina238_channel_t *measurement)
{
    char line[64] = "";
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;

    fill_text_colors(fg, bg, 64U);
    append_format_colored(line, fg, bg, &pos, COLOR_ORANGE, "%c ", channel);

    if (measurement->valid) {
        append_format_colored(line, fg, bg, &pos, COLOR_WHITE,
                              "%.2f", (double)measurement->bus_mv / 1000.0);
    } else {
        append_colored(line, fg, bg, &pos, "--.--", COLOR_GRAY);
    }
    append_colored(line, fg, bg, &pos, "V  ", COLOR_GREEN);

    if (measurement->valid) {
        char current[32];
        format_measured_current(current, sizeof(current), channel, measurement);
        append_colored(line, fg, bg, &pos, current, COLOR_WHITE);
    } else {
        append_colored(line, fg, bg, &pos, "--.--", COLOR_GRAY);
    }
    append_colored(line, fg, bg, &pos, "A", COLOR_CYAN);

    draw_rich_text(2, y, line, &roboto_18, fg, bg);
}

static void draw_bottom_actuals(const app_state_t *s)
{
    draw_compact_actual_line(126, 'A', &s->ina238.channel[0]);
    draw_compact_actual_line(148, 'B', &s->ina238.channel[1]);
}

static void draw_highlighted_value_line(int y,
                                        const char *label,
                                        const char *value,
                                        const char *suffix,
                                        uint8_t this_select,
                                        const app_state_t *s)
{
    char line[64] = "";
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;
    size_t value_start;

    fill_text_colors(fg, bg, 64U);
    append_colored(line, fg, bg, &pos, label, COLOR_GRAY);
    append_colored(line, fg, bg, &pos, " ", COLOR_GRAY);
    value_start = pos;
    append_colored(line, fg, bg, &pos, value, COLOR_YELLOW);
    if (suffix != NULL && suffix[0] != '\0') {
        append_colored(line, fg, bg, &pos, " ", COLOR_GRAY);
        append_colored(line, fg, bg, &pos, suffix, COLOR_GREEN);
    }

    if (s->control.selected_value == this_select) {
        if (s->control.selected_digit == CONTROL_DIGIT_WHOLE) {
            mark_range(fg, bg, value_start, strlen(value), COLOR_BLACK, COLOR_YELLOW);
        } else {
            mark_range(fg, bg,
                       value_start + editable_digit_offset(value, s->control.selected_digit),
                       1U,
                       COLOR_BLACK,
                       COLOR_YELLOW);
        }
    }

    draw_rich_text(8, y, line, &roboto_30, fg, bg);
}

static void draw_generator_duty_line(const app_state_t *s)
{
    char line[64] = "";
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;
    size_t duty_start;
    size_t on_start;
    char duty[8];
    const char *on_value = s->control.generator_on ? "ON" : "OFF";

    fill_text_colors(fg, bg, 64U);
    append_colored(line, fg, bg, &pos, "DUTY ", COLOR_GRAY);
    duty_start = pos;
    snprintf(duty, sizeof(duty), "%02u", (unsigned)s->control.generator_duty_percent);
    append_colored(line, fg, bg, &pos, duty, COLOR_YELLOW);
    append_colored(line, fg, bg, &pos, " %  ON ", COLOR_GREEN);
    on_start = pos;
    append_colored(line, fg, bg, &pos, on_value, COLOR_YELLOW);

    if (s->control.selected_value == CONTROL_SELECT_GEN_DUTY) {
        if (s->control.selected_digit == CONTROL_DIGIT_WHOLE) {
            mark_range(fg, bg, duty_start, strlen(duty), COLOR_BLACK, COLOR_YELLOW);
        } else {
            mark_range(fg, bg,
                       duty_start + editable_digit_offset(duty, s->control.selected_digit),
                       1U,
                       COLOR_BLACK,
                       COLOR_YELLOW);
        }
    } else if (s->control.selected_value == CONTROL_SELECT_GEN_ON) {
        mark_range(fg, bg, on_start, strlen(on_value), COLOR_BLACK, COLOR_YELLOW);
    }

    draw_rich_text(8, 68, line, &roboto_30, fg, bg);
}

static void draw_generator_screen(const app_state_t *s)
{
    char value[16];

    draw_text(8, 4, "GENERATOR", &roboto_18, COLOR_CYAN);

    snprintf(value, sizeof(value), "%06lu", (unsigned long)s->control.generator_freq_hz);
    draw_highlighted_value_line(30, "FREQ", value, "HZ", CONTROL_SELECT_GEN_FREQ, s);

    draw_generator_duty_line(s);

    draw_bottom_actuals(s);
}

static void draw_serial_header(const app_state_t *s, const char *title, uint32_t rate, uint8_t select)
{
    char line[64] = "";
    char value[16];
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;
    size_t baud_start;

    fill_text_colors(fg, bg, 64U);
    append_colored(line, fg, bg, &pos, title, COLOR_CYAN);
    append_colored(line, fg, bg, &pos, "  BAUD ", COLOR_CYAN);
    baud_start = pos;
    snprintf(value, sizeof(value), "%lu", (unsigned long)rate);
    append_colored(line, fg, bg, &pos, value, COLOR_YELLOW);

    if (s->control.selected_value == select) {
        mark_range(fg, bg, baud_start, strlen(value), COLOR_BLACK, COLOR_YELLOW);
    }

    draw_rich_text(8, 4, line, &roboto_18, fg, bg);
}

static void draw_uart_rx_line(int y, const char *text)
{
    draw_text(8, y, text[0] != '\0' ? text : "-", &roboto_30,
              text[0] != '\0' ? COLOR_WHITE : COLOR_GRAY);
}

static void draw_serial_screen(const app_state_t *s, const char *title, uint32_t rate, uint8_t select)
{
    draw_serial_header(s, title, rate, select);
    draw_uart_rx_line(28, s->uart.lines[0]);
    draw_uart_rx_line(60, s->uart.lines[1]);
    draw_uart_rx_line(88, s->uart.lines[2]);
    draw_bottom_actuals(s);
}

static void draw_i2c_sniffer_screen(const app_state_t *s)
{
    draw_text(8, 4, "I2C SNIFF", &roboto_18, COLOR_CYAN);
    draw_uart_rx_line(28, s->i2c_sniffer.lines[0]);
    draw_uart_rx_line(60, s->i2c_sniffer.lines[1]);
    draw_uart_rx_line(88, s->i2c_sniffer.lines[2]);
    draw_bottom_actuals(s);
}

static void draw_can_screen(const app_state_t *s)
{
    char value[16];
    uint16_t fg[64];
    uint16_t bg[64];

    draw_text(8, 4, "CAN", &roboto_18, COLOR_CYAN);
    snprintf(value, sizeof(value), "%lu", (unsigned long)(s->control.can_bitrate / 1000U));
    draw_highlighted_value_line(30, "BAUD", value, "KBIT", CONTROL_SELECT_CAN_BITRATE, s);

    fill_text_colors(fg, bg, 64U);
    draw_rich_text(8, 74, "ID: ---", &roboto_18, fg, bg);
    draw_rich_text(8, 98, "DATA: --", &roboto_18, fg, bg);

    draw_bottom_actuals(s);
}

static void draw_setting_line(int y,
                              const char *label,
                              const char *value,
                              uint8_t select,
                              const app_state_t *s)
{
    char line[64] = "";
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;
    size_t value_start;

    fill_text_colors(fg, bg, 64U);
    append_colored(line, fg, bg, &pos, label, COLOR_GRAY);
    append_colored(line, fg, bg, &pos, " ", COLOR_GRAY);
    value_start = pos;
    append_colored(line, fg, bg, &pos, value, COLOR_YELLOW);

    if (s->control.selected_value == select) {
        if (s->control.selected_digit == CONTROL_DIGIT_WHOLE ||
            select == CONTROL_SELECT_OVERCURRENT) {
            mark_range(fg, bg, value_start, strlen(value), COLOR_BLACK, COLOR_YELLOW);
        } else {
            mark_range(fg,
                       bg,
                       value_start + editable_digit_offset(value, s->control.selected_digit),
                       1U,
                       COLOR_BLACK,
                       COLOR_YELLOW);
        }
    }

    draw_rich_text(8, y, line, &roboto_18, fg, bg);
}

static void draw_setting_screen(const app_state_t *s)
{
    char value[8];

    draw_text(8, 4, "SETTING", &roboto_18, COLOR_CYAN);

    draw_setting_line(30,
                      "Overcurrent",
                      s->control.overcurrent_cc ? "CC" : "OFF",
                      CONTROL_SELECT_OVERCURRENT,
                      s);
    snprintf(value, sizeof(value), "%02u", (unsigned)s->control.overheat_c);
    draw_setting_line(58, "Overheat", value, CONTROL_SELECT_OVERHEAT, s);
    snprintf(value, sizeof(value), "%03u", (unsigned)s->control.overpower_w);
    draw_setting_line(86, "Overpower", value, CONTROL_SELECT_OVERPOWER, s);
    snprintf(value, sizeof(value), "%03u", (unsigned)s->control.volume_percent);
    draw_setting_line(114, "Volume", value, CONTROL_SELECT_VOLUME, s);
}

static void draw_reserved_screen(const app_state_t *s, const char *title)
{
    draw_text(8, 4, title, &roboto_18, COLOR_CYAN);
    draw_text(8, 58, "RESERVED", &roboto_30, COLOR_GRAY);
    draw_bottom_actuals(s);
}

static const char *mode_menu_name(app_mode_t mode)
{
    switch (mode) {
        case APP_MODE_POWER_SUPPLY: return "POWER SOURCE";
        case APP_MODE_GENERATOR:    return "GENERATOR";
        case APP_MODE_UART:         return "UART";
        case APP_MODE_1WIRE:        return "1WIRE";
        case APP_MODE_RS485:        return "RS485";
        case APP_MODE_CAN:          return "CAN";
        case APP_MODE_I2C:          return "I2C";
        case APP_MODE_SETTING:      return "SETTING";
        default:                    return "?";
    }
}

static void draw_mode_menu_scroll_indicator(uint8_t selected, uint8_t visible_rows)
{
    const uint8_t total_rows = (uint8_t)APP_MODE_COUNT;
    const int track_x = 5;
    const int track_y = 7;
    const int track_width = 3;
    const int track_height = 156;
    const int thumb_height = 24;

    fill_rect(0, 0, 15, TFT_HEIGHT, COLOR_BLACK);
    if (total_rows <= visible_rows) return;

    uint8_t max_selected = (uint8_t)(total_rows - 1U);
    int travel = track_height - thumb_height;
    int thumb_y = track_y;
    if (max_selected != 0U) {
        thumb_y += (travel * (int)selected + (int)max_selected / 2) / (int)max_selected;
    }

    fill_rect(track_x, track_y, track_width, track_height, COLOR_GRAY);
    fill_rect(track_x - 1, thumb_y, track_width + 2, thumb_height, COLOR_CYAN);
}

static void draw_mode_menu(const app_state_t *s, bool force)
{
    const uint8_t visible_rows = 7U;
    const int row_height = 23;
    static char previous_lines[7][64];
    static uint8_t previous_top = 0xFFU;
    static uint8_t previous_selected = 0xFFU;
    static app_mode_t previous_active = APP_MODE_COUNT;
    uint8_t top = 0U;

    if ((uint8_t)APP_MODE_COUNT > visible_rows) {
        if (s->control.menu_index >= visible_rows) {
            top = (uint8_t)(s->control.menu_index - visible_rows + 1U);
        }
    }

    if (force) {
        fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);
        for (uint8_t i = 0U; i < visible_rows; ++i) previous_lines[i][0] = '\0';
        previous_top = 0xFFU;
        previous_selected = 0xFFU;
        previous_active = APP_MODE_COUNT;
    }

    for (uint8_t row = 0U; row < visible_rows; ++row) {
        uint8_t mode_index = (uint8_t)(top + row);
        if (mode_index >= (uint8_t)APP_MODE_COUNT) break;

        char line[64];
        uint16_t fg[64];
        uint16_t bg[64];
        size_t length;
        bool selected = mode_index == s->control.menu_index;
        bool active = mode_index == (uint8_t)s->control.mode;

        snprintf(line, sizeof(line), "%c %s", active ? '*' : ' ', mode_menu_name((app_mode_t)mode_index));
        length = strlen(line);
        fill_text_colors(fg, bg, 64U);

        for (size_t i = 0U; i < length; ++i) {
            fg[i] = active ? COLOR_GREEN : COLOR_WHITE;
            bg[i] = COLOR_BLACK;
        }
        if (selected) {
            mark_range(fg, bg, 0U, length, COLOR_BLACK, COLOR_YELLOW);
        }

        bool selection_changed_here = mode_index == s->control.menu_index ||
                                      mode_index == previous_selected;
        bool active_changed_here = mode_index == (uint8_t)s->control.mode ||
                                   mode_index == (uint8_t)previous_active;

        if (force ||
            top != previous_top ||
            selection_changed_here ||
            active_changed_here ||
            strcmp(previous_lines[row], line) != 0) {
            draw_rich_text(18, 5 + (int)row * row_height, line, &roboto_18, fg, bg);
            snprintf(previous_lines[row], sizeof(previous_lines[row]), "%s", line);
        }
    }

    draw_mode_menu_scroll_indicator(s->control.menu_index, visible_rows);

    previous_top = top;
    previous_selected = s->control.menu_index;
    previous_active = s->control.mode;
}

/* draw_status_line
 * Inputs: app state.
 * Returns: none.
 * Does: draws the bottom power summary line for the power-supply screen.
 */
static void draw_status_line(int y, const app_state_t *s)
{
    char line[64] = "";
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;
    uint32_t input_mv = (s->analog.adc_mv * 19U + 1U) / 2U;
    uint32_t input_tenths = (input_mv + 50U) / 100U;
    int64_t power_mw = channel_power_mw('A', &s->ina238.channel[0]) +
                       channel_power_mw('B', &s->ina238.channel[1]);
    int64_t abs_power_mw = power_mw < 0 ? -power_mw : power_mw;
    int64_t power_tenths = (abs_power_mw + 50LL) / 100LL;

    fill_text_colors(fg, bg, 64U);

    append_format_colored(line, fg, bg, &pos, COLOR_GRAY,
                          "In %02lu.%01luV, ",
                          (unsigned long)(input_tenths / 10U),
                          (unsigned long)(input_tenths % 10U));

    append_colored(line, fg, bg, &pos, "A ", COLOR_ORANGE);
    if (s->tps55289.status_valid) {
        append_format_colored(line, fg, bg, &pos, COLOR_GRAY, "0x%02X, ", s->tps55289.status);
    } else {
        append_colored(line, fg, bg, &pos, "0x--, ", COLOR_RED);
    }

    append_colored(line, fg, bg, &pos, "B ", COLOR_ORANGE);
    if (s->lm51772.status_valid) {
        append_format_colored(line, fg, bg, &pos, COLOR_GRAY, "0x%02X, ", s->lm51772.status);
    } else {
        append_colored(line, fg, bg, &pos, "0x--, ", COLOR_RED);
    }

    append_colored(line, fg, bg, &pos, "P ", COLOR_GRAY);
    append_format_colored(line, fg, bg, &pos, COLOR_YELLOW,
                          "%s%lld.%01lldW",
                          power_mw < 0 ? "-" : "",
                          (long long)(power_tenths / 10LL),
                          (long long)(power_tenths % 10LL));

    draw_rich_text(2, y, line, &roboto_18, fg, bg);
}

/* format_voltage
 * Inputs: destination buffer and voltage in millivolts.
 * Returns: none.
 * Does: writes a fixed xx.xx voltage string.
 */
static void format_voltage(char *out, size_t size, uint16_t mv)
{
    snprintf(out, size, "%02u.%02u", (unsigned)(mv / 1000U), (unsigned)((mv % 1000U) / 10U));
}

/* format_current
 * Inputs: destination buffer and current in milliamps.
 * Returns: none.
 * Does: writes a fixed x.xx current string.
 */
static void format_current(char *out, size_t size, uint16_t ma)
{
    snprintf(out, size, "%u.%02u", (unsigned)(ma / 1000U), (unsigned)((ma % 1000U) / 10U));
}

/* editable_digit_offset
 * Inputs: formatted numeric text and editable digit number.
 * Returns: character offset of the editable digit.
 * Does: skips the decimal point in fixed xx.xx/x.xx strings.
 */
static size_t editable_digit_offset(const char *value, uint8_t digit)
{
    size_t seen_digits = 0U;
    for (size_t i = 0U; value[i] != '\0'; ++i) {
        if (value[i] >= '0' && value[i] <= '9') {
            if (seen_digits == digit) return i;
            ++seen_digits;
        }
    }
    return 0U;
}

/* display_init
 * Inputs: none.
 * Returns: ESP_OK on success, or an ESP-IDF SPI/GPIO setup error.
 * Does: configures TFT pins, initializes ST7789 and draws the startup UI.
 */
esp_err_t display_init(void)
{
    ESP_LOGI(TAG, "init start: ST7789 %dx%d offset=%d,%d spi SCL=%d SDA=%d RES=%d DC=%d CS=%d BLK=%d active=%d",
             TFT_WIDTH,
             TFT_HEIGHT,
             TFT_X_OFFSET,
             TFT_Y_OFFSET,
             TFT_SCLK_GPIO,
             TFT_MOSI_GPIO,
             TFT_RESET_GPIO,
             TFT_DC_GPIO,
             TFT_CS_GPIO,
             TFT_BACKLIGHT_GPIO,
             TFT_BACKLIGHT_ACTIVE_LEVEL);
    gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << TFT_DC_GPIO) | (1ULL << TFT_RESET_GPIO) |
                        (1ULL << TFT_CS_GPIO) | (1ULL << TFT_SCLK_GPIO) |
                        (1ULL << TFT_MOSI_GPIO) | (1ULL << TFT_BACKLIGHT_GPIO),
        .mode = GPIO_MODE_OUTPUT
    };
    ESP_ERROR_CHECK(gpio_config(&outputs));
    gpio_set_level(TFT_CS_GPIO, 1);
    gpio_set_level(TFT_DC_GPIO, 1);
    gpio_set_level(TFT_SCLK_GPIO, 0);
    gpio_set_level(TFT_MOSI_GPIO, 0);
    gpio_set_level(TFT_RESET_GPIO, 1);
    gpio_set_level(TFT_BACKLIGHT_GPIO, !TFT_BACKLIGHT_ACTIVE_LEVEL);
    ESP_RETURN_ON_ERROR(backlight_pwm_init(), TAG, "backlight init");
    log_pin_levels("after gpio defaults");
#if RGB_LED_ENABLED
    gpio_set_level(RGB_RED_GPIO, !RGB_ACTIVE_LEVEL);
    gpio_set_level(RGB_GREEN_GPIO, !RGB_ACTIVE_LEVEL);
    gpio_set_level(RGB_BLUE_GPIO, !RGB_ACTIVE_LEVEL);
#endif

    spi_bus_config_t bus = {
        .mosi_io_num = TFT_MOSI_GPIO,
        .miso_io_num = TFT_MISO_GPIO,
        .sclk_io_num = TFT_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi bus init: %s", esp_err_to_name(err));
        log_pin_levels("after spi bus fail");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "spi bus ok");
    spi_device_interface_config_t device = {
        .clock_speed_hz = 10000000,
        .mode = 0,
        .spics_io_num = TFT_CS_GPIO,
        .queue_size = 1
    };
    err = spi_bus_add_device(SPI2_HOST, &device, &s_tft);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi device add: %s", esp_err_to_name(err));
        log_pin_levels("after spi device fail");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "spi device ok");

    gpio_set_level(TFT_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(TFT_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    log_pin_levels("after reset pulse");

    ESP_LOGI(TAG, "cmd SWRESET");
    command(0x01);
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_LOGI(TAG, "cmd SLPOUT");
    command(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_LOGI(TAG, "cmd COLMOD/MADCTL/INVOFF/NORON/DISPON");
    const uint8_t format[] = {0x55}; command_data(0x3A, format, sizeof(format));
    const uint8_t madctl[] = {0xA0}; command_data(0x36, madctl, sizeof(madctl));
    command(0x20);
    command(0x13);
    command(0x29);
    log_pin_levels("after display on");

    ESP_LOGI(TAG, "draw startup splash");
    draw_splash_bitmap();
    backlight_fade_in();
    vTaskDelay(pdMS_TO_TICKS(TFT_SPLASH_VISIBLE_MS - TFT_BACKLIGHT_FADE_MS));
    ESP_LOGI(TAG, "init done");
    return ESP_OK;
}

/* display_black
 * Inputs: none.
 * Returns: none.
 * Does: clears the entire TFT to black.
 */
void display_black (void){
    fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);

}

/* display_set_rgb
 * Inputs: probe logic state.
 * Returns: none.
 * Does: drives the active-low onboard RGB LED according to logic state.
 */
void display_set_rgb(probe_logic_state_t state)
{
#if RGB_LED_ENABLED
    bool red = false, green = false, blue = false;
    switch (state) {
        case PROBE_LOW: green = true; break;
        case PROBE_HIGH: red = true; break;
        case PROBE_OPEN: break;
        case PROBE_OVERVOLTAGE: red = true; break;
        default: red = true; green = true; break;
    }
    gpio_set_level(RGB_RED_GPIO, red ? RGB_ACTIVE_LEVEL : !RGB_ACTIVE_LEVEL);
    gpio_set_level(RGB_GREEN_GPIO, green ? RGB_ACTIVE_LEVEL : !RGB_ACTIVE_LEVEL);
    gpio_set_level(RGB_BLUE_GPIO, blue ? RGB_ACTIVE_LEVEL : !RGB_ACTIVE_LEVEL);
#else
    (void)state;
#endif
}

/* display_render
 * Inputs: s points to the complete measurement snapshot to show.
 * Returns: none.
 * Does: updates only changed display lines and RGB LED state.
 */
void display_render(const app_state_t *s)
{
    static char previous_ch_a[64] = "";
    static char previous_ch_b[64] = "";
    static char previous_limits[64] = "";
    static char previous_mode_screen[256] = "";
    static app_mode_t previous_mode = APP_MODE_COUNT;
    static bool previous_menu_open;
    static uint8_t previous_menu_index = 0xFFU;
    static bool psu_screen_initialized;
    char line[64];
    char mode_line[256];

    if (previous_menu_open != s->control.menu_open) {
        fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);
        previous_ch_a[0] = '\0';
        previous_ch_b[0] = '\0';
        previous_limits[0] = '\0';
        previous_mode_screen[0] = '\0';
        psu_screen_initialized = false;
        previous_mode = APP_MODE_COUNT;
        previous_menu_index = 0xFFU;
        previous_menu_open = s->control.menu_open;
    }

    if (s->control.menu_open) {
        if (previous_menu_index != s->control.menu_index ||
            previous_mode != s->control.mode) {
            draw_mode_menu(s, previous_menu_index == 0xFFU);
            previous_menu_index = s->control.menu_index;
            previous_mode = s->control.mode;
        }
        display_set_rgb(s->analog.logic_state);
        return;
    }

    if (previous_mode != s->control.mode) {
        fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);
        previous_ch_a[0] = '\0';
        previous_ch_b[0] = '\0';
        previous_limits[0] = '\0';
        previous_mode_screen[0] = '\0';
        psu_screen_initialized = false;
        previous_mode = s->control.mode;
    }

    if (s->control.mode != APP_MODE_POWER_SUPPLY) {
        snprintf(previous_ch_a, sizeof(previous_ch_a), "%s", "");
        snprintf(previous_ch_b, sizeof(previous_ch_b), "%s", "");
        snprintf(previous_limits, sizeof(previous_limits), "%s", "");

        snprintf(mode_line, sizeof(mode_line), "M%u S%u D%u F%lu P%u O%u U%lu R%lu C%lu OC%u OH%u OP%u V%u T%s|%s|%s UE%lu I%s|%s|%s IP%lu IE%lu A%u%lu%ld B%u%lu%ld",
                 (unsigned)s->control.mode,
                 (unsigned)s->control.selected_value,
                 (unsigned)s->control.selected_digit,
                 (unsigned long)s->control.generator_freq_hz,
                 (unsigned)s->control.generator_duty_percent,
                 s->control.generator_on ? 1U : 0U,
                 (unsigned long)s->control.uart_baud,
                 (unsigned long)s->control.rs485_baud,
                 (unsigned long)s->control.can_bitrate,
                 s->control.overcurrent_cc ? 1U : 0U,
                 (unsigned)s->control.overheat_c,
                 (unsigned)s->control.overpower_w,
                 (unsigned)s->control.volume_percent,
                 (s->control.mode == APP_MODE_UART || s->control.mode == APP_MODE_RS485) ? s->uart.lines[0] : "",
                 (s->control.mode == APP_MODE_UART || s->control.mode == APP_MODE_RS485) ? s->uart.lines[1] : "",
                 (s->control.mode == APP_MODE_UART || s->control.mode == APP_MODE_RS485) ? s->uart.lines[2] : "",
                 (unsigned long)s->uart.errors,
                 s->control.mode == APP_MODE_I2C ? s->i2c_sniffer.lines[0] : "",
                 s->control.mode == APP_MODE_I2C ? s->i2c_sniffer.lines[1] : "",
                 s->control.mode == APP_MODE_I2C ? s->i2c_sniffer.lines[2] : "",
                 (unsigned long)s->i2c_sniffer.packets,
                 (unsigned long)s->i2c_sniffer.errors,
                 s->ina238.channel[0].valid ? 1U : 0U,
                 (unsigned long)s->ina238.channel[0].bus_mv,
                 (long)s->ina238.channel[0].shunt_uv,
                 s->ina238.channel[1].valid ? 1U : 0U,
                 (unsigned long)s->ina238.channel[1].bus_mv,
                 (long)s->ina238.channel[1].shunt_uv);
        if (strcmp(previous_mode_screen, mode_line) != 0) {
            switch (s->control.mode) {
                case APP_MODE_GENERATOR:
                    draw_generator_screen(s);
                    break;
                case APP_MODE_UART:
                    draw_serial_screen(s, "UART", s->control.uart_baud, CONTROL_SELECT_UART_BAUD);
                    break;
                case APP_MODE_1WIRE:
                    draw_reserved_screen(s, "1WIRE");
                    break;
                case APP_MODE_RS485:
                    draw_serial_screen(s, "RS485", s->control.rs485_baud, CONTROL_SELECT_RS485_BAUD);
                    break;
                case APP_MODE_CAN:
                    draw_can_screen(s);
                    break;
                case APP_MODE_I2C:
                    draw_i2c_sniffer_screen(s);
                    break;
                case APP_MODE_SETTING:
                    draw_setting_screen(s);
                    break;
                default:
                    break;
            }
            memcpy(previous_mode_screen, mode_line, sizeof(previous_mode_screen));
        }
        display_set_rgb(s->analog.logic_state);
        return;
    }

    if (!psu_screen_initialized) {
        fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);
        previous_ch_a[0] = '\0';
        previous_ch_b[0] = '\0';
        previous_limits[0] = '\0';
        psu_screen_initialized = true;
    }

    snprintf(line, sizeof(line), "IN%lu A%u%02X B%u%02X PA%u%lu%ld PB%u%lu%ld",
             (unsigned long)s->analog.adc_mv,
             s->tps55289.status_valid ? 1U : 0U,
             s->tps55289.status,
             s->lm51772.status_valid ? 1U : 0U,
             s->lm51772.status,
             s->ina238.channel[0].valid ? 1U : 0U,
             (unsigned long)s->ina238.channel[0].bus_mv,
             (long)s->ina238.channel[0].shunt_uv,
             s->ina238.channel[1].valid ? 1U : 0U,
             (unsigned long)s->ina238.channel[1].bus_mv,
             (long)s->ina238.channel[1].shunt_uv);
    if (strcmp(previous_limits, line) != 0) {
        draw_status_line(146, s);
        snprintf(previous_limits, sizeof(previous_limits), "%s", line);
    }

    const app_ina238_channel_t *ina = &s->ina238.channel[0];
    if (ina->valid) {
        snprintf(line, sizeof(line), "A%lu%ld%u%u%u%ld%u%u%u",
                 (unsigned long)ina->bus_mv,
                 (long)ina->shunt_uv,
                 (unsigned)ina->wide_range,
                 (unsigned)s->control.u2_mv,
                 (unsigned)s->control.i2_ma,
                 (long)ina->temperature_mc,
                 (unsigned)s->control.selected_value,
                 (unsigned)s->control.selected_digit,
                 s->tps55289.current_limit_active ? 1U : 0U);
    } else {
        snprintf(line, sizeof(line), "A--%u%u%u%u%u",
                 (unsigned)s->control.u2_mv,
                 (unsigned)s->control.i2_ma,
                 (unsigned)s->control.selected_value,
                 (unsigned)s->control.selected_digit,
                 s->tps55289.current_limit_active ? 1U : 0U);
    }
    if (strcmp(previous_ch_a, line) != 0) {
        draw_channel_actual_line(2, 'A', ina, s->tps55289.current_limit_active);
        draw_channel_set_line(40, s->control.u2_mv, s->control.i2_ma, ina,
                              2U, 3U,
                              s->control.selected_value,
                              s->control.selected_digit);
        snprintf(previous_ch_a, sizeof(previous_ch_a), "%s", line);
    }

    ina = &s->ina238.channel[1];
    if (ina->valid) {
        snprintf(line, sizeof(line), "B%lu%ld%u%u%u%ld%u%u%u",
                 (unsigned long)ina->bus_mv,
                 (long)ina->shunt_uv,
                 (unsigned)ina->wide_range,
                 (unsigned)s->control.u1_mv,
                 (unsigned)s->control.i1_ma,
                 (long)ina->temperature_mc,
                 (unsigned)s->control.selected_value,
                 (unsigned)s->control.selected_digit,
                 s->lm51772.current_limit_active ? 1U : 0U);
    } else {
        snprintf(line, sizeof(line), "B--%u%u%u%u%u",
                 (unsigned)s->control.u1_mv,
                 (unsigned)s->control.i1_ma,
                 (unsigned)s->control.selected_value,
                 (unsigned)s->control.selected_digit,
                 s->lm51772.current_limit_active ? 1U : 0U);
    }
    if (strcmp(previous_ch_b, line) != 0) {
        draw_channel_actual_line(72, 'B', ina, s->lm51772.current_limit_active);
        draw_channel_set_line(110, s->control.u1_mv, s->control.i1_ma, ina,
                              0U, 1U,
                              s->control.selected_value,
                              s->control.selected_digit);
        snprintf(previous_ch_b, sizeof(previous_ch_b), "%s", line);
    }

    display_set_rgb(s->analog.logic_state);
}
