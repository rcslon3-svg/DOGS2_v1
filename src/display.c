#include "display.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
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
#define COLOR_PURPLE RGB565(169, 92, 255)
#define COLOR_ORANGE RGB565(255, 120, 0)
#define COLOR_GRAY RGB565(155, 165, 180)
#define COLOR_LABEL RGB565(205, 215, 230)
#define COLOR_PANEL RGB565(9, 12, 18)
#define COLOR_BADGE_CV_FG COLOR_WHITE
#define COLOR_BADGE_CV_BG RGB565(18, 18, 18)
#define COLOR_BADGE_CC_FG COLOR_BLACK
#define COLOR_BADGE_CC_BG RGB565(0, 210, 220)
#define COLOR_ALERT_RED_FG COLOR_WHITE
#define COLOR_ALERT_RED_BG COLOR_RED
#define POWER_CHANNEL_FRAME_X 6
#define POWER_CHANNEL_FRAME_WIDTH 238
#define POWER_CHANNEL_INNER_X 8
#define POWER_CHANNEL_INNER_RIGHT 242
#define POWER_CURRENT_RIGHT 238
#define POWER_SIDE_FRAME_X 248
#define POWER_SIDE_FRAME_WIDTH 66
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

/* Match the pixel polarity already used by the working splash bitmap. */
static uint16_t display_pixel(uint16_t logical_color)
{
    uint16_t wire_color = (uint16_t)(logical_color ^ 0xFFFFU);
    return (uint16_t)((wire_color << 8) | (wire_color >> 8));
}

static void clear_text_buffer(int height, uint16_t logical_color)
{
    uint16_t pixel = display_pixel(logical_color);
    size_t count = (size_t)TFT_WIDTH * (size_t)height;
    for (size_t i = 0U; i < count; ++i) s_text_buffer[i] = pixel;
}

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
    uint16_t wire_color = (uint16_t)(color ^ 0xFFFFU);
    for (size_t i = 0; i < sizeof(s_fill_block); i += 2U) {
        s_fill_block[i] = (uint8_t)(wire_color >> 8);
        s_fill_block[i + 1U] = (uint8_t)wire_color;
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

/* fill_round_rect
 * Inputs: rectangle, small corner radius and RGB565 color.
 * Returns: none.
 * Does: builds a compact rounded panel from horizontal spans; intended for the
 * small radii used by the 320x170 instrument UI.
 */
static void fill_round_rect(int x, int y, int width, int height, int radius, uint16_t color)
{
    if (radius <= 0 || width <= radius * 2 || height <= radius * 2) {
        fill_rect(x, y, width, height, color);
        return;
    }

    fill_rect(x, y + radius, width, height - radius * 2, color);
    for (int row = 0; row < radius; ++row) {
        int inset = radius - row;
        fill_rect(x + inset, y + row, width - inset * 2, 1, color);
        fill_rect(x + inset, y + height - 1 - row, width - inset * 2, 1, color);
    }
}

/* draw_panel_outline
 * Draws a continuous two-pixel panel outline with slightly clipped corners.
 */
static void draw_panel_outline(int x, int y, int width, int height, uint16_t color)
{
    const int thickness = 2;
    const int corner = 3;

    fill_rect(x + corner, y, width - corner * 2, thickness, color);
    fill_rect(x + corner, y + height - thickness,
              width - corner * 2, thickness, color);
    fill_rect(x, y + corner, thickness, height - corner * 2, color);
    fill_rect(x + width - thickness, y + corner,
              thickness, height - corner * 2, color);
    fill_rect(x + 1, y + 2, 2, 1, color);
    fill_rect(x + width - 3, y + 2, 2, 1, color);
    fill_rect(x + 1, y + height - 3, 2, 1, color);
    fill_rect(x + width - 3, y + height - 3, 2, 1, color);
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
    return display_pixel(pixel);
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
    return display_pixel(pixel);
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

static void send_text_buffer_region(int x, int y, int width, int height)
{
    if (s_tft == NULL || width <= 0 || height <= 0) return;
    set_window(x, y, width, height);
    gpio_set_level(TFT_DC_GPIO, 1);
    for (int row = 0; row < height; ++row) {
        const uint8_t *bytes = (const uint8_t *)&s_text_buffer[row * TFT_WIDTH + x];
        spi_transaction_t transaction = {
            .length = (size_t)width * 2U * 8U,
            .tx_buffer = bytes
        };
        esp_err_t err = spi_device_polling_transmit(s_tft, &transaction);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "text region tx: %s", esp_err_to_name(err));
            return;
        }
    }
}

static void send_text_buffer_region_slice(int x, int y, int width,
                                          int first_row, int row_count)
{
    if (s_tft == NULL || width <= 0 || row_count <= 0) return;
    set_window(x, y, width, row_count);
    gpio_set_level(TFT_DC_GPIO, 1);
    for (int row = 0; row < row_count; ++row) {
        const uint8_t *bytes = (const uint8_t *)&s_text_buffer[
            (first_row + row) * TFT_WIDTH + x];
        spi_transaction_t transaction = {
            .length = (size_t)width * 2U * 8U,
            .tx_buffer = bytes
        };
        esp_err_t err = spi_device_polling_transmit(s_tft, &transaction);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "text slice tx: %s", esp_err_to_name(err));
            return;
        }
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
    clear_text_buffer(height, COLOR_BLACK);

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
static int compose_rich_text_on_panel(int x,
                                      const char *text,
                                      const smooth_font_t *font,
                                      const uint16_t *fg,
                                      const uint16_t *bg,
                                      uint16_t panel_background,
                                      int panel_left,
                                      int panel_right)
{
    int height = font->line_height;
    if (height > 40) height = 40;
    clear_text_buffer(height, COLOR_BLACK);

    if (panel_background != COLOR_BLACK) {
        uint16_t swapped = display_pixel(panel_background);
        if (panel_left < 0) panel_left = 0;
        if (panel_right > TFT_WIDTH) panel_right = TFT_WIDTH;
        for (int row = 0; row < height; ++row) {
            for (int column = panel_left; column < panel_right; ++column) {
                s_text_buffer[row * TFT_WIDTH + column] = swapped;
            }
        }
    }

    int cursor = x;
    size_t char_index = 0U;
    while (text[char_index] != '\0' && cursor < TFT_WIDTH) {
        uint8_t code = (uint8_t)text[char_index];
        if (code < font->first_char || code > font->last_char) code = (uint8_t)'?';
        const smooth_glyph_t *g = &font->glyphs[code - font->first_char];
        uint16_t foreground = fg[char_index];
        uint16_t background = bg[char_index] == COLOR_BLACK ? panel_background : bg[char_index];

        if (background != COLOR_BLACK) {
            int left = cursor;
            int right = cursor + g->advance;
            if (left < 0) left = 0;
            if (right > TFT_WIDTH) right = TFT_WIDTH;
            for (int row = 0; row < height; ++row) {
                for (int column = left; column < right; ++column) {
                    s_text_buffer[row * TFT_WIDTH + column] = display_pixel(background);
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

    return height;
}

static void draw_rich_text_on_panel(int x,
                                    int y,
                                    const char *text,
                                    const smooth_font_t *font,
                                    const uint16_t *fg,
                                    const uint16_t *bg,
                                    uint16_t panel_background,
                                    int panel_left,
                                    int panel_right)
{
    int height = compose_rich_text_on_panel(x, text, font, fg, bg,
                                            panel_background, panel_left, panel_right);
    send_text_buffer_region(panel_left, y, panel_right - panel_left, height);
}

static void draw_rich_text_on_panel_clipped(int x,
                                            int y,
                                            const char *text,
                                            const smooth_font_t *font,
                                            const uint16_t *fg,
                                            const uint16_t *bg,
                                            uint16_t panel_background,
                                            int panel_left,
                                            int panel_right,
                                            int clip_top,
                                            int clip_bottom)
{
    int height = compose_rich_text_on_panel(x, text, font, fg, bg,
                                            panel_background, panel_left, panel_right);
    int first_row = clip_top > y ? clip_top - y : 0;
    int end_row = clip_bottom < y + height ? clip_bottom - y : height;
    if (end_row > first_row) {
        send_text_buffer_region_slice(panel_left, y + first_row,
                                      panel_right - panel_left,
                                      first_row, end_row - first_row);
    }
}

static void draw_rich_text(int x,
                           int y,
                           const char *text,
                           const smooth_font_t *font,
                           const uint16_t *fg,
                           const uint16_t *bg)
{
    draw_rich_text_on_panel(x, y, text, font, fg, bg, COLOR_BLACK, 0, TFT_WIDTH);
}

static int text_width(const char *text, const smooth_font_t *font)
{
    int width = 0;
    while (*text != '\0') {
        uint8_t code = (uint8_t)*text++;
        if (code < font->first_char || code > font->last_char) code = (uint8_t)'?';
        width += font->glyphs[code - font->first_char].advance;
    }
    return width;
}

static int text_visual_y(const char *text, const smooth_font_t *font,
                         int frame_y, int frame_height)
{
    int top = font->line_height;
    int bottom = 0;
    while (*text != '\0') {
        uint8_t code = (uint8_t)*text++;
        if (code < font->first_char || code > font->last_char) code = (uint8_t)'?';
        const smooth_glyph_t *glyph = &font->glyphs[code - font->first_char];
        if (glyph->height == 0U) continue;
        if (glyph->y_offset < top) top = glyph->y_offset;
        int glyph_bottom = glyph->y_offset + glyph->height;
        if (glyph_bottom > bottom) bottom = glyph_bottom;
    }
    if (bottom <= top) return frame_y + (frame_height - font->line_height) / 2;
    return frame_y + (frame_height - (bottom - top)) / 2 - top;
}

static void buffer_fill_rect(int x, int y, int width, int height,
                             int buffer_height, uint16_t color)
{
    uint16_t pixel = display_pixel(color);
    for (int row = y; row < y + height && row < buffer_height; ++row) {
        for (int column = x; column < x + width && column < TFT_WIDTH; ++column) {
            if (row >= 0 && column >= 0) s_text_buffer[row * TFT_WIDTH + column] = pixel;
        }
    }
}

static void buffer_fill_round_rect(int x, int y, int width, int height, int radius,
                                   int buffer_height, uint16_t color)
{
    buffer_fill_rect(x, y + radius, width, height - radius * 2, buffer_height, color);
    for (int row = 0; row < radius; ++row) {
        int inset = radius - row;
        buffer_fill_rect(x + inset, y + row, width - inset * 2, 1, buffer_height, color);
        buffer_fill_rect(x + inset, y + height - 1 - row,
                         width - inset * 2, 1, buffer_height, color);
    }
}

static void buffer_draw_text(int x, int y, const char *text, const smooth_font_t *font,
                             uint16_t foreground, uint16_t background, int buffer_height)
{
    int cursor = x;
    while (*text != '\0') {
        uint8_t code = (uint8_t)*text++;
        if (code < font->first_char || code > font->last_char) code = (uint8_t)'?';
        const smooth_glyph_t *g = &font->glyphs[code - font->first_char];
        unsigned pixel_index = 0U;
        for (int row = 0; row < g->height; ++row) {
            int target_y = y + row + g->y_offset;
            for (int column = 0; column < g->width; ++column, ++pixel_index) {
                int target_x = cursor + g->x_offset + column;
                uint8_t packed = font->bitmap[g->bitmap_offset + pixel_index / 2U];
                uint8_t alpha = (pixel_index & 1U) == 0U ? packed >> 4 : packed & 0x0FU;
                if (alpha != 0U && target_x >= 0 && target_x < TFT_WIDTH &&
                    target_y >= 0 && target_y < buffer_height) {
                    s_text_buffer[target_y * TFT_WIDTH + target_x] =
                        blend_rgb565(foreground, background, alpha);
                }
            }
        }
        cursor += g->advance;
    }
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
        current_ua -= ((int64_t)measurement->bus_mv * CHANNEL_A_DIVIDER_LEAK_UA_PER_MV_NUM +
                       CHANNEL_A_DIVIDER_LEAK_UA_PER_MV_DEN / 2LL) /
                      CHANNEL_A_DIVIDER_LEAK_UA_PER_MV_DEN;
    }
    return current_ua;
}

static void format_measured_current(char *out,
                                    size_t size,
                                    char channel,
                                    const app_ina238_channel_t *measurement)
{
    int64_t current_ua = measured_current_ua(channel, measurement);
    if (current_ua < 0) current_ua = 0;
    uint8_t decimals = measured_current_decimals(channel);
    int64_t scale = 1;
    for (uint8_t i = 0; i < decimals; ++i) scale *= 10LL;

    int64_t scaled = (current_ua * scale + 500000LL) / 1000000LL;
    int64_t whole = scaled / scale;
    int64_t frac = scaled % scale;

    if (decimals == 3U) {
        snprintf(out, size, "%lld.%03lld", (long long)whole, (long long)frac);
    } else if (decimals == 4U) {
        snprintf(out, size, "%lld.%04lld", (long long)whole, (long long)frac);
    } else if (decimals == 6U) {
        snprintf(out, size, "%lld.%06lld", (long long)whole, (long long)frac);
    } else {
        snprintf(out, size, "%lld.%05lld", (long long)whole, (long long)frac);
    }
}

/* draw_power_screen_frame
 * Inputs: none.
 * Returns: none.
 * Does: draws the static Twin Rail structure.  Thin cyan/purple rules echo the
 * startup logo without consuming space needed by the live measurements.
 */
static void draw_power_screen_frame(void)
{
    fill_round_rect(POWER_CHANNEL_INNER_X, 4,
                    POWER_CHANNEL_INNER_RIGHT - POWER_CHANNEL_INNER_X,
                    60, 4, COLOR_PANEL);
    draw_panel_outline(POWER_CHANNEL_FRAME_X, 2,
                       POWER_CHANNEL_FRAME_WIDTH, 64, COLOR_CYAN);
    fill_round_rect(POWER_CHANNEL_INNER_X, 72,
                    POWER_CHANNEL_INNER_RIGHT - POWER_CHANNEL_INNER_X,
                    62, 4, COLOR_PANEL);
    draw_panel_outline(POWER_CHANNEL_FRAME_X, 70,
                       POWER_CHANNEL_FRAME_WIDTH, 66, COLOR_PURPLE);
    fill_round_rect(POWER_SIDE_FRAME_X + 2, 4,
                    POWER_SIDE_FRAME_WIDTH - 4, 130, 4, COLOR_PANEL);
    draw_panel_outline(POWER_SIDE_FRAME_X, 2,
                       POWER_SIDE_FRAME_WIDTH, 134, COLOR_CYAN);

    fill_round_rect(8, 142, 148, 25, 4, COLOR_PANEL);
    draw_panel_outline(6, 140, 152, 29, COLOR_CYAN);
    fill_round_rect(164, 142, 148, 25, 4, COLOR_PANEL);
    draw_panel_outline(162, 140, 152, 29, COLOR_PURPLE);
}

static void draw_channel_actual_line(int y,
                                     char channel,
                                     const app_ina238_channel_t *measurement,
                                     bool cc_active,
                                     bool channel_enabled)
{
    const int height = 38;
    char voltage[16];
    char current[32];
    uint16_t channel_bg = channel == 'A' ? COLOR_CYAN : COLOR_PURPLE;
    uint16_t channel_fg = channel == 'A' ? COLOR_BLACK : COLOR_WHITE;
    if (!channel_enabled) {
        channel_bg = COLOR_GRAY;
        channel_fg = COLOR_BLACK;
    }

    if (measurement->valid) {
        snprintf(voltage, sizeof(voltage), "%.2f", (double)measurement->bus_mv / 1000.0);
        format_measured_current(current, sizeof(current), channel, measurement);
    } else {
        snprintf(voltage, sizeof(voltage), "--.--");
        snprintf(current, sizeof(current), "--.----");
    }

    clear_text_buffer(height, COLOR_BLACK);
    buffer_fill_rect(POWER_CHANNEL_INNER_X, 0,
                     POWER_CHANNEL_INNER_RIGHT - POWER_CHANNEL_INNER_X,
                     height, height, COLOR_PANEL);

    int old_badge_width = text_width(" A ", &instrument_30);
    int badge_width = (old_badge_width * 4 + 2) / 5;
    int badge_height = (instrument_30.line_height * 4 + 2) / 5;
    buffer_fill_round_rect(8, 0, badge_width, badge_height, 3, height, channel_bg);
    char channel_text[2] = {channel, '\0'};
    const smooth_glyph_t *channel_glyph =
        &instrument_30.glyphs[(uint8_t)channel - instrument_30.first_char];
    int channel_x = 8 + (badge_width - channel_glyph->width) / 2 - channel_glyph->x_offset;
    int channel_y = (badge_height - channel_glyph->height) / 2 - channel_glyph->y_offset;
    buffer_draw_text(channel_x, channel_y, channel_text, &instrument_30,
                     channel_fg, channel_bg, height);

    uint16_t value_color = (measurement->valid && channel_enabled) ? COLOR_WHITE : COLOR_GRAY;
    int voltage_x = 8 + badge_width + 6;
    buffer_draw_text(voltage_x, 0, voltage, &instrument_30,
                     value_color, COLOR_PANEL, height);
    int unit_v_x = voltage_x + text_width(voltage, &instrument_30) + 1;
    const smooth_glyph_t *main_zero =
        &instrument_30.glyphs[(uint8_t)'0' - instrument_30.first_char];
    const smooth_glyph_t *small_v =
        &instrument_18.glyphs[(uint8_t)'V' - instrument_18.first_char];
    int unit_v_y = (main_zero->y_offset + main_zero->height) -
                   (small_v->y_offset + small_v->height);
    buffer_draw_text(unit_v_x, unit_v_y, "V", &instrument_18,
                     channel_enabled ? COLOR_GREEN : COLOR_GRAY, COLOR_PANEL, height);

    size_t current_len = strlen(current);
    size_t small_digits = measurement->valid ? (channel == 'A' ? 2U : 1U) : 0U;
    if (small_digits > current_len) small_digits = 0U;
    char current_main[32];
    char current_tail[4] = "";
    size_t main_len = current_len - small_digits;
    memcpy(current_main, current, main_len);
    current_main[main_len] = '\0';
    if (small_digits != 0U) snprintf(current_tail, sizeof(current_tail), "%s", current + main_len);
    int current_group_width = text_width(current_main, &instrument_30) +
                              text_width(current_tail, &instrument_18) + 1 +
                              text_width("A", &instrument_18);
    int current_x = POWER_CURRENT_RIGHT - current_group_width;
    uint16_t current_fg = (cc_active && channel_enabled) ? channel_fg : value_color;
    uint16_t current_bg = (cc_active && channel_enabled) ? channel_bg : COLOR_PANEL;
    if (cc_active && channel_enabled) {
        int current_width = text_width(current_main, &instrument_30) +
                            text_width(current_tail, &instrument_18);
        buffer_fill_round_rect(current_x - 2, 0, current_width + 4,
                               badge_height, 3, height, current_bg);
    }
    buffer_draw_text(current_x, 0, current_main, &instrument_30,
                     current_fg, current_bg, height);
    int tail_x = current_x + text_width(current_main, &instrument_30);
    const smooth_glyph_t *small_zero =
        &instrument_18.glyphs[(uint8_t)'0' - instrument_18.first_char];
    int tail_y = (main_zero->y_offset + main_zero->height) -
                 (small_zero->y_offset + small_zero->height);
    buffer_draw_text(tail_x, tail_y, current_tail, &instrument_18,
                     current_fg, current_bg, height);
    int unit_a_x = tail_x + text_width(current_tail, &instrument_18) + 1;
    const smooth_glyph_t *small_a =
        &instrument_18.glyphs[(uint8_t)'A' - instrument_18.first_char];
    int unit_a_y = (main_zero->y_offset + main_zero->height) -
                   (small_a->y_offset + small_a->height);
    buffer_draw_text(unit_a_x, unit_a_y, "A", &instrument_18,
                     channel_enabled ? COLOR_CYAN : COLOR_GRAY, COLOR_PANEL, height);

    send_text_buffer_region(POWER_CHANNEL_INNER_X, y,
                            POWER_CHANNEL_INNER_RIGHT - POWER_CHANNEL_INNER_X,
                            height);
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
                                  uint8_t voltage_select,
                                  uint8_t current_select,
                                  uint8_t selected,
                                  uint8_t digit,
                                  bool edit_blink_on)
{
    char line[64] = "";
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;
    char voltage[8];
    char current[8];
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
    append_colored(line, fg, bg, &pos, "A", COLOR_CYAN);

    if (selected == voltage_select) {
        if (digit == CONTROL_DIGIT_WHOLE) {
            mark_range(fg, bg, voltage_start, strlen(voltage), COLOR_BLACK, COLOR_YELLOW);
        } else {
            mark_range(fg, bg, voltage_start, strlen(voltage), COLOR_YELLOW, COLOR_BLACK);
            mark_range(fg, bg, voltage_start + editable_digit_offset(voltage, digit),
                       1U, COLOR_BLACK, COLOR_YELLOW);
        }
    } else if (selected == current_select) {
        if (digit == CONTROL_DIGIT_WHOLE || edit_blink_on) {
            mark_range(fg, bg, current_start, strlen(current), COLOR_BLACK, COLOR_YELLOW);
        }
    }
    draw_rich_text_on_panel(8, y, line, &instrument_18, fg, bg,
                            COLOR_PANEL,
                            POWER_CHANNEL_INNER_X, POWER_CHANNEL_INNER_RIGHT);
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

    draw_rich_text_on_panel_clipped(10, y, line, &instrument_18, fg, bg,
                                    COLOR_PANEL, 8, TFT_WIDTH - 8,
                                    126, 167);
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

    draw_rich_text(8, y, line, &instrument_30, fg, bg);
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

    draw_rich_text_on_panel(12, 72, line, &instrument_30, fg, bg,
                            COLOR_PANEL, 8, TFT_WIDTH - 8);
}

static void draw_generator_frequency_line(const app_state_t *s)
{
    char line[64] = "";
    char value[16];
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;
    size_t value_start;

    fill_text_colors(fg, bg, 64U);
    append_colored(line, fg, bg, &pos, "FREQ ", COLOR_GRAY);
    value_start = pos;
    snprintf(value, sizeof(value), "%06lu", (unsigned long)s->control.generator_freq_hz);
    append_colored(line, fg, bg, &pos, value, COLOR_YELLOW);
    append_colored(line, fg, bg, &pos, " HZ", COLOR_GREEN);

    if (s->control.selected_value == CONTROL_SELECT_GEN_FREQ) {
        if (s->control.selected_digit == CONTROL_DIGIT_WHOLE) {
            mark_range(fg, bg, value_start, strlen(value), COLOR_BLACK, COLOR_YELLOW);
        } else {
            mark_range(fg, bg,
                       value_start + editable_digit_offset(value, s->control.selected_digit),
                       1U, COLOR_BLACK, COLOR_YELLOW);
        }
    }
    draw_rich_text_on_panel(12, 32, line, &instrument_30, fg, bg,
                            COLOR_PANEL, 8, TFT_WIDTH - 8);
}

static void draw_generator_actual_line(int y,
                                       char channel,
                                       const app_ina238_channel_t *measurement)
{
    char line[64] = "";
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;
    uint16_t channel_color = channel == 'A' ? COLOR_CYAN : COLOR_PURPLE;

    fill_text_colors(fg, bg, 64U);
    append_format_colored(line, fg, bg, &pos, channel_color, "%c ", channel);
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
    draw_rich_text_on_panel(12, y, line, &instrument_18, fg, bg,
                            COLOR_PANEL, 8, TFT_WIDTH - 8);
}

static void draw_generator_screen(const app_state_t *s, bool draw_frame)
{
    if (draw_frame) {
        char title[] = "GENERATOR";
        uint16_t fg[16];
        uint16_t bg[16];
        fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);
        fill_round_rect(8, 4, TFT_WIDTH - 16, 110, 4, COLOR_PANEL);
        draw_panel_outline(6, 2, TFT_WIDTH - 12, 114, COLOR_CYAN);
        fill_round_rect(8, 121, TFT_WIDTH - 16, 46, 4, COLOR_PANEL);
        draw_panel_outline(6, 119, TFT_WIDTH - 12, 50, COLOR_PURPLE);
        fill_text_colors(fg, bg, 16U);
        for (size_t i = 0U; title[i] != '\0'; ++i) fg[i] = COLOR_CYAN;
        draw_rich_text_on_panel(12, 7, title, &instrument_18, fg, bg,
                                COLOR_PANEL, 8, TFT_WIDTH - 8);
    }
    draw_generator_frequency_line(s);
    draw_generator_duty_line(s);
    draw_generator_actual_line(121, 'A', &s->ina238.channel[0]);
    draw_generator_actual_line(145, 'B', &s->ina238.channel[1]);
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

    draw_rich_text_on_panel_clipped(10, 4, line, &instrument_18, fg, bg,
                                    COLOR_PANEL, 8, TFT_WIDTH - 8,
                                    3, 24);
}

static void draw_uart_rx_line(int y, const char *text)
{
    const char *shown = text[0] != '\0' ? text : "-";
    uint16_t fg[64];
    uint16_t bg[64];
    fill_text_colors(fg, bg, 64U);
    for (size_t i = 0U; shown[i] != '\0' && i < 64U; ++i) {
        fg[i] = text[0] != '\0' ? COLOR_WHITE : COLOR_GRAY;
    }
    draw_rich_text_on_panel_clipped(10, y, shown, &instrument_30, fg, bg,
                                    COLOR_PANEL, 8, TFT_WIDTH - 8,
                                    28, 121);
}

static void draw_small_rx_line(int y, const char *text)
{
    const char *shown = text[0] != '\0' ? text : "-";
    uint16_t fg[64];
    uint16_t bg[64];
    fill_text_colors(fg, bg, 64U);
    for (size_t i = 0U; shown[i] != '\0' && i < 64U; ++i) {
        fg[i] = text[0] != '\0' ? COLOR_WHITE : COLOR_GRAY;
    }
    draw_rich_text_on_panel_clipped(10, y, shown, &instrument_18, fg, bg,
                                    COLOR_PANEL, 8, TFT_WIDTH - 8,
                                    54, 120);
}

static void draw_serial_screen(const app_state_t *s, const char *title, uint32_t rate,
                               uint8_t select, bool draw_frame)
{
    if (draw_frame) {
        fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);
        fill_rect(8, 3, TFT_WIDTH - 16, 21, COLOR_PANEL);
        draw_panel_outline(6, 1, TFT_WIDTH - 12, 25, COLOR_CYAN);
        fill_rect(8, 28, TFT_WIDTH - 16, 93, COLOR_PANEL);
        draw_panel_outline(6, 26, TFT_WIDTH - 12, 97, COLOR_PURPLE);
        fill_rect(8, 126, TFT_WIDTH - 16, 41, COLOR_PANEL);
        draw_panel_outline(6, 124, TFT_WIDTH - 12, 45, COLOR_CYAN);
    }
    draw_serial_header(s, title, rate, select);
    draw_uart_rx_line(28, s->uart.lines[0]);
    draw_uart_rx_line(60, s->uart.lines[1]);
    draw_uart_rx_line(88, s->uart.lines[2]);
    draw_bottom_actuals(s);
}

static void draw_mask_line(int y, const char *mask, uint8_t select, const app_state_t *s)
{
    char line[32] = "";
    uint16_t fg[32];
    uint16_t bg[32];
    size_t pos = 0U;
    size_t value_start;

    fill_text_colors(fg, bg, 32U);
    append_colored(line, fg, bg, &pos, "MASK ", COLOR_CYAN);
    value_start = pos;
    append_colored(line, fg, bg, &pos, mask, COLOR_YELLOW);

    if (s->control.selected_value == select) {
        if (s->control.selected_digit == CONTROL_DIGIT_WHOLE) {
            mark_range(fg, bg, value_start, strlen(mask), COLOR_BLACK, COLOR_YELLOW);
        } else if (s->control.selected_digit < strlen(mask)) {
            mark_range(fg, bg, value_start + s->control.selected_digit, 1U, COLOR_BLACK, COLOR_YELLOW);
        }
    }

    draw_rich_text_on_panel_clipped(10, y, line, &instrument_18, fg, bg,
                                    COLOR_PANEL, 8, TFT_WIDTH - 8,
                                    3, 49);
}

static void draw_masked_serial_frame(void)
{
    fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);
    fill_rect(8, 3, TFT_WIDTH - 16, 46, COLOR_PANEL);
    draw_panel_outline(6, 1, TFT_WIDTH - 12, 50, COLOR_CYAN);
    fill_rect(8, 54, TFT_WIDTH - 16, 66, COLOR_PANEL);
    draw_panel_outline(6, 52, TFT_WIDTH - 12, 70, COLOR_PURPLE);
    fill_rect(8, 126, TFT_WIDTH - 16, 41, COLOR_PANEL);
    draw_panel_outline(6, 124, TFT_WIDTH - 12, 45, COLOR_CYAN);
}

static void draw_serial_screen_with_mask(const app_state_t *s,
                                         const char *title,
                                         uint32_t rate,
                                         uint8_t rate_select,
                                         const char *mask,
                                         uint8_t mask_select,
                                         bool draw_frame)
{
    if (draw_frame) draw_masked_serial_frame();
    draw_serial_header(s, title, rate, rate_select);
    draw_mask_line(27, mask, mask_select, s);
    draw_small_rx_line(54, s->uart.lines[0]);
    draw_small_rx_line(76, s->uart.lines[1]);
    draw_small_rx_line(98, s->uart.lines[2]);
    draw_bottom_actuals(s);
}

static void draw_i2c_rx_line(int y, const char *text)
{
    const char *shown = text[0] != '\0' ? text : "-";
    uint16_t fg[64];
    uint16_t bg[64];
    fill_text_colors(fg, bg, 64U);
    for (size_t i = 0U; shown[i] != '\0' && i < 64U; ++i) {
        fg[i] = text[0] != '\0' ? COLOR_WHITE : COLOR_GRAY;
    }
    draw_rich_text_on_panel_clipped(10, y, shown, &instrument_18, fg, bg,
                                    COLOR_PANEL, 8, TFT_WIDTH - 8,
                                    28, 121);
}

static void draw_i2c_sniffer_screen(const app_state_t *s, bool draw_frame)
{
    char line[64] = "";
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;
    size_t mask_start;

    if (draw_frame) {
        fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);
        fill_rect(8, 3, TFT_WIDTH - 16, 21, COLOR_PANEL);
        draw_panel_outline(6, 1, TFT_WIDTH - 12, 25, COLOR_CYAN);
        fill_rect(8, 28, TFT_WIDTH - 16, 93, COLOR_PANEL);
        draw_panel_outline(6, 26, TFT_WIDTH - 12, 97, COLOR_PURPLE);
        fill_rect(8, 126, TFT_WIDTH - 16, 41, COLOR_PANEL);
        draw_panel_outline(6, 124, TFT_WIDTH - 12, 45, COLOR_CYAN);
    }

    fill_text_colors(fg, bg, 64U);
    append_colored(line, fg, bg, &pos, "I2C SNIFFER  MASK ", COLOR_CYAN);
    mask_start = pos;
    append_colored(line, fg, bg, &pos, s->control.i2c_mask, COLOR_YELLOW);
    if (s->control.selected_value == CONTROL_SELECT_I2C_MASK) {
        if (s->control.selected_digit == CONTROL_DIGIT_WHOLE) {
            mark_range(fg, bg, mask_start, strlen(s->control.i2c_mask),
                       COLOR_BLACK, COLOR_YELLOW);
        } else if (s->control.selected_digit < strlen(s->control.i2c_mask)) {
            mark_range(fg, bg, mask_start + s->control.selected_digit, 1U,
                       COLOR_BLACK, COLOR_YELLOW);
        }
    }
    draw_rich_text_on_panel_clipped(10, 4, line, &instrument_18, fg, bg,
                                    COLOR_PANEL, 8, TFT_WIDTH - 8,
                                    3, 24);

    draw_i2c_rx_line(30, s->i2c_sniffer.lines[0]);
    draw_i2c_rx_line(52, s->i2c_sniffer.lines[1]);
    draw_i2c_rx_line(74, s->i2c_sniffer.lines[2]);
    draw_i2c_rx_line(96, s->i2c_sniffer.lines[3]);
    draw_bottom_actuals(s);
}

static void draw_i2c_master_screen(const app_state_t *s, bool draw_frame)
{
    char line[64] = "";
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;

    if (draw_frame) {
        fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);
        fill_rect(8, 3, TFT_WIDTH - 16, 21, COLOR_PANEL);
        draw_panel_outline(6, 1, TFT_WIDTH - 12, 25, COLOR_CYAN);
        fill_rect(8, 28, TFT_WIDTH - 16, 93, COLOR_PANEL);
        draw_panel_outline(6, 26, TFT_WIDTH - 12, 97, COLOR_PURPLE);
        fill_rect(8, 126, TFT_WIDTH - 16, 41, COLOR_PANEL);
        draw_panel_outline(6, 124, TFT_WIDTH - 12, 45, COLOR_CYAN);
    }

    fill_text_colors(fg, bg, 64U);
    append_colored(line, fg, bg, &pos, "I2C MASTER  100K", COLOR_CYAN);
    draw_rich_text_on_panel_clipped(10, 4, line, &instrument_18, fg, bg,
                                    COLOR_PANEL, 8, TFT_WIDTH - 8,
                                    3, 24);

    draw_i2c_rx_line(30, s->i2c_sniffer.lines[0]);
    draw_i2c_rx_line(52, s->i2c_sniffer.lines[1]);
    draw_i2c_rx_line(74, s->i2c_sniffer.lines[2]);
    draw_i2c_rx_line(96, s->i2c_sniffer.lines[3]);
    draw_bottom_actuals(s);
}

static void draw_can_screen(const app_state_t *s, bool draw_frame)
{
    char line[64] = "";
    char value[16];
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;
    size_t value_start;

    if (draw_frame) draw_masked_serial_frame();

    snprintf(value, sizeof(value), "%lu", (unsigned long)(s->control.can_bitrate / 1000U));
    fill_text_colors(fg, bg, 64U);
    append_colored(line, fg, bg, &pos, "CAN  BAUD ", COLOR_CYAN);
    value_start = pos;
    append_colored(line, fg, bg, &pos, value, COLOR_YELLOW);
    append_colored(line, fg, bg, &pos, " KBIT", COLOR_GREEN);
    if (s->control.selected_value == CONTROL_SELECT_CAN_BITRATE) {
        mark_range(fg, bg, value_start, strlen(value), COLOR_BLACK, COLOR_YELLOW);
    }
    draw_rich_text_on_panel_clipped(10, 4, line, &instrument_18, fg, bg,
                                    COLOR_PANEL, 8, TFT_WIDTH - 8,
                                    3, 24);
    draw_mask_line(27, s->control.can_mask, CONTROL_SELECT_CAN_MASK, s);

    fill_text_colors(fg, bg, 64U);
    for (size_t i = 0U; i < strlen("ID: ---"); ++i) fg[i] = COLOR_WHITE;
    draw_rich_text_on_panel_clipped(10, 65, "ID: ---", &instrument_18, fg, bg,
                                    COLOR_PANEL, 8, TFT_WIDTH - 8,
                                    54, 120);
    fill_text_colors(fg, bg, 64U);
    for (size_t i = 0U; i < strlen("DATA: --"); ++i) fg[i] = COLOR_WHITE;
    draw_rich_text_on_panel_clipped(10, 91, "DATA: --", &instrument_18, fg, bg,
                                    COLOR_PANEL, 8, TFT_WIDTH - 8,
                                    54, 120);

    draw_bottom_actuals(s);
}

static void draw_setting_line(int y,
                              const char *label,
                              const char *value,
                              uint8_t select,
                              const app_state_t *s,
                              int clip_top,
                              int clip_bottom)
{
    char line[64] = "";
    uint16_t fg[64];
    uint16_t bg[64];
    size_t pos = 0U;
    size_t value_start;

    fill_text_colors(fg, bg, 64U);
    append_colored(line, fg, bg, &pos, label, COLOR_LABEL);
    append_colored(line, fg, bg, &pos, " ", COLOR_LABEL);
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

    draw_rich_text_on_panel_clipped(12, y, line, &instrument_18, fg, bg,
                                    COLOR_PANEL, 8, TFT_WIDTH - 8,
                                    clip_top, clip_bottom);
}

static void draw_setting_screen(const app_state_t *s, bool draw_frame)
{
    char value[8];

    if (draw_frame) {
        fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);
        fill_rect(8, 29, TFT_WIDTH - 16, 28, COLOR_PANEL);
        draw_panel_outline(6, 27, TFT_WIDTH - 12, 32, COLOR_CYAN);
        fill_rect(8, 65, TFT_WIDTH - 16, 28, COLOR_PANEL);
        draw_panel_outline(6, 63, TFT_WIDTH - 12, 32, COLOR_PURPLE);
        fill_rect(8, 101, TFT_WIDTH - 16, 28, COLOR_PANEL);
        draw_panel_outline(6, 99, TFT_WIDTH - 12, 32, COLOR_CYAN);
        fill_rect(8, 137, TFT_WIDTH - 16, 30, COLOR_PANEL);
        draw_panel_outline(6, 135, TFT_WIDTH - 12, 34, COLOR_PURPLE);
    }
    draw_text(8, 4, "SETTING", &instrument_18, COLOR_CYAN);

    draw_setting_line(31,
                      "Overcurrent",
                      s->control.overcurrent_cc ? "CC" : "OFF",
                      CONTROL_SELECT_OVERCURRENT,
                      s, 29, 57);
    snprintf(value, sizeof(value), "%02u", (unsigned)s->control.overheat_c);
    draw_setting_line(67, "Overheat", value, CONTROL_SELECT_OVERHEAT, s, 65, 93);
    snprintf(value, sizeof(value), "%03u", (unsigned)s->control.overpower_w);
    draw_setting_line(103, "Overpower", value, CONTROL_SELECT_OVERPOWER, s, 101, 129);
    snprintf(value, sizeof(value), "%03u", (unsigned)s->control.volume_percent);
    draw_setting_line(139, "Volume", value, CONTROL_SELECT_VOLUME, s, 137, 167);
}

static void draw_reserved_screen(const app_state_t *s, const char *title)
{
    draw_text(8, 4, title, &instrument_18, COLOR_CYAN);
    draw_text(8, 58, "RESERVED", &instrument_30, COLOR_GRAY);
    draw_bottom_actuals(s);
}

static const char *mode_menu_name(app_mode_t mode)
{
    switch (mode) {
        case APP_MODE_POWER_SUPPLY: return "POWER SOURCE";
        case APP_MODE_GENERATOR:    return "GENERATOR";
        case APP_MODE_UART:         return "UART";
        case APP_MODE_LIN:          return "LIN";
        case APP_MODE_1WIRE:        return "1WIRE";
        case APP_MODE_RS485:        return "RS485";
        case APP_MODE_CAN:          return "CAN";
        case APP_MODE_I2C:          return "I2C SNIFFER";
        case APP_MODE_I2C_MASTER:   return "I2C MASTER";
        case APP_MODE_SETTING:      return "SETTING";
        default:                    return "?";
    }
}

static void draw_mode_menu(const app_state_t *s, bool force)
{
    const uint8_t rows_per_column = 5U;
    const int frame_x[2] = {4, 162};
    const int frame_width = 154;
    const int frame_height = 32;
    const int row_pitch = 33;
    static char previous_lines[APP_MODE_COUNT][64];
    static uint8_t previous_selected = 0xFFU;
    static app_mode_t previous_active = APP_MODE_COUNT;

    if (force) {
        fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);
        for (uint8_t mode_index = 0U; mode_index < (uint8_t)APP_MODE_COUNT; ++mode_index) {
            int column = mode_index / rows_per_column;
            int row = mode_index % rows_per_column;
            int x = frame_x[column];
            int y = 3 + row * row_pitch;
            fill_rect(x + 2, y + 2, frame_width - 4, frame_height - 4, COLOR_PANEL);
            draw_panel_outline(x, y, frame_width, frame_height,
                               column == 0 ? COLOR_CYAN : COLOR_PURPLE);
            previous_lines[mode_index][0] = '\0';
        }
        previous_selected = 0xFFU;
        previous_active = APP_MODE_COUNT;
    }

    for (uint8_t mode_index = 0U; mode_index < (uint8_t)APP_MODE_COUNT; ++mode_index) {
        char line[64];
        uint16_t fg[64];
        uint16_t bg[64];
        size_t length;
        int column = mode_index / rows_per_column;
        int row = mode_index % rows_per_column;
        int x = frame_x[column];
        int y = 3 + row * row_pitch;
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
            selection_changed_here ||
            active_changed_here ||
            strcmp(previous_lines[mode_index], line) != 0) {
            int text_y = text_visual_y(line, &instrument_18, y, frame_height);
            draw_rich_text_on_panel(x + 4, text_y,
                                    line, &instrument_18, fg, bg,
                                    COLOR_PANEL, x + 2, x + frame_width - 2);
            snprintf(previous_lines[mode_index], sizeof(previous_lines[mode_index]), "%s", line);
        }
    }
    previous_selected = s->control.menu_index;
    previous_active = s->control.mode;
}

/* draw_status_line
 * Inputs: app state.
 * Returns: none.
 * Does: draws the bottom power summary line for the power-supply screen.
 */
static void draw_status_line(const app_state_t *s)
{
    char left[32] = "";
    char right[24] = "";
    uint16_t left_fg[32];
    uint16_t left_bg[32];
    uint16_t right_fg[24];
    uint16_t right_bg[24];
    size_t left_pos = 0U;
    size_t right_pos = 0U;
    const app_ina238_channel_t *a = &s->ina238.channel[0];
    const app_ina238_channel_t *b = &s->ina238.channel[1];
    bool temperature_valid = a->valid || b->valid;
    int32_t temperature_mc = 0;
    if (a->valid && b->valid) {
        temperature_mc = a->temperature_mc > b->temperature_mc ?
                         a->temperature_mc : b->temperature_mc;
    } else if (a->valid) {
        temperature_mc = a->temperature_mc;
    } else if (b->valid) {
        temperature_mc = b->temperature_mc;
    }
    uint32_t input_mv = s->analog.input_mv;
    uint32_t input_tenths = (input_mv + 50U) / 100U;
    uint32_t frequency_hz = s->timing.frequency_hz > 999999.0f ?
                            999999U : (uint32_t)(s->timing.frequency_hz + 0.5f);

    fill_text_colors(left_fg, left_bg, 32U);
    append_colored(left, left_fg, left_bg, &left_pos, "T", COLOR_GRAY);
    size_t temperature_start = left_pos;
    if (temperature_valid) {
        append_format_colored(left, left_fg, left_bg, &left_pos, COLOR_WHITE,
                              "%.0f", (double)temperature_mc / 1000.0);
    } else {
        append_colored(left, left_fg, left_bg, &left_pos, "--", COLOR_WHITE);
    }
    if (temperature_valid &&
        temperature_mc >= (int32_t)s->control.overheat_c * 1000) {
        mark_range(left_fg, left_bg, temperature_start,
                   left_pos - temperature_start,
                   COLOR_ALERT_RED_FG, COLOR_ALERT_RED_BG);
    }
    append_colored(left, left_fg, left_bg, &left_pos, "C ", COLOR_GREEN);
    append_colored(left, left_fg, left_bg, &left_pos, "V_IN", COLOR_GRAY);
    append_format_colored(left, left_fg, left_bg, &left_pos, COLOR_WHITE,
                          "%02lu.%01lu",
                          (unsigned long)(input_tenths / 10U),
                          (unsigned long)(input_tenths % 10U));
    append_colored(left, left_fg, left_bg, &left_pos, "V", COLOR_GREEN);

    fill_text_colors(right_fg, right_bg, 24U);
    append_colored(right, right_fg, right_bg, &right_pos, "FREQ ", COLOR_GRAY);
    append_format_colored(right, right_fg, right_bg, &right_pos, COLOR_WHITE,
                          "%06lu", (unsigned long)frequency_hz);
    append_colored(right, right_fg, right_bg, &right_pos, "Hz", COLOR_GREEN);

    int left_y = text_visual_y(left, &instrument_18, 140, 29);
    int right_y = text_visual_y(right, &instrument_18, 140, 29);
    draw_rich_text_on_panel_clipped(10, left_y, left, &instrument_18,
                                    left_fg, left_bg, COLOR_PANEL,
                                    8, 156, 142, 167);
    draw_rich_text_on_panel_clipped(166, right_y, right, &instrument_18,
                                    right_fg, right_bg, COLOR_PANEL,
                                    164, 312, 142, 167);
}

static uint16_t probe_voltage_badge_color(uint32_t voltage_mv, uint32_t test_span_mv)
{
    if (voltage_mv < 800U) return COLOR_GREEN;
    if (voltage_mv > 2200U) return COLOR_RED;
    if (test_span_mv < 88U) return COLOR_YELLOW;
    return COLOR_GRAY;
}

static void draw_power_probe_label(void)
{
    int inner_x = POWER_SIDE_FRAME_X + 2;
    int inner_width = POWER_SIDE_FRAME_WIDTH - 4;
    int label_x = inner_x + (inner_width - text_width("PRB", &instrument_18)) / 2;
    int label_y = text_visual_y("PRB", &instrument_18, 6, 23);
    uint16_t fg[4];
    uint16_t bg[4];
    fill_text_colors(fg, bg, 4U);
    mark_range(fg, bg, 0U, 3U, COLOR_GRAY, COLOR_PANEL);
    draw_rich_text_on_panel_clipped(label_x, label_y, "PRB", &instrument_18,
                                    fg, bg, COLOR_PANEL,
                                    inner_x, inner_x + inner_width, 4, 29);
}

static void format_probe_voltage(char *out, size_t size, uint32_t tenths)
{
    if (tenths > 999U) tenths = 999U;
    snprintf(out, size, "%lu.%01lu",
             (unsigned long)(tenths / 10U),
             (unsigned long)(tenths % 10U));
}

static void draw_power_probe_badge(uint32_t voltage_mv,
                                   uint32_t test_span_mv,
                                   uint32_t voltage_tenths)
{
    char value[8];
    uint16_t fg[8];
    uint16_t bg[8];
    int inner_x = POWER_SIDE_FRAME_X + 2;
    int inner_width = POWER_SIDE_FRAME_WIDTH - 4;
    int badge_size = inner_width - 6;
    int badge_x = inner_x + (inner_width - badge_size) / 2;
    uint16_t badge_color = probe_voltage_badge_color(voltage_mv, test_span_mv);

    fill_round_rect(badge_x, 31, badge_size, badge_size, 3,
                    badge_color);

    format_probe_voltage(value, sizeof(value), voltage_tenths);
    fill_text_colors(fg, bg, 12U);
    mark_range(fg, bg, 0U, strlen(value), COLOR_BLACK, badge_color);
    int value_x = inner_x + (inner_width - text_width(value, &instrument_30)) / 2;
    int value_y = text_visual_y(value, &instrument_30, 31, badge_size);
    draw_rich_text_on_panel_clipped(value_x, value_y, value, &instrument_30,
                                    fg, bg, badge_color,
                                    badge_x, badge_x + badge_size,
                                    31, 31 + badge_size);
}

static void draw_power_probe_span(uint32_t span_mv)
{
    char value[12];
    uint16_t fg[12];
    uint16_t bg[12];
    int inner_x = POWER_SIDE_FRAME_X + 2;
    int inner_width = POWER_SIDE_FRAME_WIDTH - 4;
    if (span_mv > 9999U) span_mv = 9999U;
    snprintf(value, sizeof(value), "%lumV", (unsigned long)span_mv);
    fill_text_colors(fg, bg, 8U);
    mark_range(fg, bg, 0U, strlen(value), COLOR_WHITE, COLOR_PANEL);
    int value_x = inner_x + (inner_width - text_width(value, &instrument_18)) / 2;
    int value_y = text_visual_y(value, &instrument_18, 94, 38);
    draw_rich_text_on_panel_clipped(value_x, value_y, value, &instrument_18,
                                    fg, bg, COLOR_PANEL,
                                    inner_x, inner_x + inner_width, 94, 132);
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
    const uint8_t madctl[] = {0x60}; command_data(0x36, madctl, sizeof(madctl));
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
    static char previous_mode_screen[512] = "";
    static app_mode_t previous_mode = APP_MODE_COUNT;
    static bool previous_menu_open;
    static uint8_t previous_menu_index = 0xFFU;
    static bool psu_screen_initialized;
    static bool generator_screen_initialized;
    static bool serial_screen_initialized;
    static bool setting_screen_initialized;
    static uint16_t previous_probe_color = 0xFFFFU;
    static uint32_t previous_probe_tenths = UINT32_MAX;
    static uint32_t previous_probe_span_mv = UINT32_MAX;
    static uint32_t displayed_probe_tenths = UINT32_MAX;
    char line[64];
    char mode_line[512];
    uint8_t edit_blink_phase = (uint8_t)((esp_timer_get_time() / 500000LL) & 1LL);

    if (previous_menu_open != s->control.menu_open) {
        fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BLACK);
        previous_ch_a[0] = '\0';
        previous_ch_b[0] = '\0';
        previous_limits[0] = '\0';
        previous_mode_screen[0] = '\0';
        psu_screen_initialized = false;
        generator_screen_initialized = false;
        serial_screen_initialized = false;
        setting_screen_initialized = false;
        previous_probe_color = 0xFFFFU;
        previous_probe_tenths = UINT32_MAX;
        previous_probe_span_mv = UINT32_MAX;
        displayed_probe_tenths = UINT32_MAX;
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
        generator_screen_initialized = false;
        serial_screen_initialized = false;
        setting_screen_initialized = false;
        previous_mode = s->control.mode;
    }

    if (s->control.mode != APP_MODE_POWER_SUPPLY) {
        snprintf(previous_ch_a, sizeof(previous_ch_a), "%s", "");
        snprintf(previous_ch_b, sizeof(previous_ch_b), "%s", "");
        snprintf(previous_limits, sizeof(previous_limits), "%s", "");

        snprintf(mode_line, sizeof(mode_line), "M%u S%u D%u F%lu P%u O%u U%lu L%lu R%lu C%lu LM%s CM%s IM%s OC%u OH%u OP%u V%u T%s|%s|%s UE%lu I%s|%s|%s|%s IP%lu IE%lu A%u%lu%ld B%u%lu%ld",
                 (unsigned)s->control.mode,
                 (unsigned)s->control.selected_value,
                 (unsigned)s->control.selected_digit,
                 (unsigned long)s->control.generator_freq_hz,
                 (unsigned)s->control.generator_duty_percent,
                 s->control.generator_on ? 1U : 0U,
                 (unsigned long)s->control.uart_baud,
                 (unsigned long)s->control.lin_baud,
                 (unsigned long)s->control.rs485_baud,
                 (unsigned long)s->control.can_bitrate,
                 s->control.lin_mask,
                 s->control.can_mask,
                 s->control.i2c_mask,
                 s->control.overcurrent_cc ? 1U : 0U,
                 (unsigned)s->control.overheat_c,
                 (unsigned)s->control.overpower_w,
                 (unsigned)s->control.volume_percent,
                 (s->control.mode == APP_MODE_UART || s->control.mode == APP_MODE_LIN ||
                  s->control.mode == APP_MODE_RS485) ? s->uart.lines[0] : "",
                 (s->control.mode == APP_MODE_UART || s->control.mode == APP_MODE_LIN ||
                  s->control.mode == APP_MODE_RS485) ? s->uart.lines[1] : "",
                 (s->control.mode == APP_MODE_UART || s->control.mode == APP_MODE_LIN ||
                  s->control.mode == APP_MODE_RS485) ? s->uart.lines[2] : "",
                 (unsigned long)s->uart.errors,
                 s->control.mode == APP_MODE_I2C ? s->i2c_sniffer.lines[0] : "",
                 s->control.mode == APP_MODE_I2C ? s->i2c_sniffer.lines[1] : "",
                 s->control.mode == APP_MODE_I2C ? s->i2c_sniffer.lines[2] : "",
                 s->control.mode == APP_MODE_I2C ? s->i2c_sniffer.lines[3] : "",
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
                    draw_generator_screen(s, !generator_screen_initialized);
                    generator_screen_initialized = true;
                    break;
                case APP_MODE_UART:
                    draw_serial_screen(s, "UART", s->control.uart_baud,
                                       CONTROL_SELECT_UART_BAUD, !serial_screen_initialized);
                    serial_screen_initialized = true;
                    break;
                case APP_MODE_LIN:
                    draw_serial_screen_with_mask(s,
                                                 "LIN",
                                                 s->control.lin_baud,
                                                 CONTROL_SELECT_LIN_BAUD,
                                                 s->control.lin_mask,
                                                 CONTROL_SELECT_LIN_MASK,
                                                 !serial_screen_initialized);
                    serial_screen_initialized = true;
                    break;
                case APP_MODE_1WIRE:
                    draw_reserved_screen(s, "1WIRE");
                    break;
                case APP_MODE_RS485:
                    draw_serial_screen(s, "RS485", s->control.rs485_baud,
                                       CONTROL_SELECT_RS485_BAUD, !serial_screen_initialized);
                    serial_screen_initialized = true;
                    break;
                case APP_MODE_CAN:
                    draw_can_screen(s, !serial_screen_initialized);
                    serial_screen_initialized = true;
                    break;
                case APP_MODE_I2C:
                    draw_i2c_sniffer_screen(s, !serial_screen_initialized);
                    serial_screen_initialized = true;
                    break;
                case APP_MODE_I2C_MASTER:
                    draw_i2c_master_screen(s, !serial_screen_initialized);
                    serial_screen_initialized = true;
                    break;
                case APP_MODE_SETTING:
                    draw_setting_screen(s, !setting_screen_initialized);
                    setting_screen_initialized = true;
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
        draw_power_screen_frame();
        draw_power_probe_label();
        previous_ch_a[0] = '\0';
        previous_ch_b[0] = '\0';
        previous_limits[0] = '\0';
        previous_probe_color = 0xFFFFU;
        previous_probe_tenths = UINT32_MAX;
        previous_probe_span_mv = UINT32_MAX;
        displayed_probe_tenths = UINT32_MAX;
        psu_screen_initialized = true;
    }

    uint32_t footer_frequency_hz = s->timing.frequency_hz > 999999.0f ?
                                   999999U : (uint32_t)(s->timing.frequency_hz + 0.5f);
    snprintf(line, sizeof(line), "IN%lu TA%u%ld TB%u%ld OH%u F%lu M%u",
             (unsigned long)s->analog.input_mv,
             s->ina238.channel[0].valid ? 1U : 0U,
             (long)s->ina238.channel[0].temperature_mc,
             s->ina238.channel[1].valid ? 1U : 0U,
             (long)s->ina238.channel[1].temperature_mc,
             (unsigned)s->control.overheat_c,
             (unsigned long)footer_frequency_hz,
             s->timing.signal_missing ? 1U : 0U);
    if (strcmp(previous_limits, line) != 0) {
        draw_status_line(s);
        snprintf(previous_limits, sizeof(previous_limits), "%s", line);
    }

    uint32_t raw_probe_tenths = (s->analog.voltage_mv + 50U) / 100U;
    if (raw_probe_tenths > 999U) raw_probe_tenths = 999U;
    if (displayed_probe_tenths == UINT32_MAX) {
        displayed_probe_tenths = raw_probe_tenths;
    } else {
        uint32_t displayed_center_mv = displayed_probe_tenths * 100U;
        if (s->analog.voltage_mv >= displayed_center_mv + 200U ||
            s->analog.voltage_mv + 200U <= displayed_center_mv) {
            displayed_probe_tenths = raw_probe_tenths;
        }
    }
    uint16_t probe_color = probe_voltage_badge_color(s->analog.voltage_mv,
                                                     s->analog.test_span_mv);
    if (previous_probe_color != probe_color ||
        previous_probe_tenths != displayed_probe_tenths) {
        draw_power_probe_badge(s->analog.voltage_mv,
                               s->analog.test_span_mv,
                               displayed_probe_tenths);
        previous_probe_color = probe_color;
        previous_probe_tenths = displayed_probe_tenths;
    }
    if (previous_probe_span_mv != s->analog.test_span_mv) {
        draw_power_probe_span(s->analog.test_span_mv);
        previous_probe_span_mv = s->analog.test_span_mv;
    }

    const app_ina238_channel_t *ina = &s->ina238.channel[0];
    if (ina->valid) {
        snprintf(line, sizeof(line), "A%lu%ld%u%u%u%u%u%u%u%u",
                 (unsigned long)ina->bus_mv,
                 (long)ina->shunt_uv,
                 (unsigned)ina->wide_range,
                 (unsigned)s->control.u2_mv,
                 (unsigned)s->control.i2_ma,
                 s->control.channel_a_enabled ? 1U : 0U,
                 (unsigned)s->control.selected_value,
                 (unsigned)s->control.selected_digit,
                 s->tps55289.current_limit_active ? 1U : 0U,
                 (s->control.selected_value == CONTROL_SELECT_I2 &&
                  s->control.selected_digit != CONTROL_DIGIT_WHOLE) ? edit_blink_phase : 2U);
    } else {
        snprintf(line, sizeof(line), "A--%u%u%u%u%u%u%u",
                 (unsigned)s->control.u2_mv,
                 (unsigned)s->control.i2_ma,
                 s->control.channel_a_enabled ? 1U : 0U,
                 (unsigned)s->control.selected_value,
                 (unsigned)s->control.selected_digit,
                 s->tps55289.current_limit_active ? 1U : 0U,
                 (s->control.selected_value == CONTROL_SELECT_I2 &&
                  s->control.selected_digit != CONTROL_DIGIT_WHOLE) ? edit_blink_phase : 2U);
    }
    if (strcmp(previous_ch_a, line) != 0) {
        draw_channel_actual_line(6, 'A', ina,
                                 s->tps55289.current_limit_active,
                                 s->control.channel_a_enabled);
        draw_channel_set_line(40, s->control.u2_mv, s->control.i2_ma,
                              2U, 3U,
                              s->control.selected_value,
                              s->control.selected_digit,
                              edit_blink_phase == 0U);
        snprintf(previous_ch_a, sizeof(previous_ch_a), "%s", line);
    }

    ina = &s->ina238.channel[1];
    if (ina->valid) {
        snprintf(line, sizeof(line), "B%lu%ld%u%u%u%u%u%u%u%u",
                 (unsigned long)ina->bus_mv,
                 (long)ina->shunt_uv,
                 (unsigned)ina->wide_range,
                 (unsigned)s->control.u1_mv,
                 (unsigned)s->control.i1_ma,
                 s->control.channel_b_enabled ? 1U : 0U,
                 (unsigned)s->control.selected_value,
                 (unsigned)s->control.selected_digit,
                 s->lm51772.current_limit_active ? 1U : 0U,
                 (s->control.selected_value == CONTROL_SELECT_I1 &&
                  s->control.selected_digit != CONTROL_DIGIT_WHOLE) ? edit_blink_phase : 2U);
    } else {
        snprintf(line, sizeof(line), "B--%u%u%u%u%u%u%u",
                 (unsigned)s->control.u1_mv,
                 (unsigned)s->control.i1_ma,
                 s->control.channel_b_enabled ? 1U : 0U,
                 (unsigned)s->control.selected_value,
                 (unsigned)s->control.selected_digit,
                 s->lm51772.current_limit_active ? 1U : 0U,
                 (s->control.selected_value == CONTROL_SELECT_I1 &&
                  s->control.selected_digit != CONTROL_DIGIT_WHOLE) ? edit_blink_phase : 2U);
    }
    if (strcmp(previous_ch_b, line) != 0) {
        draw_channel_actual_line(74, 'B', ina,
                                 s->lm51772.current_limit_active,
                                 s->control.channel_b_enabled);
        draw_channel_set_line(110, s->control.u1_mv, s->control.i1_ma,
                              0U, 1U,
                              s->control.selected_value,
                              s->control.selected_digit,
                              edit_blink_phase == 0U);
        snprintf(previous_ch_b, sizeof(previous_ch_b), "%s", line);
    }

    display_set_rgb(s->analog.logic_state);
}
