#pragma once
#include <stdint.h>

typedef struct {
    uint16_t bitmap_offset;
    uint8_t width;
    uint8_t height;
    uint8_t advance;
    int8_t x_offset;
    int8_t y_offset;
} smooth_glyph_t;

typedef struct {
    const uint8_t *bitmap;
    const smooth_glyph_t *glyphs;
    uint8_t first_char;
    uint8_t last_char;
    uint8_t line_height;
} smooth_font_t;

extern const smooth_font_t roboto_18;
extern const smooth_font_t roboto_30;