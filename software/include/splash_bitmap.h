#pragma once

#include <stdint.h>
#include <stddef.h>

#define SPLASH_BITMAP_WIDTH 320U
#define SPLASH_BITMAP_HEIGHT 170U

extern const uint8_t splash_bitmap_rgb565[SPLASH_BITMAP_WIDTH * SPLASH_BITMAP_HEIGHT * 2U];
extern const size_t splash_bitmap_rgb565_size;