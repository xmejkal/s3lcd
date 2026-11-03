/*
 * The blend of two RGB565 colours, moved out of s3lcd.c (MIT, Copyright (c) 2020-2023 Russ Hughes) so a Mac test
 * builds the same code (tests/test_blend.py).
 *
 * P603: this fork's frame buffer holds colours in the panel's byte order. The games run the driver with
 * swap_color_bytes=False and swap each colour once when they make it (games/gfx.py c565, games/lcd.py). So the blend
 * swaps the bytes of both colours in, mixes red, green and blue, and swaps the result back.
 */

#ifndef BLEND565_H
#define BLEND565_H

#include <stdint.h>

#define _swap_bytes(val) ((((val) >> 8) | ((val) << 8)) & 0xFFFF)

static inline uint16_t alpha_blend_565(uint16_t fg, uint16_t bg, uint8_t alpha) {
    if (alpha == 0) {
        return bg;
    }

    if (alpha == 255) {
        return fg;
    }

    fg = _swap_bytes(fg);
    bg = _swap_bytes(bg);
    uint8_t r = ((fg >> 11) * alpha + (bg >> 11) * (255 - alpha)) >> 8;
    uint8_t g = (((fg >> 5) & 0x3f) * alpha + ((bg >> 5) & 0x3f) * (255 - alpha)) >> 8;
    uint8_t b = ((fg & 0x1f) * alpha + (bg & 0x1f) * (255 - alpha)) >> 8;
    return _swap_bytes((uint16_t)((r << 11) | (g << 5) | b));
}

#endif
