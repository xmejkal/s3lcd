// The frame: the framebuffer memory and its send to the panel.
//
// It talks to the panel only through esp_lcd's panel API (draw_bitmap and the
// color-done callback), so it knows no controller command and works for any
// panel esp_lcd drives. Its settings come from Python through frame_config().

#ifndef __FRAME_H__
#define __FRAME_H__

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "py/obj.h"

// Where the framebuffer lives.
typedef enum {
    FRAME_IN_DMA,       // "dma": internal DMA RAM, sent as it is, with no copy
    FRAME_IN_STRIPS,    // "strips": PSRAM, copied through two internal strips
} frame_memory_t;

// The settings Python sets through config().
typedef struct {
    uint32_t timeout_ms;        // show() raises when its windows have not ended by then (see config())
    frame_memory_t memory;      // asked for; read at init()
    bool partial;               // send only the tiles that changed, not the whole frame
    uint8_t tile_shift;         // a tile is 1 << tile_shift pixels square
    uint8_t windows;            // past this many windows, show() sends the whole frame instead
    bool swap;                  // swap the two bytes of each pixel on send; needs "strips"
} frame_config_t;

// The marks of one tile: drawn in this frame, drawn in the frame before.
#define FRAME_MARK_NOW 1
#define FRAME_MARK_BEFORE 2

typedef struct {
    frame_config_t config;
    frame_config_t defaults;        // what config(defaults=True) restores
    frame_memory_t memory;          // in use since init(); differs from config.memory after a fallback
    esp_lcd_panel_handle_t panel;
    uint16_t *pixels;               // the framebuffer
    size_t size;                    // its size in bytes
    uint16_t *strips[2];            // internal DMA buffers: the copy of one overlaps the send of the other
    uint16_t strip_rows;            // rows per strip
    size_t strip_size;              // bytes per strip
    volatile int in_flight;         // windows queued and not yet on the wire, changed under lock
    portMUX_TYPE lock;              // guards in_flight between the task and the interrupt
    SemaphoreHandle_t done;         // given by the interrupt each time a window ends
    uint8_t *marks;                 // one byte per tile at the smallest tile size, FRAME_MARK_* bits
    uint16_t grid;                  // tiles per row of marks: the longest axis over the smallest tile
    uint16_t queued;                // windows the last show() queued, strip loads included
    bool tracking;                  // draws mark their tiles; off while Python repaints the still picture
} frame_t;

// A draw changed pixels x0 up to x1 and y0 up to y1, already clipped to the
// frame. Inline, because every pixel a line draws calls it.
static inline void frame_dirty(frame_t *frame, int x0, int y0, int x1, int y1) {
    if (!frame->tracking || x1 <= x0 || y1 <= y0) {
        return;
    }
    int shift = frame->config.tile_shift;
    for (int ty = y0 >> shift; ty <= (y1 - 1) >> shift; ty++) {
        uint8_t *row = frame->marks + ty * frame->grid;
        for (int tx = x0 >> shift; tx <= (x1 - 1) >> shift; tx++) {
            row[tx] |= FRAME_MARK_NOW;
        }
    }
}

void frame_defaults(frame_t *frame, uint16_t strip_rows, bool swap);
void frame_alloc(frame_t *frame, uint16_t width, uint16_t height);
size_t frame_max_transfer(const frame_t *frame);
void frame_attach(frame_t *frame, esp_lcd_panel_handle_t panel);
bool frame_on_color_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
void frame_dirty_all(frame_t *frame);
uint16_t frame_show(frame_t *frame, uint16_t width, uint16_t height);
void frame_free(frame_t *frame);
mp_obj_t frame_config(frame_t *frame, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args);

#endif // __FRAME_H__
