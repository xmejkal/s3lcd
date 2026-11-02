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
    uint32_t timeout_ms;        // the longest time one show() may take; past it show() raises
    frame_memory_t memory;      // asked for; read at init()
} frame_config_t;

typedef struct {
    frame_config_t config;
    frame_memory_t memory;          // in use since init(); differs from config.memory after a fallback
    esp_lcd_panel_handle_t panel;
    bool swap;                      // swap the two bytes of each pixel on send (strips only)
    uint16_t *pixels;               // the framebuffer
    size_t size;                    // its size in bytes
    uint16_t *strips[2];            // internal DMA buffers: the copy of one overlaps the send of the other
    uint16_t strip_rows;            // rows per strip
    size_t strip_size;              // bytes per strip
    volatile int in_flight;         // windows queued and not yet on the wire, changed under lock
    portMUX_TYPE lock;              // guards in_flight between the task and the interrupt
    SemaphoreHandle_t done;         // given by the interrupt each time a window ends
} frame_t;

void frame_defaults(frame_t *frame, uint16_t strip_rows);
void frame_alloc(frame_t *frame, uint16_t width, uint16_t height, bool swap);
size_t frame_max_transfer(const frame_t *frame);
void frame_attach(frame_t *frame, esp_lcd_panel_handle_t panel);
bool frame_on_color_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
void frame_show(frame_t *frame, uint16_t width, uint16_t height);
void frame_free(frame_t *frame);
mp_obj_t frame_config(frame_t *frame, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args);

#endif // __FRAME_H__
