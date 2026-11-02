// The frame: the framebuffer memory and its send to the panel. See frame.h.
//
// A send is one or more windows, each one esp_lcd_panel_draw_bitmap call.
// esp_lcd splits a window into chunks of at most 32 KB and calls
// frame_on_color_done once, after the last chunk. The task counts each window
// up before it queues it, and the interrupt counts it down when it ends, so
// the count says how many windows the panel has still to receive.

#include <string.h>

#include "esp_heap_caps.h"
#include "py/mperrno.h"
#include "py/runtime.h"

#include "frame.h"

#define FRAME_TIMEOUT_MS 100            // default: a full frame takes 31 ms at the real 40 MHz
#define FRAME_TIMEOUT_MAX_MS 60000      // a longer wait would only hide a dead bus
#define FRAME_DMA_CAPS (MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#define FRAME_STRIP_COUNT 2
#define FRAME_ALL_SENT 0                // windows in flight when the panel has the whole frame
#define FRAME_OTHER_STRIP 1             // windows in flight when a strip may be filled again

// The callback runs inside the bus interrupt. If that interrupt lives in IRAM,
// so must the callback, or a flash write that turns the cache off crashes it.
#if CONFIG_SPI_MASTER_ISR_IN_IRAM
#define FRAME_ISR IRAM_ATTR
#else
#define FRAME_ISR
#endif

// Before init(): the default settings and no memory yet.
void frame_defaults(frame_t *frame, uint16_t strip_rows) {
    memset(frame, 0, sizeof(*frame));
    frame->config.timeout_ms = FRAME_TIMEOUT_MS;
    frame->config.memory = FRAME_IN_DMA;
    frame->strip_rows = strip_rows;
    portMUX_INITIALIZE(&frame->lock);
}

static void frame_alloc_strips(frame_t *frame, uint16_t longest_axis) {
    frame->strip_size = (size_t)frame->strip_rows * longest_axis * sizeof(uint16_t);
    for (int i = 0; i < FRAME_STRIP_COUNT; i++) {
        frame->strips[i] = heap_caps_malloc(frame->strip_size, FRAME_DMA_CAPS);
        if (frame->strips[i] == NULL) {
            mp_raise_OSError(MP_ENOMEM);
        }
    }
}

// At init(), before the bus: the framebuffer first, so that it gets the
// largest internal block, then the strips. "dma" falls back to "strips" with a
// notice when the swap needs a copy or no block is large enough.
void frame_alloc(frame_t *frame, uint16_t width, uint16_t height, bool swap) {
    frame->swap = swap;
    frame->size = (size_t)width * height * sizeof(uint16_t);
    frame->memory = frame->config.memory;
    if (frame->memory == FRAME_IN_DMA && swap) {
        mp_printf(&mp_plat_print, "frame: a byte swap needs a copy, so framebuffer=\"strips\"\n");
        frame->memory = FRAME_IN_STRIPS;
    }
    if (frame->memory == FRAME_IN_DMA) {
        frame->pixels = heap_caps_malloc(frame->size, FRAME_DMA_CAPS);
        if (frame->pixels == NULL) {
            mp_printf(&mp_plat_print, "frame: no free internal block of %u bytes, so framebuffer=\"strips\"\n",
                (unsigned)frame->size);
            frame->memory = FRAME_IN_STRIPS;
        }
    }
    if (frame->memory == FRAME_IN_STRIPS) {
        frame->pixels = m_malloc(frame->size);      // MicroPython's heap: PSRAM on this board
    }
    memset(frame->pixels, 0, frame->size);
    frame_alloc_strips(frame, width > height ? width : height);
    frame->in_flight = 0;
    frame->done = xSemaphoreCreateBinary();
    if (frame->done == NULL) {
        mp_raise_OSError(MP_ENOMEM);
    }
}

// The largest single send the bus must take: the whole frame from internal
// RAM, or else one strip.
size_t frame_max_transfer(const frame_t *frame) {
    return frame->memory == FRAME_IN_DMA ? frame->size : frame->strip_size;
}

void frame_attach(frame_t *frame, esp_lcd_panel_handle_t panel) {
    frame->panel = panel;
}

// In the bus interrupt, once per window, after its last chunk. esp_lcd drops
// the return value, and the tick is 10 ms, so the yield happens here: without
// it the waiting task wakes up to one tick late.
FRAME_ISR bool frame_on_color_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    frame_t *frame = user_ctx;
    BaseType_t woken = pdFALSE;
    portENTER_CRITICAL_ISR(&frame->lock);
    frame->in_flight--;
    portEXIT_CRITICAL_ISR(&frame->lock);
    xSemaphoreGiveFromISR(frame->done, &woken);
    portYIELD_FROM_ISR(woken);
    return woken == pdTRUE;
}

static void frame_count(frame_t *frame, int change) {
    portENTER_CRITICAL(&frame->lock);
    frame->in_flight += change;
    portEXIT_CRITICAL(&frame->lock);
}

// Rounded up, so that a 1 ms timeout waits one tick and not zero.
static TickType_t frame_ticks(uint32_t ms) {
    return (ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
}

// True when at most `most` windows are in flight within the timeout. The
// interrupt gives the semaphore after every window, and a give can be left
// over from an earlier frame, so the count decides, never the give alone.
static bool frame_wait(frame_t *frame, int most) {
    TickType_t limit = frame_ticks(frame->config.timeout_ms);
    TickType_t start = xTaskGetTickCount();
    while (frame->in_flight > most) {
        TickType_t spent = xTaskGetTickCount() - start;
        bool given = spent < limit && xSemaphoreTake(frame->done, limit - spent) == pdTRUE;
        if (!given && frame->in_flight > most) {
            return false;
        }
    }
    return true;
}

static void frame_wait_or_raise(frame_t *frame, int most) {
    if (!frame_wait(frame, most)) {
        mp_raise_OSError(MP_ETIMEDOUT);
    }
}

// Queue one window: rows y0 up to y1. The count goes up first, because the
// interrupt can end the window before draw_bitmap returns.
static void frame_send(frame_t *frame, uint16_t width, uint16_t y0, uint16_t y1, const uint16_t *src) {
    frame_count(frame, +1);
    esp_err_t err = esp_lcd_panel_draw_bitmap(frame->panel, 0, y0, width, y1, src);
    if (err != ESP_OK) {
        frame_count(frame, -1);
        mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("frame: draw_bitmap failed: %s"), esp_err_to_name(err));
    }
}

static void frame_copy(const frame_t *frame, uint16_t *dst, const uint16_t *src, size_t count) {
    if (!frame->swap) {
        memcpy(dst, src, count * sizeof(uint16_t));
        return;
    }
    for (size_t i = 0; i < count; i++) {
        dst[i] = __builtin_bswap16(src[i]);
    }
}

// The PSRAM frame, strip by strip: the copy of one strip overlaps the send of
// the other. Windows end in order, so when at most one is in flight, the
// window that used this strip two steps back has ended.
static void frame_send_strips(frame_t *frame, uint16_t width, uint16_t height) {
    const uint16_t *src = frame->pixels;
    for (uint16_t y = 0, k = 0; y < height; y += frame->strip_rows, k++) {
        uint16_t rows = height - y < frame->strip_rows ? height - y : frame->strip_rows;
        size_t count = (size_t)rows * width;
        uint16_t *strip = frame->strips[k % FRAME_STRIP_COUNT];
        frame_wait_or_raise(frame, FRAME_OTHER_STRIP);
        frame_copy(frame, strip, src, count);
        frame_send(frame, width, y, y + rows, strip);
        src += count;
    }
}

// Send the whole framebuffer, and return when the panel has all of it.
void frame_show(frame_t *frame, uint16_t width, uint16_t height) {
    // A window of a frame that timed out can still end now. Wait for it first,
    // so that it cannot count as a window of this frame.
    frame_wait_or_raise(frame, FRAME_ALL_SENT);
    if (frame->memory == FRAME_IN_DMA) {
        frame_send(frame, width, 0, height, frame->pixels);
    } else {
        frame_send_strips(frame, width, height);
    }
    frame_wait_or_raise(frame, FRAME_ALL_SENT);
}

// At deinit() or at a soft reset, through the finalizer, so it never raises.
// It waits for the last window first, because DMA can still read the memory.
// If the bus is dead, that wait times out and the memory is freed anyway.
void frame_free(frame_t *frame) {
    if (frame->done != NULL) {
        frame_wait(frame, FRAME_ALL_SENT);
        vSemaphoreDelete(frame->done);
        frame->done = NULL;
    }
    if (frame->pixels != NULL) {
        if (frame->memory == FRAME_IN_DMA) {
            heap_caps_free(frame->pixels);
        } else {
            m_free(frame->pixels);
        }
        frame->pixels = NULL;
    }
    for (int i = 0; i < FRAME_STRIP_COUNT; i++) {
        heap_caps_free(frame->strips[i]);
        frame->strips[i] = NULL;
    }
}

static uint32_t frame_parse_timeout(mp_obj_t value) {
    mp_int_t ms = mp_obj_get_int(value);
    if (ms < 1 || ms > FRAME_TIMEOUT_MAX_MS) {
        mp_raise_ValueError(MP_ERROR_TEXT("timeout must be 1 to 60000 ms"));
    }
    return (uint32_t)ms;
}

// After init() the memory is fixed. The value in use, or the one asked for,
// is still accepted, so that config(**config()) works.
static frame_memory_t frame_parse_memory(const frame_t *frame, mp_obj_t value) {
    qstr name = mp_obj_str_get_qstr(value);
    frame_memory_t memory;
    if (name == MP_QSTR_dma) {
        memory = FRAME_IN_DMA;
    } else if (name == MP_QSTR_strips) {
        memory = FRAME_IN_STRIPS;
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("framebuffer must be \"dma\" or \"strips\""));
    }
    bool fixed = frame->pixels != NULL && memory != frame->memory && memory != frame->config.memory;
    if (fixed) {
        mp_raise_ValueError(MP_ERROR_TEXT("set framebuffer before init()"));
    }
    return memory;
}

// After init() it reports the memory in use, which differs after a fallback.
static mp_obj_t frame_config_dict(const frame_t *frame) {
    frame_memory_t memory = frame->pixels != NULL ? frame->memory : frame->config.memory;
    mp_obj_t dict = mp_obj_new_dict(0);
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_timeout), mp_obj_new_int(frame->config.timeout_ms));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_framebuffer),
        MP_OBJ_NEW_QSTR(memory == FRAME_IN_DMA ? MP_QSTR_dma : MP_QSTR_strips));
    return dict;
}

// config(**settings): check every value given, then apply them all, so that a
// bad one changes nothing. Returns every setting as a dict.
mp_obj_t frame_config(frame_t *frame, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_timeout, ARG_framebuffer };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_framebuffer, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    frame_config_t next = frame->config;
    if (args[ARG_timeout].u_obj != MP_OBJ_NULL) {
        next.timeout_ms = frame_parse_timeout(args[ARG_timeout].u_obj);
    }
    if (args[ARG_framebuffer].u_obj != MP_OBJ_NULL) {
        next.memory = frame_parse_memory(frame, args[ARG_framebuffer].u_obj);
    }
    frame->config = next;
    return frame_config_dict(frame);
}
