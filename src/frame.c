// The frame: the framebuffer memory and its send to the panel. See frame.h.
//
// A send is one or more windows, each one esp_lcd_panel_draw_bitmap call.
// esp_lcd splits a window into chunks of at most 32 KB and calls
// frame_on_color_done once, after the last chunk. The task counts each window
// up before it queues it, and the interrupt counts it down when it ends, so
// the count says how many windows the panel has still to receive.
//
// With partial on, show() sends only the tiles drawn in this frame or the one
// before, as rectangles. Draws mark their tiles through frame_dirty().

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
#define FRAME_TILE_SHIFT 4              // default tile: 16 px, as in config(tile=16)
#define FRAME_TILE_SHIFT_MIN 3          // tile=8, the smallest; the marks grid is sized for it
#define FRAME_TILE_SHIFT_MAX 5          // tile=32
#define FRAME_WINDOWS 40                // default: a window costs about 60-120 us of bus setup
#define FRAME_WINDOWS_MAX 64            // the size of show()'s window list, on the stack

// The callback runs inside the bus interrupt. If that interrupt lives in IRAM,
// so must the callback, or a flash write that turns the cache off crashes it.
#if CONFIG_SPI_MASTER_ISR_IN_IRAM
#define FRAME_ISR IRAM_ATTR
#else
#define FRAME_ISR
#endif

// portYIELD_FROM_ISR calls this trace hook. FreeRTOS.h gives its empty default
// only under ESP_PLATFORM, which MicroPython's user C modules do not define.
#ifndef traceISR_EXIT_TO_SCHEDULER
#define traceISR_EXIT_TO_SCHEDULER()
#endif

// Before init(): the default settings and no memory yet.
void frame_defaults(frame_t *frame, uint16_t strip_rows) {
    memset(frame, 0, sizeof(*frame));
    frame->config.timeout_ms = FRAME_TIMEOUT_MS;
    frame->config.memory = FRAME_IN_DMA;
    frame->config.tile_shift = FRAME_TILE_SHIFT;
    frame->config.windows = FRAME_WINDOWS;
    frame->strip_rows = strip_rows;
    frame->tracking = true;
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

// Square, so that a rotation needs no new grid; internal RAM, because every
// draw writes it.
static void frame_alloc_marks(frame_t *frame, uint16_t longest_axis) {
    int smallest = 1 << FRAME_TILE_SHIFT_MIN;
    frame->grid = (longest_axis + smallest - 1) / smallest;
    frame->marks = heap_caps_malloc((size_t)frame->grid * frame->grid, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (frame->marks == NULL) {
        mp_raise_OSError(MP_ENOMEM);
    }
    frame_dirty_all(frame);
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
    uint16_t longest_axis = width > height ? width : height;
    frame_alloc_strips(frame, longest_axis);
    frame_alloc_marks(frame, longest_axis);
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

// True when at most `most` windows are in flight before the timeout, counted
// from `start`: one show() has one deadline, however many waits it makes. The
// interrupt gives the semaphore after every window, and a give can be left
// over from an earlier frame, so the count decides, never the give alone.
static bool frame_wait(frame_t *frame, int most, TickType_t start) {
    TickType_t limit = frame_ticks(frame->config.timeout_ms);
    while (frame->in_flight > most) {
        TickType_t spent = xTaskGetTickCount() - start;
        bool given = spent < limit && xSemaphoreTake(frame->done, limit - spent) == pdTRUE;
        if (!given && frame->in_flight > most) {
            return false;
        }
    }
    return true;
}

static void frame_wait_or_raise(frame_t *frame, int most, TickType_t start) {
    if (!frame_wait(frame, most, start)) {
        mp_raise_OSError(MP_ETIMEDOUT);
    }
}

// Pixels x0 up to x1 and rows y0 up to y1 of the frame.
typedef struct {
    uint16_t x0, y0, x1, y1;
} frame_rect_t;

// Tile columns c0 up to c1 and tile rows r0 up to r1.
typedef struct {
    uint8_t c0, c1, r0, r1;
} frame_window_t;

// One show(): the frame's width in this rotation, the start of its one
// deadline, and how many strips it filled so far, across windows.
typedef struct {
    frame_t *frame;
    uint16_t width;
    TickType_t start;
    unsigned turn;
} frame_send_t;

// Queue one window. The count goes up first, because the interrupt can end
// the window before draw_bitmap returns.
static void frame_queue(frame_t *frame, frame_rect_t rect, const uint16_t *src) {
    frame_count(frame, +1);
    frame->queued++;
    esp_err_t err = esp_lcd_panel_draw_bitmap(frame->panel, rect.x0, rect.y0, rect.x1, rect.y1, src);
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

// The rectangle through the strips, as many rows at a time as a strip holds:
// the copy of one strip overlaps the send of the other. Windows end in order,
// so when at most one is in flight, the window that used this strip two
// turns back has ended.
static void frame_send_strips(frame_send_t *send, frame_rect_t rect) {
    frame_t *frame = send->frame;
    uint16_t w = rect.x1 - rect.x0;
    uint16_t step = frame->strip_size / sizeof(uint16_t) / w;     // rows per strip
    for (uint16_t y = rect.y0; y < rect.y1; y += step) {
        uint16_t rows = rect.y1 - y < step ? rect.y1 - y : step;
        uint16_t *strip = frame->strips[send->turn++ % FRAME_STRIP_COUNT];
        frame_wait_or_raise(frame, FRAME_OTHER_STRIP, send->start);
        const uint16_t *src = frame->pixels + (size_t)y * send->width + rect.x0;
        for (uint16_t r = 0; r < rows; r++) {
            frame_copy(frame, strip + (size_t)r * w, src + (size_t)r * send->width, w);
        }
        frame_queue(frame, (frame_rect_t){rect.x0, y, rect.x1, y + rows}, strip);
    }
}

// Whole rows of an internal framebuffer are one block of memory, so DMA reads
// them where they are. Anything else goes through the strips.
static void frame_send_rect(frame_send_t *send, frame_rect_t rect) {
    frame_t *frame = send->frame;
    bool whole_rows = rect.x0 == 0 && rect.x1 == send->width;
    if (frame->memory == FRAME_IN_DMA && whole_rows) {
        frame_queue(frame, rect, frame->pixels + (size_t)rect.y0 * send->width);
    } else {
        frame_send_strips(send, rect);
    }
}

// A tile goes out when it was drawn in this frame or in the one before: a
// tile that held a moving object last frame holds only the still picture now,
// and the panel still shows the object.
static bool frame_tile_changed(const frame_t *frame, int tx, int ty) {
    return frame->marks[ty * frame->grid + tx] != 0;
}

// Grow the window that ends just above this run and has the same columns, or
// add a new one. Returns the new count, or -1 past the windows setting.
static int frame_add_run(const frame_t *frame, frame_window_t *windows, int count, int c0, int c1, int r) {
    for (int i = 0; i < count; i++) {
        if (windows[i].r1 == r && windows[i].c0 == c0 && windows[i].c1 == c1) {
            windows[i].r1 = r + 1;
            return count;
        }
    }
    if (count == frame->config.windows) {
        return -1;
    }
    windows[count] = (frame_window_t){c0, c1, r, r + 1};
    return count + 1;
}

// The changed tiles as windows, top to bottom: each run of changed tiles along
// a tile row, joined to the window above when the columns match. Returns the
// count, or -1 when the whole frame is the cheaper send.
static int frame_find_windows(const frame_t *frame, frame_window_t *windows, int cols, int rows) {
    int count = 0;
    for (int r = 0; r < rows && count >= 0; r++) {
        int c = 0;
        while (c < cols && count >= 0) {
            if (!frame_tile_changed(frame, c, r)) {
                c++;
                continue;
            }
            int c0 = c;
            while (c < cols && frame_tile_changed(frame, c, r)) {
                c++;
            }
            count = frame_add_run(frame, windows, count, c0, c, r);
        }
    }
    return count;
}

static uint16_t frame_min(uint16_t a, uint16_t b) {
    return a < b ? a : b;
}

// A window in pixels; the last tile of a row or column can stick out of the frame.
static frame_rect_t frame_window_rect(const frame_t *frame, frame_window_t win, uint16_t width, uint16_t height) {
    int shift = frame->config.tile_shift;
    return (frame_rect_t){
        win.c0 << shift, win.r0 << shift,
        frame_min(win.c1 << shift, width), frame_min(win.r1 << shift, height),
    };
}

// After a show(): this frame's marks become the frame before's, and the next
// frame starts unmarked.
static void frame_age_marks(frame_t *frame) {
    size_t count = (size_t)frame->grid * frame->grid;
    for (size_t i = 0; i < count; i++) {
        frame->marks[i] = (frame->marks[i] & FRAME_MARK_NOW) ? FRAME_MARK_BEFORE : 0;
    }
}

// Every tile changed: at init(), a rotation, a fill(), a clear(), and any draw
// that writes the framebuffer outside _setpixel, _fill_rect and blit_buffer.
// It marks even with tracking off, so such a draw is never left unsent.
void frame_dirty_all(frame_t *frame) {
    if (frame->marks == NULL) {     // before init(): nothing to mark yet
        return;
    }
    memset(frame->marks, FRAME_MARK_NOW, (size_t)frame->grid * frame->grid);
}

// Send what changed, or the whole frame, and return when the panel has all of
// it. Returns the count of windows queued: each costs bus setup.
uint16_t frame_show(frame_t *frame, uint16_t width, uint16_t height) {
    frame_send_t send = {frame, width, xTaskGetTickCount(), 0};
    frame->queued = 0;
    // A window of a frame that timed out can still end now. Wait for it first,
    // so that it cannot count as a window of this frame.
    frame_wait_or_raise(frame, FRAME_ALL_SENT, send.start);
    int shift = frame->config.tile_shift;
    int cols = (width + (1 << shift) - 1) >> shift;
    int rows = (height + (1 << shift) - 1) >> shift;
    frame_window_t windows[FRAME_WINDOWS_MAX];
    int count = frame->config.partial ? frame_find_windows(frame, windows, cols, rows) : -1;
    if (count < 0) {
        frame_send_rect(&send, (frame_rect_t){0, 0, width, height});
    }
    for (int i = 0; i < count; i++) {
        frame_send_rect(&send, frame_window_rect(frame, windows[i], width, height));
    }
    frame_wait_or_raise(frame, FRAME_ALL_SENT, send.start);
    frame_age_marks(frame);
    return frame->queued;
}

// At deinit() or at a soft reset, through the finalizer, so it never raises.
// It waits for the last window first, because DMA can still read the memory.
// If the bus is dead, that wait times out and the memory is freed anyway.
void frame_free(frame_t *frame) {
    if (frame->done != NULL) {
        frame_wait(frame, FRAME_ALL_SENT, xTaskGetTickCount());
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
    heap_caps_free(frame->marks);
    frame->marks = NULL;
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

// 8, 16 or 32 px, kept as its shift, so a mark costs no division.
static uint8_t frame_parse_tile(mp_obj_t value) {
    mp_int_t tile = mp_obj_get_int(value);
    for (int shift = FRAME_TILE_SHIFT_MIN; shift <= FRAME_TILE_SHIFT_MAX; shift++) {
        if (tile == 1 << shift) {
            return shift;
        }
    }
    mp_raise_ValueError(MP_ERROR_TEXT("tile must be 8, 16 or 32"));
}

static uint8_t frame_parse_windows(mp_obj_t value) {
    mp_int_t windows = mp_obj_get_int(value);
    if (windows < 1 || windows > FRAME_WINDOWS_MAX) {
        mp_raise_ValueError(MP_ERROR_TEXT("windows must be 1 to 64"));
    }
    return (uint8_t)windows;
}

// After init() it reports the memory in use, which differs after a fallback.
static mp_obj_t frame_config_dict(const frame_t *frame) {
    frame_memory_t memory = frame->pixels != NULL ? frame->memory : frame->config.memory;
    mp_obj_t dict = mp_obj_new_dict(0);
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_timeout), mp_obj_new_int(frame->config.timeout_ms));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_framebuffer),
        MP_OBJ_NEW_QSTR(memory == FRAME_IN_DMA ? MP_QSTR_dma : MP_QSTR_strips));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_partial), mp_obj_new_bool(frame->config.partial));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_tile), mp_obj_new_int(1 << frame->config.tile_shift));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_windows), mp_obj_new_int(frame->config.windows));
    return dict;
}

// config(**settings): check every value given, then apply them all, so that a
// bad one changes nothing. Returns every setting as a dict.
mp_obj_t frame_config(frame_t *frame, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_timeout, ARG_framebuffer, ARG_partial, ARG_tile, ARG_windows };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_framebuffer, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_partial, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_tile, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_windows, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
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
    if (args[ARG_partial].u_obj != MP_OBJ_NULL) {
        next.partial = mp_obj_is_true(args[ARG_partial].u_obj);
    }
    if (args[ARG_tile].u_obj != MP_OBJ_NULL) {
        next.tile_shift = frame_parse_tile(args[ARG_tile].u_obj);
    }
    if (args[ARG_windows].u_obj != MP_OBJ_NULL) {
        next.windows = frame_parse_windows(args[ARG_windows].u_obj);
    }
    // Marks made at the old tile size point at the wrong tiles.
    bool retiled = next.tile_shift != frame->config.tile_shift;
    frame->config = next;
    if (retiled) {
        frame_dirty_all(frame);
    }
    return frame_config_dict(frame);
}
