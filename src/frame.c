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
//
// With framebuffer="double", the frame is in PSRAM and show() copies the
// windows into `outgoing`, an internal buffer, queues them and returns. The
// next frame is drawn while they go out; the next show() waits for them first.

#include <string.h>

#include "esp_heap_caps.h"
#include "py/mperrno.h"
#include "py/runtime.h"

#include "frame.h"

#define FRAME_TIMEOUT_MS 100            // default: a full frame takes 31 ms at the real 40 MHz
#define FRAME_TIMEOUT_MAX_MS 60000      // a longer wait would only hide a dead bus
#define FRAME_DMA_CAPS (MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#define FRAME_PSRAM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)     // outside MicroPython's heap: gc never scans it
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

// Before init(): the default settings and no memory yet. The swap's default
// is the bus's swap_color_bytes flag; config(swap=...) overrides it.
void frame_defaults(frame_t *frame, uint16_t strip_rows, bool swap) {
    memset(frame, 0, sizeof(*frame));
    frame->config.timeout_ms = FRAME_TIMEOUT_MS;
    frame->config.memory = FRAME_IN_DMA;
    frame->config.tile_shift = FRAME_TILE_SHIFT;
    frame->config.windows = FRAME_WINDOWS;
    frame->config.swap = swap;
    frame->defaults = frame->config;
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

// At init(), before the bus: the framebuffer (or "double"'s outgoing buffer)
// first, so that it gets the largest internal block, then the strips. "dma"
// falls back to "strips" with a notice when the swap needs a copy or no block
// is large enough, and so does "double" when no block is large enough.
void frame_alloc(frame_t *frame, uint16_t width, uint16_t height) {
    if (frame->pixels != NULL) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("init() again: call deinit() first"));
    }
    frame->size = (size_t)width * height * sizeof(uint16_t);
    frame->memory = frame->config.memory;
    if (frame->memory == FRAME_IN_DMA && frame->config.swap) {
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
    if (frame->memory == FRAME_DOUBLE) {
        frame->outgoing = heap_caps_malloc(frame->size, FRAME_DMA_CAPS);
        if (frame->outgoing == NULL) {
            mp_printf(&mp_plat_print, "frame: no free internal block of %u bytes, so framebuffer=\"strips\"\n",
                (unsigned)frame->size);
            frame->memory = FRAME_IN_STRIPS;
        } else {
            frame->pixels = heap_caps_malloc(frame->size, FRAME_PSRAM_CAPS);
        }
        if (frame->outgoing != NULL && frame->pixels == NULL) {
            heap_caps_free(frame->outgoing);
            frame->outgoing = NULL;     // or the next frame_free() frees it again
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("frame: no PSRAM for the frame"));
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
// RAM or from `outgoing`, or else one strip. The i80 bus reads it too, untested
// with "double".
size_t frame_max_transfer(const frame_t *frame) {
    return frame->memory == FRAME_IN_STRIPS ? frame->strip_size : frame->size;
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
    if (frame->done != NULL) {      // deinit() deletes the bus first; this guards any other order
        xSemaphoreGiveFromISR(frame->done, &woken);
    }
    portYIELD_FROM_ISR(woken);
    return woken == pdTRUE;
}

static void frame_count(frame_t *frame, int change) {
    portENTER_CRITICAL(&frame->lock);
    frame->in_flight += change;
    portEXIT_CRITICAL(&frame->lock);
}

// Rounded up, plus one tick: the start is read part way through a tick, so
// without it a 1 ms timeout could end before any time passed.
static TickType_t frame_ticks(uint32_t ms) {
    return (ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS + 1;
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
// deadline, how many strips it filled so far, across windows, and with
// "double" how many pixels it packed into `outgoing` so far.
typedef struct {
    frame_t *frame;
    uint16_t width;
    TickType_t start;
    unsigned turn;
    size_t packed;
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
    if (!frame->config.swap) {
        memcpy(dst, src, count * sizeof(uint16_t));
        return;
    }
    for (size_t i = 0; i < count; i++) {
        dst[i] = __builtin_bswap16(src[i]);
    }
}

// The rectangle's rows, packed into dst, then queued from there. Whole rows
// are one block in the frame, so they are one copy.
static void frame_load(frame_send_t *send, frame_rect_t rect, uint16_t *dst) {
    frame_t *frame = send->frame;
    uint16_t w = rect.x1 - rect.x0;
    uint16_t rows = rect.y1 - rect.y0;
    const uint16_t *src = frame->pixels + (size_t)rect.y0 * send->width + rect.x0;
    if (w == send->width) {
        frame_copy(frame, dst, src, (size_t)w * rows);
    } else {
        for (uint16_t r = 0; r < rows; r++) {
            frame_copy(frame, dst + (size_t)r * w, src + (size_t)r * send->width, w);
        }
    }
    frame_queue(frame, rect, dst);
}

// The rectangle through the strips, as many rows at a time as a strip holds:
// the copy of one strip overlaps the send of the other. Windows end in order,
// so when at most one is in flight, the window that used this strip two
// turns back has ended. esp_lcd itself also waits for the last window before
// it starts the next, so this wait is the guard, not the pace.
static void frame_send_strips(frame_send_t *send, frame_rect_t rect) {
    frame_t *frame = send->frame;
    uint16_t step = frame->strip_size / sizeof(uint16_t) / (rect.x1 - rect.x0);     // rows per strip
    for (uint16_t y = rect.y0; y < rect.y1; y += step) {
        uint16_t rows = rect.y1 - y < step ? rect.y1 - y : step;
        uint16_t *strip = frame->strips[send->turn++ % FRAME_STRIP_COUNT];
        frame_wait_or_raise(frame, FRAME_OTHER_STRIP, send->start);
        frame_load(send, (frame_rect_t){rect.x0, y, rect.x1, y + rows}, strip);
    }
}

// "double": the rectangle into `outgoing`, after the windows this show()
// packed before it, so the copy of one window runs while the one before is
// sent. Windows never share a tile, so they fit in one frame's size. Nothing
// waits here: the first wait of show() made sure the last frame was out.
static void frame_send_packed(frame_send_t *send, frame_rect_t rect) {
    frame_load(send, rect, send->frame->outgoing + send->packed);
    send->packed += (size_t)(rect.x1 - rect.x0) * (rect.y1 - rect.y0);
}

// Whole rows of an internal framebuffer are one block of memory, so DMA reads
// them where they are. "double" packs into `outgoing`. Anything else goes
// through the strips.
static void frame_send_rect(frame_send_t *send, frame_rect_t rect) {
    frame_t *frame = send->frame;
    bool whole_rows = rect.x0 == 0 && rect.x1 == send->width;
    if (frame->memory == FRAME_DOUBLE) {
        frame_send_packed(send, rect);
    } else if (frame->memory == FRAME_IN_DMA && whole_rows) {
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
// it. With "double", return once it is queued: the next show() waits for it.
// Returns the count of windows queued: each costs bus setup.
uint16_t frame_show(frame_t *frame, uint16_t width, uint16_t height) {
    frame_send_t send = {frame, width, xTaskGetTickCount(), 0, 0};
    frame->queued = 0;
    // A window of a frame that timed out can still end now, and with "double"
    // the last frame is usually still going out. Wait for it first, so that it
    // cannot count as a window of this frame and `outgoing` is free to fill.
    // So with "double" the timeout covers the last send and this show(), and a
    // dead bus raises one show() late.
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
    if (frame->memory != FRAME_DOUBLE) {
        frame_wait_or_raise(frame, FRAME_ALL_SENT, send.start);
    }
    frame_age_marks(frame);
    return frame->queued;
}

// At deinit() or at a soft reset, through the finalizer, so it never raises.
// deinit() deletes the bus first, which waits for the windows on the wire.
// The wait here guards any other caller; on a dead bus it times out and the
// memory is freed anyway. The handle is cleared before the delete, because
// the interrupt gives only to a handle that is not NULL.
void frame_free(frame_t *frame) {
    if (frame->done != NULL) {
        frame_wait(frame, FRAME_ALL_SENT, xTaskGetTickCount());
        SemaphoreHandle_t done = frame->done;
        frame->done = NULL;
        vSemaphoreDelete(done);
    }
    if (frame->pixels != NULL) {
        if (frame->memory == FRAME_IN_STRIPS) {
            m_free(frame->pixels);
        } else {
            heap_caps_free(frame->pixels);
        }
        frame->pixels = NULL;
    }
    heap_caps_free(frame->outgoing);
    frame->outgoing = NULL;
    for (int i = 0; i < FRAME_STRIP_COUNT; i++) {
        heap_caps_free(frame->strips[i]);
        frame->strips[i] = NULL;
    }
    heap_caps_free(frame->marks);
    frame->marks = NULL;
}

// The settings config() knows, in the order of its dict.
static const qstr frame_keys[] = {
    MP_QSTR_timeout, MP_QSTR_framebuffer, MP_QSTR_partial, MP_QSTR_tile, MP_QSTR_windows, MP_QSTR_swap,
    MP_QSTR_defaults,
};

// An unknown key names itself: mp_arg_parse_all's own error does not, in
// this build's terse error text.
static void frame_check_keys(const mp_map_t *kw_args) {
    for (size_t i = 0; i < kw_args->alloc; i++) {
        if (!mp_map_slot_is_filled(kw_args, i)) {
            continue;
        }
        qstr key = mp_obj_str_get_qstr(kw_args->table[i].key);
        bool known = false;
        for (size_t k = 0; k < MP_ARRAY_SIZE(frame_keys); k++) {
            known = known || key == frame_keys[k];
        }
        if (!known) {
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("config() has no setting '%q'"), key);
        }
    }
}

// An int in lo to hi, or a ValueError that names the key and the range.
static mp_int_t frame_parse_int(mp_obj_t value, qstr key, mp_int_t lo, mp_int_t hi) {
    mp_int_t n = lo - 1;
    if (mp_obj_is_int(value) && !mp_obj_is_bool(value)) {
        n = mp_obj_get_int(value);
    }
    if (n < lo || n > hi) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("%q must be %d to %d"), key, (int)lo, (int)hi);
    }
    return n;
}

static bool frame_parse_bool(mp_obj_t value, mp_rom_error_text_t message) {
    if (!mp_obj_is_bool(value)) {
        mp_raise_ValueError(message);
    }
    return value == mp_const_true;
}

// After init() the memory is fixed. The value in use, or the one asked for,
// is still accepted, so that config(**config()) works.
static frame_memory_t frame_parse_memory(const frame_t *frame, mp_obj_t value) {
    qstr name = mp_obj_is_str(value) ? mp_obj_str_get_qstr(value) : MP_QSTRnull;
    frame_memory_t memory;
    if (name == MP_QSTR_dma) {
        memory = FRAME_IN_DMA;
    } else if (name == MP_QSTR_strips) {
        memory = FRAME_IN_STRIPS;
    } else if (name == MP_QSTR_double) {
        memory = FRAME_DOUBLE;
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("framebuffer must be \"dma\", \"strips\" or \"double\""));
    }
    bool fixed = frame->pixels != NULL && memory != frame->memory && memory != frame->config.memory;
    if (fixed) {
        mp_raise_ValueError(MP_ERROR_TEXT("set framebuffer before init()"));
    }
    return memory;
}

// 8, 16 or 32 px, kept as its shift, so a mark costs no division.
static uint8_t frame_parse_tile(mp_obj_t value) {
    mp_int_t tile = mp_obj_is_int(value) && !mp_obj_is_bool(value) ? mp_obj_get_int(value) : 0;
    for (int shift = FRAME_TILE_SHIFT_MIN; shift <= FRAME_TILE_SHIFT_MAX; shift++) {
        if (tile == 1 << shift) {
            return shift;
        }
    }
    mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("tile must be %d, %d or %d"),
        1 << FRAME_TILE_SHIFT_MIN, 1 << (FRAME_TILE_SHIFT_MIN + 1), 1 << FRAME_TILE_SHIFT_MAX);
}

// An internal framebuffer goes out with no copy, so nothing can swap it.
// Checked on the result, so that defaults=True cannot bring a swap back.
static void frame_check_swap(const frame_t *frame, const frame_config_t *next) {
    if (next->swap && frame->pixels != NULL && frame->memory == FRAME_IN_DMA) {
        mp_raise_ValueError(MP_ERROR_TEXT("swap needs framebuffer=\"strips\" or \"double\"; set both before init()"));
    }
}

// Each value given, checked; the rest as they are, or the defaults first.
static frame_config_t frame_parse(const frame_t *frame, const mp_arg_val_t *args) {
    enum { ARG_timeout, ARG_framebuffer, ARG_partial, ARG_tile, ARG_windows, ARG_swap, ARG_defaults };
    bool restore = args[ARG_defaults].u_obj != MP_OBJ_NULL
        && frame_parse_bool(args[ARG_defaults].u_obj, MP_ERROR_TEXT("defaults must be True or False"));
    frame_config_t next = restore ? frame->defaults : frame->config;
    if (args[ARG_timeout].u_obj != MP_OBJ_NULL) {
        next.timeout_ms = frame_parse_int(args[ARG_timeout].u_obj, MP_QSTR_timeout, 1, FRAME_TIMEOUT_MAX_MS);
    }
    if (args[ARG_framebuffer].u_obj != MP_OBJ_NULL) {
        next.memory = frame_parse_memory(frame, args[ARG_framebuffer].u_obj);
    }
    if (args[ARG_partial].u_obj != MP_OBJ_NULL) {
        next.partial = frame_parse_bool(args[ARG_partial].u_obj, MP_ERROR_TEXT("partial must be True or False"));
    }
    if (args[ARG_tile].u_obj != MP_OBJ_NULL) {
        next.tile_shift = frame_parse_tile(args[ARG_tile].u_obj);
    }
    if (args[ARG_windows].u_obj != MP_OBJ_NULL) {
        next.windows = frame_parse_int(args[ARG_windows].u_obj, MP_QSTR_windows, 1, FRAME_WINDOWS_MAX);
    }
    if (args[ARG_swap].u_obj != MP_OBJ_NULL) {
        next.swap = frame_parse_bool(args[ARG_swap].u_obj, MP_ERROR_TEXT("swap must be True or False"));
    }
    frame_check_swap(frame, &next);
    return next;
}

// The names of frame_memory_t, in its order.
static const qstr frame_memory_names[] = {MP_QSTR_dma, MP_QSTR_strips, MP_QSTR_double};

// After init() it reports the memory in use, which differs after a fallback.
static mp_obj_t frame_config_dict(const frame_t *frame) {
    frame_memory_t memory = frame->pixels != NULL ? frame->memory : frame->config.memory;
    mp_obj_t dict = mp_obj_new_dict(0);
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_timeout), mp_obj_new_int(frame->config.timeout_ms));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_framebuffer),
        MP_OBJ_NEW_QSTR(frame_memory_names[memory]));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_partial), mp_obj_new_bool(frame->config.partial));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_tile), mp_obj_new_int(1 << frame->config.tile_shift));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_windows), mp_obj_new_int(frame->config.windows));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_swap), mp_obj_new_bool(frame->config.swap));
    return dict;
}

// config(**settings): check every value given, then apply them all, so that a
// bad one changes nothing. Returns every setting as a dict.
mp_obj_t frame_config(frame_t *frame, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_framebuffer, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_partial, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_tile, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_windows, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_swap, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_defaults, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
    };
    frame_check_keys(kw_args);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    frame_config_t next = frame_parse(frame, args);
    // Marks made at the old tile size point at the wrong tiles.
    bool retiled = next.tile_shift != frame->config.tile_shift;
    frame->config = next;
    if (retiled) {
        frame_dirty_all(frame);
    }
    return frame_config_dict(frame);
}
