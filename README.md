# s3lcd (fork)

A MicroPython display driver for the ESP32-S3, forked from [russhughes/s3lcd](https://github.com/russhughes/s3lcd)
at `f254daf5` (MIT, see `LICENSE`). [pixel-engine](https://github.com/xmejkal/pixel-engine) uses it as its
`driver/` submodule, and [pixel-games](https://github.com/xmejkal/pixel-games) builds its firmware from both.

## What the fork changes

| file | change |
| --- | --- |
| `src/frame.c`, `src/frame.h` | a frame buffer sent by DMA, with partial updates: only the tiles drawn in this frame or the one before go to the panel (P556) |
| `src/s3lcd.c` | builds on MicroPython v1.29, a key colour in `blit_buffer`, pictures in PSRAM outside the collected heap (P562) |
| `src/blend565.h` | the RGB565 alpha blend in the panel's byte order, plain C, shared with the engine's fire (P603) |

The history before this repository lives in pixel-games. A PR number in a commit message (for example `#18`) is a
pixel-games PR.

## Build

The firmware build passes `src/micropython.cmake` to MicroPython as a user C module:

```sh
make BOARD=ESP32_GENERIC_S3 BOARD_VARIANT=SPIRAM_OCT USER_C_MODULES=/path/to/s3lcd/src/micropython.cmake
```

pixel-games' `build.sh` does this for you.
