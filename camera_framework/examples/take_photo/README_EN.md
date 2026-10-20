# take_photo Example

[中文](README.md)

## Overview

This example uses the high-level `camera_handle.h` API to capture RGB565 single frames and print the frame-buffer address so the raw image can be exported from the target later.

Location: `examples/take_photo`

## Command

```text
msh> take_photo <framesize|RGB565> <quality> <count>
```

## Select a Sensor

Select OV2640, GC032A, or BF30A2 under
`Camera drivers -> Sensor settings -> Active camera sensor` in menuconfig.
Only the selected sensor driver is compiled.

Parameters:

- `framesize`: `QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA / 240X320`
- `RGB565`: automatically selects the only framesize supported by the active RGB565 sensor
- `quality`: ignored for RGB565; use `0`
- `count`: number of frames to capture, must be `>= 1`

Example:

```text
msh> take_photo RGB565 0 1
```

## Call Sequence

1. `camera_handler_instance_init()`
2. `camera_get_capabilities()`
3. `camera_change_settings()` with `PIXFORMAT_RGB565`
4. loop `camera_capture_single()`
5. `camera_deinit()`

## Sample Output

```text
RGB565 capture: 240x320, 153600 bytes/frame, buffer @ 0x6000001c
Frame 1 captured: 153600 bytes @ 0x6000001c (RGB565 240x320)
Export the buffer with the SDK script, e.g.:
  sftool ... read_mem 0x6000001c 153600 rgb565.bin
```

## Buffer Notes

This example builds its own PSRAM heap on the application side and allocates the frame buffer through `psram_heap_malloc()`.

Important:

- this is **example-local allocation logic**, not a public `camera_framework` allocator
- higher-resolution RGB565 capture typically requires PSRAM because internal SRAM is not large enough
- when capturing multiple frames, the same buffer is reused, so only the last frame usually remains after the command finishes

## Exporting The Image

The command prints the frame-buffer address and byte count. You can then use SDK tools to read back the memory region and convert it into a viewable image.

## Notes

- SCCB / SPI / sensor RESET / XCLK pin muxing is owned by the board's `bsp_pinmux.c`; the camera framework never calls `HAL_PIN_Set()` and carries no pin option, and the application does not mux pads either
- this example is mainly intended to validate the RGB565 single-shot capture path
