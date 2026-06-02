# take_photo Example

[中文](README.md)

## Overview

This example uses the high-level `camera_handle.h` API to capture RGB565 single frames and print the frame-buffer address so the raw image can be exported from the target later.

Location: `examples/take_photo`

## Command

```text
msh> take_photo <framesize> <count>
```

Parameters:

- `framesize`: `QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA`
- `count`: number of frames to capture, must be `>= 1`

Example:

```text
msh> take_photo QVGA 1
```

## Call Sequence

1. `camera_handler_instance_init()`
2. `camera_get_capabilities()`
3. `camera_change_settings()` with `PIXFORMAT_RGB565`
4. loop `camera_capture_single()`
5. `camera_deinit()`

## Sample Output

```text
RGB565 capture: 320x240, 153600 bytes/frame, buffer @ 0x20100000
Frame 1 captured: 153600 bytes @ 0x20100000 (RGB565 320x240)
Export the buffer with the SDK script, e.g.:
  sftool ... read_mem 0x20100000 153600 rgb565.bin
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

- SCCB / DVP / XCLK pin muxing is handled inside the OV2640 driver; the application does not call `HAL_PIN_Set()`
- this example is mainly intended to validate the RGB565 single-shot capture path
