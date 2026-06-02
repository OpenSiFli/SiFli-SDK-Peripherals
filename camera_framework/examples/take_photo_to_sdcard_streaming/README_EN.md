# take_photo_to_sdcard (Streaming) Example

[中文](README.md)

## Overview

This example uses the streaming APIs in `camera_handle.h` to capture JPEG frames continuously and write each completed frame to the SD card immediately.

Current JPEG streaming path:

- DVP DMA writes incoming bytes into internal ring buffers
- the handle-layer JPEG parser thread assembles complete frames
- `camera_get_stream_frame()` returns complete frame descriptors to the application
- the application writes each frame to `/photo/photo_NNN.jpg`

## Command

```text
msh> take_photo <framesize> <quality> <count>
```

Parameters:

- `framesize`: `QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA`
- `quality`: JPEG quality, `0` is best and `63` is the most compressed
- `count`: number of frames to save, must be `>= 1`

## Call Sequence

1. `camera_handler_instance_init()`
2. `camera_get_capabilities()`
3. `camera_change_settings()` with `PIXFORMAT_JPEG`
4. allocate two PSRAM buffers and call `camera_start_stream()`
5. loop `camera_get_stream_frame()` and save each frame to the SD card
6. `camera_stop_stream()`
7. `camera_deinit()`

## Double-Buffer Notes

The example allocates two buffers:

- `buffer[0]`: primary streaming buffer
- `buffer[1]`: normalize workspace for frame assembly

These buffers come from example-local PSRAM allocation logic, not from a framework-wide allocator.

## Notes

- the ready-frame queue depth is currently `4`
- if SD write throughput is too low, you may see sequence jumps or dropped-frame growth
- this example is intended for JPEG streaming only
- SCCB / DVP / XCLK pin muxing is still handled by the OV2640 driver
