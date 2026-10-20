# take_photo_to_sdcard (Streaming) Example

[中文](README.md)

## Overview

This example uses the streaming APIs in `camera_handle.h` to capture JPEG or
RGB565 frames continuously and write each completed frame to the SD card.

Current JPEG streaming path:

- DVP DMA writes incoming bytes into internal ring buffers
- the handle-layer JPEG parser thread assembles complete frames
- `camera_get_stream_frame()` returns complete frame descriptors to the application
- the application writes each frame to `/photo/photo_NNN.jpg`

For RGB565, the sensor driver supplies double-buffered frames. The application
copies each completed frame before converting it to `/photo/photo_NNN.ppm`, so
SD-card writes cannot race with buffer reuse.

## Command

```text
msh> take_photo <framesize|RGB565> <quality> <count>
```

## Select a Sensor

Select a sensor under
`Camera drivers -> Sensor settings -> Active camera sensor` in menuconfig.
Only the selected driver is compiled. OV2640 uses JPEG, while GC032A serial and
BF30A2 use RGB565.

Parameters:

- `framesize`: `QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA / 240X320`
- `RGB565`: automatically selects the only framesize supported by the active RGB565 sensor
- `quality`: JPEG quality, `0` is best and `63` is the most compressed; ignored for RGB565
- `count`: number of frames to save, must be `>= 1`

## Call Sequence

1. `camera_handler_instance_init()`
2. `camera_get_capabilities()`
3. select `PIXFORMAT_JPEG` or `PIXFORMAT_RGB565` from the sensor capabilities
4. allocate PSRAM buffers and call `camera_start_stream()`
5. loop `camera_get_stream_frame()` and save each frame to the SD card
6. `camera_stop_stream()`
7. `camera_deinit()`

## Double-Buffer Notes

JPEG mode allocates two buffers. RGB565 mode adds a stable save buffer:

- `buffer[0]`: primary streaming buffer
- `buffer[1]`: normalize workspace for frame assembly
- `rgb565_save_buffer`: stable copy used while writing the PPM file

These buffers come from example-local PSRAM allocation logic, not from a framework-wide allocator.

## Notes

- the ready-frame queue depth is currently `4`
- if SD write throughput is too low, you may see sequence jumps or dropped-frame growth
- JPEG frames are saved as `.jpg`; RGB565 frames are converted to `.ppm`
- SCCB / SPI / XCLK pin muxing is owned by the board's `bsp_pinmux.c`; the camera framework never calls `HAL_PIN_Set()` and carries no pin option
