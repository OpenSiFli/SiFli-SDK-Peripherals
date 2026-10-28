# take_photo_to_sdcard Example

[中文](README.md)

## Overview

This example uses `camera_handle.h` to capture single frames and save them
under `/photo` on the SD card. OV2640 uses JPEG. GC032A serial and BF30A2 use
RGB565 at VGA and 240x320, respectively.

## Command

```text
msh> take_photo <framesize|RGB565> <quality> <count>
msh> take_photo_async <framesize> <quality>
```

## Select a Sensor

Select a sensor under
`Camera drivers -> Sensor settings -> Active camera sensor` in menuconfig.
After selecting GC032A, choose either `8-bit DVP` (SF32LB57X only; 52x/56x has
no parallel DVP capture path) or `2-bit SPI`. Only the selected sensor and data
backend are compiled.
BF30A2 uses a fixed RGB565/240x320 mode.

- `take_photo` queries the selected sensor capabilities. It saves `.jpg` when
  JPEG is supported, otherwise it captures RGB565 and saves `.ppm`.
- `take_photo_async` is available only for JPEG-capable sensors.

## GC032A 2-bit SPI Wiring

The GPIO sampling (GPIO-DMA) backend that used to carry the 2-bit serial stream
on 52x/56x has been removed. 2-bit serial capture is only available through the
DCMI SPI packet mode on 57x, whose clock and data pads are muxed by the board's
`bsp_pinmux.c`; this example does not wire any pad itself.

Parameters:

- `framesize`: `QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA / 240X320`
- `RGB565`: automatically selects the only framesize supported by the active RGB565 sensor
- `quality`: JPEG quality, `0` is best and `63` is the most compressed; it is
  ignored for RGB565 capture
- `count`: number of photos to capture, must be `>= 1`

Example:

```text
msh> take_photo VGA 10 3
msh> take_photo RGB565 0 1
msh> take_photo_async VGA 10
```

## Call Sequence

1. `camera_handler_instance_init()`
2. `camera_get_capabilities()`
3. select `PIXFORMAT_JPEG` or `PIXFORMAT_RGB565` from the capabilities
4. call `camera_change_settings()`
5. loop `camera_capture_single()`
6. save each frame as `/photo/photo_NNN.jpg` or `/photo/photo_NNN.ppm`
7. call `camera_deinit()`

## Output Files

Results are saved like this:

```text
/photo/photo_001.jpg
/photo/photo_002.jpg
/photo/photo_003.jpg
```

GC032A or BF30A2 RGB565 output looks like this:

```text
/photo/photo_001.ppm
/photo/photo_002.ppm
```

## Buffer Notes

The example builds an application-local PSRAM heap using `rt_memheap` and
allocates capture buffers through `psram_heap_malloc()`.

Important:

- this is example-local allocation logic, not a public framework allocator
- JPEG frame size is variable, so the example estimates a reasonable buffer size from the selected resolution
- RGB565 uses a fixed `width * height * 2` byte buffer; PPM output does not
  perform JPEG compression
- make sure enough PSRAM is available for higher resolutions

## Notes

- SCCB / SPI / XCLK pin muxing is owned by the board's `bsp_pinmux.c`; the camera framework never calls `HAL_PIN_Set()` and carries no pin option
- `camera_change_settings()` already includes the required AEC/AWB settle delay internally
- if you see `sd card not found` or mount failures, first check the `sd0` device, filesystem format, and board wiring
