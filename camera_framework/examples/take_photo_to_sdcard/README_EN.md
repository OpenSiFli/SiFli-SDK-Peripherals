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
After selecting GC032A, choose either `8-bit DVP` or `2-bit SPI`. Only the
selected sensor and data backend are compiled.
BF30A2 uses a fixed RGB565/240x320 mode.

- `take_photo` queries the selected sensor capabilities. It saves `.jpg` when
  JPEG is supported, otherwise it captures RGB565 and saves `.ppm`.
- `take_photo_async` is available only for JPEG-capable sensors.

## GC032A 2-bit SPI Wiring

All GC032A serial examples in this repository use a GPTIM1 external clock to
trigger GPIO-DMA, so the `handle` test and this example share one wiring
layout. SPI1 remains dedicated to the on-board TF card.

| GC032A module pin | SF32LB52 board | Purpose |
| --- | --- | --- |
| GND (1, 10) | GND | Common ground |
| 3V3 (2) | 3V3 | Power |
| IIC_SDA (3) | PA33 | SCCB / I2C1 SDA |
| IIC_SCL (4) | PA30 | SCCB / I2C1 SCL |
| PWDN (5) | GND | Active operation at low level |
| MCLK (6) | PA9 | 6 MHz XCLK |
| SPI_CLK (7) | PA39 | GPTIM1 ETR sample clock |
| SPI_D1 (8) | PA38 | GPIO-DMA data bit 1 |
| SPI_D0 (9) | PA37 | GPIO-DMA data bit 0 |

PA37 and PA38 are sampled together on each PA39 edge. SPI2 is not used and no
CS wire is required.

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

- SCCB / DVP or 2-bit SPI / XCLK pin muxing is handled by the camera framework
- `camera_change_settings()` already includes the required AEC/AWB settle delay internally
- if you see `sd card not found` or mount failures, first check the `sd0` device, filesystem format, and board wiring
