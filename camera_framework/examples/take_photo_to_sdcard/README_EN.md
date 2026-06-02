# take_photo_to_sdcard Example

[中文](README.md)

## Overview

This example uses `camera_handle.h` to capture JPEG single frames and save each frame to the SD card under `/photo`.

## Command

```text
msh> take_photo <framesize> <quality> <count>
```

Parameters:

- `framesize`: `QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA`
- `quality`: JPEG quality, `0` is best and `63` is the most compressed
- `count`: number of photos to capture, must be `>= 1`

Example:

```text
msh> take_photo VGA 10 3
```

## Call Sequence

1. `camera_handler_instance_init()`
2. `camera_get_capabilities()`
3. `camera_change_settings()` with `PIXFORMAT_JPEG`
4. loop `camera_capture_single()`
5. save each frame as `/photo/photo_NNN.jpg`
6. `camera_deinit()`

## Output Files

Results are saved like this:

```text
/photo/photo_001.jpg
/photo/photo_002.jpg
/photo/photo_003.jpg
```

## Buffer Notes

The example builds an application-local PSRAM heap using `rt_memheap` and allocates the JPEG buffer through `psram_heap_malloc()`.

Important:

- this is example-local allocation logic, not a public framework allocator
- JPEG frame size is variable, so the example estimates a reasonable buffer size from the selected resolution
- make sure enough PSRAM is available for higher resolutions

## Notes

- SCCB / DVP / XCLK pin muxing is handled by the OV2640 driver
- `camera_change_settings()` already includes the required AEC/AWB settle delay internally
- if you see `sd card not found` or mount failures, first check the `sd0` device, filesystem format, and board wiring
