# OV2640 Camera Component

[中文](README.md) | [English](README_EN.md)

RT-Thread based OV2640 camera component for the SiFli SF32LB52/SF32LB56 platform. The component is built around a layered architecture: sensor driver, DVP data bus (with a generic bus-adapter abstraction), and a generic handle layer. Frame buffers are supplied by the caller — the component does not reserve any PSRAM. Upper-layer applications drive the camera through the handle-layer API.

## Current Capabilities

- Supports JPEG, RGB565, YUV422, and RAW8 pixel formats
- Supports single-shot capture
- Supports continuous capture with double-buffer streaming
- Supports a generic handle-layer API for future multi-camera or driver replacement scenarios
- Supports common image controls such as frame size, quality, mirror, flip, exposure, white balance, gain, and special effects

## Directory Layout

```text
ov2640/
├── camera/
│   ├── bus/
│   │   ├── control/
│   │   │   ├── sccb.c              # SCCB (I2C) control bus
│   │   │   └── sccb.h
│   │   └── data/
│   │       ├── data_bus_adapter.c  # Generic data-bus adapter abstraction
│   │       ├── data_bus_adapter.h
│   │       ├── dvp.c               # DVP DMA capture backend
│   │       └── dvp.h
│   ├── driver/
│   │   └── ov2640/
│   │       ├── ov2640.c            # OV2640 RT-Thread device driver
│   │       ├── ov2640.h            # Control-command macros and public API
│   │       ├── ov2640_regs.h       # Register address definitions
│   │       └── ov2640_settings.h   # Register initialization sequences
│   ├── handle/
│   │   ├── camera_handle.c         # High-level handle API implementation
│   │   └── camera_handle.h         # Type definitions and API declarations

├── examples/
│   ├── take_photo/                  # RGB565 single-shot MSH command example
│   └── take_photo_to_sdcard/        # Capture and save to SD card example
├── Kconfig
├── SConscript
├── README.md
└── README_EN.md
```

## Architecture

```text
┌────────────────────────────────────────────────────┐
│               Application / Example Layer          │
│         camera_handle_* APIs / MSH                 │
└───────────────────────┬────────────────────────────┘
                        │
┌───────────────────────▼────────────────────────────┐
│              handle layer: camera_handle           │
│  camera_device_ops_t (compile-time binding)        │
└───────────────────────┬────────────────────────────┘
                        │
┌───────────────────────▼────────────────────────────┐
│         OV2640 driver layer: camera/driver         │
│     DVP resource init and frame dispatching        │
└───────────────────────┬────────────────────────────┘
                        │
┌───────────────────────▼────────────────────────────┐
│            DVP data layer: camera/bus/data         │
│ GPIO + GPTIM + DMA, JPEG parsing, rearm for stream │
└───────────────────────┬────────────────────────────┘
                        │
                     OV2640 hardware
```

## Hardware Connections

| OV2640 Pin | MCU Pin | Function | Notes |
|------------|---------|----------|-------|
| D0 ~ D7 | Platform default data pin group | 8-bit parallel data | Applied through DVP default resource macros |
| PCLK | Default PA41 | Pixel clock | Default DVP mapping to GPTIM1_ETR |
| HSYNC/HREF | Default PA43 | Line sync | Default DVP mapping to GPTIM1_CH1 |
| VSYNC | Configured by Kconfig | Frame sync | Used as frame trigger interrupt |
| SDA | Default PA39 | SCCB data | Must be configured before open |
| SCL | Default PA40 | SCCB clock | Must be configured before open |
| XCLK | Default PA08 | Sensor clock input | Optional GPTIM2_CH1 output |

Notes:

- DVP hardware resources are initialized inside the DVP backend from board macros; OV2640 accesses DVP only through the bus-adapter layer
- Pin muxing for SCCB (I2C), DVP (PCLK / HSYNC), and XCLK is handled internally during initialization — the application does not need to call `HAL_PIN_Set()`

## Kconfig Options

All options are nested under `Camera drivers`, then configure:

- `Sensor settings` -> `Use OV2640` -> `OV2640 settings`
- `SCCB settings`
- `Data bus settings` -> `Use DVP`

### OV2640 settings

| Option | Default | Range | Description |
|--------|---------|-------|-------------|
| `CAMERA_READ_TIMEOUT_MS` | 1000 | 100–10000 | Single-shot capture wait timeout (ms) |

### SCCB settings

| Option | Default | Range | Description |
|--------|---------|-------|-------------|
| `CAMERA_SCCB_I2C_BUS_NAME` | `"i2c1"` | — | RT-Thread I2C bus device name |
| `CAMERA_SCCB_TIMEOUT_MS` | 1000 | — | I2C transaction timeout (ms) |
| `CAMERA_SCCB_MAX_HZ` | 100000 | 10000–400000 | I2C maximum frequency (Hz) |

### Data bus settings -> Use DVP

| Option | Default | Range | Description |
|--------|---------|-------|-------------|
| `CAMERA_DVP_PINGPONG_POOL_SIZE` | 10240 | 1024–65536 | OV2640 driver-owned ping-pong pool size ceiling |
| `CAMERA_DVP_PINGPONG_BUFFER_SIZE` | 8192 | 1024–65536 | DVP DMA ping-pong buffer size in bytes; minimum is row-width × 2 |
| `CAMERA_DVP_PINGPONG_USE_SECTION` | `n` | — | Place the OV2640 driver-owned ping-pong pool in `.dvp_pingpong` |
| `CAMERA_DVP_PCLK_PIN` | 41 | — | PCLK input pin index (PAx) |
| `CAMERA_DVP_HSYNC_PIN` | 43 | — | HSYNC/HREF input pin index (PAx) |
| `CAMERA_DVP_VSYNC_PIN` | 42 | — | VSYNC GPIO interrupt pin index (PAx number) |
| `CAMERA_DVP_XCLK_PIN` | -1 | — | XCLK output pin (PAx); -1 disables |
| `CAMERA_DVP_XCLK_FREQ` | 12 MHz | 6 / 12 MHz | XCLK frequency; derived from GPTIM2 running at 24 MHz |

> `CAMERA_DVP_DATA_PIN_BASE` is reserved for internal use — do not modify.

## Usage

### Recommended: Handle-Layer API

Drive the camera through `camera_handle.h` for a clean abstraction over the concrete sensor driver. The call order is always five steps:

```c
#include "camera_handle.h"

/* 1. Initialize the handle instance — selects the compile-time driver ops
 *    (SENSOR_USING_OV2640), calls ops->open(), and applies
 *    JPEG/VGA/quality-10 defaults. Upper-layer code stays sensor-agnostic. */
camera_handler_instance_t instance;
camera_handler_instance_init(&instance);

/* 2. Push format + resolution (internally inserts ~500 ms AEC/AWB settle) */
camera_capture_config_t cfg = {
    .pixformat = PIXFORMAT_RGB565,
    .framesize = FRAMESIZE_VGA,
    .quality   = 10,   /* JPEG only; ignored for RGB565/RAW8/YUV422 */
};
camera_change_settings(&instance, &cfg);

/* 4a. Blocking single-shot capture */
camera_capture_request_t req = {
    .buffer      = buffer,
    .buffer_size = buffer_size,
    .frame_size  = 0,
};
camera_capture_single(&instance, &req);
/* req.frame_size now holds the actual captured byte count */

/* 4b. Continuous double-buffer streaming (mutually exclusive with 4a) */
camera_stream_config_t stream_cfg = {
    .buffers     = {buffer0, buffer1},
    .buffer_size = frame_bytes,
};
camera_start_stream(&instance, &stream_cfg);

camera_stream_frame_t frame;
camera_get_stream_frame(&instance, &frame, 5 * RT_TICK_PER_SECOND);
/* frame.buffer / frame.frame_size / frame.sequence are valid until the next two frames arrive */

camera_stop_stream(&instance);

/* 5. Close the device */
camera_deinit(&instance);
```

### PSRAM Allocation Note

VGA-and-above RGB565 frames (e.g. VGA = 640×480×2 = 614 400 bytes) exceed internal SRAM capacity. The caller must allocate the frame buffer from PSRAM. RT-Thread’s `rt_memheap` is the recommended approach; the example projects contain a ready-to-copy inline implementation (`examples/take_photo/src/main.c`).

### OV2640 Internal Control Commands

`OV2640_CMD_*` are internal dispatch IDs used inside `ov2640_control()`. They are not part of the public API. The table below is provided as a reference for driver developers.

## Control Commands

### Format and Resolution

| Command | Value | Argument | Description |
|---------|-------|----------|-------------|
| `OV2640_CMD_SET_PIXFORMAT` | 0x01 | `pixformat_t` | Set pixel format |
| `OV2640_CMD_SET_FRAMESIZE` | 0x02 | `framesize_t` | Set frame size |

### Image Quality

| Command | Value | Argument | Description |
|---------|-------|----------|-------------|
| `OV2640_CMD_SET_BRIGHTNESS` | 0x03 | int −2…+2 | Brightness |
| `OV2640_CMD_SET_CONTRAST` | 0x04 | int −2…+2 | Contrast |
| `OV2640_CMD_SET_SATURATION` | 0x05 | int −2…+2 | Saturation |
| `OV2640_CMD_SET_QUALITY` | 0x06 | int 0…63 | JPEG quality (0 = best) |
| `OV2640_CMD_SET_GAINCEILING` | 0x1C | `gainceiling_t` | AGC gain ceiling |

### Image Processing

| Command | Value | Argument | Description |
|---------|-------|----------|-------------|
| `OV2640_CMD_SET_HMIRROR` | 0x07 | int 0/1 | Horizontal mirror |
| `OV2640_CMD_SET_VFLIP` | 0x08 | int 0/1 | Vertical flip |
| `OV2640_CMD_SET_COLORBAR` | 0x09 | int 0/1 | Color bar test pattern |
| `OV2640_CMD_SET_SPECIAL_EFFECT` | 0x14 | int 0…6 | Special effect |
| `OV2640_CMD_SET_DCW` | 0x17 | int 0/1 | Downsize control |
| `OV2640_CMD_SET_BPC` | 0x18 | int 0/1 | Bad pixel correction |
| `OV2640_CMD_SET_WPC` | 0x19 | int 0/1 | White pixel correction |
| `OV2640_CMD_SET_RAW_GMA` | 0x1A | int 0/1 | RAW gamma |
| `OV2640_CMD_SET_LENC` | 0x1B | int 0/1 | Lens correction |

### White Balance

| Command | Value | Argument | Description |
|---------|-------|----------|-------------|
| `OV2640_CMD_SET_WHITEBAL` | 0x0A | int 0/1 | AWB on/off |
| `OV2640_CMD_SET_AWB_GAIN` | 0x0E | int 0/1 | AWB gain on/off |
| `OV2640_CMD_SET_WB_MODE` | 0x15 | int 0…4 | WB mode (0 = auto) |

### Exposure and Gain

| Command | Value | Argument | Description |
|---------|-------|----------|-------------|
| `OV2640_CMD_SET_GAIN_CTRL` | 0x0B | int 0/1 | AGC on/off |
| `OV2640_CMD_SET_EXPOSURE_CTRL` | 0x0C | int 0/1 | AEC on/off |
| `OV2640_CMD_SET_AEC2` | 0x0D | int 0/1 | AEC DSP mode on/off |
| `OV2640_CMD_SET_AGC_GAIN` | 0x0F | int 0…30 | Manual AGC gain |
| `OV2640_CMD_SET_AEC_VALUE` | 0x13 | int 0…1200 | Manual AEC exposure |
| `OV2640_CMD_SET_AE_LEVEL` | 0x16 | int −2…+2 | AE target brightness offset |

### Data Bus and Buffers

| Command | Value | Argument | Description |
|---------|-------|----------|-------------|
| `OV2640_CMD_SET_FRAME_BUFFER` | 0x1F | `void *` | Set frame buffer pointer |
| `OV2640_CMD_SET_FRAME_BUFFER_SIZE` | 0x20 | `uint32_t` | Set frame buffer size |

### Capture Control

| Command | Value | Argument | Description |
|---------|-------|----------|-------------|
| `OV2640_CMD_START_CAPTURE` | 0x10 | `void *` (NULL = keep current) | Start single-shot capture |
| `OV2640_CMD_STOP_CAPTURE` | 0x11 | — | Abort current single capture |
| `OV2640_CMD_GET_FRAME_SIZE` | 0x12 | `uint32_t *` | Get captured frame byte count |
| `OV2640_CMD_START_STREAM` | 0x22 | `camera_stream_start_args_t *` | Start continuous double-buffer stream |
| `OV2640_CMD_STOP_STREAM` | 0x23 | — | Stop continuous stream |

### Supported Pixel Formats

- PIXFORMAT_JPEG
- PIXFORMAT_RGB565
- PIXFORMAT_YUV422
- PIXFORMAT_RAW8

### Supported Frame Sizes

- FRAMESIZE_96X96
- FRAMESIZE_QQVGA
- FRAMESIZE_128X128
- FRAMESIZE_QCIF
- FRAMESIZE_HQVGA
- FRAMESIZE_240X240
- FRAMESIZE_QVGA
- FRAMESIZE_320X320
- FRAMESIZE_CIF
- FRAMESIZE_HVGA
- FRAMESIZE_VGA
- FRAMESIZE_SVGA
- FRAMESIZE_XGA
- FRAMESIZE_HD
- FRAMESIZE_SXGA
- FRAMESIZE_UXGA

## Example Commands

## Examples

| Directory | Description |
|-----------|-------------|
| `examples/take_photo/` | MSH command `take_photo`: RGB565 single-shot capture, prints frame buffer address |
| `examples/take_photo_to_sdcard/` | Capture and save image to SD card |

### take_photo

```text
msh> take_photo <framesize> <count>

msh> take_photo QVGA 1
msh> take_photo VGA 3
```

After capture the frame buffer address is printed. View the image directly via J-Link (tools: `C:\WORK\SiFli-SDK\main\tools\bin2bmp\`):

```
python jlinkbin2bmp.py "-if SWD -speed 12000" rgb565 <width> <height> <addr>
```

Example (QVGA):

```
python jlinkbin2bmp.py "-if SWD -speed 12000" rgb565 320 240 0x20100000
```

## Notes

1. **Continuous capture** uses double buffering; the caller must supply two DMA-accessible frame buffers and process each frame before the next two arrive.
2. **Single-shot and streaming are mutually exclusive**: do not call `camera_capture_single` while a stream is active.
3. **JPEG buffer sizing**: output length is variable — allow at least 300 KB for VGA; UXGA may need 1–2 MB.
4. **Fixed-size modes** (RGB565 / YUV422 / RAW8): buffer must be exactly `width × height × bytes_per_pixel` (RGB565/YUV422 = 2, RAW8 = 1).
5. **After changing format or resolution**: `camera_change_settings` inserts ~500 ms internally; when using raw commands, call `rt_thread_mdelay(500)` manually.
6. **Auto resize rule**: after a successful frame-size change in non-JPEG mode, the driver automatically updates DVP ping-pong size to `line_bytes × 2`. JPEG mode keeps its current ping-pong size.
7. **Ping-pong pool and active size**: `CAMERA_DVP_PINGPONG_POOL_SIZE` is the compile-time ceiling; `CAMERA_DVP_PINGPONG_BUFFER_SIZE` is the initial active size.
8. **Timeout or first-frame-only**: check VSYNC pin, PCLK / HSYNC / XCLK configuration, and DVP ping-pong buffer size first.

## Dependencies

- RT-Thread device framework
- RT-Thread I2C driver framework
- SiFli BSP HAL (GPIO, DMA, GPTIM)
- An I2C bus device available as `i2c1` (default)

## Related Documents

- Chinese version: [README.md](README.md)
- take_photo example: [examples/take_photo/README_EN.md](examples/take_photo/README_EN.md)
