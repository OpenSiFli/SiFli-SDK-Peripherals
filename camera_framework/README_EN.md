# Camera Framework

[中文](README.md) | [English](README_EN.md)

RT-Thread camera framework for the SiFli SF32 series, shipped as plain sources. It contains a `camera_handle` API layer, one of three sensor drivers (OV2640 / GC032A / BF30A2, picked in Kconfig), an SCCB/I2C control bus, three data paths (DCMI parallel DVP, DCMI SPI 2-bit packet mode, ArduCam SPI FIFO), and optional software ISP/JPEG.

Pin muxing, pulls, drive strength and peripheral instances belong to the board (`customer/boards/<board>/bsp_pinmux.c` + `Kconfig.board`); the framework only uses RT-Thread device names and never carries a pin or device-name option of its own. Frame buffers are supplied by the caller — the framework reserves no PSRAM.

## Current Capabilities

- One sensor driver and one data-bus backend per firmware image (see `SConscript`)
- Pixel formats: JPEG, RGB565, YUV422, RAW8
- Single-shot capture: blocking, with optional timeout, or asynchronous with a completion callback
- Bounded JPEG burst capture (`camera_capture_frames_timeout()`, keeps the last frame)
- Double-buffer streaming, plus a half-frame mode driven by DMA HT/TC events
- Frame validity query (`camera_stream_frame_is_valid()`) so a consumer can tell whether a dequeued descriptor still describes live data
- Capability query, unified status codes, 0/180 degree rotation
- Image controls are limited to what the sensor driver implements: frame size, quality and rotation. There is no global mirror/flip/exposure/white-balance/gain API.

## Directory Layout

```text
camera_framework/
├── Kconfig                  sensor / transport / software ISP+JPEG (no pin, no device name)
├── SConscript               source selection (one backend + one driver per config)
├── core/
│   ├── handle/             camera_handle.[ch] (public API), camera_handle_internal.h
│   ├── driver/
│   │   ├── camera_driver_desc.[ch]      driver link section and discovery
│   │   ├── camera_sensor_runtime.[ch]   shared capture/stream logic
│   │   ├── camera_jpeg_assembler.[ch]   JPEG frame assembly
│   │   └── {ov2640,gc032a,bf30a2}/      sensor drivers and register tables
│   └── bus/
│       ├── control/sccb.[ch]            control bus (I2C)
│       └── data/
│           ├── data_bus_adapter.[ch]    registry and wrapper API
│           ├── camera_xclk.[ch]         MCLK (drives only the board's PWM)
│           ├── camera_xclk_calc.c       MCLK period calculation
│           ├── dcmi.c                   SDK DCMI: parallel DVP / 2-bit SPI packet mode
│           └── arducam_fifo*.c          SPI FIFO module backend
├── software/{isp,jpeg}/    software ISP and JPEG codecs (vendored sources included)
├── examples/               runnable examples
├── tests/                  static checks and on-target tests
├── README.md
├── README_EN.md
└── develop.md              developer handbook (architecture, Kconfig, API, adding a sensor/bus)
```

## Architecture

```text
Application / example
    │  camera_handle.h only
    ▼
camera_handle            instance state machine, API mutex, stream queue, drop counters
    │  camera_device_ops_t (function table, one implementation per build)
    ▼
sensor driver + camera_sensor_runtime
    │  bus_adapter_ops_t, sccb.h, camera_xclk.h
    ▼
data-bus backend (dcmi / arducam_fifo) + control bus (sccb) + MCLK (camera_xclk)
    │  RT-Thread device API: find "dcmi" / "pwmt2" / "i2c1" / "spiN" by name
    ▼
SDK / board (bsp_pinmux.c, drv_dcmi.c, drv_pwm, drv_i2c, drv_spi)
```

The dependency is strictly one-way downwards: no lower layer calls back into the handle, and the framework never writes board state.

## Hardware Connections

| OV2640 Pin | MCU Pin | Function | Notes |
|------------|---------|----------|-------|
| D0 ~ D7 | Muxed by the board (`bsp_pinmux.c`) | 8-bit parallel data | Owned by the SDK DCMI backend |
| PCLK | Muxed by the board (`bsp_pinmux.c`) | Pixel clock | DCMI samples the rising PCLK edge by default |
| HSYNC/HREF | Muxed by the board (`bsp_pinmux.c`) | Line sync | Owned by the SDK DCMI backend |
| VSYNC | Muxed by the board (`bsp_pinmux.c`) | Frame sync | Owned by the SDK DCMI backend |
| SDA | Muxed by the board (`bsp_pinmux.c`) | SCCB data | The board owns the pad; the framework only opens `CAMERA_I2C_DEVICE_NAME` |
| SCL | Muxed by the board (`bsp_pinmux.c`) | SCCB clock | Same as SDA |
| XCLK | Muxed by the board (`bsp_pinmux.c`) | Sensor clock input | The board names the PWM device/channel (`CAMERA_XCLK_PWM_DEVICE_NAME` / `_CHANNEL`); a module with its own oscillator declares `CAMERA_XCLK_EXTERNAL_OSC` instead of `CAMERA_XCLK_ENABLE` |

Notes:

- Parallel DVP uses the SDK's `dcmi` device driver. The component no longer filters by SoC model; the board is responsible for only enabling `BSP_USING_CAMERA1` when it actually exposes DCMI. OV2640 accesses DVP only through the bus-adapter layer
- **Every pad is the board's.** `bsp_pinmux.c` muxes the DVP/SPI pads, the MCLK pad and the camera I2C pads; the framework calls no `HAL_PIN_*` and carries no pin or device-name option of its own — it only uses the RT-Thread device names the board declares
- `SCCB settings` no longer exists as a menu: the bus name is the board's `CAMERA_I2C_DEVICE_NAME`, and the SCCB timeout/clock rate are constants inside each sensor driver

## Kconfig Path

Run from the example project:

```sh
scons --board=dpi-hdk_lb57eybbn6_n16_qspi --menuconfig
```

or from the repository root:

```sh
scons -C camera_framework/examples/take_photo_to_screen/project \
      --board=dpi-hdk_lb57eybbn6_n16_qspi --menuconfig
```

The menu exists only after the component Kconfig is sourced. In-repo examples do
that with `rsource "../../../Kconfig"` in `project/Kconfig.proj`; with SF-PKG,
`sf-pkgs/Kconfig.conandeps` sources it instead, under the same
`Camera drivers` menu.

If `Camera drivers` is missing, check that:

- the board has `BSP_USING_CAMERA1=y` (otherwise `sf-pkg.yaml` does not pull the
  component);
- the component is installed (`sf-pkgs/Kconfig.conandeps` exists) or the in-repo
  example still has `rsource "../../../Kconfig"` in `project/Kconfig.proj`;
- `PKG_USING_CAMERA_FRAMEWORK=y` (when it is off, all submenus are hidden).

Menu path:

```text
Camera drivers (PKG_USING_CAMERA_FRAMEWORK)
├── Camera selection
│   ├── Camera model               OV2640 / GC032A / BF30A2 (single choice)
│   └── <model> interface          only the links that model supports
├── Camera hardware
│   ├── I2C device name            CAMERA_I2C_DEVICE_NAME
│   ├── Camera XCLK (MCLK) output  CAMERA_XCLK_ENABLE
│   └── XCLK PWM device name / channel
├── Camera software ISP
│   ├── ISP pipeline               Sensor hardware ISP only / Software ISP
│   └── Camera software ISP execution core
└── Camera software JPEG
    ├── BitBank JPEGENC encoder
    ├── ChaN TJpgDec decoder
    └── Camera software JPEG execution core
```

Configuration files:

| Path | Purpose |
|------|---------|
| `camera_framework/Kconfig` | root switch `PKG_USING_CAMERA_FRAMEWORK`, model/interface choices, `Camera hardware`, derived backend symbols |
| `camera_framework/software/isp/Kconfig` | `Camera software ISP` |
| `camera_framework/software/jpeg/Kconfig` | `Camera software JPEG` |
| `customer/boards/<board>/Kconfig.board` | default-only stubs for `choice SENSOR_MODEL` / `choice CAMERA_<MODEL>_INTERFACE`; hardware defaults via `configdefault CAMERA_I2C_DEVICE_NAME` / `CAMERA_XCLK_*` |
| `project/Kconfig.proj` | in-repo examples source the framework with `rsource "../../../Kconfig"`; with SF-PKG the component is sourced by `sf-pkgs/Kconfig.conandeps` instead |
| `project/proj.conf` | project overrides; in the examples, menuconfig saves here |
| `build_<board>_<core>/.config`, `rtconfig.h` | generated final configuration and macros; do not edit by hand |

Precedence, lowest to highest:

```text
board Kconfig.board configdefault
    → project proj.conf / menuconfig selection
        → generated .config / rtconfig.h
```

- The board only supplies defaults; the component choices pick the final model and
  interface.
- After changing a board default, remove any explicit `CONFIG_SENSOR_USING_*` /
  `CONFIG_CAMERA_<MODEL>_INTERFACE_*` from `proj.conf` or re-select it in
  menuconfig, otherwise the old value wins.
- bootloader/ACPU passes do not source the component, so
  `PKG_USING_CAMERA_FRAMEWORK` is undefined, the default-only choice stub is inert
  and every board `configdefault` stays inactive. This avoids `defined without a
  prompt`, `default not contained in the choice` and `active but symbol undefined`
  warnings.

### Sensor and interface defaults

- `SENSOR_USING_OV2640` / `_GC032A` / `_BF30A2` are members of the single-choice
  `SENSOR_MODEL`, whose full definition is in the component. The board only reopens
  the same named choice to set its default:

  ```kconfig
  choice SENSOR_MODEL
      default SENSOR_USING_OV2640
  endchoice
  ```

  In a component-less bootloader/ACPU pass this default-only stub has an inactive
  dependency and the SDK's `check_undefined_configdefaults()` treats it as inert.
  Do not use `configdefault` on a choice member itself.
- Each model has its own interface choice. The board only needs one default-only
  stub for its **default model** (OV2640 here):

  ```kconfig
  choice CAMERA_OV2640_INTERFACE
      default CAMERA_OV2640_INTERFACE_DVP
  endchoice
  ```

  The component owns the full choice and keeps fallback defaults for other models
  (OV2640/GC032A → DVP, BF30A2 → SPI1). Add another model's stub only if that
  model needs a non-default interface.
- `CAMERA_USING_*` and `CAMERA_*_BACKEND_*` / `CAMERA_SERIAL_SPI_2BIT` are derived
  symbols selected by the interface choice; do not edit them from a board or a
  `.conf` fragment.
- `BSP_USING_CAMERA1` is the board's Camera 1 connector switch and only controls
  whether the SF-PKG dependency is pulled in.

The complete board contract (symbol list, optional `BOARD_CAMERA_HW` grouping,
derived `CAMERA_BOARD_HAS_MCLK` / `CAMERA_XCLK_REQUIRED`, verification greps) is in
`develop.md` sections 4.1, 4.3 and Appendix A.

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
6. **Auto resize rule**: after a successful frame-size change in non-JPEG mode, the driver calls the bus adapter's `set_pingpong_size` op with `line_bytes × 2`. Backends that own no ping-pong pool (for example the DCMI backend) only validate the value. JPEG mode leaves the size untouched.
7. **Timeout or first-frame-only**: check the VSYNC/PCLK/HSYNC pad muxing in the board's `bsp_pinmux.c` and the XCLK configuration first.

## Dependencies

- RT-Thread device framework
- RT-Thread I2C driver framework
- SiFli BSP HAL (GPIO, DMA, GPTIM)
- An I2C bus device available as `i2c1` (default)

## Related Documents

- Chinese version: [README.md](README.md)
- take_photo example: [examples/take_photo/README_EN.md](examples/take_photo/README_EN.md)

## Hardware DCMI backend

Select GC032A `8-bit DVP` or OV2640 `DVP`. The component does not filter by SoC model: the board only enables `BSP_USING_CAMERA1` when it actually exposes DCMI. Parallel DVP has a single backend — `CAMERA_DVP_BACKEND_DCMI` is a hidden symbol that defaults to `y` and automatically `select`s `BSP_USING_DCMI`, which must stay enabled. This backend uses the SDK's `dcmi` device driver: the SDK owns the peripheral, the DMA channel and the DCMI interrupt vectors, while the framework programs capture and completes JPEG frames. DMA half/full/error notifications arrive through `rt_device_set_rx_indicate`; the SDK's own `HAL_DCMI_FrameEventCallback` turns DCMI frame/overflow/packet-size events into the same notification, and the framework only overrides `HAL_DCMI_ErrorCallback`. The board's own `bsp_pinmux.c` muxes D0-D7, PCLK, HREF, VSYNC and the MCLK pad in its `BSP_USING_DCMI` block, so the application supplies no `HAL_DCMI_MspInit()`; the board's `BSP_CAMERA_PowerUp()` / `BSP_CAMERA_PowerDown()` hooks are called by the backend init/deinit. SCCB uses the board's `CAMERA_I2C_DEVICE_NAME` plus the driver's timing constants, and capture samples the rising PCLK edge by default (the board can select the falling edge with `CAMERA_DCMI_PCLK_FALLING`). Do not enable `BSP_USING_CAMERA1` on a board without DCMI.

OV2640 defaults to SVGA/RGB565 with hardware DCMI and supports native sensor JPEG capture. The ArduCAM FIFO backend retains its JPEG default. The OV2640 RGB565 settings write `0x09` to the DSP `IMAGE_MODE` register, sending the low byte of each pixel first. Applications using native RGB565 on a little-endian MCU can copy these pixels directly.

`camera_start_stream()` defaults to `CAMERA_STREAM_MODE_FRAME` and submits two adjacent, 64-byte-aligned frame buffers. Each frame size must also be a multiple of 64 bytes. DMA half/full completion identifies complete frames; DCMI runs continuously without per-frame stop/restart.

`camera_get_stream_frame()` returns a borrowed descriptor. A negative `error` reports capture failure; `timestamp_ticks` records the RTOS tick at the frame event. Consumers perform any required DMA cache invalidation in thread context, then call `camera_stream_frame_is_valid()` before and after copying pixels to stable storage. Publish the copy only when both checks pass. In `FRAME` mode, the check covers generation and DMA source/destination progress; a queued descriptor alone does not guarantee that its pixels remain valid.

Use `camera_start_stream_mode(instance, &config, CAMERA_STREAM_MODE_HALF_FRAME)` for half-frame capture. `config.buffers[0]` points to one complete raw-image buffer aligned to 64 bytes; `config.buffers[1]` must be `NULL`, `config.buffer_size` must equal the full image size, the image height must be even, and each half must contain a multiple of 64 bytes. For example, an 800 × 600 OV2640 RGB565 image uses one 960000-byte buffer, with 480000 bytes per half.

DMA half transfer (HT) emits `CAMERA_STREAM_EVENT_HALF_FIRST`, and transfer completion (TC) emits `CAMERA_STREAM_EVENT_HALF_SECOND`. Both events for one image share the same `sequence`. Their `buffer` and `frame_size` always describe the full image; consumers use `event_type` to select its upper or lower half. Among successful events, only `HALF_SECOND` sets `is_complete` to true. Interrupt callbacks only enqueue metadata and wake the waiter; a normal thread uses `camera_get_stream_frame()` to consume events in FIFO order and process pixels.

In `HALF_FRAME` mode, `camera_stream_frame_is_valid()` compares only the notified half's generation against the latest HT/TC state, without polling DMA progress. `HALF_FIRST` expires at its corresponding TC, and `HALF_SECOND` expires at the next HT. Stop, error and restart invalidate old descriptors. Consumers must check validity before and after processing each half and perform the required cache maintenance. Publish an assembled image only after both halves with the same `sequence` have been processed successfully. A validity check does not prevent DMA from overwriting the buffer.

Failed stop/close retains state and buffer references for retry. Buffers are returned only after successful shutdown. Single-shot completion runs on the system workqueue after hardware has stopped.
