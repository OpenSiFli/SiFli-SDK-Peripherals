# take_photo 示例

[English](README_EN.md)

## 概述

该示例使用 `camera_handle.h` 的高层 API 采集 RGB565 单帧，并把帧缓冲地址打印到串口，方便后续从目标板导出原始图像数据。

命令位于：`examples/take_photo`

## 命令

```text
msh> take_photo <framesize> <count>
```

参数：

- `framesize`：`QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA`
- `count`：采集帧数，必须大于等于 `1`

示例：

```text
msh> take_photo QVGA 1
```

## 调用流程

1. `camera_handler_instance_init()`
2. `camera_get_capabilities()`
3. `camera_change_settings()`，设置 `PIXFORMAT_RGB565`
4. `camera_capture_single()` 循环采集
5. `camera_deinit()`

## 输出示例

```text
RGB565 capture: 320x240, 153600 bytes/frame, buffer @ 0x20100000
Frame 1 captured: 153600 bytes @ 0x20100000 (RGB565 320x240)
Export the buffer with the SDK script, e.g.:
  sftool ... read_mem 0x20100000 153600 rgb565.bin
```

## 缓冲区说明

该示例在应用侧自建了一个基于 `rt_memheap` 的 PSRAM heap，并通过 `psram_heap_malloc()` 为帧缓冲分配空间。

注意：

- 这是 **示例自己的分配方式**，不是 `camera_framework` 提供的公共接口
- 对高分辨率 RGB565，通常需要 PSRAM，内部 SRAM 往往不够
- 多帧采集时会复用同一块缓冲区，因此命令结束后保留下来的通常只有最后一帧

## 导出图像

示例会打印帧缓冲地址和字节数。你可以用 SDK 工具从目标板读出内存，再转换为 BMP 或其他可视化格式。

## 备注

- 引脚复用（SCCB / DVP / XCLK）由 OV2640 driver 内部完成，应用层不需要调用 `HAL_PIN_Set()`
- 该示例适合验证 RGB565 单帧采集链路是否正常
