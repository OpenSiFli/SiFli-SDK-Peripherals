# take_photo 示例

[English](README_EN.md)

## 概述

该示例使用 `camera_handle.h` 的高层 API 采集 RGB565 单帧，并把帧缓冲地址打印到串口，方便后续从目标板导出原始图像数据。

命令位于：`examples/take_photo`

## 命令

```text
msh> take_photo <framesize|RGB565> <quality> <count>
```

## 选择摄像头

在 menuconfig 的 `Camera drivers -> Sensor settings -> Active camera sensor`
中选择 OV2640、GC032A 或 BF30A2。构建系统只会编译选中的传感器驱动。

参数：

- `framesize`：`QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA / 240X320`
- `RGB565`：自动选择当前 RGB565 摄像头唯一支持的分辨率
- `quality`：RGB565 模式忽略该参数，建议填 `0`
- `count`：采集帧数，必须大于等于 `1`

示例：

```text
msh> take_photo RGB565 0 1
```

## 调用流程

1. `camera_handler_instance_init()`
2. `camera_get_capabilities()`
3. `camera_change_settings()`，设置 `PIXFORMAT_RGB565`
4. `camera_capture_single()` 循环采集
5. `camera_deinit()`

## 输出示例

```text
RGB565 capture: 240x320, 153600 bytes/frame, buffer @ 0x6000001c
Frame 1 captured: 153600 bytes @ 0x6000001c (RGB565 240x320)
Export the buffer with the SDK script, e.g.:
  sftool ... read_mem 0x6000001c 153600 rgb565.bin
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

- 引脚复用（SCCB / SPI / 传感器 RESET / XCLK）一律由开发板自己的 `bsp_pinmux.c` 负责，camera framework 不调用 `HAL_PIN_Set()`、也不提供引脚项；应用层同样不需要自己复用引脚
- 该示例适合验证 RGB565 单帧采集链路是否正常
