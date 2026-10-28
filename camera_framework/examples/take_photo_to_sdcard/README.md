# take_photo_to_sdcard 示例

[English](README_EN.md)

## 概述

该示例使用 `camera_handle.h` 采集单帧，并把图像保存到 SD 卡文件系统中的
`/photo` 目录。OV2640 使用 JPEG；GC032A serial 和 BF30A2 使用 RGB565，
分别输出 VGA 和 240×320 图像。

## 命令

```text
msh> take_photo <framesize|RGB565> <quality> <count>
msh> take_photo_async <framesize> <quality>
```

## 选择摄像头

在 menuconfig 的 `Camera drivers -> Sensor settings -> Active camera sensor`
中选择摄像头；选择 GC032A 后还可以选择 `8-bit DVP`（仅 SF32LB57X，
52x/56x 上没有并行 DVP 采集路径）或 `2-bit SPI`，构建系统只会编译选中的
sensor 和 data backend。
BF30A2 使用固定的 RGB565/240×320 模式。

- `take_photo` 会查询当前摄像头能力：支持 JPEG 时保存 `.jpg`，否则使用
  RGB565 并保存 `.ppm`。
- `take_photo_async` 仅适用于支持 JPEG 的摄像头。

## GC032A 2-bit SPI 接线

52x/56x 上用于 2-bit 串行的 GPIO 采样（GPIO-DMA）后端已整体删除。2-bit 串行采集
只剩 57x 的 DCMI SPI 包模式，其时钟/数据 pad 由开发板 `bsp_pinmux.c` 复用，本例程
不自己复用任何引脚。

参数：

- `framesize`：`QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA / 240X320`
- `RGB565`：自动选择当前 RGB565 摄像头唯一支持的分辨率
- `quality`：JPEG 质量，`0` 最好、`63` 压缩最强；RGB565 模式会忽略
  该参数
- `count`：拍照次数，必须大于等于 `1`

示例：

```text
msh> take_photo VGA 10 3
msh> take_photo RGB565 0 1
msh> take_photo_async VGA 10
```

## 调用流程

1. `camera_handler_instance_init()`
2. `camera_get_capabilities()`
3. 根据能力选择 `PIXFORMAT_JPEG` 或 `PIXFORMAT_RGB565`
4. `camera_change_settings()`
5. 循环 `camera_capture_single()`
6. 每帧保存为 `/photo/photo_NNN.jpg` 或 `/photo/photo_NNN.ppm`
7. `camera_deinit()`

## 输出文件

保存结果类似：

```text
/photo/photo_001.jpg
/photo/photo_002.jpg
/photo/photo_003.jpg
```

GC032A 或 BF30A2 的 RGB565 输出类似：

```text
/photo/photo_001.ppm
/photo/photo_002.ppm
```

## 缓冲区说明

示例在应用侧自建了 `rt_memheap` 风格的 PSRAM heap，并通过
`psram_heap_malloc()` 分配采集缓冲区。

注意：

- 这是示例自己的分配逻辑，不是框架公共接口
- JPEG 帧长可变，示例会按分辨率估算缓冲区大小
- RGB565 使用 `宽 × 高 × 2` 字节的固定缓冲区；PPM 保存过程不会进行
  JPEG 压缩
- 高分辨率模式下应确认 PSRAM 容量足够

## 备注

- 引脚复用（SCCB / SPI / XCLK）一律由开发板 `bsp_pinmux.c` 完成，camera framework 不调用 `HAL_PIN_Set()`，也不提供引脚项
- `camera_change_settings()` 内部已经处理必要的 AEC/AWB 稳定等待
- 如果看到 `sd card not found` 或挂载失败，请优先检查 `sd0` 设备、文件系统格式和板级连线
