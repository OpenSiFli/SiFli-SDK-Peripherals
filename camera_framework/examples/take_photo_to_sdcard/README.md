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
中选择摄像头；选择 GC032A 后还可以选择 `8-bit DVP` 或
`2-bit SPI`，构建系统只会编译选中的 sensor 和 data backend。
BF30A2 使用固定的 RGB565/240×320 模式。

- `take_photo` 会查询当前摄像头能力：支持 JPEG 时保存 `.jpg`，否则使用
  RGB565 并保存 `.ppm`。
- `take_photo_async` 仅适用于支持 JPEG 的摄像头。

## GC032A 2-bit SPI 接线

仓库中的 GC032A 串行例程统一使用 GPTIM1 外部时钟触发 GPIO-DMA，因此
`handle` 测试和本例程可以共用同一套接线；SPI1 保留给开发板上的 TF 卡。

| GC032A 模块引脚 | SF32LB52 开发板 | 说明 |
| --- | --- | --- |
| GND（1、10） | GND | 共地 |
| 3V3（2） | 3V3 | 电源 |
| IIC_SDA（3） | PA33 | SCCB / I2C1 SDA |
| IIC_SCL（4） | PA30 | SCCB / I2C1 SCL |
| PWDN（5） | GND | 低电平保持工作 |
| MCLK（6） | PA9 | 6 MHz XCLK |
| SPI_CLK（7） | PA39 | GPTIM1 ETR 采样时钟 |
| SPI_D1（8） | PA38 | GPIO-DMA 数据位 1 |
| SPI_D0（9） | PA37 | GPIO-DMA 数据位 0 |

PA37、PA38 会在 PA39 的每个采样沿被同时读取，不使用 SPI2，也不需要连接
CS。

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

- 引脚复用（SCCB / DVP 或 2-bit SPI / XCLK）由 camera framework 内部完成
- `camera_change_settings()` 内部已经处理必要的 AEC/AWB 稳定等待
- 如果看到 `sd card not found` 或挂载失败，请优先检查 `sd0` 设备、文件系统格式和板级连线
