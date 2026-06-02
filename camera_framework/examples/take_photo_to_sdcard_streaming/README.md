# take_photo_to_sdcard（流式采集）示例

[English](README_EN.md)

## 概述

该示例使用 `camera_handle.h` 的流式 API 连续采集 JPEG，并在每次取到完整帧后立即写入 SD 卡。

当前 JPEG 流式路径为：

- DVP DMA 将数据写入内部环形缓冲
- handle 层 JPEG parser thread 识别完整帧
- `camera_get_stream_frame()` 向应用返回完整帧描述
- 应用立即写 `/photo/photo_NNN.jpg`

## 命令

```text
msh> take_photo <framesize> <quality> <count>
```

参数：

- `framesize`：`QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA`
- `quality`：JPEG 质量，`0` 最好、`63` 压缩最强
- `count`：保存帧数，必须大于等于 `1`

## 调用流程

1. `camera_handler_instance_init()`
2. `camera_get_capabilities()`
3. `camera_change_settings()`，设置 `PIXFORMAT_JPEG`
4. 分配两块 PSRAM 缓冲并调用 `camera_start_stream()`
5. 循环 `camera_get_stream_frame()`，每帧保存到 SD 卡
6. `camera_stop_stream()`
7. `camera_deinit()`

## 双缓冲说明

示例会分配两块缓冲：

- `buffer[0]`：流式采集主缓冲
- `buffer[1]`：内部归一化工作缓冲

这两块缓冲都是示例自己的 PSRAM 分配逻辑，不是框架提供的统一分配器。

## 备注

- ready-frame queue 深度当前为 `4`
- 如果 SD 写入速度跟不上，可能看到 `dseq` 跳变或丢帧统计增加
- 该示例只适用于 JPEG 流式采集
- 引脚复用仍由 OV2640 driver 内部完成
