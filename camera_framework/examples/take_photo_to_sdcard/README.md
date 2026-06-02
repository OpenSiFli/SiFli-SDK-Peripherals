# take_photo_to_sdcard 示例

[English](README_EN.md)

## 概述

该示例使用 `camera_handle.h` 采集 JPEG 单帧，并把每一帧保存到 SD 卡文件系统中的 `/photo` 目录。

## 命令

```text
msh> take_photo <framesize> <quality> <count>
```

参数：

- `framesize`：`QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA`
- `quality`：JPEG 质量，`0` 最好、`63` 压缩最强
- `count`：拍照次数，必须大于等于 `1`

示例：

```text
msh> take_photo VGA 10 3
```

## 调用流程

1. `camera_handler_instance_init()`
2. `camera_get_capabilities()`
3. `camera_change_settings()`，设置 `PIXFORMAT_JPEG`
4. 循环 `camera_capture_single()`
5. 每帧保存为 `/photo/photo_NNN.jpg`
6. `camera_deinit()`

## 输出文件

保存结果类似：

```text
/photo/photo_001.jpg
/photo/photo_002.jpg
/photo/photo_003.jpg
```

## 缓冲区说明

示例在应用侧自建了 `rt_memheap` 风格的 PSRAM heap，并通过 `psram_heap_malloc()` 为 JPEG 缓冲区分配空间。

注意：

- 这是示例自己的分配逻辑，不是框架公共接口
- JPEG 帧长可变，示例会按分辨率估算缓冲区大小
- 高分辨率模式下应确认 PSRAM 容量足够

## 备注

- 引脚复用（SCCB / DVP / XCLK）由 OV2640 driver 内部完成
- `camera_change_settings()` 内部已经处理必要的 AEC/AWB 稳定等待
- 如果看到 `sd card not found` 或挂载失败，请优先检查 `sd0` 设备、文件系统格式和板级连线
