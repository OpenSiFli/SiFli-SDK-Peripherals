# Camera 框架说明

[English](README_EN.md)

## 概述

`camera_framework` 是一个面向 RT-Thread 的摄像头框架，当前默认集成：

- `camera_handle` 高层接口
- OV2640 sensor driver
- SCCB 控制总线
- DVP 数据总线

应用层只依赖 `camera_handle.h`。驱动层通过 `camera_device_ops_t` 接口接入，数据总线通过 `bus_adapter_t` 接口接入。

当前实现已经不再提供 `psram_heap_*` 或 `mem/` 目录下的帧缓冲分配器；最终帧缓冲区由应用或板级代码负责提供。

## 当前能力

- 支持 `PIXFORMAT_JPEG`、`PIXFORMAT_RGB565`、`PIXFORMAT_YUV422`、`PIXFORMAT_RAW8`
- 支持单帧阻塞采集 `camera_capture_single`
- 支持单帧异步采集 `camera_capture_single_async`
- 支持双缓冲连续流式采集 `camera_start_stream` / `camera_get_stream_frame`
- 支持 capability 查询 `camera_get_capabilities`
- 支持统一错误码 `camera_handle_status_t`

## 目录结构

```text
camera_framework/
├── camera/
│   ├── bus/
│   │   ├── control/
│   │   │   ├── sccb.c
│   │   │   └── sccb.h
│   │   └── data/
│   │       ├── data_bus_adapter.c
│   │       ├── data_bus_adapter.h
│   │       ├── dvp.c
│   │       └── dvp.h
│   ├── driver/
│   │   ├── camera_driver_desc.c
│   │   ├── camera_driver_desc.h
│   │   └── ov2640/
│   │       ├── ov2640.c
│   │       ├── ov2640.h
│   │       ├── ov2640_regs.h
│   │       └── ov2640_settings.h
│   └── handle/
│       ├── camera_handle.c
│       ├── camera_handle.h
│       └── camera_handle_internal.h
├── examples/
├── tests/
├── Kconfig
├── SConscript
├── README.md
├── README_EN.md
└── develop.md
```

## 分层关系

```text
Application
    ↓
camera_handle.h API
    ↓
camera_device_ops_t
    ↓
Sensor driver (当前为 OV2640)
    ↓
bus_adapter_t
    ↓
Data bus backend (当前为 DVP)
```

## Driver 发现方式

当前 camera framework 采用和 SDK 中 LCD driver 类似的静态导出方式：

- 每个 sensor driver 通过 `CAMERA_DRIVER_EXPORT(name, ops)` 导出一条描述符
- 描述符进入 `CameraDriverDescTab` section
- `camera_handle` 通过 `camera_driver_get_default_ops()` 扫描这张表，拿到默认 driver 的 `camera_device_ops_t`
- `CAMERA_HANDLE_TESTING` 打开时，测试注入的 fake ops 优先级高于默认 driver

这意味着：

- 新增 sensor driver 时，不需要再改 `camera_handle.c` 里的硬编码 `#ifdef`
- 但链接脚本必须保留 `CameraDriverDescTab` section，否则运行时无法发现 driver

## Kconfig 结构

顶层配置入口：`Camera drivers`

当前主要分成三层：

- `CAMERA_FRAMEWORK_ENABLE`：是否编译整个 camera framework
- `Sensor settings`：当前激活哪个 sensor driver
- `Data bus settings`：当前激活哪个数据总线 backend

当前默认项：

- `Active camera sensor` -> `Use OV2640`
- `Active camera data bus` -> `Use DVP`

## 公共 API

头文件：`camera/handle/camera_handle.h`

主要接口：

- `camera_handler_instance_init()`：初始化 handle 并打开 driver
- `camera_deinit()`：关闭 driver 并释放 handle
- `camera_get_capabilities()`：查询能力表
- `camera_change_settings()`：设置 `pixformat` / `framesize` / `quality`
- `camera_capture_single()`：阻塞单帧采集
- `camera_capture_single_async()`：异步单帧采集
- `camera_start_stream()`：启动双缓冲流式采集
- `camera_get_stream_frame()`：取下一帧流数据
- `camera_stop_stream()`：停止流式采集
- `camera_get_stream_dropped_count()`：查询丢帧数

## 最小使用流程

```c
#include "camera_handle.h"

camera_handler_instance_t *cam = RT_NULL;
camera_capture_config_t cfg = {
    .pixformat = PIXFORMAT_JPEG,
    .framesize = FRAMESIZE_VGA,
    .quality = 10,
};
camera_capture_request_t req = {
    .buffer = frame_buf,
    .buffer_size = frame_buf_size,
};

if (camera_handler_instance_init(&cam) != CAMERA_OK)
    return;

if (camera_change_settings(cam, &cfg) != CAMERA_OK)
    goto out;

if (camera_capture_single(cam, &req) == CAMERA_OK)
{
    /* req.frame_size 为实际帧长 */
}

out:
camera_deinit(&cam);
```

## 内存模型说明

当前链路不是“DMA 直接写用户最终缓冲区”。以 JPEG / DVP 路径为例：

- DVP DMA 先写 driver 持有的内部缓冲
- driver / handle 负责整理完整帧
- 最终再交给应用缓冲区或流式帧队列

因此：

- 应用侧仍需提供足够大的最终帧缓冲
- 高分辨率 RGB565 / JPEG 示例通常使用 PSRAM
- `camera_framework` 本身不负责分配 PSRAM

## 相关文档

- 详细开发说明：`develop.md`
- RGB565 单帧示例：`examples/take_photo/README.md`
- JPEG 保存到 SD 卡：`examples/take_photo_to_sdcard/README.md`
- JPEG 流式保存到 SD 卡：`examples/take_photo_to_sdcard_streaming/README.md`
- 测试说明：`tests/README.md`
