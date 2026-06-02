# Camera 框架使用与开发手册

> 适用范围：`camera_framework/camera/` 目录内的 Handle 层、OV2640 设备驱动、SCCB 控制总线、DVP 数据总线，以及与其直接配套的 `camera_framework/Kconfig` 配置项。  
> 目标平台：SF32LB52X / SF32LB56X，RT-Thread 操作系统

---

## 目录

1. [架构概述](#1-架构概述)  
2. [目录结构](#2-目录结构)  
3. [快速上手（用户指南）](#3-快速上手用户指南)  
   - 3.1 单帧拍照  
    - 3.2 非阻塞单帧拍照  
    - 3.3 连续流式采集  
    - 3.4 修改图像参数  
4. [配置参考](#4-配置参考)  
   - 4.1 Kconfig 选项  
   - 4.2 关键宏与常量  
5. [缓冲区与内存约束](#5-缓冲区与内存约束)  
6. [API 参考](#6-api-参考)  
   - 6.1 实例生命周期与状态机  
   - 6.2 Handle 层 API  
   - 6.3 错误码  
   - 6.4 OV2640 扩展控制命令表  
7. [开发指南——接入新摄像头传感器](#7-开发指南接入新摄像头传感器)  
8. [开发指南——接入新数据总线](#8-开发指南接入新数据总线)  
9. [分层交互时序](#9-分层交互时序)  
10. [常见问题](#10-常见问题)  
11. [架构决策记录（ADR）](#11-架构决策记录adr)  

---

## 1. 架构概述

```
┌─────────────────────────────────────────────────────────┐
│                    应用层代码                            │
│   camera_handle.h API  +  应用自管最终帧缓冲区           │
└───────────────────────┬─────────────────────────────────┘
                        │ camera_device_ops_t（编译期绑定）
┌───────────────────────▼─────────────────────────────────┐
│               Handle 层  camera_handle.c                │
│  · 统一错误码  · 两槽就绪队列  · 信号量同步              │
│  · 单帧拍照 / 异步单帧  · 连续流式采集                  │
└───────────┬───────────────────────────┬─────────────────┘
            │ ops->open/capture/        │ camera_stream_start_args_t
            │ start_stream/stop_stream  │
┌───────────▼───────────────────────────────────────────┐
│           驱动层  ov2640                               │
│               driver/ov2640/ov2640.c                  │
│  · 传感器寄存器配置  · 总线模式切换  · 帧分发           │
└──────────┬──────────────────────────────┬─────────────┘
           │ bus_adapter_t *              │ SCCB/I2C
  ┌────────▼──────────┐          ┌────────▼────────────┐
  │  DVP 数据总线      │          │  控制总线 SCCB       │
  │  bus/data/dvp.c   │          │  bus/control/sccb.c │
  │  GPTIM + DMA      │          │  寄存器读写          │
  └───────────────────┘          └─────────────────────┘
```

**关键设计原则**

| 原则 | 当前实现 |
|------|------|
| Handle 与传感器驱动解耦 | 应用层只依赖 `camera_handle.h`；Handle 层通过 Kconfig 在编译期绑定具体 sensor ops，不要求业务代码包含 `ov2640.h`。 |
| 驱动层拥有总线硬件配置 | OV2640 驱动内部持有 SCCB / DVP 配置，并在 `ov2640_open()` 时显式下发给底层总线。 |
| 流状态有单一真值 | 驱动层以 `stream.frame_callback != RT_NULL` 作为“流已武装”的唯一判定条件。 |
| 丢帧可观测 | Handle 层就绪队列固定 2 槽，满时覆盖最旧帧并递增 `instance->stream.dropped_count`。 |
| 帧数据不是 DMA 直达用户缓冲 | DVP DMA 先写入 OV2640 驱动持有的内部乒乓缓冲池（SRAM），随后驱动再把数据拷贝或拼接到应用提供的最终帧缓冲区。 |
| `camera/` 不再提供分配器 | 当前目录已无 `mem/` / `psram_heap_*`；应用负责提供最终帧缓冲区。 |
| 驱动选择在编译期确定 | Kconfig 宏 `SENSOR_USING_OV2640` 在编译期绑定具体驱动 ops；Handle 层通过 `camera_get_board_ops()` 取得 ops 指针，直接调用 `ops->open()` / `ops->capture()` 等，不经过 RT-Thread camera 设备注册路径。 |

---

## 2. 目录结构

```
camera/
├── bus/
│   ├── control/
│   │   ├── sccb.h               ← SCCB/I2C 控制总线接口
│   │   └── sccb.c
│   └── data/
│       ├── data_bus_adapter.h   ← 数据总线抽象层接口（注册/查找/分发）
│       ├── data_bus_adapter.c
│       ├── dvp.h                ← DVP 具体实现接口
│       └── dvp.c
├── driver/
│   └── ov2640/
│       ├── ov2640.h             ← OV2640 内部类型 + 控制命令表（供驱动/Handle 集成使用）
│       ├── ov2640.c
│       ├── ov2640_regs.h        ← 寄存器定义
│       └── ov2640_settings.h    ← 分辨率/格式预设表
└── handle/
    ├── camera_handle.h          ← 公共 API 与类型定义
    └── camera_handle.c          ← Handle 层实现
```

与旧版本相比，当前 `camera/` 目录有两个明显变化：

- 已删除 `mem/` 子目录，文档不再引用 `psram_heap_*`。
- 数据总线示例应与 `dvp.c/.h` 同级放在 `camera/bus/data/` 下，不再按旧文档使用额外子目录层级。

---

## 3. 快速上手（用户指南）

常规应用只需要包含 `camera_handle.h`。`ov2640.h` 仅用于驱动开发或查阅 OV2640 内部控制命令定义，不应成为业务层初始化路径的依赖。

```c
#include "camera_handle.h"
```

> **缓冲区说明**：以下示例用静态数组表示“最终帧缓冲区”。当前实现里，DVP DMA 并不会直接写这里，而是先写入内部 SRAM 乒乓缓冲，再由驱动拷贝或拼接到这些用户缓冲区。若你的板卡使用 PSRAM 或其他外部 RAM，请替换为板级/应用自己的分配方式，但不要再引用已删除的 `psram_heap_*` 接口。

### 3.1 单帧拍照

```c
/* 1. 声明实例和单帧缓冲区 */
static camera_handler_instance_t g_cam;
static uint8_t g_frame_buf[200 * 1024];

void app_camera_init(void)
{
    /* camera_handler_instance_init 根据 Kconfig（SENSOR_USING_OV2640）
     * 在编译期选好驱动并打开驱动，应用层无需感知传感器型号。 */
    if (camera_handler_instance_init(&g_cam) != CAMERA_OK)
    {
        LOG_E("camera_handler_instance_init failed");
        return;
    }

    /* 可选：覆盖默认 JPEG/VGA/quality=10 */
    camera_capture_config_t cfg = {
        .pixformat = PIXFORMAT_JPEG,
        .framesize = FRAMESIZE_SVGA,
        .quality   = 12,
    };

    if (camera_change_settings(&g_cam, &cfg) != CAMERA_OK)
    {
        LOG_E("camera_change_settings failed");
    }
}

void app_capture_one_frame(void)
{
    camera_capture_request_t req = {
        .buffer      = g_frame_buf,
        .buffer_size = sizeof(g_frame_buf),
    };

    camera_handle_status_t ret = camera_capture_single(&g_cam, &req);
    if (ret == CAMERA_OK)
    {
        /* req.frame_size 为本帧实际字节数 */
        do_something_with_jpeg(g_frame_buf, req.frame_size);
    }
    else
    {
        LOG_E("capture failed: %d", ret);
    }
}
```

若应用需要根据驱动真实能力动态选择格式、分辨率或预估缓冲区大小，可在初始化后先查询 capability：

```c
const camera_capabilities_t *caps;

if (camera_get_capabilities(&g_cam, &caps) == CAMERA_OK)
{
    /* caps->pixformats / caps->framesizes / caps->max_buffer_size */
}
```

> **说明**：`camera_handler_instance_init()` 只会打开驱动并在 Handle 实例里记录默认的 `active_config`（JPEG / VGA / quality=10），不会在初始化阶段主动把这些默认值下发到传感器。若业务需要确定的输出格式、分辨率和质量，请在首次采集前显式调用 `camera_change_settings()`。

### 3.2 非阻塞单帧拍照

```c
static camera_handler_instance_t g_cam;
static uint8_t g_async_frame_buf[200 * 1024];

static void app_capture_done(void *context,
                             camera_handle_status_t status,
                             rt_size_t frame_size)
{
    (void)context;

    if (status != CAMERA_OK)
    {
        LOG_E("async capture failed: %d", status);
        return;
    }

    do_something_with_jpeg(g_async_frame_buf, frame_size);
}

void app_capture_one_frame_async(void)
{
    camera_capture_request_t req = {
        .buffer = g_async_frame_buf,
        .buffer_size = sizeof(g_async_frame_buf),
    };

    camera_handle_status_t ret = camera_capture_single_async(&g_cam,
                                                             &req,
                                                             app_capture_done,
                                                             RT_NULL);
    if (ret != CAMERA_OK)
    {
        LOG_E("camera_capture_single_async failed: %d", ret);
    }
}
```

> **注意**：异步单帧一次只允许一个请求在途；同一实例上一个异步拍照未完成前再次调用，会返回 `CAMERA_ERRORRESOURCE`。并且 `request.buffer` 必须一直保持有效，直到回调触发。

### 3.3 连续流式采集

```c
/* 双缓冲：作为最终帧缓冲区，在 stop_stream 返回前保持有效 */
static uint8_t g_stream_buf[2][200 * 1024];

void app_start_stream(void)
{
    camera_stream_config_t cfg = {
        .buffers = {
            g_stream_buf[0],
            g_stream_buf[1],
        },
        .buffer_size = sizeof(g_stream_buf[0]),
    };

    if (camera_start_stream(&g_cam, &cfg) != CAMERA_OK)
    {
        LOG_E("start_stream failed");
        return;
    }
}

void stream_consumer_thread(void *param)
{
    camera_stream_frame_t frame;
    while (1)
    {
        camera_handle_status_t ret =
            camera_get_stream_frame(&g_cam, &frame, rt_tick_from_millisecond(500));
        if (ret != CAMERA_OK)
            continue;

        /* frame.buffer      → 当前用户流缓冲槽地址（后续帧会复用） */
        /* frame.frame_size  → 本帧有效字节数 */
        /* frame.sequence    → 单调递增序号 */
        /* frame.buffer_index→ 本帧来自哪一个双缓冲槽 */
        consume_frame(frame.buffer, frame.frame_size, frame.sequence);
    }
}

void app_stop_stream(void)
{
    camera_stop_stream(&g_cam);
}
```

> **注意**：`camera_get_stream_frame()` 返回的是浅拷贝；`frame.buffer` 仍指向应用提供给 `camera_start_stream()` 的双缓冲区，后续帧会继续复用这些缓冲槽。若要长期保留数据，调用方必须自行复制。

### 3.4 修改图像参数

`camera_change_settings()` 不可在流运行时调用；需先 `camera_stop_stream()`，修改后再重新 `camera_start_stream()`。

```c
camera_capture_config_t cfg = {
    .pixformat = PIXFORMAT_RGB565,
    .framesize = FRAMESIZE_QVGA,
    .quality   = 10,
};

camera_change_settings(&g_cam, &cfg);
/* 函数内部会阻塞 500 ms，等待 AEC/AWB 收敛 */
```

若需修改亮度、对比度等细粒度参数，当前框架没有公开的细粒度控制 API；`OV2640_CMD_*` 是驱动内部的分发命令 ID，不对业务层暴露。如有需求，请在 Handle 层扩展相应控制接口，或直接修改驱动的默认値。

当 `pixformat` 发生变化时，当前 `ov2640` 驱动还会把像素格式映射到 `bus_capture_mode_t`，并在需要时通过 `bus_adapter_set_mode()` 切换总线采集模式。

---

## 4. 配置参考

### 4.1 Kconfig 选项

当前实现中，OV2640 驱动会把自身持有的 SCCB / DVP 配置集中在驱动内部静态结构里，并在 `ov2640_open()` 时显式下发给 SCCB 和 DVP 层。因此下面这些 Kconfig 选项属于**驱动拥有、并在运行时注入到底层总线的真实生效参数**。

对应到 `ov2640.c`，这些配置并不是零散直接使用，而是先被组装进驱动内部的静态硬件配置 `g_ov2640_hw_config`：

```c
typedef struct {
    sccb_config_t sccb;
    ov2640_data_bus_config_t data_bus;
} ov2640_hw_config_t;

static const ov2640_hw_config_t g_ov2640_hw_config = {
    .sccb = {
        .bus_name   = CAMERA_SCCB_I2C_BUS_NAME,
        .timeout_ms = CAMERA_SCCB_TIMEOUT_MS,
        .max_hz     = CAMERA_SCCB_MAX_HZ,
    },
    .data_bus = {
        .name                 = CAMERA_DATA_BUS_ADAPTER_NAME,
        .default_mode         = BUS_CAPTURE_MODE_JPEG,
        .frame_timeout_ms     = CAMERA_READ_TIMEOUT_MS,
        .pingpong_pool_size   = CAMERA_DVP_PINGPONG_POOL_SIZE,
        .pingpong_buffer_size = CAMERA_DVP_PINGPONG_BUFFER_SIZE,
        .resources            = DVP_RESOURCE_CONFIG(CAMERA_DVP_PCLK_PIN,
                                                    CAMERA_DVP_HSYNC_PIN,
                                                    CAMERA_DVP_VSYNC_PIN),
        .xclk_pin             = CAMERA_DVP_XCLK_PIN,
        .xclk_freq            = CAMERA_DVP_XCLK_FREQ,
    },
};
```

这段结构在当前实现里的作用可以直接理解为两步：

- `sccb` 子结构统一承载控制总线参数，随后在 `ov2640_open()` 中通过 `sccb_init(&g_ov2640_hw_config.sccb)` 下发。
- `data_bus` 子结构统一承载数据总线参数，随后在 `ov2640_open()` 中先用 `bus_adapter_find(g_ov2640_hw_config.data_bus.name)` 选中总线，再通过 `bus_adapter_*` 接口与总线交互。

其中有三个点容易被使用者忽略：

- `data_bus.name` 不应在驱动里手写常量，而是由 Kconfig 导出的 `CAMERA_DATA_BUS_ADAPTER_NAME` 决定。当前如果选择 `CAMERA_USING_DVP`，它会解析成 `"dvp"`。
- `data_bus.default_mode` 当前固定为 `BUS_CAPTURE_MODE_JPEG`，表示驱动打开后的默认总线采集模式先按 JPEG 组织；后续如果业务调用 `camera_change_settings()` 切到 RGB565 / YUV422，驱动会再按像素格式把总线模式切过去。
- `data_bus.frame_timeout_ms` 不是 DVP 自己的独立配置项，而是由 `CAMERA_READ_TIMEOUT_MS` 注入，最终被 `ov2640_capture()` 转成 tick 用于等待单帧完成。

同时要注意：虽然数据总线名字现在来自 Kconfig 选择，但 OV2640 当前实现仍然只编写了 DVP backend 的配置与初始化路径，也就是必须启用 `CAMERA_USING_DVP`。若后续增加其他 backend，除了补 Kconfig 选项外，还需要同时补 `ov2640_open()` 内对应 backend 的配置下发逻辑。

**DVP 配置**

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `CAMERA_DVP_PINGPONG_POOL_SIZE` | 10240 | OV2640 驱动持有的静态乒乓缓冲池总大小；运行期任何 `dvp_set_pingpong_size()` 调整都不能超过它。当前该调整能力仅在 DVP/驱动内部使用，Handle 层没有对业务直接暴露。 |
| `CAMERA_DVP_PINGPONG_BUFFER_SIZE` | 8192 | DVP 初始化时启用的乒乓缓冲大小。底层虽然支持运行期调整，但要求总线已停止且当前未在采集；常规业务建议通过 Kconfig 先设好初始值，而不是在 Handle 层运行中动态改。 |
| `CAMERA_DVP_PINGPONG_USE_SECTION` | `n` | 置 `y` 后，OV2640 驱动持有的乒乓缓冲池会被放到 `.dvp_pingpong` 链接段，由板级链接脚本决定实际内存区域。 |
| `CAMERA_DVP_DATA_PIN_BASE` | 0 | DVP D0-D7 数据引脚基准值，仅供内部计算资源宏使用，通常不要修改。 |
| `CAMERA_DVP_PCLK_PIN` | 41 | PCLK 输入引脚的 PAx 索引；驱动会自动复用到 `GPTIM1_ETR`。 |
| `CAMERA_DVP_HSYNC_PIN` | 43 | HSYNC/HREF 输入引脚的 PAx 索引；驱动会自动复用到 `GPTIM1_CH1`。 |
| `CAMERA_DVP_VSYNC_PIN` | 42 | VSYNC GPIO 中断引脚的 PAx 索引；由 DVP 在初始化时按板级宏装配。 |

**SCCB（I²C 控制总线）配置**

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `CAMERA_SCCB_I2C_BUS_NAME` | `"i2c1"` | SCCB 使用的 RT-Thread I²C 总线设备名。 |
| `CAMERA_SCCB_TIMEOUT_MS` | 1000 | SCCB 单次 I²C 操作超时（毫秒）。 |
| `CAMERA_SCCB_MAX_HZ` | 100000 | SCCB 最大时钟频率（Hz），范围 10000–400000。 |

**摄像头通用配置**

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `CAMERA_READ_TIMEOUT_MS` | 1000 | `ov2640_capture()` 等待单帧完成的超时时间。Handle 层超时会表现为 `CAMERA_ERRORTIMEOUT`。 |
| `CAMERA_DVP_XCLK_PIN` | -1 | XCLK 输出 PAx 索引；`-1` 表示禁用 XCLK 输出。 |
| `CAMERA_DVP_XCLK_FREQ` | 12000000 | XCLK 频率（Hz），由 choice 选项在 6 MHz / 12 MHz 之间生成。 |

> **XCLK 说明**：当前 `dvp_xclk_start()` 已在驱动内部自动调用 `HAL_PIN_Set(PAD_PA00 + pin, GPTIM2_CH1, ...)` 配置复用，不再需要文档旧版本中那种“必须在 `main.c` 手工配 pin”的步骤。你只需要保证 Kconfig 里填的 PAx 索引与硬件连接一致，且不会被别的模块二次改写。

### 4.2 关键宏与常量

| 宏 / 类型 | 定义位置 | 说明 |
|-----------|---------|------|
| `CAMERA_DATA_BUS_ADAPTER_NAME` | `Kconfig` | 当前 Kconfig 选择出的数据总线适配器名；启用 `CAMERA_USING_DVP` 时默认为 `"dvp"`。 |
| `DVP_BUS_ADAPTER_NAME` | `dvp.h` | DVP 总线适配器注册名，当前为 `"dvp"`。 |
| `BUS_ADAPTER_MAX` | `data_bus_adapter.c` | 总线适配器静态注册表容量，当前为 4。 |
| `camera_stream_start_args_t` | `camera_handle.h` | Handle 层传给传感器驱动的启动流参数结构体。 |
| `OV2640_CMD_*` | `ov2640.h` | OV2640 驱动内部 `ov2640_control()` 的分发命令 ID，仅供驱动层内部使用，不对业务层暴露。 |
| `DEBUG_DVP` | `dvp.c` | 置 1 可开启 DVP 调试快照和 hex dump 代码。 |

---

## 5. 缓冲区与内存约束

当前 `camera/` 目录**不再提供** `psram_heap_init()`、`psram_heap_malloc()`、`psram_heap_free()` 之类的帧缓冲分配器。并且需要特别注意：当前采集链路是“DVP DMA -> 内部 SRAM 乒乓缓冲 -> 软件拷贝/拼接 -> 用户最终帧缓冲”，不是 DMA 直写用户缓冲。内存职责划分如下：

- **应用层负责输出最终帧缓冲区**  
    单帧拍照通过 `camera_capture_request_t.buffer` 提供目标缓冲区；连续流通过 `camera_stream_config_t.buffers[0/1]` 提供双缓冲。这些缓冲区是最终图像承载区，不是 DVP DMA 的直接目的地址。

- **总线层负责内部中转缓冲区**  
    OV2640 驱动内部维护静态乒乓缓冲池，DVP 初始化时启用大小由 `CAMERA_DVP_PINGPONG_BUFFER_SIZE` 决定。非 JPEG 模式下分辨率切换后，驱动会通过 `bus_adapter_set_pingpong_size()` 触发 DVP 自动调整为 `line_bytes * 2`；这仍是总线/驱动内部能力，不是 Handle 公共 API。

- **流模式缓冲区的生命周期要求更严格**  
  `camera_start_stream()` 返回后，两块用户缓冲区都必须持续有效，直到 `camera_stop_stream()` 返回。

- **最终帧缓冲必须对 CPU 可访问**  
    当前采集路径由 CPU 把 SRAM 乒乓缓冲中的数据拷贝到最终帧缓冲。因此这块内存至少要对 CPU 可访问。若后续还有显示 DMA 或其他硬件模块直接读取它，再额外确认那些模块能访问该地址范围。

- **DCache 一致性仍由应用处理**  
  驱动层不会在帧派发前自动做 Cache Invalidate / Clean。若平台开启 D-Cache，请在 CPU 读取帧数据前做 Invalidate，在下一跳 DMA 消费前按需做 Clean。

```c
/* DMA 写完后，CPU 读取前 */
mpu_dcache_invalidate((uint32_t *)frame.buffer, (uint32_t)frame.frame_size);

/* 若下一跳硬件 DMA 要读这块内存 */
mpu_dcache_clean(frame.buffer, frame.frame_size);
```

---

## 6. API 参考

### 6.1 实例生命周期与状态机

当前实现里，`camera_handler_instance_init()` 会通过编译期绑定的 `camera_device_ops_t` 直接调用驱动 `open`，并在 Handle 实例内初始化默认 `active_config`。这些默认值不会在 init 阶段主动下发到传感器，因此若业务需要确定的输出格式、分辨率和质量，应在首次采集前显式调用 `camera_change_settings()`。实例调用后进入 IDLE 状态，可进行单帧拍照、异步单帧拍照或流式采集。更准确的生命周期如下：

```
       camera_handler_instance_init()
            │  (调用 ops->open() + 初始化 active_config 默认值)
            ┌───────▼──────────┐   camera_change_settings()
            │       IDLE       │◄────────────────────────────┐
            └──────┬─────┬─────┘        (500 ms AEC 等待)    │
 camera_start_stream()   │ camera_capture_single() /         │
        │            │ camera_capture_single_async()     │
        │            │ (阻塞或异步完成后回到 IDLE)       │
   ┌────────▼────────┐   └───────────────────────────────────┘
   │    STREAMING    │
   │                 │ camera_get_stream_frame()（消费帧）
   └────────┬────────┘
            │ camera_stop_stream()
            ▼
           IDLE ──── camera_deinit() ──► camera_handler_instance_init() ──► IDLE
```

| 状态 | 含义 | 可调 API |
|------|------|----------|
| IDLE | 设备已打开，未在采集 | `camera_get_capabilities`, `camera_change_settings`, `camera_capture_single`, `camera_capture_single_async`, `camera_start_stream`, `camera_deinit` |
| STREAMING | 连续流采集中 | `camera_get_stream_frame`, `camera_stop_stream`, `camera_deinit` |

> `camera_handler_instance_init()` 不是 reset API：如果实例已经打开，它会直接返回 `CAMERA_OK`，不会重置流状态、异步 worker 或任何运行时现场。若需要完整重建链路，应按 `camera_stop_stream()`（如有）→ `camera_deinit()` → `camera_handler_instance_init()` 的顺序执行。

> `camera_deinit()` 会调用 `ops->close()`、`rt_sem_detach()` Handle 层帧信号量，并在异步单帧 worker 已创建时停止 worker、发送 STOP 邮件并 `rt_mb_detach()`；驱动的编译期绑定不会改变，因此同一实例后续可再次调用 `camera_handler_instance_init()` 重新打开。

---

### 6.2 Handle 层 API

> **线程安全总则**：所有 Handle 层 API 都假定运行在线程上下文；内部会使用 `ops->capture`、`rt_sem_take`、`rt_sem_init` 等接口，不适合在 ISR 中直接调用。

#### `camera_handler_instance_init`

```c
camera_handle_status_t camera_handler_instance_init(
    camera_handler_instance_t *instance);
```

作用：

- 幂等：若实例已打开，直接返回 `CAMERA_OK`。
- 通过 `camera_get_board_ops()` 获取编译期绑定的 `camera_device_ops_t`；若无可用驱动，返回 `CAMERA_ERRORRESOURCE`。
- 调用 `ops->open()` 打开驱动（初始化 SCCB、读取传感器 ID、初始化 DVP 资源）。
- 清零实例状态，并在 Handle 实例内记录默认 `active_config`：`PIXFORMAT_JPEG` / `FRAMESIZE_VGA` / `quality=10`。
- 清空流队列并把 `stream.sem_initialized` / `stream.enabled` 置为 `RT_FALSE`。

> 注意 1：这里记录的是 Handle 层默认状态，不是一次真正的传感器配置下发；若业务需要确定的格式/分辨率/质量，请继续调用 `camera_change_settings()`。

> 注意 2：若实例已经处于打开状态，再次调用只会直接返回 `CAMERA_OK`，不会把当前实例重新清零。因此不要把 `camera_handler_instance_init()` 当成“重新初始化/故障恢复”接口使用。

#### `camera_get_capabilities`

```c
camera_handle_status_t camera_get_capabilities(
    camera_handler_instance_t      *instance,
    const camera_capabilities_t   **caps);
```

作用：

- 返回当前编译期选中驱动的静态能力描述。
- `caps->pixformats` 给出支持的像素格式列表。
- `caps->framesizes` 给出支持的分辨率列表。
- `caps->max_buffer_size` 给出单帧最坏情况下的最大缓冲需求估计。

限制：

- `instance == NULL` 或 `caps == NULL` 时返回 `CAMERA_ERRORPARAMETER`。
- 若驱动没有提供 capability 描述，则返回 `CAMERA_ERRORRESOURCE`。

#### `camera_deinit`

```c
camera_handle_status_t camera_deinit(camera_handler_instance_t *instance);
```

作用：

- 若流仍在运行，先调用 `camera_stop_stream()`。
- 调用 `ops->close()` 关闭驱动。
- 若 Handle 层流信号量已初始化，则执行 `rt_sem_detach()`。
- 若异步单帧 worker 曾被创建，则发送 STOP 命令、删除 worker 线程并 `rt_mb_detach()` 对应 mailbox。

当前实现不会销毁实例本身，也不会清空已编译期绑定的 `device_ops`；因此可在之后再次调用 `camera_handler_instance_init()` 重新打开。

#### `camera_change_settings`

```c
camera_handle_status_t camera_change_settings(
    camera_handler_instance_t      *instance,
    const camera_capture_config_t  *config);
```

作用：

- 依次下发 `set_pixformat`、`set_framesize`、`set_quality` 三条控制命令。
- 成功后把 `config` 保存到 `instance->active_config`。
- 额外阻塞 500 ms，等待 AEC/AWB 收敛。

限制：

- 流模式下禁止调用，返回 `CAMERA_ERRORRESOURCE`。
- 设备未打开时禁止调用，返回 `CAMERA_ERRORRESOURCE`。

补充：当前 `ov2640` 驱动在处理 `OV2640_CMD_SET_PIXFORMAT` 时，还会把像素格式映射为 `bus_capture_mode_t`，并在模式变化时调用 `bus_adapter_stop()` → `bus_adapter_set_mode()` → `bus_adapter_start()` 重启总线硬件。

#### `camera_capture_single`

```c
camera_handle_status_t camera_capture_single(
    camera_handler_instance_t *instance,
    camera_capture_request_t  *request);
```

作用：

- 调用 `ops->capture(request->buffer, request->buffer_size)` 触发一次阻塞式单帧采集。
- 成功时将驱动返回的字节数写入 `request->frame_size`。
- 若驱动返回 0，则将 `request->frame_size` 置 0，并映射为 `CAMERA_ERRORTIMEOUT`。

当前 `ov2640_capture()` 的行为补充：

- 启动采集前会清空旧的信号量令牌。
- 超时后会调用 `bus_adapter_abort_capture()` 中止当前采集。
- 若开启超时诊断，会通过 `bus_adapter_dump_state()` 输出总线状态。

#### `camera_capture_single_async`

```c
camera_handle_status_t camera_capture_single_async(
    camera_handler_instance_t        *instance,
    camera_capture_request_t         *request,
    camera_capture_done_callback_t    callback,
    void                             *context);
```

作用：

- 把一次单帧采集请求投递给 Handle 层内部 worker thread，立即返回，不阻塞调用线程。
- 采集完成或超时后，在 worker thread 上下文调用 `callback(context, status, frame_size)`。
- worker thread 与 mailbox 采用懒初始化方式，首次异步调用时创建，`camera_deinit()` 时销毁。

限制：

- `request->buffer` 必须在回调触发前保持有效。
- `callback` 不能为空。
- 流模式运行期间不可调用。
- 若上一笔异步采集仍在执行，则返回 `CAMERA_ERRORRESOURCE`。
- 若 worker thread 创建失败，则返回 `CAMERA_ERRORNOMEMORY`。

#### `camera_start_stream`

```c
camera_handle_status_t camera_start_stream(
    camera_handler_instance_t      *instance,
    const camera_stream_config_t   *config);
```

作用：

- 首次调用时懒初始化 `instance->stream.frame_sem`。
- 排空历史信号量令牌。
- 清空两槽就绪队列和 `dropped_count`。
- 构造 `camera_stream_start_args_t`，把内部 `camera_stream_frame_ready_callback()` 注入到底层驱动。
- 调用 `ops->start_stream(&args)` 启动流。

限制：

- `config->buffers[0]`、`config->buffers[1]` 均不可为 `NULL`。
- `config->buffer_size` 必须大于 0。
- 流已激活时再次调用会返回 `CAMERA_ERRORRESOURCE`。

#### `camera_get_stream_frame`

```c
camera_handle_status_t camera_get_stream_frame(
    camera_handler_instance_t *instance,
    camera_stream_frame_t     *frame,
    rt_int32_t                 timeout);
```

作用：

- 在 `instance->stream.frame_sem` 上阻塞等待下一帧。
- 成功后从两槽环形队列尾部取出一帧，浅拷贝到 `*frame`。

返回特征：

- 超时返回 `CAMERA_ERRORTIMEOUT`。
- 流未启动或信号量未初始化返回 `CAMERA_ERRORRESOURCE`。
- 若出现“拿到信号量但队列计数为 0”的异常保护分支，也返回 `CAMERA_ERRORRESOURCE`。

#### `camera_stop_stream`

```c
camera_handle_status_t camera_stop_stream(camera_handler_instance_t *instance);
```

作用：

- 向底层驱动发送 `stop_stream` 控制命令。
- 无论驱动返回值如何，都会本地清除 `stream.enabled`，并重置队列与 `dropped_count`。

特性：

- 幂等；若流本来未激活，直接返回 `CAMERA_OK`。

---

### 6.3 错误码

| 枚举值 | 数值 | 含义 | 典型触发场景 |
|--------|------|------|-------------|
| `CAMERA_OK` | 0 | 成功 | — |
| `CAMERA_ERROR` | 1 | 通用错误 | RT-Thread 或驱动返回了未映射错误码 |
| `CAMERA_ERRORTIMEOUT` | 2 | 操作超时 | `camera_capture_single` 超时；`camera_get_stream_frame` 等待超时 |
| `CAMERA_ERRORRESOURCE` | 3 | 资源不可用 | 未选中任何驱动；设备未打开；流已激活时调用 `camera_change_settings`；异步拍照已有请求在途 |
| `CAMERA_ERRORPARAMETER` | 4 | 参数非法 | `instance == NULL`、`config == NULL`、缓冲区为空或大小为 0 |
| `CAMERA_ERRORNOMEMORY` | 5 | 内存不足 | `camera_start_stream()` 内部 `rt_sem_init()` 失败；或异步拍照 worker thread 创建失败 |
| `CAMERA_ERRORISR` | 6 | ISR 上下文不允许 | 预留，当前版本未在 Handle 层显式返回 |

---

### 6.4 OV2640 内部控制命令参考

`OV2640_CMD_*` 是 OV2640 驱动内部 `ov2640_control()` 函数的分发 ID，仅供驱动层内部使用，不对业务层直接暴露。如需了解驱动支持的控制能力（例如在实现新 Handle 层 API 时），可参考此表。整数参数通常以 `(void *)(rt_ubase_t)value` 形式传入。

**图像格式与分辨率**

| 命令 | 值 | 参数类型 | 说明 |
|------|----|----------|------|
| `OV2640_CMD_SET_PIXFORMAT` | 0x01 | `pixformat_t` | 像素格式（JPEG / RGB565 / YUV422 / RAW8） |
| `OV2640_CMD_SET_FRAMESIZE` | 0x02 | `framesize_t` | 帧分辨率（96×96 → UXGA 1600×1200） |

**图像质量**

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| `OV2640_CMD_SET_BRIGHTNESS` | 0x03 | int −2…+2 | 亮度 |
| `OV2640_CMD_SET_CONTRAST` | 0x04 | int −2…+2 | 对比度 |
| `OV2640_CMD_SET_SATURATION` | 0x05 | int −2…+2 | 饱和度 |
| `OV2640_CMD_SET_QUALITY` | 0x06 | int 0…63 | JPEG 压缩质量 |
| `OV2640_CMD_SET_GAINCEILING` | 0x1C | `gainceiling_t` | AGC 增益上限 |
| `OV2640_CMD_SET_SHARPNESS` | 0x1D | int −2…+2 | 锐度；当前 OV2640 上为 no-op |
| `OV2640_CMD_SET_DENOISE` | 0x1E | int level | 降噪；当前 OV2640 上为 no-op |

**白平衡**

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| `OV2640_CMD_SET_WHITEBAL` | 0x0A | int 0/1 | 硬件白平衡开关 |
| `OV2640_CMD_SET_AWB_GAIN` | 0x0E | int 0/1 | AWB 增益开关 |
| `OV2640_CMD_SET_WB_MODE` | 0x15 | int 0–4 | 0=自动, 1=晴天, 2=阴天, 3=办公室, 4=家庭 |

**曝光与增益**

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| `OV2640_CMD_SET_GAIN_CTRL` | 0x0B | int 0/1 | AGC 开关 |
| `OV2640_CMD_SET_EXPOSURE_CTRL` | 0x0C | int 0/1 | AEC 开关 |
| `OV2640_CMD_SET_AEC2` | 0x0D | int 0/1 | AEC2 开关 |
| `OV2640_CMD_SET_AGC_GAIN` | 0x0F | int 0–30 | AGC 手动增益 |
| `OV2640_CMD_SET_AEC_VALUE` | 0x13 | int 0–1200 | AEC 手动曝光值 |
| `OV2640_CMD_SET_AE_LEVEL` | 0x16 | int −2…+2 | 自动曝光目标亮度偏置 |

**图像处理**

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| `OV2640_CMD_SET_HMIRROR` | 0x07 | int 0/1 | 水平镜像 |
| `OV2640_CMD_SET_VFLIP` | 0x08 | int 0/1 | 垂直翻转 |
| `OV2640_CMD_SET_COLORBAR` | 0x09 | int 0/1 | 彩条测试图 |
| `OV2640_CMD_SET_SPECIAL_EFFECT` | 0x14 | int 0–6 | 特效 |
| `OV2640_CMD_SET_DCW` | 0x17 | int 0/1 | 数字裁剪缩放 |
| `OV2640_CMD_SET_BPC` | 0x18 | int 0/1 | 坏点校正 |
| `OV2640_CMD_SET_WPC` | 0x19 | int 0/1 | 白点校正 |
| `OV2640_CMD_SET_RAW_GMA` | 0x1A | int 0/1 | RAW gamma 开关 |
| `OV2640_CMD_SET_LENC` | 0x1B | int 0/1 | 镜头阴影校正 |

**数据总线与缓冲区**

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| `OV2640_CMD_SET_FRAME_BUFFER` | 0x1F | `void *` | 更新底层总线层的目标帧缓冲区指针 |
| `OV2640_CMD_SET_FRAME_BUFFER_SIZE` | 0x20 | `uint32_t` | 更新底层目标帧缓冲区大小 |
| `OV2640_CMD_SET_PINGPONG_SIZE` | 0x21 | `uint32_t` | 运行期调整 DVP 乒乓缓冲大小 |

**捕获控制**

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| `OV2640_CMD_START_CAPTURE` | 0x10 | 无常用参数 | 直接启动一次底层采集；通常不建议绕过 Handle 层使用 |
| `OV2640_CMD_STOP_CAPTURE` | 0x11 | 无 | 停止当前底层采集 |
| `OV2640_CMD_GET_FRAME_SIZE` | 0x12 | `uint32_t *` | 读取最近一帧字节数 |
| `OV2640_CMD_START_STREAM` | 0x22 | `camera_stream_start_args_t *` | 启动连续流；通常由 `camera_start_stream()` 封装调用 |
| `OV2640_CMD_STOP_STREAM` | 0x23 | 无 | 停止连续流；通常由 `camera_stop_stream()` 封装调用 |

---

## 7. 开发指南——接入新摄像头传感器

以下以接入虚构的 `OV5640` 为例，按**当前 `ov2640` 驱动的真实接口**说明接入方式。重点有四条：

- 应用层不需要修改；但要把新驱动接进 Handle 层的编译期选择分支。
- 传感器驱动与数据总线之间只通过 `bus_adapter_*` 交互。
- 板级硬件参数应尽量放在总线层或 Kconfig 中，不要再沿用旧文档里“上层传 `dvp_cfg` 给总线”的模式。
- 当前工程的构建和配置入口仍然偏向 OV2640；新增传感器时，必须先补齐 `SConscript` 和 `Kconfig` 接线，否则代码即使写完也可能根本没有参与编译或没有 menuconfig 入口。

### 开始前：先判断是“替换现有传感器”还是“与现有传感器并存”

- **替换现有 OV2640**：这是当前框架最现实、最省改动的接入路径。你主要需要补传感器驱动、命令号映射，以及构建/配置入口。
- **与现有 OV2640 并存**：这不是“多复制一个驱动目录”就能完成的工作。当前 `ov2640` 驱动本身带有 single-instance 约束，底层 `sccb.c`、`dvp.c` 也仍有文件级状态和单实例假设。如果客户需求是“板上同时挂两种摄像头并可切换/并发”，要先评估驱动架构重构，而不是直接照着本节抄一份新驱动。

### 步骤零：先接通构建与配置入口

在开始写 `ov5640.c` 之前，先把“代码会不会被编译、菜单里能不能选到、Handle 层能不能选到它”这三个问题解决掉：

1. 更新 `camera_framework/Kconfig`。当前文件只定义了 `SENSOR_USING_OV2640`，如果新增 `OV5640`，至少要补一个对应的使能符号，以及它的总线/控制接口配置入口。
2. 更新 `camera_framework/SConscript`。当前整个 camera framework 组仍然受 `SENSOR_USING_OV2640` 控制；如果你只新增 `SENSOR_USING_OV5640` 而不改这里，旧开关一关，整个驱动组都可能不再参与构建。
3. 更新 `camera/handle/camera_handle.c` 的编译期选择分支，例如增加 `#elif defined(SENSOR_USING_OV5640)` 并返回 `&ov5640_ops`。
4. 若应用侧确实需要直接 `#include "ov5640.h"` 做底层调试，还要检查 `SConscript` 的 `CPPPATH` 是否已经覆盖新驱动目录。当前 include path 明确写了 `./camera/driver/ov2640`，并没有自动暴露新传感器头文件目录。
5. 如果只是临时替换 OV2640 做快速 bring-up，可以短期复用旧配置开关；但在准备交付给客户或长期维护前，应把 Kconfig/SConscript 改成“包级开关”和“具体传感器选择”分离的结构。

### 步骤一：创建驱动目录

```
camera/driver/ov5640/
    ov5640.h
    ov5640.c
    ov5640_regs.h
    ov5640_settings.h    ← 可选：如果有分辨率/格式表
```

### 步骤二：定义设备私有状态

建议对齐当前 `ov2640_device_t` 的职责拆分：

```c
typedef struct {
    bus_capture_mode_t mode;
    uint8_t           *frame_buffer;
    uint32_t           buffer_size;
} ov5640_bus_snapshot_t;

typedef struct {
    uint8_t                        *buffers[2];
    rt_size_t                       buffer_size;
    rt_uint8_t                      active_buffer_index;
    rt_uint32_t                     sequence;
    camera_stream_frame_callback_t  frame_callback;
    void                           *callback_context;
} ov5640_stream_state_t;

typedef struct {
    ov5640_t               sensor;
    bus_adapter_t         *data_bus;
    ov5640_bus_snapshot_t  bus_snapshot;
    struct rt_semaphore    frame_sem;
    ov5640_stream_state_t  stream;
    struct rt_mutex        ctrl_lock;   /* 若控制总线需要串行化，可加锁 */
} ov5640_device_t;
```

这里有两个接口名要特别注意：

- 流回调类型现在是 `camera_stream_frame_callback_t`，不是旧文档里的 `camera_frame_callback_t`。
- 单帧信号量当前 `ov2640` 用的是 `struct rt_semaphore` + `rt_sem_init/rt_sem_detach`，不是旧文档里的 `rt_sem_t` 动态创建风格。

### 步骤三：实现 `open` / `close`

推荐对齐当前 `ov2640_open()` / `ov2640_close()` 的职责顺序：

1. `rt_sem_init(&cam->frame_sem, "cam_frm", 0, RT_IPC_FLAG_FIFO)`。
2. 初始化控制总线，例如 `sccb_init()`。
3. 读取传感器 ID、下发默认寄存器表。
4. `cam->data_bus = bus_adapter_find(CAMERA_DATA_BUS_ADAPTER_NAME)`；当前如果 Kconfig 选择的是 `CAMERA_USING_DVP`，这里就会解析到 `"dvp"`。
5. `bus_adapter_set_frame_callback(cam->data_bus, ov5640_frame_ready_callback, dev)`。
6. `bus_adapter_init(cam->data_bus)`。

关闭顺序则反过来：

1. 清空流状态。
2. `bus_adapter_abort_capture()`。
3. `bus_adapter_stop()`。
4. `bus_adapter_deinit()`。
5. 控制总线 `deinit()`。
6. `rt_sem_detach(&cam->frame_sem)`。

### 步骤四：实现帧回调与单帧 `read`

当前 `ov2640` 的模式是“同一个总线回调同时服务单帧和流式”。最小骨架如下：

```c
static void ov5640_frame_ready_callback(bus_adapter_t *self,
                                        const bus_frame_t *bus_frame,
                                        void *user)
{
    ov5640_device_t *cam = (ov5640_device_t *)user;

    rt_base_t level = rt_hw_interrupt_disable();
    camera_stream_frame_callback_t cb = cam->stream.frame_callback;
    void *cb_ctx = cam->stream.callback_context;
    rt_hw_interrupt_enable(level);

    if (cb != RT_NULL)
    {
        rt_uint8_t completed = cam->stream.active_buffer_index;
        camera_stream_frame_t frame = {
            .buffer       = cam->stream.buffers[completed],
            .buffer_size  = cam->stream.buffer_size,
            .frame_size   = bus_frame->length,
            .sequence     = ++cam->stream.sequence,
            .buffer_index = completed,
        };

        cam->stream.active_buffer_index ^= 1U;
        if (bus_adapter_rearm_capture(self,
                                      cam->stream.buffers[cam->stream.active_buffer_index],
                                      cam->stream.buffer_size) != BUS_OK)
        {
            level = rt_hw_interrupt_disable();
            rt_memset(&cam->stream, 0, sizeof(cam->stream));
            rt_hw_interrupt_enable(level);
            bus_adapter_abort_capture(self);
            return;
        }

        cb(cb_ctx, &frame);
        return;
    }

    rt_sem_release(&cam->frame_sem);
    bus_adapter_stop(self);
    bus_adapter_abort_capture(self);
}
```

对应的 `read` 路径要点：

- 启动采集前清掉旧的信号量令牌。
- `bus_adapter_start_capture(cam->data_bus, buffer, size)` 只负责启动底层 DMA，不在这里传额外回调参数。
- `rt_sem_take()` 超时后要 `bus_adapter_abort_capture()`，必要时再输出诊断。

### 步骤五：实现 `control`

与当前 `ov2640_control()` 保持一致，最重要的是三件事：

1. **像素格式切换时同步切换总线模式**

```c
case OV5640_CMD_SET_PIXFORMAT:
{
    bus_capture_mode_t new_mode;
    ov5640_set_pixformat(&cam->sensor, format);
    ov5640_pixformat_to_bus_mode(format, &new_mode);

    if (cam->bus_snapshot.mode != new_mode)
    {
        bus_adapter_stop(cam->data_bus);
        bus_adapter_set_mode(cam->data_bus, new_mode);
        cam->bus_snapshot.mode = new_mode;
        bus_adapter_start(cam->data_bus);
    }
    return RT_EOK;
}
```

2. **`START_STREAM` 的顺序必须是：先填字段，再发布回调，再启动采集**

```c
case OV5640_CMD_START_STREAM:
{
    camera_stream_start_args_t *sa = (camera_stream_start_args_t *)args;
    cam->stream.buffers[0] = sa->buffers[0];
    cam->stream.buffers[1] = sa->buffers[1];
    cam->stream.buffer_size = sa->buffer_size;
    cam->stream.active_buffer_index = 0;
    cam->stream.sequence = 0;

    level = rt_hw_interrupt_disable();
    cam->stream.callback_context = sa->callback_context;
    cam->stream.frame_callback = sa->frame_callback;
    rt_hw_interrupt_enable(level);

    return bus_adapter_start_capture(cam->data_bus,
                                     cam->stream.buffers[0],
                                     cam->stream.buffer_size) == BUS_OK ? RT_EOK : -RT_ERROR;
}
```

3. **`STOP_STREAM` 的顺序必须是：先停底层采集，再清流状态**

```c
case OV5640_CMD_STOP_STREAM:
    bus_adapter_abort_capture(cam->data_bus);
    level = rt_hw_interrupt_disable();
    rt_memset(&cam->stream, 0, sizeof(cam->stream));
    rt_hw_interrupt_enable(level);
    return RT_EOK;
```

### 步骤六：暴露 camera_device_ops_t 并接入 Handle 层

驱动需要实现一个全局的 `camera_device_ops_t` 实例，将其所有函数指针实现为具体驱动函数：

```c
const camera_device_ops_t ov5640_ops = {
    .capabilities    = &g_ov5640_caps,
    .open            = ov5640_open,
    .close           = ov5640_close,
    .set_pixformat   = ov5640_apply_pixformat,
    .set_framesize   = ov5640_apply_framesize,
    .set_quality     = ov5640_apply_quality,
    .capture         = ov5640_capture,
    .start_stream    = ov5640_start_stream,
    .stop_stream     = ov5640_stop_stream,
};
```

再在 `camera/handle/camera_handle.c` 里接入分支：

```c
#elif defined(SENSOR_USING_OV5640)
#include "../driver/ov5640/ov5640.h"
static const camera_device_ops_t *camera_get_board_ops(void)
{
    return &ov5640_ops;
}
```

`camera_handler_instance_init()` 会通过 `camera_get_board_ops()` 取得 ops，并直接调用 `ops->open()`，不要求驱动注册到 RT-Thread camera 设备树。不要再使用 `INIT_DEVICE_EXPORT(ov5640_xxx)` 之类的自注册模式。

### 步骤七：第一次 bring-up 验证顺序

对第一次接入新传感器的人来说，建议按下面顺序验证，避免一上来就在 JPEG 图像质量或 AE/AWB 上浪费时间：

1. 先确认 menuconfig 里已经能看到新的传感器选项，且编译日志里新驱动源码确实被纳入构建。
2. 上电后验证 `camera_handler_instance_init()` 不返回错误，即 `ops->open()` 内 SCCB 初始化和传感器 ID 读取成功。
3. 只验证 `camera_handler_instance_init()`，先把“Handle 能选到新 ops、驱动能 open 成功”走通。
4. 先跑单帧模式，再跑流模式。单帧更容易定位是“设备没开起来”还是“流控时序有问题”。
5. 若新传感器支持测试图或 color bar，优先打开测试图验证数据通路，确认总线和缓冲区链路没问题，再去调真实图像参数。
6. 跑流模式时，先只看 `frame.sequence` 是否递增、`instance->stream.dropped_count` 是否稳定，再决定是否进入图像质量优化。
7. 如果 `camera_capture_single()` 或流启动超时，优先看驱动选择分支、总线名、Kconfig 引脚与时钟配置，再看图像配置本身。

### 常见陷阱

| 陷阱 | 当前正确做法 |
|------|-------------|
| 新驱动代码已经写完，但 menuconfig 里没有入口 | 先补 `camera_framework/Kconfig`，不要等驱动写完再补。 |
| menuconfig 有新传感器选项，但一关 `SENSOR_USING_OV2640` 就整个 camera framework 都不编译 | 说明 `camera_framework/SConscript` 还绑定在旧的 OV2640 开关上，需要先拆构建依赖。 |
| 新传感器已经编译进来了，但 `camera_handler_instance_init()` 还是选不到它 | 说明 `camera_handle.c` 还没加 `#elif defined(SENSOR_USING_XXX)` 分支。 |
| 希望新传感器和 OV2640 同时并存 | 先评估 single-instance 约束和底层总线状态设计，不能只靠复制目录解决。 |
| 继续使用旧类型名 `camera_frame_callback_t` | 使用 `camera_stream_frame_callback_t`。 |
| 继续按旧接口写 `bus_adapter_init(cam->data_bus, &cfg)` | 当前签名只有 `bus_adapter_init(bus_adapter_t *self)`。总线配置应在总线层内部完成。 |
| START_STREAM 先发布回调，再填 buffers/size | 先填缓冲区和索引，再发布回调，再启动底层采集。 |
| STOP_STREAM 先清 `stream` 再停总线 | 先 `bus_adapter_abort_capture()`，再在临界区清 `stream`。 |
| 假设 `camera/` 自带 PSRAM 分配器 | 当前目录没有这类接口，应用必须自备缓冲区来源。 |
| 改 `pixformat` 时只配传感器，不配总线 | 驱动需要同步更新 `bus_capture_mode_t`。 |
| 继续依赖 `INIT_DEVICE_EXPORT` 做 sensor 注册 | 当前没有 RT-Thread camera 设备注册路径；驱动需实现 `camera_device_ops_t` 表并展露 `xxx_ops`，由 Handle 层编译期选择。 |

---

## 8. 开发指南——接入新数据总线

当前代码结构下，新总线建议与 `dvp.c/.h` 同级放在 `camera/bus/data/`，不要再按旧文档放到额外子目录中。下面以接入一条虚构的 `dcmi` 总线为例说明当前接口契约。

> **先明确一件事**：只注册一个新的 `bus_adapter_t` 实例并不等于系统已经“切换到新总线”。具体使用哪条总线，是由传感器驱动决定的。以当前 `ov2640_open()` 为例，它会根据驱动内部配置去查找 `data_bus.name`，并在当前实现里要求该名字等于 `"dvp"`。所以如果你想让 OV2640 跑在新总线上，除了新增总线适配器，还必须同时修改传感器驱动的总线选择与配置注入逻辑。

### 步骤零：先接通总线选择与配置入口

在开始写 `dcmi.c` 之前，先确认以下几件事：

1. 你的目标是“给现有 OV2640 换总线”，还是“给未来的新传感器准备一条新总线”。前者必须同步修改传感器驱动内部的数据总线配置和选择逻辑。
2. 新总线需要哪些 Kconfig 或板级配置项。当前 DVP 的硬件资源几乎全部来自 `camera_framework/Kconfig`；如果新总线也需要引脚、时钟、DMA、lane 数等参数，最好一开始就把配置入口设计好。
3. 如果新总线文件不放在 `camera/bus/data/` 根下，而是新建额外子目录，要同步检查 `SConscript` 的 source glob 和 include path 是否仍然覆盖得到。
4. 如果要引入新的总线类型枚举，先修改 `data_bus_adapter.h` 的 `bus_type_t`，不要等到最后再回填类型系统。

### 步骤一：创建适配器文件

```
camera/bus/data/dcmi.h
camera/bus/data/dcmi.c
```

### 步骤二：定义私有状态

```c
#define DCMI_BUS_ADAPTER_NAME "dcmi"

typedef struct {
    void                        *active_buffer;
    uint32_t                     active_buffer_size;
    uint32_t                     last_frame_size;
    uint32_t                     frame_sequence;
    bus_capture_mode_t           mode;
    bus_frame_ready_callback_t   user_callback;
    void                        *user_data;

    /* 你的硬件句柄 */
    void *dma_handle;
    void *ctrl_base;
} dcmi_priv_t;
```

### 步骤三：按当前 `bus_adapter_ops_t` 实现接口

当前 `data_bus_adapter.h` 里的接口签名如下，文档旧版本的“`init(self, cfg)`”已经失效：

```c
typedef struct bus_adapter_ops {
    int      (*init)(bus_adapter_t *self);
    int      (*deinit)(bus_adapter_t *self);
    int      (*start)(bus_adapter_t *self);
    int      (*stop)(bus_adapter_t *self);
    int      (*set_frame_notify_callback)(bus_adapter_t *self,
                                   bus_frame_notify_callback_t callback,
                                   void *user_data);
    int      (*start_capture)(bus_adapter_t *self, void *buffer, uint32_t size);
    int      (*rearm_capture)(bus_adapter_t *self, void *buffer, uint32_t size);
    int      (*abort_capture)(bus_adapter_t *self);
    int      (*set_pingpong_size)(bus_adapter_t *self, uint32_t size);
    int      (*set_mode)(bus_adapter_t *self, bus_capture_mode_t mode);
    void     (*dump_state)(bus_adapter_t *self);
} bus_adapter_ops_t;
```

这意味着：

- `init()` 本身不接收外部配置结构体；若你的总线需要由传感器驱动显式注入硬件资源，可像当前 DVP 一样额外提供一个 `xxx_apply_config()` 入口，再由驱动在 `bus_adapter_init()` 前调用。
- `set_mode()` 和 `dump_state()` 虽然是可选，但若你的总线支持 JPEG / RAW / RGB565 / YUV422 多模式，建议实现。
- 若某个 op 不支持，可以留 `NULL`，上层 wrapper 会返回 `BUS_ERR_NOT_SUPPORTED`。

### 步骤四：实现最小骨架

```c
static int dcmi_init(bus_adapter_t *self)          { /* 从驱动注入配置或总线私有配置取资源 */ }
static int dcmi_deinit(bus_adapter_t *self)        { }
static int dcmi_start(bus_adapter_t *self)         { }
static int dcmi_stop(bus_adapter_t *self)          { }
static int dcmi_start_capture(bus_adapter_t *self, void *buffer, uint32_t size) { }
static int dcmi_rearm_capture(bus_adapter_t *self, void *buffer, uint32_t size) { }
static int dcmi_abort_capture(bus_adapter_t *self) { }
static int dcmi_set_mode(bus_adapter_t *self, bus_capture_mode_t mode) { }
static void dcmi_dump_state(bus_adapter_t *self)   { }

static int dcmi_set_frame_notify_callback(bus_adapter_t *self,
                                   bus_frame_notify_callback_t callback,
                                   void *user_data)
{
    dcmi_priv_t *priv = (dcmi_priv_t *)self->priv;
    priv->user_callback = callback;
    priv->user_data = user_data;
    return BUS_OK;
}
```

### 步骤五：ISR 中的职责

总线 ISR 只负责把“硬件完成一帧”的事实转成 `bus_frame_t` 并回调给上层，最小职责是：

1. 读取本帧实际字节数。
2. 填写 `bus_frame_t.buffer / length / timestamp / sequence`。
3. 若已注册 `user_callback`，则在 ISR 上下文调用它。

```c
static void dcmi_frame_isr(void *arg)
{
    bus_adapter_t *self = (bus_adapter_t *)arg;
    dcmi_priv_t *priv = (dcmi_priv_t *)self->priv;

    bus_frame_t frame = {
        .buffer    = priv->active_buffer,
        .length    = priv->last_frame_size,
        .timestamp = rt_tick_get(),
        .sequence  = ++priv->frame_sequence,
    };

    if (priv->user_callback != RT_NULL)
    {
        priv->user_callback(self, &frame, priv->user_data);
    }
}
```

### 步骤六：注册适配器实例

```c
static const bus_adapter_ops_t s_dcmi_ops = {
    .init               = dcmi_init,
    .deinit             = dcmi_deinit,
    .start              = dcmi_start,
    .stop               = dcmi_stop,
    .set_frame_callback = dcmi_set_frame_callback,
    .start_capture      = dcmi_start_capture,
    .rearm_capture      = dcmi_rearm_capture,
    .abort_capture      = dcmi_abort_capture,
    .update_buffer      = dcmi_update_buffer,
    .get_frame_size     = dcmi_get_frame_size,
    .set_mode           = dcmi_set_mode,
    .dump_state         = dcmi_dump_state,
};

static dcmi_priv_t s_dcmi_priv;
static bus_adapter_t s_dcmi_adapter = {
    .name = DCMI_BUS_ADAPTER_NAME,
    .type = BUS_TYPE_DCMI,
    .ops  = &s_dcmi_ops,
    .priv = &s_dcmi_priv,
};

static int dcmi_adapter_register(void)
{
    return bus_adapter_register(&s_dcmi_adapter);
}
INIT_BOARD_EXPORT(dcmi_adapter_register);
```

> 若现有 `bus_type_t` 没有合适枚举值，请先扩展 `data_bus_adapter.h` 中的 `bus_type_t`，不要再沿用旧文档里的 `BUS_TYPE_CSI` 这种当前代码里并不存在的枚举名。

### 步骤七：在传感器驱动中选择总线

```c
cam->data_bus = bus_adapter_find(DCMI_BUS_ADAPTER_NAME);
if (cam->data_bus == RT_NULL)
{
    LOG_E("dcmi adapter not found");
    return -RT_ERROR;
}
```

如果你是在“现有 OV2640 切换到底层新总线”的场景下工作，还需要把当前驱动里“按驱动配置选择总线名并下发总线资源”的逻辑一起替换掉：

```c
cam_dev->data_bus = bus_adapter_find(g_ov2640_hw_config.data_bus.name);
```

### 步骤八：第一次 bring-up 验证顺序

1. 启动后先确认新的 `INIT_BOARD_EXPORT()` 注册函数确实执行了；最直接的信号是 `bus_adapter_find(你的总线名)` 不再返回 `NULL`。
2. 再确认对应传感器驱动确实选择并配置了这条新总线，而不是仍然保持当前的 DVP 配置路径。
3. 先只验证 `bus_adapter_init()` 和单帧 `bus_adapter_start_capture()` 路径，再验证连续流的 `rearm_capture()`。
4. 如果是固定帧长模式，重点检查 VSYNC 对齐和帧边界；如果是 JPEG 模式，重点检查 SOI/EOI 解析和跨半缓冲拼接。
5. 超时时第一反应不是重写上层 API，而是优先实现或调用 `dump_state()`，把 DMA/计数器/中断状态打出来。

### `bus_adapter_ops_t` 当前字段合约

| 字段 | 是否建议实现 | 说明 |
|------|-------------|------|
| `init` | 是 | 初始化硬件和私有状态。当前签名只有 `self`。 |
| `deinit` | 是 | 释放资源，和 `init` 成对。 |
| `start` | 推荐 | 启动连续数据通路。 |
| `stop` | 推荐 | 停止连续数据通路。 |
| `set_frame_notify_callback` | 是 | 注册 ISR 上报回调。 |
| `start_capture` | 是 | 启动一次底层采集。 |
| `rearm_capture` | 推荐 | 流模式下切下一块缓冲区。 |
| `abort_capture` | 推荐 | 中止当前采集。 |
| `set_pingpong_size` | DVP 类总线推荐 | 调整内部中转缓冲大小。 |
| `set_mode` | 多像素格式总线推荐 | 响应 `pixformat -> bus_capture_mode_t` 切换。 |
| `dump_state` | 强烈建议 | 超时诊断时输出硬件状态。 |

---

## 9. 分层交互时序

### 单帧拍照

```
应用                Handle 层            OV2640 驱动            DVP
  │                    │                      │                   │
  ├─camera_capture_single()                   │                   │
  │                    ├─ops->capture()───────►                   │
  │                    │                      ├─bus_adapter_start_capture()
  │                    │                      │                   ├─DMA start
  │                    │                      │                   │  ···帧到来···
  │                    │                      │◄──dvp_dispatch_frame()
  │                    │                      ├─ov2640_frame_ready_callback()
  │                    │                      ├─rt_sem_release()  │
  │                    │◄─────────────────────┤                   │
  │◄──frame_size───────┤                      │                   │
```

### 连续流

```
应用线程             Handle 层             OV2640 驱动            DVP
  │                     │                      │                   │
  ├─camera_start_stream()                      │                   │
  │                     ├─ops->start_stream(&args)               │
  │                     │                      ├─填 buffers/size/callback
  │                     │                      ├─bus_adapter_start_capture(first buffer)
  │                     │                      │                   ├─DMA start
  │  (等待)             │                      │                   │
  │                     │                      │◄──dvp_dispatch_frame()
  │                     │                      ├─ov2640_frame_ready_callback()
  │                     │                      ├─bus_adapter_rearm_capture(next buffer)
  │                     │◄──camera_stream_frame_ready_callback()  │
  │                     ├─入队 + sem_release()                    │
  │◄─camera_get_stream_frame() 返回                               │
  │  处理帧 …           │                      │                   │
  │                     ├─camera_stop_stream()                    │
  │                     ├─ops->stop_stream()──────────────────────►│
  │                     │                      ├─bus_adapter_abort_capture()
  │                     │                      └─清 stream 状态   │
```

---

## 10. 常见问题

**Q1：`camera_handler_instance_init()` 返回 `CAMERA_ERRORRESOURCE`（找不到驱动）**  
确认 Kconfig 里是否已选中传感器项（`SENSOR_USING_OV2640`），且 `camera_get_board_ops()` 返回非 NULL；再验证 `ops->open()` 内部的 SCCB 初始化和传感器 ID 读取是否成功。

**Q2：`camera_handler_instance_init()` 返回 `CAMERA_ERROR`（设备存在但 open 失败）**  
优先检查底层驱动初始化、SCCB/I2C 总线、DVP 引脚与时钟配置，而不是上层 Handle 参数；当前 Handle 层不再要求应用传入 `device_ops`。

**Q3：串流一段时间后日志出现 `camera: stream queue full`**  
这是消费线程慢于传感器出帧导致的正常保护行为。Handle 层队列固定只有 2 槽，满时会覆盖最旧帧，并递增 `instance->stream.dropped_count`。

**Q4：`camera_change_settings()` 后首帧画面偏暗或偏色**  
这是正常现象。当前实现会固定等待 500 ms 让 AEC/AWB 收敛；若业务更敏感，可在正式取图前丢弃 1 到 2 帧。

**Q5：VSYNC 引脚到底应该填什么值？**  
当前 Kconfig `CAMERA_DVP_VSYNC_PIN` 需要填写 **PAx 的索引值**，例如 PA42 就填 `42`。OV2640 驱动会在内部通过 `DVP_RESOURCE_CONFIG(...)` 把 PCLK / HSYNC / VSYNC 组合成 DVP 资源描述，不需要在应用层传 `PAD_PA42` 这样的 PAD 宏。

**Q6：如何开启 DVP 调试快照？**  
把 `dvp.c` 顶部的 `#define DEBUG_DVP 0` 改成 `1`，重新编译后 `dump_buffer_hex()`、`snapshot_and_dump_buffer()` 以及对应调试快照缓冲区才会被编入。

**Q7：`bus_adapter_find()` 返回 `NULL`**  
说明目标总线适配器还没注册，或者注册名不一致。当前 DVP 通过 `INIT_BOARD_EXPORT(dvp_bus_adapter_register)` 自动注册为 `"dvp"`；新总线也应遵循同样模式。

**Q8：`camera_capture_single()` 超时后该怎么办？**  
当前 `ov2640_capture()` 超时时会主动调用 `bus_adapter_abort_capture()`，并在开启超时诊断时输出总线状态。因此第一步应检查引脚、XCLK、SCCB 和 Kconfig 是否正确；只有在连续超时且硬件状态异常时，才需要额外做一次 `camera_deinit()` + `camera_handler_instance_init()` 复位整个链路。

**Q9：`camera_stop_stream()` 之后还会残留旧帧吗？**  
Handle 层 `camera_stop_stream()` 会清空本地队列；下次 `camera_start_stream()` 还会主动排空历史信号量令牌，因此正常情况下不会把上一次流的旧帧带到下一次采集中。

**Q10：如何读取丢帧计数？**  
当前字段在 `instance->stream.dropped_count`，而不是旧文档里的 `instance->dropped_count`。例如：

```c
camera_stop_stream(&g_cam);
LOG_I("dropped frames: %u", g_cam.stream.dropped_count);
```

**Q11：为什么这版文档不再使用 `psram_heap_*`？**  
因为当前 `camera_framework/camera/` 目录中已经没有 `mem/` 和 `psram_heap_*` 实现。帧缓冲分配属于应用/板级职责，不再由摄像头框架目录直接提供。

**Q12：我已经把新传感器驱动文件加进来了，为什么 menuconfig 里看不到、编译里也没生效？**  
先检查三处入口：`camera_framework/Kconfig` 里是否真的新增了传感器开关；`camera_framework/SConscript` 是否仍然只依赖 `SENSOR_USING_OV2640`；`camera/handle/camera_handle.c` 是否已经把新宏接到对应 `xxx_ops`。当前工程不是“往目录里丢一个 `.c` 文件就自动接通所有配置和构建入口”的结构。

**Q13：新总线已经注册成功了，为什么 OV2640 还是走 DVP？**  
因为“总线已注册”和“传感器选择了这条总线”是两回事。当前 OV2640 驱动虽然已经把总线名放入驱动配置，但实现仍然要求数据总线后端是 DVP；如果不改传感器驱动的数据总线配置与初始化路径，新增总线不会被实际使用。

**Q14：能不能像文档示例那样直接再加一个新传感器，与 OV2640 同时存在？**  
如果你的意思是“替换 OV2640”，通常可以；如果你的意思是“与 OV2640 并存并同时维护两套实例”，当前实现不是现成支持的。需要先处理 single-instance 约束、底层总线状态共享、设备选择策略等架构问题。

**Q15：`camera_handler_instance_init()` 能不能当作 reset 或故障恢复接口反复调用？**  
不能。当前实现里，如果实例已经打开，`camera_handler_instance_init()` 会直接返回 `CAMERA_OK`，不会重置流状态、异步 worker、mailbox 或驱动运行态。若要做完整恢复，应按 `camera_stop_stream()`（如有）→ `camera_deinit()` → `camera_handler_instance_init()` 的顺序执行。

---

## 11. 架构决策记录（ADR）

### ADR-001：驱动注册控制权归 Handle 层

**决定日期**：2026-05-25

**背景**：旧实现中，sensor 驱动通过 `INIT_DEVICE_EXPORT(...)` 在启动阶段自注册 RT-Thread 设备，再额外通过全局注入方式把 ops 交给 Handle 层。这会让驱动注册时机、Handle 生命周期和应用初始化顺序分散在多个入口里，不利于维护，也让应用层容易感知具体 sensor 类型。

**决定**：

- 驱动不再使用 `INIT_DEVICE_EXPORT` 自注册。
- `camera_device_ops_t` 包含 `open`、`close`、`capture`、`start_stream`、`stop_stream` 等函数指针，各 sensor 驱动按该表提供实现。
- `camera_handler_instance_init()` 内部通过 `camera_get_board_ops()` 根据 Kconfig 选择具体 driver ops，并直接调用 `ops->open()` 完成驱动初始化。
- 应用初始化路径固定为：`camera_handler_instance_init(&cam)`（内部完成驱动打开）；业务层只依赖 `camera_handle.h`。

**当前绑定方式**：

```c
#ifdef SENSOR_USING_OV2640
#include "../driver/ov2640/ov2640.h"
static const camera_device_ops_t *camera_get_board_ops(void)
{
    return &ov2640_ops;
}
#else
#error "No camera sensor driver selected."
#endif
```

**新增传感器的最小接入面**：

1. 在 `camera_framework/Kconfig` 增加 `SENSOR_USING_XXX`。
2. 在 `camera_framework/SConscript` 接通对应构建依赖。
3. 在 `camera/handle/camera_handle.c` 增加 `#elif defined(SENSOR_USING_XXX)` 分支。
4. 在新驱动里暴露 `extern const camera_device_ops_t xxx_ops`，并实现其中每个函数指针。
5. 应用层 `main.c` 不改。
