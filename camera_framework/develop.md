# Camera 框架使用与开发手册

> 适用范围：`camera_framework/` 目录内的 Handle 层、传感器驱动（OV2640 / GC032A / BF30A2）、
> SCCB 控制总线、DVP / DCMI / SPI 数据总线，以及内置的软件 ISP / JPEG 编解码。
>
> 目标平台：SF32LB57X，RT-Thread + SiFli SDK。
>
> 应用层只需要包含一个头文件：`core/handle/camera_handle.h`。

---

## 目录

**第一部分 如何使用**

1. [架构概述](#1-架构概述)
2. [目录结构](#2-目录结构)
3. [快速上手（用户指南）](#3-快速上手用户指南)
   - 3.1 把框架接入工程
   - 3.2 选择传感器与数据接口
   - 3.3 单帧拍照（同步）
   - 3.4 非阻塞单帧拍照
   - 3.5 连续流式采集
   - 3.6 修改图像参数
   - 3.7 使用约束速查
4. [配置参考](#4-配置参考)
5. [缓冲区与内存约束](#5-缓冲区与内存约束)
6. [API 参考](#6-api-参考)

**第二部分 框架结构**

7. [源码选择与后端矩阵](#7-源码选择与后端矩阵)
8. [分层职责与采集调用链](#8-分层职责与采集调用链)

**第三部分 开发指南**

9. [接入新摄像头传感器](#9-接入新摄像头传感器)
10. [接入新数据总线（协议）](#10-接入新数据总线协议)
11. [接入“相机 + 控制器”外挂模块](#11-接入相机--控制器外挂模块)
12. [分层交互时序](#12-分层交互时序)
13. [测试与实板验收](#13-测试与实板验收)
14. [常见问题](#14-常见问题)

**附录**

- [A. 配置符号速查](#附录-a-配置符号速查)
- [B. 公共 API 速查](#附录-b-公共-api-速查)
- [C. 文件索引](#附录-c-文件索引)
- [D. OV2640 内部控制命令（驱动内部）](#附录-d-ov2640-内部控制命令驱动内部)

---

## 1. 架构概述

```text
┌─────────────────────────────────────────────────────────┐
│                    应用层代码                            │
│   camera_handle.h API  +  应用自管最终帧缓冲区            │
└───────────────────────┬─────────────────────────────────┘
                        │ camera_*()  公共 API
┌───────────────────────▼─────────────────────────────────┐
│              Handle 层  core/handle                      │
│  · 统一错误码  · 实例状态机  · 采集/流/异步  · 丢帧计数    │
└───────────────────────┬─────────────────────────────────┘
                        │ camera_device_ops_t
┌───────────────────────▼─────────────────────────────────┐
│  驱动发现  core/driver/camera_driver_desc                │
│  链接段 CameraDriverDescTab → camera_driver_get_default_ops│
└───────────────────────┬─────────────────────────────────┘
                        │ camera_device_ops_t
┌───────────────────────▼─────────────────────────────────┐
│  传感器驱动 core/driver/<sensor>（OV2640 / GC032A / BF30A2）│
│  · 能力表  · 寄存器表（SCCB）  · 模式切换  · 帧分发        │
└──────┬────────────────────────────────────┬─────────────┘
       │ camera_sensor_runtime_*()          │ sccb_init/write/read
┌──────▼────────────────────────────┐  ┌────▼─────────────────┐
│ 总线注册表 core/bus/data           │  │ 控制总线 SCCB         │
│ data_bus_adapter（名字 + 类型查找） │  │ core/bus/control      │
└──────┬────────────────────────────┘  └──────────────────────┘
       │ bus_adapter_ops_t
┌──────▼───────────────────────────────────────────────────┐
│ 数据总线实现                                              │
│ dcmi.c（DVP / 2-bit SPI）                                 │
│ arducam_fifo*.c（SPI FIFO）                               │
└──────────────────────┬───────────────────────────────────┘
                       │
                 SoC HAL / RT-Thread 设备
```

应用只看 handle；handle 通过链接段找到驱动；驱动配传感器；驱动通过注册表找到数据总线；总线负责搬数据；控制总线（SCCB）与数据总线（DVP/SPI）彼此独立。

**关键设计原则**

| 原则 | 当前实现 |
| --- | --- |
| 应用与传感器解耦 | 应用只包含 `camera_handle.h`，不感知传感器型号，也不需要 `ov2640.h` |
| 驱动发现不写死 | 每个驱动用 `CAMERA_DRIVER_EXPORT(name, ops)` 把描述符放进链接段 `CameraDriverDescTab`，handle 用 `camera_driver_get_default_ops()` 取当前编译进来的那一个 |
| 驱动不自己找总线 | 驱动只知道总线的**名字**（来自 Kconfig 的 `CAMERA_DATA_BUS_ADAPTER_NAME`）和**类型**，由 `bus_adapter_find()` 查找 |
| 切换摄像头＝改配置 | 传感器与接口都是 Kconfig 单选，换型号只需改 menuconfig 重新编译，应用代码不动 |
| 帧缓冲走向随后端不同 | DCMI 后端（并行 DVP 与 2-bit SPI 包模式）由 DMA 直写调用方缓冲，并自行维护该缓冲的 cache：`camera_dcmi_prepare_locked()` 把 DMA 目标设成调用方 `buffer`，`camera_dcmi_arm_stream_locked()`（启动流前）和 `camera_dcmi_single_work()`（单帧/JPEG 收尾后）各做一次 `SCB_InvalidateDCache_by_Addr`，两处都**不做 clean**；ArduCAM 后端先写内部乒乓缓冲，再整理成完整帧交给应用缓冲（见第 5 节） |
| 框架不提供分配器 | 没有 `psram_heap_*`；最终帧缓冲由应用或板级提供 |
| 缓冲归属与 cache 责任 | 流模式下缓冲借给驱动，必须有效到 `camera_stop_stream()` 返回。cache 维护以调用方为主；DCMI 后端已自行维护，不可重复维护 |

**驱动发现方式**

1. 每个传感器驱动导出一条描述符：`CAMERA_DRIVER_EXPORT(ov2640, &ov2640_ops)`。
2. 描述符进入链接段 `CameraDriverDescTab`。
3. Handle 层用 `camera_driver_get_default_ops()` 扫描该段，得到当前编译进来的驱动 ops。
4. 如果链接脚本删掉了这个 section，运行时就会找不到驱动 —— 新增驱动时不要改链接脚本。

---

## 2. 目录结构

```text
camera_framework/
├── Kconfig                 传感器 / 接口 / 总线 / 参数
├── SConscript              源码选择规则（一个配置只编一个后端 + 一个驱动）
├── README.md               能力概览
├── develop.md              本文件（唯一说明文档）
├── core/
│   ├── handle/
│   │   ├── camera_handle.h           ← 公共 API（应用只需要这一个头文件）
│   │   ├── camera_handle_internal.h  驱动与 handle 的内部契约
│   │   └── camera_handle.c
│   ├── driver/
│   │   ├── camera_driver_desc.[ch]   驱动链接段与发现
│   │   ├── camera_sensor_runtime.[ch] 采集/流公共逻辑
│   │   ├── camera_jpeg_assembler.[ch] JPEG 组帧
│   │   └── {ov2640,gc032a,bf30a2}/   传感器驱动与寄存器表
│   └── bus/
│       ├── control/sccb.[ch]         控制总线
│       └── data/
│           ├── data_bus_adapter.[ch] 注册表与包装 API
│           ├── camera_xclk.[ch]      MCLK 输出
│           ├── dcmi.c                DVP(DCMI) / 2-bit SPI
│           └── arducam_fifo*.c       SPI FIFO 模组
├── software/{isp,jpeg}/    软件 ISP 与 JPEG 编解码（含 vendor 源与 LICENSES）
├── examples/               可运行示例（take_photo_to_screen 等）
└── tests/                  框架测试
```


## 3. 快速上手（用户指南）

常规应用**只需要包含一个头文件**：

```c
#include "camera_handle.h"
```

`ov2640.h` / `gc032a.h` 等只在驱动开发或查阅内部命令时使用，不应成为业务层初始化路径的依赖。

### 3.1 把框架接入工程

框架是纯源码组件，不依赖独立库。接入一个 RT-Thread 工程只需要两处改动。

**(1) 在工程的 Kconfig 里引入框架的配置树**（例如 `project/Kconfig.proj`）：

```kconfig
rsource "../../camera_framework/Kconfig"      # 路径按实际相对位置调整
```

**(2) 在工程的 SConscript 里加入框架源码**（例如 `project/SConscript`）：

```python
camera_root = os.path.normpath(os.path.join(cwd, '../../camera_framework'))
objs.extend(SConscript(os.path.join(camera_root, 'SConscript'),
                       variant_dir='camera_framework',
                       duplicate=0))
```

框架的 `SConscript` 会同时把内置的软件 ISP / JPEG 组件（`software/`）挂进构建，工程侧不需要再单独添加它们。

**(3) 构建**（在工程目录下，先加载 SDK 环境）：

```sh
scons --board=<board> --menuconfig      # 选择传感器、接口、引脚
scons --board=<board> -j8               # 编译
```

> 板级引脚、SD 卡、按键等**工程相关**配置放在工程的 `proj.conf` 里；传感器/接口/总线相关配置由框架的 Kconfig 提供。

### 3.2 选择传感器与数据接口

所有摄像头配置都在 menuconfig 的 `Camera drivers` 菜单下：

| 菜单路径 | 作用 |
| --- | --- |
| `Sensor settings → Active camera sensor` | 选择当前使用的传感器（OV2640 / GC032A / BF30A2） |
| `Sensor settings → <传感器> data/output interface` | 该传感器用哪种**物理接口**输出图像（`8-bit DVP` / `2-bit SPI` / `Arducam SPI FIFO`） |
| `SCCB settings` | 控制总线（I2C 总线名、超时、速率、SCL/SDA 引脚） |
| `Data bus settings` | **数据总线后端**及其引脚/参数（2-bit 采集方式、ArduCAM FIFO 参数） |
| `<传感器> settings` | 该传感器专属参数（如 BF30A2 的缓冲） |
| `Camera software ISP` / `Camera software JPEG` | 是否使用软件 ISP / 编解码，以及运行在 HCPU 还是 ACPU |

选择逻辑是**两级**的：先选传感器，再选该传感器的接口。接口决定了框架编译哪个数据总线实现（见第 7 节）。

#### 两个必须知道的坑

1. **接口选择不会自动持久化。** 框架的配置来自 `Kconfig` + 工程的 `proj.conf`，构建目录里的 `.config` 是**输出**而不是输入。只靠 menuconfig 选的接口，在下一次普通 `scons` 时会回到默认值。要让某个接口配置可复现，请写进工程的 `proj.conf`：

```conf
CONFIG_CAMERA_OV2640_INTERFACE_ARDUCAM_FIFO=y
```

2. **并行 DVP 只有 SDK DCMI 一个后端，而且只支持 SF32LB57X。** 选择 `8-bit DVP` / `DVP` 接口后，`CAMERA_DVP_BACKEND_DCMI` 是隐藏符号（没有菜单项，无需选择），在 SF32LB57X 上 `default y` 并自动 `select BSP_USING_DCMI`，该后端直接使用 SDK 的 `dcmi` 设备驱动；框架会在编译期 `#error` 要求 `BSP_USING_DCMI` 已启用，外设、DMA 通道和中断向量都由 SDK 驱动持有。52x/56x 上没有任何并行 DVP 采集路径：GC032A 请选 `2-bit SPI`，OV2640 请选 `Arducam SPI FIFO`。

### 3.3 单帧拍照（同步）

> 以下 §3.3、§3.4、§3.5 的样例共用 `FRAME_BUFFER_BYTES`，需在文件开头定义，
> 取值不小于 `camera_get_capabilities()->max_buffer_size`：
>
> ```c
> #define FRAME_BUFFER_BYTES (640U * 480U * 2U)   /* 示例值：VGA RGB565 */
> ```
>
> §3.4 使用 `LOG_E`，另需 `#include <rtdbg.h>`。

```c
#include "camera_handle.h"

/* 大于等于能力表报告的最大一帧；DMA 可见，建议按缓存行对齐 */
static uint8_t frame_buffer[FRAME_BUFFER_BYTES] __attribute__((aligned(64)));
static camera_handler_instance_t *cam;

static int take_one_photo(void)
{
    const camera_capabilities_t *caps;
    camera_capture_config_t settings;
    camera_capture_request_t request;

    if (camera_handler_instance_init(&cam) != CAMERA_OK)      /* 打开驱动会话 */
        return -1;

    if (camera_get_capabilities(cam, &caps) != CAMERA_OK)    /* 查询能力表 */
        goto fail;

    /* 只在能力表里挑组合：pixformat ∈ caps->pixformats，
       framesize ∈ caps->framesizes */
    settings.pixformat = PIXFORMAT_RGB565;
    settings.framesize = FRAMESIZE_QVGA;
    settings.quality   = 0;
    if (camera_change_settings(cam, &settings) != CAMERA_OK)
        goto fail;

    request.buffer      = frame_buffer;
    request.buffer_size = sizeof(frame_buffer);
    request.frame_size  = 0;
    if (camera_capture_single_timeout(cam, &request, 1000) != CAMERA_OK)
        goto fail;

    /* 关键：任何返回路径上都要先成功 stop，再复用/释放目标缓冲区 */
    if (camera_stop_capture(cam) != CAMERA_OK)
        goto fail;

    /* 到这里 request.buffer 里是 request.frame_size 字节有效数据 */
    camera_deinit(&cam);
    return 0;

fail:
    camera_deinit(&cam);
    return -1;
}
```

> `camera_handler_instance_init()` 打开驱动，并把驱动的 `ops->default_config` 原样记入
> `active_config`（`camera_handle.c` 里 `handle->active_config = *ops->default_config;`）。该默认值随传感器变化：
>
> | 传感器 | 默认 `pixformat` | 默认 `framesize` | 默认 `quality` |
> | --- | --- | --- | --- |
> | OV2640（DCMI 后端） | `PIXFORMAT_RGB565` | `FRAMESIZE_SVGA` | 10 |
> | OV2640（非 DCMI 后端） | `PIXFORMAT_JPEG` | `FRAMESIZE_SVGA` | 10 |
> | GC032A（2-bit SPI） | `PIXFORMAT_RGB565` | `FRAMESIZE_VGA` | 0 |
> | GC032A（8-bit DVP） | `PIXFORMAT_RGB565` | `FRAMESIZE_QVGA` | 0 |
> | BF30A2 | `PIXFORMAT_RGB565` | `FRAMESIZE_240X320` | 0 |
>
> `open()` 阶段会下发其中的 `pixformat`（OV2640、GC032A）和 GC032A 的 `framesize`；
> `quality` 与 OV2640 的 `framesize` 不下发。需要确定的格式、分辨率或质量时，
> 在首次采集前调用 `camera_change_settings()`。

> **DCMI 后端的 JPEG 拍照**：DCMI 从 `CAPTURE` 置位那一刻就开始搬数据，而传感器是连续
> 出帧的，所以缓冲区开头可能是置位前就在传的那一帧的尾巴。用
> `camera_capture_frames_timeout()` 一次连续接收 `count` 帧（典型 9 帧预热 + 1 帧保留），
> 后端会在整段接收数据里找**最后一个能完整解析的 JPEG**（跳过开头的残帧、截到 EOI），
> 把它移到缓冲区开头作为结果。因此：
>
> - 缓冲区要能放下全部 `count` 帧（UXGA 建议 ≥ 1.9 MB，实测每帧不到 192 KB）；
> - 超时值要覆盖整段接收（UXGA 每帧约 146 ms，10 帧用 5 s 足够）；
> - 置位后若一帧都收不满，采集按超时失败。
>
> **拍照前必须让两条通道都真正停下**：`camera_stop_stream()` 成功还不够，还要
> `camera_stop_capture()` 成功（参考 `ai_camera` 的 `camera_sensor_stop_idle()`），
> 否则 DCMI 可能仍按上一次 stream 的 DMA 目标继续写入。若拍照缓冲区是借用预览的
> 双缓冲（两者同一块内存），漏停这一条的表现是：采集"完成"了、字节数也合理，但缓冲区
> 里还是预览像素，JPEG 标记一个都找不到。改设置/拍照前用
> `camera_stop_stream()` + `camera_stop_capture()` 双停并确认无 `CAMERA_ERRORRESOURCE`
> 重试即可。

### 3.4 非阻塞单帧拍照

> 沿用 §3.3 的 `cam` 与 `frame_buffer`；`do_something_with()` 为应用自己的处理函数。

```c
static void capture_done(void *context,
                         camera_handle_status_t status,
                         rt_size_t frame_size)
{
    (void)context;
    if (status != CAMERA_OK)
        return;
    do_something_with(frame_buffer, frame_size);
}

void capture_one_frame_async(void)
{
    camera_capture_request_t req = {
        .buffer      = frame_buffer,
        .buffer_size = sizeof(frame_buffer),
    };

    if (camera_capture_single_async(cam, &req, capture_done, RT_NULL) != CAMERA_OK)
        LOG_E("async capture start failed");
    /* req.buffer 要保持有效，直到回调触发 */
}
```

同一时刻只允许一笔在途请求；上一笔未完成前再次调用会返回 `CAMERA_ERRORRESOURCE`。

### 3.5 连续流式采集

> 沿用 §3.3 的 `cam`；`running` 为应用自己的循环标志，`use_frame()` 为帧消费函数。

```c
static uint8_t stream_buffer[2][FRAME_BUFFER_BYTES] __attribute__((aligned(64)));

void start_preview(void)
{
    camera_stream_config_t cfg = {
        .buffers     = { stream_buffer[0], stream_buffer[1] },
        .buffer_size = sizeof(stream_buffer[0]),
    };
    camera_start_stream(cam, &cfg);
}

void preview_thread(void *param)
{
    camera_stream_frame_t frame;

    while (running)
    {
        if (camera_get_stream_frame(cam, &frame,
                                    rt_tick_from_millisecond(1000)) == CAMERA_OK)
        {
            /* frame.buffer / frame.frame_size / frame.sequence / frame.buffer_index */
            use_frame(&frame);
        }
    }
}

void stop_preview(void)
{
    camera_stop_stream(cam);
}
```

> `camera_get_stream_frame()` 返回的是**浅拷贝**：`frame.buffer` 仍指向你提供的双缓冲，后续帧会继续复用这些槽位。要长期保留数据，调用方必须自行复制；CPU 读取前也要自行做 cache 维护。

需要 HT/TC 半帧通知时用 `camera_start_stream_mode(cam, &cfg, CAMERA_STREAM_MODE_HALF_FRAME)`：此时只用 `buffers[0]`，`buffers[1]` 必须为 `NULL`，`buffer_size` 必须等于一幅图像的大小，且图像高度为偶数（细节见 8.3）。

### 3.6 修改图像参数

`camera_change_settings()` **不可在流运行时调用**；需先 `camera_stop_stream()`，修改后再重新 `camera_start_stream()`。

```c
camera_capture_config_t cfg = {
    .pixformat = PIXFORMAT_RGB565,
    .framesize = FRAMESIZE_QVGA,
    .quality   = 10,
};
camera_change_settings(cam, &cfg);
```

改完之后传感器需要重新收敛 AE/AWB，建议正式取图前丢弃 1～2 帧。若需修改亮度、对比度等细粒度参数，当前框架没有公开 API（OV2640 驱动内部只有 `CMD_SET_PIXFORMAT` / `CMD_SET_FRAMESIZE` / `CMD_SET_QUALITY` 三个命令，见附录 D），可在 Handle 层扩展控制接口或直接改驱动默认值。

### 3.7 使用约束速查

| 约束 | 说明 |
| --- | --- |
| **缓冲区** | DMA 可见、建议 64 字节对齐；`buffer_size` 是总容量，写满即失败 |
| **先停后复用** | 任何单帧/连拍返回后必须先 `camera_stop_capture()` 成功；流用 `camera_stop_stream()` |
| **cache 维护** | 各后端不同：DCMI 自行维护调用方缓冲，不可重复维护；ArduCAM 不代调用方维护（见第 5 节） |
| **单实例** | 一个固件只绑定一个传感器，同一时刻只允许一个相机会话 |
| **忙状态** | 采集中改配置、流运行中启动单帧、异步在途时再次启动，都会返回 `CAMERA_ERRORRESOURCE` |
| **回调上下文** | 帧回调在中断/DMA 完成上下文：短小、不阻塞、不 `malloc` |
| **旋转** | `camera_set_rotation()` 只支持 0/180，且必须在停止采集时调用 |
| **格式能力** | 不要假设“能编就是能用”，请以 `camera_get_capabilities()` 的返回为准 |

## 4. 配置参考

### 4.1 Kconfig 选项

框架的型号/接口菜单已经移到 SDK 的 `customer/peripherals/camera/Kconfig`
（`Board Peripherals -> Camera 1`）；`camera_framework/Kconfig` 只保留
`PKG_USING_CAMERA_FRAMEWORK`、MCLK 派生符号和 `software/`。下表的符号名不变，
但 `SENSOR_USING_*` / `CAMERA_*` 由 SDK Camera 模块定义。

**传感器与接口**

| 选项 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `PKG_USING_CAMERA_FRAMEWORK` | bool | n | 框架使能符号（SF-PKG 入口）；由 Camera 模块在选中型号时 `select`，会 `select RT_USING_SYSTEM_WORKQUEUE` |
| `SENSOR_USING_OV2640` / `SENSOR_USING_GC032A` / `SENSOR_USING_BF30A2` | choice | GC032A | 当前激活的传感器（单选，在 SDK 的 Camera 1 菜单里）；通信形式在其子菜单里选 |

> 帧等待超时不再是菜单项，改由驱动自己的常量决定：
> `OV2640_FRAME_TIMEOUT_MS`（30000）、`GC032A_FRAME_TIMEOUT_MS`（1000）、
> `BF30A2_FRAME_TIMEOUT_MS`（1000）、`ARDUCAM_FIFO_FRAME_WAIT_MS`（30000）。

**控制总线（SCCB）**

框架 Kconfig **没有** SCCB 菜单了：

- 总线名与引脚都是板级事实：板级用 `CAMERA_I2C_DEVICE_NAME` 给出总线名，并在自己的 `bsp_pinmux.c` 里复用 SCL/SDA（框架不 `HAL_PIN_Set` 任何引脚）。
- 总线时序是驱动常量：`OV2640_SCCB_TIMEOUT_MS` / `_MAX_HZ`、`GC032A_SCCB_TIMEOUT_MS` / `_MAX_HZ`（BF30A2 在自己的 `i2c_init()` 里给出）。

**数据总线（通信形式）**

菜单跟着所选摄像头变：

| 所选传感器 | 菜单内容 |
| --- | --- |
| OV2640 | `OV2640 communication form`：`8-bit DVP（并行）` / `SPI（ArduCam FIFO 模组）` |
| GC032A | `GC032A communication form`：`8-bit DVP（并行）` / `2-bit SPI（串行）` |
| BF30A2 | `BF30A2 SPI bus`：`SPI1` / `SPI2`（它固定走 SPI） |
| OV2640 + ArduCam FIFO | 额外出现 `ArduCam SPI bus`：`SPI1` / `SPI2` |

| 选项 | 说明 |
| --- | --- |
| `CAMERA_OV2640_INTERFACE_DVP` / `_ARDUCAM_FIFO` | 8 线并行，或经 ArduCam FIFO 模组走 SPI（OV2640 本身没有串行像素输出） |
| `CAMERA_GC032A_INTERFACE_DVP_8BIT` / `_SERIAL_2BIT` | 8 线并行，或 GC032A 自带 2-bit 串行（进 DCMI SPI 包模式） |
| `CAMERA_ARDUCAM_SPI_BUS1/2` | ArduCam 模组挂在哪条 SPI；会 `select` 对应 SPI 驱动与 DMA |
| `CAMERA_BF30A2_SPI_BUS1/2` | BF30A2 用哪条 SPI；会 `select` 对应 SPI 驱动与环形 DMA |
| `CAMERA_DVP_BACKEND_DCMI` | 隐藏：并行 DVP 的唯一后端（SDK `dcmi` 设备驱动） |
| `CAMERA_SERIAL_SPI_2BIT` | 隐藏：2-bit 串行采集（同一个 DCMI 的 SPI 包模式） |

> 引脚永远不是菜单项：复用/上下拉/驱动能力都在板级 `bsp_pinmux.c`；SPI 设备名、SPI 频率、FIFO scratch/worker 栈、2-bit 串行包格式都是驱动/后端常量。
>
> 2-bit 串行的包格式属于传感器协议，由 `gc032a.c` 的 `bus_serial_config_t` 常量定义，运行时经 `bus_adapter_config_t.serial` 下发给 `dcmi.c`；驱动没给就直接拒绝配置。
>
> DCMI 系列后端（并行 DVP、`DCMI SPI 2-bit packet mode`）的引脚复用由开发板自己的 `bsp_pinmux.c` 在 `BSP_USING_DCMI` 块里完成，PCLK 采样沿由板级 `CAMERA_DCMI_PCLK_FALLING` 决定；应用不需要提供 `HAL_DCMI_MspInit()`。

**XCLK / 软件 ISP / 软件 JPEG**

| 选项 | 说明 |
| --- | --- |
| `CAMERA_XCLK_ENABLE` | **板级符号**：是否由 MCU 输出 MCLK；编进来的传感器需要时钟（OV2640 走 DVP / GC032A / BF30A2）而板级没定义它时，`camera_xclk.c` 编译期 `#error` |
| `CAMERA_XCLK_EXTERNAL_OSC` | **板级符号**：模组自带晶振，抵消上面那条需求检查（不需要 PWM 设备名/通道） |
| `CAMERA_XCLK_PWM_DEVICE_NAME` / `CAMERA_XCLK_PWM_CHANNEL` | **板级符号**：驱动 MCLK 的 PWM 设备与通道（MCLK pad 与定时器通道由 `bsp_pinmux.c` 复用） |
| `CAMERA_ISP_HARDWARE` / `CAMERA_ISP_SOFTWARE` | ISP 走传感器自带还是软件管线 |
| `CAMERA_SW_ISP_RUN_ON_HCPU` / `_ACPU`、`CAMERA_SW_ISP_GAMMA/AWB/TONE` | 软件 ISP 的运行核与各级开关 |
| `CAMERA_SW_JPEG`、`CAMERA_SW_JPEG_ENCODER` / `_DECODER`、`CAMERA_SW_JPEG_RUN_ON_HCPU` / `_ACPU` | 软件 JPEG 编解码 |
| `BSP_USING_DCMI` | 并行 DVP（或 2-bit SPI 包模式）时**必须为 y**（`CAMERA_DVP_BACKEND_DCMI` 会自动 select，框架编译期强制检查） |

### 4.2 关键宏与常量

| 宏 / 类型 | 定义位置 | 说明 |
| --- | --- | --- |
| `CAMERA_DATA_BUS_ADAPTER_NAME` | `Kconfig` | 当前数据总线适配器名，驱动与总线共用 |
| `CAMERA_DRIVER_EXPORT(name, ops)` | `camera_driver_desc.h` | 把驱动描述符放进链接段 `CameraDriverDescTab` |
| `camera_device_ops_t` | `camera_handle_internal.h` | 传感器驱动对 Handle 层的实现契约 |
| `camera_stream_start_args_t` | `camera_handle_internal.h` | Handle 层传给驱动的启动流参数 |
| `bus_adapter_t` / `bus_adapter_ops_t` | `data_bus_adapter.h` | 数据总线适配器与操作表 |
| `bus_type_t` | `data_bus_adapter.h` | `BUS_TYPE_DVP` / `BUS_TYPE_SPI` / `BUS_TYPE_DCMI` |
| `bus_status_t` | `data_bus_adapter.h` | `BUS_OK` / `BUS_ERR_*` |
| `camera_sensor_runtime_config_t` | `camera_sensor_runtime.h` | 驱动交给运行时层的总线选择信息 |
| `CMD_SET_PIXFORMAT` / `CMD_SET_FRAMESIZE` / `CMD_SET_QUALITY` | `core/driver/ov2640/ov2640.h` | OV2640 驱动内部命令 ID（仅这三个），不对业务层暴露（见附录 D） |

### 4.3 板级 `Kconfig.board` 示例（`sf32lb57-dpi-hdk_base`）

框架侧只读板级符号；下面是 SDK 里已经落地并可编译的完整写法
（`customer/boards/sf32lb57-dpi-hdk_base/Kconfig.board`）：

```kconfig
# ---- 相机 I2C：只有总线名，没有引脚 -------------------------------------
config CAMERA_I2C_DEVICE_NAME
    string
    default "i2c1"

menu "SF32LB57-SPI-HDK Configuration"      # 板级原样：菜单名沿用了 SPI-HDK 板

# ---- 引脚复用选择权在板级；bsp_pinmux.c 按所选 PINMAP 复用 DVP/MCLK ----
choice
    prompt "DVP Pinmap"
    default BOARD_DVP_PINMAP_0
    config BOARD_DVP_PINMAP_0
        bool "DVP_VSYNC on PA00"      # MCLK = PA02 -> GPTIM2_CH1
    config BOARD_DVP_PINMAP_1
        bool "TDVP_VSYNC on PA26"     # MCLK = PA28 -> GPTIM2_CH1
endchoice

# ---- 板级自己的分组开关（纯板级符号，框架不读它）-----------------------
config BOARD_CAMERA_HW
    bool "Camera connector wired (DVP pads + MCLK)"
    default y if BSP_USING_DCMI
    help
      Turn this off in a project that fits no camera: the PWM instance below is
      then not allocated at all. It matters for BOARD_DVP_PINMAP_0 in
      particular, whose DVP pads share PA00..PA10 with SD1/SD2.

if BOARD_CAMERA_HW

# ---- 外设分配：板级知道自己用的是哪个 PWM 驱动实例 ----------------------
config CAMERA_XCLK_ENABLE
    bool "MCU feeds the sensor MCLK"
    default y
    select BSP_USING_PWM
    select BSP_USING_PWMT2
    help
      Drive the sensor master clock from the board timer. Set to n for a module
      that carries its own oscillator; the PWM driver is then not pulled in
      either.

config CAMERA_XCLK_PWM_DEVICE_NAME
    string
    default "pwmt2"

config CAMERA_XCLK_PWM_CHANNEL
    int
    range 1 4
    default 1

endif # BOARD_CAMERA_HW

# ---- Camera 1 只做硬件开关；型号/接口在 SDK Camera 菜单里选 --------------
config BSP_USING_CAMERA1
    bool "Camera 1"
    default n
    help
      Enable this in proj.conf for projects that have a camera on the
      Camera 1 connector. The sensor model and communication form are
      selected under Board Peripherals -> Camera 1.

endmenu
```

要点：

1. **只出现 SDK 域和板级符号**：`BSP_USING_DCMI`、`BSP_USING_PWM(T2)` 是 SDK 域，`BOARD_*` / `CAMERA_*` 是板级；**不能出现** `CAMERA_FRAMEWORK_*` / `CAMERA_USING_*`。`BOARD_CAMERA_HW` 是板级内部的分组开关，框架不读它。
2. **`DVP Pinmap` choice 不要塞进 `if BOARD_CAMERA_HW`**：`bsp_pinmux.c` 的门控仍是 `#if defined(BSP_USING_DCMI)`（框架 select 的），把 choice 藏起来会让 `#ifdef BOARD_DVP_PINMAP_*` 两个分支都不成立 → 一个 pad 都不复用 → 相机静默不工作。
3. **两套 PINMAP 的 MCLK 都是 `GPTIM2_CH1`**，所以设备名/通道不需要按 PINMAP 分支；设备名 `"pwmt2"` 来自 SDK 的 `PWMT2_CONFIG`（`customer/boards/include/config/sf32lb57x/pwm_config.h`）。
4. **模组自带晶振**时不要打开 `CAMERA_XCLK_ENABLE`，而是在同一段里声明
   `config CAMERA_XCLK_EXTERNAL_OSC` + `bool`，这样框架的需求检查才会放过它。
5. **想让"关掉相机"也释放引脚**（`BOARD_DVP_PINMAP_0` 与 SD1/SD2 冲突时有用），把 `bsp_pinmux.c` 的
   `#if defined(BSP_USING_DCMI)` 换成 `#if defined(BOARD_CAMERA_HW)`；不改也没问题，只是关掉相机后引脚仍被复用。
6. **验证**：编译后 `grep -E "BOARD_CAMERA_HW|CAMERA_XCLK|BSP_USING_PWMT2" build_<board>_hcpu/rtconfig.h`，
   期望 `BOARD_CAMERA_HW=y`、`CAMERA_XCLK_ENABLE=y`、`CAMERA_XCLK_PWM_DEVICE_NAME="pwmt2"`、`CAMERA_XCLK_PWM_CHANNEL=1`、`BSP_USING_PWMT2=y`；
   板级目录里再 `grep -rnE "CAMERA_FRAMEWORK_|CAMERA_USING_" customer/boards/`，应无输出。

---

## 5. 缓冲区与内存约束

采集链路随后端实现不同，不存在统一的“先落内部缓冲、再整理给应用”顺序。

**DCMI 后端（SF32LB57x）：DMA 直写调用方缓冲**

```text
传感器 → DCMI + DMA → 应用提供的缓冲
```

`camera_dcmi_prepare_locked()` 将 DMA 目标设为调用方传入的 `buffer`；
`camera_dcmi_arm_stream_locked()`（启动流前）和 `camera_dcmi_single_work()`（单帧收尾后、按实际收到的长度）对调用方缓冲执行 `SCB_InvalidateDCache_by_Addr`。

这两处**都不做 clean**：缓冲是纯 DMA 目的地址，CPU 没有往里面写过需要回写的数据，
clean 只会把马上被 DMA 覆盖的旧内容写回 PSRAM。启动流时 invalidate 是为了丢掉上一轮
遗留的行；单帧则在传输结束、按实际收到的长度 invalidate，因此开始前不需要动整块容量
（JPEG 连拍的 arena 可达数 MB）。

**ArduCAM 后端：经内部乒乓缓冲中转**

```text
传感器 → 总线 DMA → 驱动/总线内部缓冲 → 驱动整理为完整帧 → 应用提供的最终帧缓冲
```

内存职责划分如下：

- **应用负责提供最终帧缓冲区。** 单帧用 `camera_capture_request_t.buffer`，连续流用 `camera_stream_config_t.buffers[0/1]`。
- **缓冲必须 DMA 可见**（内部 RAM / PSRAM，且不是 cache 别名），**建议 64 字节对齐**；DCMI 后端的流缓冲要求对齐到 64 字节，且单帧大小是 64 字节的整数倍。
- **容量按“能力表报告的最大一帧”准备**：`camera_capabilities_t.max_buffer_size` 就是最坏情况。
- **`buffer_size` 是总容量**：`camera_capture_frames_timeout()` 写满即失败，不会回绕覆盖。
- **流模式下缓冲是“借给驱动”的**：`camera_start_stream()` 返回后两块缓冲必须一直有效，直到 `camera_stop_stream()` 返回。
- **cache 一致性**：ArduCAM 不在帧派发前后自动 invalidate / clean，CPU 读取前需自行 invalidate，下一跳硬件 DMA 读取前按需 clean：

```c
/* DMA 写完后、CPU 读取前 */
mpu_dcache_invalidate((uint32_t *)frame.buffer, (uint32_t)frame.frame_size);

/* 若下一跳硬件 DMA 要读这块内存 */
mpu_dcache_clean(frame.buffer, frame.frame_size);
```

  DCMI 后端已维护调用方缓冲的 cache，不可重复维护。

- **不提供分配器**：框架没有 `psram_heap_*` / `mem/`，高分辨率 RGB565 / JPEG 示例通常使用 PSRAM，由应用或板级自行分配。

## 6. API 参考

> **线程安全总则**：所有 Handle 层 API 都假定运行在**线程上下文**；内部会用到信号量、互斥锁与阻塞等待，不适合在 ISR 中直接调用。

### 6.1 实例生命周期与状态机

```text
        camera_handler_instance_init()
             │  (查找驱动 ops → ops->open() → 记录默认 active_config)
        ┌────▼─────┐   camera_change_settings()
        │   IDLE   │◄───────────────────────────────┐
        └──┬────┬──┘        (下发格式/分辨率/质量)   │
           │    │                                  │
 camera_start_stream()  camera_capture_single*() / async
           │    │        (完成后回到 IDLE)          │
     ┌─────▼────┐└─────────────────────────────────┘
     │STREAMING │
     └─────┬────┘  camera_get_stream_frame()（消费帧）
           │ camera_stop_stream()
           ▼
          IDLE ──── camera_deinit() ──► （实例可再次 init）
```

| 状态 | 含义 | 可调用 API |
| --- | --- | --- |
| IDLE | 设备已打开，未在采集 | `camera_get_capabilities`、`camera_change_settings`、`camera_capture_single*`、`camera_capture_single_async`、`camera_start_stream[_mode]`、`camera_set_rotation`、`camera_deinit` |
| STREAMING | 连续流采集中 | `camera_get_stream_frame`、`camera_stream_frame_is_valid`、`camera_get_stream_dropped_count`、`camera_stop_stream`、`camera_deinit` |

两条重要约定：

- `camera_handler_instance_init()` **不是 reset 接口**：实例已打开时直接返回 `CAMERA_OK`，不会重置流状态或异步 worker。需要完整重建时按 `camera_stop_stream()`（如有）→ `camera_deinit()` → `camera_handler_instance_init()`。
- `camera_deinit()` 会关闭驱动会话并释放 handle 资源；它**不会**销毁实例本身，之后可以再次 init。

### 6.2 Handle 层 API

#### `camera_handler_instance_init`

```c
camera_handle_status_t camera_handler_instance_init(
                          camera_handler_instance_t **instance);
```

- 通过链接段找到当前编译进来的驱动 ops（`camera_driver_get_default_ops()`）；没有可用驱动时返回 `CAMERA_ERRORRESOURCE`。
- 调用 `ops->open()` 打开驱动（初始化 SCCB、读传感器 ID、初始化数据总线）。
- 在实例内记录默认 `active_config`：把驱动的 `ops->default_config` 原样拷贝进来（`camera_handle.c` 的 `active_config = *ops->default_config`），内容随传感器而变，典型值见 3.3 节的表。
- 该默认值中的 `pixformat`（OV2640 / GC032A）以及 GC032A 的 `framesize` 由 `ops->open()` 下发到传感器；`quality` 与 OV2640 的 `framesize` 不下发，需另行调用 `camera_change_settings()`。

#### `camera_get_capabilities`

```c
camera_handle_status_t camera_get_capabilities(
                          camera_handler_instance_t      *instance,
                          const camera_capabilities_t   **caps);
```

返回当前驱动的静态能力描述：`pixformats[]`、`framesizes[]`、`max_buffer_size`。应用应**只在能力表里挑组合**。

#### `camera_change_settings`

```c
camera_handle_status_t camera_change_settings(
                          camera_handler_instance_t      *instance,
                          const camera_capture_config_t  *config);
```

- 依次下发 `set_pixformat` → `set_framesize` → `set_quality`，成功后缓存为活动配置。
- **流运行中禁止调用**（返回 `CAMERA_ERRORRESOURCE`），需先 `camera_stop_stream()`。
- 当 `pixformat` 变化时，驱动会把像素格式映射为 `bus_capture_mode_t` 并切换总线采集模式。

#### `camera_capture_single` / `camera_capture_single_timeout`

```c
camera_handle_status_t camera_capture_single(
                          camera_handler_instance_t *instance,
                          camera_capture_request_t  *request);

camera_handle_status_t camera_capture_single_timeout(
                          camera_handler_instance_t *instance,
                          camera_capture_request_t  *request,
                          uint32_t                   timeout_ms);
```

- 阻塞式单帧采集；成功时 `request->frame_size` 为实际字节数。
- **任何返回路径上都必须先成功调用 `camera_stop_capture()`**，之后才能改配置或复用/释放目标缓冲。

#### `camera_capture_frames_timeout`

```c
camera_handle_status_t camera_capture_frames_timeout(
                          camera_handler_instance_t *instance,
                          camera_capture_request_t  *request,
                          uint32_t                   frame_count,
                          uint32_t                   timeout_ms);
```

- 一次硬件运行连续接收 `frame_count` 帧，只把**最后一帧**留在 `request->buffer` 起始处（`frame_count >= 1`）。
- `buffer_size` 是**总容量**，写满即失败（不会回绕覆盖）。
- 后端不支持时直接失败，**不会**退化成多次单帧。

#### `camera_capture_single_async`

```c
camera_handle_status_t camera_capture_single_async(
                          camera_handler_instance_t       *instance,
                          camera_capture_request_t        *request,
                          camera_capture_done_callback_t   callback,
                          void                            *context);
```

- 立即返回，完成或超时后在内部 worker 上下文调用 `callback(context, status, frame_size)`。
- `request->buffer` 必须一直有效到回调触发；同一时刻只允许一笔在途请求（否则 `CAMERA_ERRORRESOURCE`）。
- 流运行期间不可调用。

#### `camera_stop_capture`

```c
camera_handle_status_t camera_stop_capture(camera_handler_instance_t *instance);
```

停止同步/异步采集，并确认硬件与完成回调都已退出。返回 `CAMERA_OK` 之后才能复用目标缓冲。
返回 `CAMERA_ERRORRESOURCE` 可能只是完成回调还在返回途中——让出 CPU 后重试。

#### `camera_start_stream` / `camera_start_stream_mode`

```c
camera_handle_status_t camera_start_stream(
                          camera_handler_instance_t      *instance,
                          const camera_stream_config_t   *config);

camera_handle_status_t camera_start_stream_mode(
                          camera_handler_instance_t      *instance,
                          const camera_stream_config_t   *config,
                          camera_stream_mode_t            mode);
```

- 双缓冲连续流：`buffers[0]`、`buffers[1]` 均不可为 `NULL`，`buffer_size` 必须大于 0。
- `HALF_FRAME` 模式下只用 `buffers[0]`（`buffers[1]` 必须为 `NULL`），`buffer_size` 必须等于整幅图像大小（详见 8.3）。
- 两块缓冲必须持续有效，直到 `camera_stop_stream()` 返回。

#### `camera_get_stream_frame`

```c
camera_handle_status_t camera_get_stream_frame(
                          camera_handler_instance_t *instance,
                          camera_stream_frame_t     *frame,
                          rt_int32_t                 timeout);
```

在内部信号量上等待下一帧并**浅拷贝**描述符到 `*frame`。超时返回 `CAMERA_ERRORTIMEOUT`。
`frame.buffer` 仍指向你提供的那两块缓冲，后续帧会复用 —— 要长期保留必须自己复制。

#### `camera_stream_frame_is_valid`

```c
rt_bool_t camera_stream_frame_is_valid(camera_handler_instance_t       *instance,
                                       const camera_stream_frame_t     *frame);
```

判断该帧所指缓冲是否仍是“这一代”可读数据（见 8.3）。不做 cache 维护、不预留内存，也不阻止 DMA 重写缓冲。

#### `camera_stop_stream` / `camera_get_stream_dropped_count`

```c
camera_handle_status_t camera_stop_stream(camera_handler_instance_t *instance);

camera_handle_status_t camera_get_stream_dropped_count(
                          camera_handler_instance_t *instance,
                          rt_uint32_t               *dropped_count);
```

停止流并清空队列/计数；丢帧计数统计的是**本次启动以来**被覆盖掉的帧数。

#### `camera_set_rotation`

```c
camera_handle_status_t camera_set_rotation(camera_handler_instance_t *instance,
                                           uint16_t                   degrees);
```

只支持 0 / 180 度，且必须在**停止采集**时调用（流中或异步在途返回 `CAMERA_ERRORRESOURCE`）。重开或改配置后需要重新下发。

#### `camera_deinit`

```c
camera_handle_status_t camera_deinit(camera_handler_instance_t **instance);
```

关闭驱动会话、释放 handle 资源并把 `*instance` 置空。注意参数是**二级指针**。

### 6.3 错误码

| 枚举值 | 数值 | 含义 | 典型触发场景 |
| --- | --- | --- | --- |
| `CAMERA_OK` | 0 | 成功 | — |
| `CAMERA_ERROR` | 1 | 通用错误 | 驱动或底层返回了未映射的错误 |
| `CAMERA_ERRORTIMEOUT` | 2 | 超时 | 单帧采集超时；`camera_get_stream_frame()` 等待超时 |
| `CAMERA_ERRORRESOURCE` | 3 | 资源不可用 | 没有可用驱动；实例未打开；流运行中改配置；异步已有请求在途；停止时回调未退出 |
| `CAMERA_ERRORPARAMETER` | 4 | 参数非法 | `instance`/`config` 为空；缓冲为空或大小为 0 |
| `CAMERA_ERRORNOMEMORY` | 5 | 内存不足 | 信号量初始化失败；异步 worker 创建失败 |
| `CAMERA_ERRORISR` | 6 | ISR 上下文不允许 | 预留 |

## 7. 源码选择与后端矩阵

`camera_framework/SConscript` 的规则：**一个配置只编译一个数据总线后端 + 一个传感器驱动**。

```python
if GetDepend('CAMERA_USING_DVP'):
    src += ['./core/bus/data/dcmi.c']
elif GetDepend('CAMERA_USING_SERIAL'):
    src += ['./core/bus/data/dcmi.c']
elif GetDepend('CAMERA_USING_ARDUCAM_FIFO'):
    src += ['./core/bus/data/arducam_fifo*.c']
```

当前可选组合：

| 传感器接口 | 采集方式 | 编译的后端 | 注册名 / 类型 | 引脚复用责任 |
| --- | --- | --- | --- | --- |
| OV2640 / GC032A `8-bit DVP`（仅 57x） | SDK DCMI device driver | `dcmi.c` | `CAMERA_DATA_BUS_ADAPTER_NAME`（默认 `"dvp"`）/ `BUS_TYPE_DVP` | **板级** `bsp_pinmux.c`（`BSP_USING_DCMI` 块） |
| GC032A `2-bit SPI`（仅 57x） | DCMI SPI 2-bit packet mode | `dcmi.c`（`CAMERA_SERIAL_SPI_2BIT`） | `"dcmi"` / `BUS_TYPE_SPI` | **板级** `bsp_pinmux.c`（`BSP_USING_DCMI` 块，CLK/DI0/DI1） |
| OV2640 `Arducam SPI FIFO` | 模组自带 FIFO | `arducam_fifo.c` + `arducam_fifo_controller.c` | `"arducam_fifo"` / `BUS_TYPE_SPI` | **板级**（SPI pad）；传感器驱动只管 SCCB |
| BF30A2 | 模组自带 SPI 输出 | `bf30a2.c` 驱动 + SPI | 见驱动内声明 | **板级**（SPI + RESET pad） |

> 52x/56x 没有采图后端：GPIO 采样 + 定时器 DMA 的 2-bit 串行实现（`camera_serial*.c`）
> 已整体删除，这些平台只保留 SCCB/XCLK 等控制侧能力。

其他与源码选择相关的配置：

- `CAMERA_DATA_BUS_ADAPTER_NAME`：由 Kconfig 根据所选接口自动解析，**驱动与总线双方都用它**，不要手写常量。
- 控制总线（SCCB）、XCLK 与软件 ISP/JPEG 的开关见附录 A。

---

## 8. 分层职责与采集调用链

### 8.1 各层职责与关键文件

| 层 | 关键文件 | 职责 | 对外接口 |
| --- | --- | --- | --- |
| Handle | `core/handle/camera_handle.h/.c` | 实例生命周期、能力查询、同步/异步采集、连续流队列与丢帧计数、超时与忙状态保护 | `camera_*()` 公共 API（附录 B） |
| 驱动发现 | `core/driver/camera_driver_desc.h/.c` | 驱动通过 `CAMERA_DRIVER_EXPORT(name, ops)` 把描述符放进链接段 `CameraDriverDescTab`；handle 用 `camera_driver_get_default_ops()` 取当前编译进来的驱动 | `CAMERA_DRIVER_EXPORT` |
| 传感器驱动 | `core/driver/<sensor>/*.c` | 声明能力与默认配置；操作寄存器（SCCB）；把 `pixformat/framesize/quality` 落到具体寄存器或模式表；实现采集/流 | `camera_device_ops_t` |
| 传感器运行时 | `core/driver/camera_sensor_runtime.h/.c` | 采集/流的公共逻辑：帧信号量、超时、异步回调派发、JPEG 组帧、总线模式切换 | `camera_sensor_runtime_*()` |
| 总线注册表 | `core/bus/data/data_bus_adapter.h/.c` | 静态注册表与按“名字 + 类型”查找；`bus_adapter_*` 包装函数统一做 `self/ops` 校验 | `bus_adapter_register/find`、`bus_adapter_*` |
| 数据总线实现 | `core/bus/data/{dcmi,arducam_fifo}.c` | 真正搬像素：外设/DMA 配置、缓冲乒乓、帧边界、模式切换 | `bus_adapter_ops_t` |
| 控制总线 | `core/bus/control/sccb.c` | SCCB（I2C）读写（pad 由板级复用） | `sccb_init(&config)`、`sccb_write/read*` |
| XCLK | `core/bus/data/camera_xclk.c` | 只驱动板级声明的那一个 PWM 设备/通道（`CAMERA_XCLK_PWM_DEVICE_NAME` / `_CHANNEL`），不碰 pad；传感器需要时钟而板级没给时编译期 `#error`，板级声明 `CAMERA_XCLK_EXTERNAL_OSC` 或传感器不需要时钟时是空操作 | `camera_xclk_start(hz)` / `camera_xclk_stop(void)` |
| 软件 ISP/JPEG | `software/isp`、`software/jpeg` | 可选的后处理与编解码，独立于传感器与总线 | 见各自 README |

关键约定：

- **驱动不自己找总线实现**：`camera_sensor_runtime_open()` 内部用 `bus_adapter_find(config->adapter_name)` 并按 `adapter_type` 校验；名字来自 Kconfig 生成的 `CAMERA_DATA_BUS_ADAPTER_NAME`，类型由驱动按所选接口声明（如 `BUS_TYPE_DVP` / `BUS_TYPE_SPI`）。
- **总线实现不关心是谁在用**：同一份 `dcmi.c` 既能做 8-bit DVP，也能做 2-bit SPI 包模式，只是编译期宏和 `.type` 不同。
- **板级代码只在必须的地方出现**：DCMI 系列的 DVP/MCLK pad 由开发板自己的 `bsp_pinmux.c`（`BSP_USING_DCMI` 块）复用，应用不提供 `HAL_DCMI_MspInit()`；SPI 系列（ArduCAM / BF30A2）的 pad 同样由板级复用，适配器只用总线名。

### 8.2 一次采集的调用链

**同步单帧**

```text
camera_capture_single_timeout()
  └─ handle: 取 API 互斥锁 → 校验状态 → ops->capture_timeout(buffer, size, timeout)
       └─ sensor driver: camera_sensor_runtime_capture_timeout()
            └─ bus_adapter_start_capture(buffer, size)           // 硬件只搬一帧
            └─ 等帧信号量（bus_adapter_set_frame_notify_callback 注册的回调 release）
       └─ 返回后：调用方 camera_stop_capture()
            └─ bus_adapter_stop() / abort_capture()              // 释放硬件与回调
```

**连续流（FRAME 模式）**

```text
camera_start_stream()
  └─ handle: 注入内部帧回调 → ops->start_stream(args)
       └─ sensor driver: camera_sensor_runtime_start_stream()
            └─ bus_adapter_start_stream(&bus_stream_config)      // 两个挂起缓冲轮转
                 └─ 每完成一帧：回调（ISR/DMA 上下文） → runtime → handle 入队 + 计数
camera_get_stream_frame()   // 从队列取帧，队列满则覆盖最旧帧并计入 dropped_count
camera_stop_stream()
```

**异步单帧**：在内部 worker 上投递请求，完成或超时后回调 `camera_capture_done_callback_t`；同一时刻只允许一笔在途请求。

### 8.3 硬件 DCMI 后端（SF32LB57x）

GC032A 选择 `8-bit DVP`，或 OV2640 选择 `DVP`（两者都只在 SF32LB57X 上可选）；并行 DVP 只有 SDK DCMI 一个后端，`CAMERA_DVP_BACKEND_DCMI` 隐藏且默认启用，会自动 `select BSP_USING_DCMI`，不需要手动改动。
此后端通过 SDK 的 `dcmi` 设备驱动使用 DCMI 与 DMA：外设、DMA 通道和中断向量由 SDK 驱动持有，框架负责采集编程与 JPEG 帧收尾。DMA 半传输/全传输/错误经 `rt_device_set_rx_indicate` 上报；DCMI 的帧/溢出/包长事件由 **SDK 自己的** `HAL_DCMI_FrameEventCallback`（`drv_dcmi.c`）记录并通过同一个通知抛出，框架只额外重写 `HAL_DCMI_ErrorCallback`。D0-D7、PCLK、HREF、VSYNC 与 MCLK 的引脚复用由开发板自己的 `bsp_pinmux.c`（`BSP_USING_DCMI` 块）完成，应用不提供 `HAL_DCMI_MspInit()`；板级 `BSP_CAMERA_PowerUp()` / `BSP_CAMERA_PowerDown()` 由后端 init/deinit 调用。默认在 PCLK 上升沿采样，板级可用 `CAMERA_DCMI_PCLK_FALLING` 改为下降沿。

`camera_start_stream()` 默认使用 `CAMERA_STREAM_MODE_FRAME`：一次提交两块相邻、64 字节对齐的帧缓冲，单帧大小也必须是 64 字节的整数倍。DMA 半传输/全传输通知分别对应两个完整帧，后端不会逐帧停止再重启 DCMI。

**半帧模式（HALF_FRAME）**

用 `camera_start_stream_mode(instance, &config, CAMERA_STREAM_MODE_HALF_FRAME)`：

- `config.buffers[0]` 指向一块 64 字节对齐的**完整原始图像**缓冲，`config.buffers[1]` 必须为 `NULL`；
- `config.buffer_size` 必须等于整帧字节数，图像高度必须为偶数，每半幅字节数也须是 64 字节的整数倍；
- 例如 OV2640 800×600 RGB565 用一块 960000 字节缓冲，每半幅 480000 字节。

DMA 半传输（HT）产生 `CAMERA_STREAM_EVENT_HALF_FIRST`，全传输（TC）产生 `CAMERA_STREAM_EVENT_HALF_SECOND`，同一帧的两个事件 `sequence` 相同。
两个描述符的 `buffer` / `frame_size` 始终指向完整图像，消费者按 `event_type` 取上半幅或下半幅；成功事件中只有 `HALF_SECOND` 的 `is_complete` 为真。

**帧有效性校验**

`camera_stream_frame_is_valid()` 用来判断一个已出队的描述符所指的缓冲是否**仍是这一代数据**：

- `FRAME` 模式：比较采集序号与 DMA 的源读/目标写进度；队列里的旧描述符不能作为“像素仍有效”的依据。
- `HALF_FRAME` 模式：只比较事件所指半幅的最新 HT/TC 代次，不轮询 DMA 进度。`HALF_FIRST` 在对应 TC 到来时失效，`HALF_SECOND` 在下一次 HT 到来时失效；停止、错误和重新启动会使旧描述符失效。
- 该查询**不会**阻止 DMA 重写缓冲，也不做 cache 维护、不预留内存；只有同一 `sequence` 的两半都处理成功，才可发布拼接后的完整图像。

## 9. 接入新摄像头传感器

### 步骤 0：先判断是“替换”还是“并存”

`Active camera sensor` 是**单选**：一个固件只绑定一个传感器驱动。多个驱动可以同时存在于源码树里，靠 Kconfig 选择，但运行时只有一个是激活的（驱动内部多为单实例静态状态）。

### 步骤 1：Kconfig + SConscript 接线

```kconfig
config SENSOR_USING_MY_SENSOR
    bool "Use MY_SENSOR"

choice
    prompt "MY_SENSOR output interface"
    depends on SENSOR_USING_MY_SENSOR
    default CAMERA_MY_SENSOR_INTERFACE_DVP
config CAMERA_MY_SENSOR_INTERFACE_DVP
    bool "8-bit DVP"
config CAMERA_MY_SENSOR_INTERFACE_SPI
    bool "2-bit SPI"
endchoice

config CAMERA_USING_DVP
    bool
    default y if CAMERA_MY_SENSOR_INTERFACE_DVP
```

```python
elif GetDepend('SENSOR_USING_MY_SENSOR'):
    src += Glob('./core/driver/my_sensor/*.c')
    include += ['./core/driver/my_sensor']
```

### 步骤 2：声明能力表与默认配置

```c
static const pixformat_t s_pixformats[] = { PIXFORMAT_RGB565 };
static const framesize_t s_framesizes[] = { FRAMESIZE_QVGA, FRAMESIZE_VGA };
static const camera_capture_config_t s_default_config = {
    .pixformat = PIXFORMAT_RGB565, .framesize = FRAMESIZE_QVGA, .quality = 0,
};
static const camera_capabilities_t s_capabilities = {
    .pixformats      = s_pixformats,
    .num_pixformats  = sizeof(s_pixformats) / sizeof(s_pixformats[0]),
    .framesizes      = s_framesizes,
    .num_framesizes  = sizeof(s_framesizes) / sizeof(s_framesizes[0]),
    .max_buffer_size = 640U * 480U * 2U,      /* 最坏情况：应用据此判断缓冲是否够大 */
};
```

**编写能力表的三条纪律**

1. **只报真实能给的组合**：应用会优先选 RGB565，然后才 JPEG；某个格式在当前接口下不可用（例如外挂模块只搬 JPEG）就不要放进 `pixformats`。
2. **帧尺寸要与寄存器/模式表匹配**：能配出来的分辨率才放进 `framesizes`。
3. **`max_buffer_size` 给最坏情况**。

### 步骤 3：实现 camera_device_ops_t

把 SCCB、数据总线、时钟三组参数绑在一起，交给运行时层转发：

```c
/* 驱动私有的硬件配置 */
typedef struct {
    sccb_config_t                  sccb;
    camera_sensor_runtime_config_t runtime;
    uint32_t                       xclk_frequency_hz;
} my_sensor_hw_config_t;

static const my_sensor_hw_config_t s_hw_config = {
    .sccb = {
        .bus_name   = SCCB_I2C_BUS_NAME,     /* 板级 CAMERA_I2C_DEVICE_NAME */
        .timeout_ms = MY_SENSOR_SCCB_TIMEOUT_MS,  /* 驱动常量 */
        .max_hz     = MY_SENSOR_SCCB_MAX_HZ,      /* 驱动常量 */
    },
    .runtime = {
        .adapter_name     = CAMERA_DATA_BUS_ADAPTER_NAME,   /* 不要手写常量 */
        .adapter_type     = BUS_TYPE_DVP,                   /* 与接口选择一致 */
        .frame_timeout_ms = MY_SENSOR_FRAME_TIMEOUT_MS,     /* 驱动常量 */
        .default_config   = &s_default_config,
#if defined(MY_SENSOR_2BIT_SERIAL)
        .serial           = &s_serial_2bit,  /* 2-bit 串行包格式（驱动常量） */
#endif
    },
    .xclk_frequency_hz = 24000000U,   /* 模块自带时钟时为 0 */
};

static int my_sensor_open(void)
{
    /* 1) 需要 MCU 供时钟时：camera_xclk_start(s_hw_config.xclk_frequency_hz)（不可用则为空操作）
       2) sccb_init(&s_hw_config.sccb)                    ← 控制总线
       3) 读 ID 校验（强烈建议，见第 11 节）
       4) 写初始化表 / 应用默认模式
       5) camera_sensor_runtime_open(&s_device.runtime, &s_hw_config.runtime)  ← 打开数据总线
       错误路径按逆序回滚 */
}

static rt_size_t my_sensor_capture(void *buffer, rt_size_t size)
{ return camera_sensor_runtime_capture(&s_device.runtime, buffer, size); }

const camera_device_ops_t my_sensor_ops = {
    .capabilities           = &s_capabilities,
    .default_config         = &s_default_config,
    .open                   = my_sensor_open,
    .close                  = my_sensor_close,
    .set_pixformat          = my_sensor_set_pixformat,
    .set_framesize          = my_sensor_set_framesize,
    .set_quality            = NULL,      /* 不支持填 NULL */
    .capture                = my_sensor_capture,
    .capture_timeout        = my_sensor_capture_timeout,
    .capture_frames_timeout = my_sensor_capture_frames_timeout,   /* 可选 */
    .capture_async          = my_sensor_capture_async,
    .start_stream           = my_sensor_start_stream,
    .stop_stream            = my_sensor_stop_stream,
    .is_stream_frame_valid  = my_sensor_stream_frame_is_valid,
    .set_rotation           = NULL,      /* 可选 */
};
```

返回约定：

| 操作 | 返回 | 说明 |
| --- | --- | --- |
| `open` / `close` / `set_*` / `start_stream` / `stop_stream` | `int` | 0 成功，负值失败 |
| `capture` / `capture_timeout` / `capture_frames_timeout` | `rt_size_t` | **实际写入字节数**，0 表示失败 |
| `is_stream_frame_valid` | `rt_bool_t` | 该帧对应缓冲是否仍可读 |

### 步骤 4：导出驱动描述符

```c
CAMERA_DRIVER_EXPORT(my_sensor, &my_sensor_ops);   /* 放进链接段 CameraDriverDescTab */
```

Handle 层会自动扫描到这个描述符，**不需要改 `camera_handle.c`**。请确认链接脚本保留了 `CameraDriverDescTab` section，否则运行时找不到驱动。

### 步骤 5：参考实现

| 参考 | 适合学习 |
| --- | --- |
| `core/driver/gc032a/gc032a.c` | 同一颗传感器两种接口（8-bit DVP / 2-bit SPI）分支怎么写；能力表随接口收敛 |
| `core/driver/ov2640/ov2640.c` | 直连传感器（DVP，含窗口/裁剪 + 回读校验）与外挂模块（ArduCAM，JPEG 模式表）两条路径，以及“先总线握手、再配传感器”的顺序 |
| `core/driver/bf30a2/bf30a2.c` | 自带 SPI 输出、固定分辨率的模组 |

### 步骤 6：bring-up 验证顺序

1. 先确认 menuconfig 里能看到新传感器选项，且编译日志里新驱动源码确实参与构建。
2. 先只验证 `camera_handler_instance_init()` 不返回错误（即 `open()` 内 SCCB 初始化与 ID 读取成功）。
3. 再跑单帧，最后跑流。单帧更容易区分“设备没开起来”和“流控时序有问题”。
4. 若传感器支持彩条/测试图，优先打开它验证数据通路，再去调图像参数。
5. 跑流时先看 `frame.sequence` 是否递增、`camera_get_stream_dropped_count()` 是否稳定，再进入图像质量优化。

### 步骤 7：常见陷阱

| 陷阱 | 正确做法 |
| --- | --- |
| 驱动写完，menuconfig 里没有入口 | 先补 `camera_framework/Kconfig` |
| menuconfig 有选项，但整个框架不编译 | 检查 `camera_framework/SConscript` 的源码选择分支 |
| 编译进来了但 `init` 选不到驱动 | 检查是否用了 `CAMERA_DRIVER_EXPORT`，以及链接脚本是否保留 `CameraDriverDescTab` |
| 用 `INIT_DEVICE_EXPORT` 自注册设备 | 当前没有 RT-Thread camera 设备注册路径，只用 `CAMERA_DRIVER_EXPORT` |
| 改 `pixformat` 只配传感器、不切总线模式 | 驱动需同步把像素格式映射为 `bus_capture_mode_t` 并 `bus_adapter_set_mode()` |
| 想让两个传感器同时并存并按需切换 | 当前是编译期单选 + 单实例，需要先做架构评估 |

---

## 10. 接入新数据总线（协议）

适用场景：新芯片外设（CSI / 新 DVP 控制器）、新的串行协议、新的模组接口。三类改动：**Kconfig → 源码 → 注册**。

### 步骤 0：先决定“谁能用它”

- **谁复用引脚？** 一律由**开发板**的 `bsp_pinmux.c` 在对应的 `BSP_USING_xxx` 块里复用，框架只通过 RT-Thread 设备名使用外设，**不允许**适配器自己 `HAL_PIN_Set`（参考 `dcmi.c` 使用 SDK `drv_dcmi.c`、`arducam_fifo.c` 只用总线名）。
- **需要哪些参数？** 把这些参数做成 Kconfig 选项（包长、marker、同步字、SPI 频率），不要写死在代码里。

### 步骤 1：Kconfig 接线

```kconfig
config CAMERA_USING_MY_BUS
    bool
    default y if CAMERA_MY_SENSOR_INTERFACE_MY_MODE

config CAMERA_DATA_BUS_ADAPTER_NAME
    string
    default "my_bus" if CAMERA_USING_MY_BUS      # 名字必须与适配器里 .name 一致
```

同一个 `string` 符号可以有多个 `default ... if ...` 分支；把新分支放在合适的优先级上。

### 步骤 2：SConscript 加分支

```python
elif GetDepend('CAMERA_USING_MY_BUS'):
    src += ['./core/bus/data/my_bus.c']
```

### 步骤 3：实现适配器

当前 `bus_adapter_ops_t` 的字段（任一项可为 `NULL`，表示不支持；包装函数会返回 `BUS_ERR_NOT_SUPPORTED`）：

```c
typedef struct bus_adapter_ops {
    int (*config)(bus_adapter_t *self, const bus_adapter_config_t *config);
    int (*init)(bus_adapter_t *self);
    int (*deinit)(bus_adapter_t *self);
    int (*start)(bus_adapter_t *self);
    int (*stop)(bus_adapter_t *self);
    int (*set_frame_notify_callback)(bus_adapter_t *self,
                                     bus_frame_notify_callback_t callback,
                                     void *user_data);
    int (*start_capture)(bus_adapter_t *self, void *buffer, uint32_t size);
    int (*rearm_capture)(bus_adapter_t *self, void *buffer, uint32_t size);
    int (*abort_capture)(bus_adapter_t *self);
    int (*set_pingpong_size)(bus_adapter_t *self, uint32_t size);
    int (*set_mode)(bus_adapter_t *self, bus_capture_mode_t mode);
    void (*dump_state)(bus_adapter_t *self);
    int (*start_stream)(bus_adapter_t *self, const bus_stream_config_t *config);
    int (*stream_frame_valid)(bus_adapter_t *self, const bus_stream_frame_t *frame);
    int (*start_capture_frames)(bus_adapter_t *self, void *buffer,
                                uint32_t size, uint32_t frame_count);
} bus_adapter_ops_t;
```

帧通知回调签名（**在 ISR 上下文调用，只发布元数据**）：

```c
typedef void (*bus_frame_notify_callback_t)(void *buffer,
                                            uint32_t length,
                                            void *user_data);
```

最小骨架：

```c
#include "data_bus_adapter.h"

typedef struct { /* 私有状态：句柄、锁、DMA、缓冲、回调… */ } my_bus_t;
static my_bus_t s_my_bus;

static int my_bus_config(bus_adapter_t *self, const bus_adapter_config_t *config)
{
    /* 只校验并保存模式（BUS_CAPTURE_MODE_JPEG/RGB565/YUV422/RAW），不碰硬件 */
}

static int my_bus_init(bus_adapter_t *self)  { /* 一次性初始化：外设、DMA、引脚、信号量 */ }

static int my_bus_start_capture(bus_adapter_t *self, void *buffer, uint32_t size)
{
    /* 启动一次传输；完成后调用已注册的 frame_callback */
}

static int my_bus_stop(bus_adapter_t *self)
{
    /* 静默硬件与回调：停 DMA、清标志、确保没有回调在返回途中 */
}

static const bus_adapter_ops_t s_my_bus_ops = {
    .config                    = my_bus_config,
    .init                      = my_bus_init,
    .stop                      = my_bus_stop,
    .abort_capture             = my_bus_stop,
    .set_frame_notify_callback = my_bus_set_callback,
    .start_capture             = my_bus_start_capture,
    .set_mode                  = my_bus_set_mode,
    /* 不支持的能力留 NULL：deinit / start / rearm_capture / start_stream /
       stream_frame_valid / start_capture_frames / set_pingpong_size / dump_state */
};

static bus_adapter_t s_my_bus_adapter = {
    .name = CAMERA_DATA_BUS_ADAPTER_NAME,
    .type = BUS_TYPE_SPI,                     /* 按协议选 bus_type_t */
    .ops  = &s_my_bus_ops,
    .priv = &s_my_bus,
};

static int my_bus_register(void)
{
    return bus_adapter_register(&s_my_bus_adapter);
}
INIT_BOARD_EXPORT(my_bus_register);           /* 开机自动注册 */
```

### 步骤 4：操作合约

| 操作 | 谁调用 / 何时 | 实现要点 |
| --- | --- | --- |
| `config` | 驱动 open / 改设置时 | 只校验并保存模式，不动硬件；不支持的模式返回 `BUS_ERR_NOT_SUPPORTED` |
| `init` / `deinit` | 驱动 open / close | 可重入保护（已初始化直接返回 `BUS_OK`）；`deinit` 必须让硬件与回调彻底静默 |
| `start` / `stop` | 生命周期 | 保证 `stop` 之后可以安全复用调用方的缓冲 |
| `set_frame_notify_callback` | 驱动 open 时 | 回调在 **ISR/DMA 上下文**执行：只发布元数据，不做长活 |
| `start_capture(buffer,size)` | 同步单帧 | 一次传输，容量不足要能失败（不要回绕覆写） |
| `rearm_capture` | 需要重新武装的硬件 | 不支持填 `NULL` |
| `abort_capture` | 停止/错误路径 | 与 `stop` 等价的“静默”语义 |
| `set_pingpong_size` | 运行期调整乒乓大小（可选） | 返回 `BUS_OK`/负错误码，不要直接改硬件 |
| `set_mode` | 运行期改采集模式 | 先 abort/stop，再改内部状态；返回时保持原有生命周期状态 |
| `dump_state` | 诊断（可选） | 打印 DMA/定时器寄存器等，便于现场定位 |
| `start_stream(config)` | 连续流 | FRAME 模式轮转两个图像缓冲；HALF_FRAME 只用 `buffers[0]`；不支持返回 `BUS_ERR_NOT_SUPPORTED` |
| `stream_frame_valid(frame)` | 流消费侧校验（可能关中断调用） | 只做“这一代缓冲是否仍可读”的短查询，不做 cache 维护、不加锁 |
| `start_capture_frames(buffer,size,count)` | JPEG 连拍取最后一帧 | 硬件只启动一次，收到的最后一帧放在 `buffer`；不支持返回 `BUS_ERR_NOT_SUPPORTED`（不要退化成多次单帧） |

返回码统一用 `bus_status_t`：`BUS_OK(0)`、`BUS_ERR_INVALID(-1)`、`BUS_ERR_NOT_SUPPORTED(-2)`、`BUS_ERR_NO_SLOT(-3)`、`BUS_ERR_HW(-4)`。

### 步骤 5：通用 checklist

- [ ] `.name` 与 Kconfig 的 `CAMERA_DATA_BUS_ADAPTER_NAME` 一致；`.type` 用现有 `bus_type_t`
- [ ] 用 `INIT_BOARD_EXPORT()` 注册，注册失败要有日志
- [ ] 长度/对齐校验（建议 64 字节对齐）
- [ ] 所有返回码用 `BUS_OK` / `BUS_ERR_*`，不要混用 RT-Thread 错误码
- [ ] 缓冲所有权与 cache 责任写清楚（是 DMA 直写调用方缓冲，还是经内部乒乓缓冲中转；若后端会代做 cache 维护，必须写明，参考 DCMI 的做法）
- [ ] 错误路径上不能留下已启动的 DMA 或悬空回调
- [ ] 超时交给上层（驱动常量 `*_FRAME_TIMEOUT_MS`），总线不要无限等待

## 11. 接入“相机 + 控制器”外挂模块

有些模组（典型是 ArduCAM SPI FIFO 系列）在相机前面还有一颗控制器/CPLD：主机通过 SPI 与它交互，通过 SCCB 配置里面的相机。接入这类模块时，下面几条经验能省掉大量时间：

1. **顺序：先让控制器握手/复位，再访问相机 SCCB。**
   控制器没放通之前，SCCB 上往往仍能读到相机 ID（硬连线）却写不进任何寄存器、读回来全是 `0xFF`。因此驱动要**先打开数据总线**（`camera_sensor_runtime_open()` 会初始化适配器并完成控制器握手），**再**写传感器寄存器表。顺序反过来就会出现“ID 正常、其余全 `0xFF`、写入无效”的迷惑现象。
2. **握手后立刻校验相机 ID**（如 `PID/VER`），失败就报错退出。这样“模块里没有相机 / 控制器没起来”会在 open 阶段暴露，而不是等到采集超时。
3. **能力收敛**：这类模块通常只搬某种固定格式（ArduCAM FIFO 常见是 JPEG）。只把这种格式报给上层，让应用自动走对应路径。
4. **用模块自带的寄存器表**，不要套用“直连传感器”的窗口/裁剪逻辑。模块交付时配套的模式表（初始化表 + 每分辨率一张表）才是它工作状态的来源。
5. **不要给自带时钟的模块配 MCLK。** 这类模组自带晶振，板级声明 `CAMERA_XCLK_EXTERNAL_OSC` 而不是 `CAMERA_XCLK_ENABLE`；若某个 pad 同时被 XCLK 和 SPI 使用，后 `HAL_PIN_Set` 的一方会把前一方“抢走”，且没有任何报错 —— 把引脚唯一性做成板级编译期检查可以避免这类问题。
6. **间歇性故障要留证据**：寄存器扫描（把整片低地址寄存器打出来）、写回测试（写一个已知值再读回）、带返回码打印，比“猜”有效得多。

---

## 12. 分层交互时序

### 12.1 同步单帧

```text
应用                Handle 层            传感器驱动            数据总线
  │                    │                      │                   │
  ├─camera_capture_single_timeout()           │                   │
  │                    ├─取 API 互斥锁、校验状态 │                   │
  │                    ├─ops->capture_timeout()►                   │
  │                    │                      ├─bus_adapter_start_capture()
  │                    │                      │                   ├─DMA start
  │                    │                      │   ···帧到来（ISR）···
  │                    │                      │◄──bus 回调────────┤
  │                    │                      ├─release 帧信号量   │
  │                    │◄─────────────────────┤                   │
  │◄──frame_size───────┤                      │                   │
  ├─camera_stop_capture()                     │                   │
  │                    ├─ops / bus stop + abort───────────────────►│
```

### 12.2 连续流（FRAME 模式）

```text
应用线程             Handle 层            传感器驱动            数据总线
  │                     │                      │                   │
  ├─camera_start_stream()                      │                   │
  │                     ├─注入内部帧回调       │                   │
  │                     ├─ops->start_stream(&args)                │
  │                     │                      ├─bus_adapter_start_stream()
  │                     │                      │                   ├─DMA start
  │  (等待)             │                      │                   │
  │                     │                      │◄──bus 帧回调（ISR）┤
  │                     │◄──runtime → handle 入队 + sem_release    │
  │◄─camera_get_stream_frame() 返回             │                   │
  │  处理帧 …           │                      │                   │
  ├─camera_stop_stream()│                      │                   │
  │                     ├─ops->stop_stream()──────────────────────►│
```

> 帧回调运行在 **ISR / DMA 完成上下文**：只发布元数据、入队并唤醒等待者，不做长活、不阻塞、不 `malloc`。

---

## 13. 测试与实板验收

### 13.1 框架自带测试

`camera_framework/tests/` 下分三类：

- **主机侧静态检查**（不需要板子，`PYTHONPATH` 指向 SDK 的 `kconfiglib.py`）：
  - `camera_xclk_ownership_test.py`：SCCB 不碰 pad；`camera_xclk.c` 只驱动板级给的 PWM（无 pad 扫描、无写死设备名、缺名字必须 `#error`）；XCLK 先于 SCCB 启动，每条 open 失败路径都 stop
  - `camera_examples_capability_test.py`、`camera_sdcard_example_test.py`：示例的 Kconfig/`proj.conf` 与它宣称的能力一致
  - `bf30a2_driver_review_test.py`、`gc032a_settings_test.py`：驱动寄存器表/参数表检查
  - `camera_kconfig_test.py`：菜单结构、派生符号、板级符号方向；它需要一个 SoC 桩（`Kconfig.soc_harness`），**该桩不在仓库里**，缺它会打印 `SKIP` 并以 0 退出
- **纯逻辑单测**（主机上单独编译）：`ov2640_jpeg_assembler_test.c`、`camera_xclk_test.c`
- **实板套件**：`tests/handle`（fake `camera_device_ops_t`）+ `run_camera_sensor_smoke`

> 真正的端到端验证以实板为准：`scons --board=<board>` 编过 + 启动日志里传感器 ID / 帧率正常。
> 文档层面的一致性由 `develop.md` 各节自述的契约（§4.1、§4.3、附录 A）保证，不依赖测试脚本。

运行方式见 `tests/README.md`。

### 13.2 实板验收顺序

1. **单帧同步采集**：日志里出现相机 ID，`camera_capture_single_timeout()` 成功，`frame_size > 0`，文件/内存里能看到正确图像。
2. **连续流**：`camera_get_stream_frame()` 稳定出帧，`camera_get_stream_dropped_count()` 不持续增长，停止后再次启动仍正常。
3. **参数切换**：反复切换 `pixformat/framesize/quality`，确认每次都在 stop 之后进行且结果正确。
4. **错误恢复**：故意制造超时（拔掉数据线 / 让模块 busy），确认 API 返回错误码、能重新初始化并再次采集。
5. **边界**：最小/最大分辨率、最长超时，以及（若支持）JPEG 连拍与半帧流。

### 13.3 示例提供的控制台命令

以 `examples/take_photo_to_screen` 为例：

| 命令 | 作用 |
| --- | --- |
| `camera_stats [window_ms]` | 按时间窗口统计帧数、ISP/拍照耗时与双核占用 |
| `camera_preview [rotate 0\|1] [background 0\|1] [swap 0\|1]` | 运行时改显示旋转/背景层/字节序，确认方向与颜色 |
| `camera_photo` | 立即拍一张并写入文件系统 |

### 13.4 日志 tag

框架使用 RT-Thread ulog：`camera.dcmi`、`cam.ardufifo`、`ov2640`、`gc032a`、`sccb`、`camera.xclk`、`bf30a2`。
出错时优先看三类：SCCB 写失败、握手失败、帧超时。启动日志里打印的传感器 ID 不对，就不必再往下看图像了。

## 14. 常见问题

### 14.1 快速排查表

| 现象 | 可能原因与处理 |
| --- | --- |
| 编译报 `The hardware DCMI camera backend requires SF32LB57X` 或 `The camera DCMI backend requires the SDK DCMI driver: enable BSP_USING_DCMI` | 并行 DVP 只支持 SF32LB57X：52x/56x 上没有 DVP 采集路径，请改用 GC032A 的 `2-bit SPI` 或 OV2640 的 `Arducam SPI FIFO`。若是后一条，说明 `BSP_USING_DCMI` 没打开，应让 `CAMERA_DVP_BACKEND_DCMI` 自动 select 它 |
| `camera_handler_instance_init()` 返回 `CAMERA_ERRORRESOURCE` | 没有可用驱动：确认 `Camera drivers` 里选中了传感器，且链接脚本保留了 `CameraDriverDescTab` section |
| `camera_handler_instance_init()` 返回 `CAMERA_ERROR` | 驱动 open 失败：优先看 SCCB/I2C、引脚、XCLK 与传感器 ID 读取，而不是上层参数 |
| 日志出现 `adapter not found` 或总线类型不匹配 | Kconfig 选的接口与驱动声明的总线类型不一致；检查 `Data bus settings` 与 `CAMERA_DATA_BUS_ADAPTER_NAME` |
| 改完 menuconfig，下一次 `scons` 又变回去了 | 接口选择要写进工程的 `proj.conf`（见 3.2 节） |
| SCCB 扫描能看到地址，但读寄存器全是 `0xFF`、写不进 | 典型的“相机还没被放通”：外挂模块（如 ArduCAM）需要先做控制器握手再访问相机 SCCB，见第 11 节 |
| 帧一直超时 | 数据链路没起来：DVP 要确认开发板 `bsp_pinmux.c` 的 `BSP_USING_DCMI` 块已复用相机 pad 且 `BSP_USING_DCMI` 已打开，SPI FIFO 要确认模块握手成功；超时值本身在驱动常量（`*_FRAME_TIMEOUT_MS`）里，不够就改驱动 |
| 画面方向/颜色不对 | 用示例的 `camera_preview` 运行时试换旋转/字节序，再把结论写回工程；注意软件 ISP 只接受大端 RGB565 |
| 帧率远低于预期 | 软件 ISP/JPEG 在 ACPU 或 HCPU 上串行执行；用示例的 `camera_stats` 把 `isp/write/display` 拆开看时间花在哪一层 |
| 图像看起来像“半张 / 撕裂” | 流模式下多半是 cache 维护或缓冲区归属问题：CPU 读之前自行 invalidate，并确认 `frame.buffer` 仍是本代数据（`camera_stream_frame_is_valid()`） |

### 14.2 常见问答

**Q：`camera_handler_instance_init()` 是 reset / 故障恢复接口吗？**

不是。实例已经打开时它直接返回 `CAMERA_OK`，不会重置流状态、异步 worker 或驱动运行态。需要完整重建时按
`camera_stop_stream()`（如有）→ `camera_deinit()` → `camera_handler_instance_init()` 的顺序执行。

**Q：`camera_change_settings()` 之后第一帧偏暗或偏色？**

改格式/分辨率后传感器需要重新收敛 AE/AWB，属正常现象。示例的拍照路径会先接收若干帧预热、只保留最后一帧，可以直接照抄这个做法。

**Q：`camera_capture_single*()` 返回 `CAMERA_ERRORRESOURCE`？**

通常是有别的操作在途：流正在运行、异步采集未完成，或上一帧的完成回调还没退出。让出 CPU 后重试，或先 `camera_stop_capture()` / `camera_stop_stream()`。

**Q：流模式下取到的帧可以长期保存吗？**

不可以。`camera_get_stream_frame()` 返回的是浅拷贝，`frame.buffer` 指向你自己提供的那两块缓冲，后续帧会继续复用。要长期保留必须自己复制一份。

**Q：`CAMERA_ERRORRESOURCE` 出现在 `camera_stop_capture()` 上怎么办？**

可能意味着完成回调还没退出（后端拒绝停止）。让出 CPU 后重试；示例中的做法是在 200 ms 预算内循环重试。

**Q：VSYNC / PCLK 这些引脚填什么值？**

不用填。框架已经**没有任何引脚项**：所有 pad、pad 功能、上下拉/驱动能力都必须在开发板自己的 `bsp_pinmux.c` 里定义，框架只按 RT-Thread 设备名使用外设。板级没定义时框架会在编译期 `#error`（缺什么写明）或运行期返回错误，不会自己兜底复用一遍 pad 去和板级打架。

**Q：丢帧怎么观测？**

`camera_get_stream_dropped_count()` 返回自本次启动以来丢弃的帧数；示例的 `camera_stats` 还会按时间窗口打印 `captured/displayed/dropped/failed`。

**Q：框架还提供 PSRAM 帧缓冲分配器吗？**

不提供。`mem/` 与 `psram_heap_*` 已经删除，最终帧缓冲由应用或板级代码提供。

**Q：想改亮度/对比度/白平衡怎么做？**

框架目前没有公开的细粒度控制 API。可以在 Handle 层扩展控制接口，或直接改驱动默认值；OV2640 驱动内部只有 `CMD_SET_PIXFORMAT` / `CMD_SET_FRAMESIZE` / `CMD_SET_QUALITY` 三个命令，且不对业务层暴露（见附录 D）。

---

## 附录 A. 配置符号速查

| 符号 | 类型 | 含义 |
| --- | --- | --- |
| `PKG_USING_CAMERA_FRAMEWORK` | bool | 框架总开关（`Camera drivers` 菜单，SF-PKG 入口符号） |
| `SENSOR_USING_<X>` | choice | 当前传感器 |
| `CAMERA_<X>_INTERFACE_<Y>` | choice | 通信形式（`Data bus settings` 里按所选传感器显示）；决定 `CAMERA_USING_DVP/SERIAL/ARDUCAM_FIFO` |
| `CAMERA_ARDUCAM_SPI_BUS1/2`、`CAMERA_BF30A2_SPI_BUS1/2` | choice | 模组挂在哪条 SPI，并 `select` 对应 SPI 驱动 + DMA；派生 `CAMERA_*_SPI_BUS_NAME` |
| `CAMERA_USING_DVP` / `CAMERA_USING_SERIAL` / `CAMERA_USING_ARDUCAM_FIFO` | bool | 后端大类（由接口选择自动置位，不要手改） |
| `CAMERA_DVP_BACKEND_DCMI` | bool（隐藏，无提示） | 并行 DVP 的唯一实现：SDK `dcmi` 设备驱动（仅 SF32LB57X，自动 `select BSP_USING_DCMI`） |
| `CAMERA_SERIAL_SPI_2BIT` | bool（隐藏） | 2-bit 采集的唯一实现：DCMI SPI 包模式（仅 SF32LB57X，自动 `select BSP_USING_DCMI`）；GPIO 采样后端已删除 |
| `CAMERA_DATA_BUS_ADAPTER_NAME` | string | 当前数据总线适配器名（驱动与总线共用） |
| `CAMERA_BOARD_HAS_MCLK` | bool（隐藏，派生） | 板级给了 MCLK：`default y if CAMERA_XCLK_ENABLE`。框架只读板级符号，反向不成立 |
| `CAMERA_XCLK_REQUIRED` | bool（隐藏，派生） | 编进来的传感器需要主时钟（OV2640 走 ArduCam FIFO 时不算；GC032A/BF30A2 总是需要） |
| `CAMERA_ISP_HARDWARE` / `CAMERA_ISP_SOFTWARE` | choice | ISP 走传感器自带还是软件管线 |
| `CAMERA_SW_ISP`、`CAMERA_SW_ISP_RUN_ON_HCPU` / `_ACPU`、`CAMERA_SW_ISP_GAMMA/AWB/TONE/VIVID/DITHER` | choice/bool | 软件 ISP 运行核与各级开关 |
| `CAMERA_SW_JPEG`、`CAMERA_SW_JPEG_ENCODER` / `_DECODER`、`CAMERA_SW_JPEG_RUN_ON_HCPU` / `_ACPU` | bool/choice | 软件 JPEG 编解码 |
| `BSP_USING_DCMI` | bool | 并行 DVP（或 2-bit SPI 包模式）时**必须为 y**（`CAMERA_DVP_BACKEND_DCMI` / `CAMERA_SERIAL_SPI_2BIT` 会自动 select，框架编译期强制检查） |

> 以上是框架 Kconfig 的**全部**内容（`Data bus settings` 里按所选摄像头出现的通信形式与 SPI 实例二选一也在此列）。
>
> 下面这些符号框架会读，但**由板级 `Kconfig.board` 定义**：`CAMERA_I2C_DEVICE_NAME`、`CAMERA_XCLK_ENABLE`、`CAMERA_XCLK_PWM_DEVICE_NAME`、`CAMERA_XCLK_PWM_CHANNEL`、`CAMERA_XCLK_EXTERNAL_OSC`、`CAMERA_DCMI_PCLK_FALLING`、`CAMERA_BF30A2_RESET_PIN`；板级还可以用**自己的**分组开关（例如 `BOARD_CAMERA_HW`，纯板级符号）把"分配 PWM 实例、给设备名/通道"这一组收进 `if BOARD_CAMERA_HW`，并在这组里 `select BSP_USING_PWM` / 具体 PWM 实例。
>
> **依赖方向只能是自下而上**：板级声明能力，框架判断需求。
> 需要时钟 + 板级给了 → 编 PWM 实现；需要时钟 + 板级没给 + 没声明 `CAMERA_XCLK_EXTERNAL_OSC` → 编译期 `#error`；不需要时钟（ArduCam 模组）或有 `CAMERA_XCLK_EXTERNAL_OSC` → 编空实现并 `LOG_W`。
> 因此板级 `Kconfig.board` 里**不允许**出现任何 `CAMERA_FRAMEWORK_*` / `CAMERA_USING_*` 符号。
>
> SCCB 超时/频率、帧等待超时、SPI 频率/设备名、FIFO/worker 尺寸、2-bit 包格式则在驱动/后端源码里作为常量。

## 附录 B. 公共 API 速查

| API | 说明 |
| --- | --- |
| `camera_handler_instance_init(&instance)` | 分配并打开一个 handle 实例（会打开底层驱动会话） |
| `camera_get_capabilities(instance, &caps)` | 取能力表（pixformat / framesize / max_buffer_size） |
| `camera_change_settings(instance, &config)` | 下发 `{pixformat, framesize, quality}` 并缓存为活动配置 |
| `camera_capture_single(instance, &request)` | 阻塞单帧（超时用驱动配置） |
| `camera_capture_single_timeout(instance, &request, timeout_ms)` | 阻塞单帧，指定等待超时 |
| `camera_capture_frames_timeout(instance, &request, count, timeout_ms)` | 连续接收 `count` 帧 JPEG 并只保留最后一帧（后端需支持） |
| `camera_capture_single_async(instance, &request, cb, ctx)` | 非阻塞单帧，完成/超时回调 |
| `camera_stop_capture(instance)` | 停止同步/异步采集并确认回调已退出；返回成功后才能复用缓冲 |
| `camera_start_stream(instance, &config)` | 双缓冲连续流（整帧通知） |
| `camera_start_stream_mode(instance, &config, mode)` | 指定 `CAMERA_STREAM_MODE_FRAME/HALF_FRAME` |
| `camera_get_stream_frame(instance, &frame, timeout)` | 取下一帧（浅拷贝），消费者负责 cache 维护 |
| `camera_stream_frame_is_valid(instance, &frame)` | 判断该帧对应的缓冲是否仍可读（不预留、不做 cache 维护） |
| `camera_stop_stream(instance)` | 停止流并清空队列/计数 |
| `camera_get_stream_dropped_count(instance, &count)` | 自本次启动以来丢弃的帧数 |
| `camera_set_rotation(instance, degrees)` | 0/180 度翻转（需停止采集；可选能力） |
| `camera_deinit(&instance)` | 关闭驱动会话、释放 handle 资源 |

错误码：`CAMERA_OK`、`CAMERA_ERROR`、`CAMERA_ERRORTIMEOUT`、`CAMERA_ERRORRESOURCE`、`CAMERA_ERRORPARAMETER`、`CAMERA_ERRORNOMEMORY`、`CAMERA_ERRORISR`。

## 附录 C. 文件索引

```text
camera_framework/
├── Kconfig                 传感器 / 接口 / 总线 / 参数
├── SConscript              源码选择规则
├── core/
│   ├── handle/             camera_handle.[ch]（公共 API）、camera_handle_internal.h
│   ├── driver/
│   │   ├── camera_driver_desc.[ch]      驱动链接段与发现
│   │   ├── camera_sensor_runtime.[ch]   采集/流公共逻辑
│   │   └── {ov2640,gc032a,bf30a2}/      传感器驱动与寄存器表
│   └── bus/
│       ├── control/sccb.[ch]            控制总线
│       └── data/
│           ├── data_bus_adapter.[ch]    注册表与包装 API
│           ├── camera_xclk.[ch]         MCLK
│           ├── dcmi.c                   DVP(DCMI) / 2-bit SPI
│           └── arducam_fifo*.c          SPI FIFO 模组
├── software/{isp,jpeg}/    软件 ISP 与 JPEG 编解码（含 vendor 源与 LICENSES）
├── examples/               可运行示例
└── tests/                  框架测试
```

## 附录 D. OV2640 内部控制命令（驱动内部）

`core/driver/ov2640/ov2640.h` 声明了三个内部控制命令 ID，由 `ov2640.c` 的
`static rt_err_t sensor_control(sensor_device_t *cam_dev, int cmd, void *args)` 分发，
不对外暴露，仅供驱动开发参考。

| 命令 | 值 | 参数 | 说明 |
| --- | --- | --- | --- |
| `CMD_SET_PIXFORMAT` | 0x01 | `pixformat_t` | 像素格式（JPEG / RGB565 / YUV422 / RAW8） |
| `CMD_SET_FRAMESIZE` | 0x02 | `framesize_t` | 帧分辨率（96×96 → UXGA 1600×1200） |
| `CMD_SET_QUALITY` | 0x06 | `int` 0…63 | JPEG 压缩质量 |

调用侧以 `(void *)(rt_ubase_t)value` 传参（`ov2640.c` 的 `sensor_set_pixformat()` /
`sensor_set_framesize()` / `sensor_set_quality()`），被调用侧按具体类型取回：
pixformat 与 framesize 为 `(…)(rt_ubase_t)args`，quality 为 `(int)(rt_base_t)args`。

亮度、对比度、饱和度、白平衡与曝光/增益控制尚未实现。`ov2640_settings.h` 中有对应的
寄存器表，但没有命令路径引用；需要这些能力时扩展驱动，或在 Handle 层增加控制接口。

采集与流控制不经过命令分发，由 `camera_device_ops_t` 的 `start_stream`、`stop_stream`
与 `capture*` 承担。
