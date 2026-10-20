# camera_framework — SiFli 相机框架（SF-PKG 源码包）

| 项 | 值 |
|---|---|
| 包名 | `camera_framework` |
| 版本 | `0.0.1` |
| 许可证 | Apache-2.0 |
| 支持的 SiFli SDK | `^2.4` |
| 依赖 | 仅 SiFli SDK（DCMI / I2C / PWM / SPI 设备驱动），由使用方提供 |
| Kconfig 根开关 | `PKG_USING_CAMERA_FRAMEWORK` |

## 概述

`camera_framework` 是一个面向 RT-Thread 的摄像头框架（纯源码组件），本身也是一个
可直接 `sdk.py sf-pkg build/upload` 的 SF-PKG 源码包：

- `camera_handle` 高层接口（应用只依赖 `core/handle/camera_handle.h`）
- 3 个传感器驱动：OV2640 / GC032A / BF30A2，Kconfig 里**单选一个**
- SCCB 控制总线（I2C；总线名与 pad 由板级给出）
- 3 类数据通路：DCMI 并行 DVP、DCMI SPI 2-bit 包模式、ArduCam SPI FIFO 模组
- 可选软件 ISP / 软件 JPEG（`software/{isp,jpeg}`）

驱动层通过 `camera_device_ops_t` 接入，数据总线通过 `bus_adapter_t` 接入；**一个固件只编译一个传感器驱动 + 一个数据总线后端**（见 `SConscript`）。

框架不提供 `psram_heap_*` / `mem/` 帧缓冲分配器，最终帧缓冲由应用或板级提供。引脚复用、上下拉、驱动能力、外设实例也都在板级（`customer/boards/<board>/bsp_pinmux.c` + `Kconfig.board`），框架只按 RT-Thread 设备名使用外设，不持有任何引脚或设备名。

## 目录结构

```text
camera_framework/
├── conanfile.py            SF-PKG recipe（包名/版本/命名空间/导出规则）
├── Kconfig                 包入口 PKG_USING_CAMERA_FRAMEWORK + 传感器/通信形式/软件 ISP/JPEG
├── SConscript              源码选择规则（一个配置只编一个后端 + 一个驱动）
├── LICENSE                 Apache-2.0
├── README.md               本文件（包说明 + 框架说明）
├── README_EN.md            英文说明
├── develop.md              开发手册（架构、配置、API、接入新传感器/总线）
├── core/
│   ├── handle/             camera_handle.[ch]（公共 API）、camera_handle_internal.h
│   ├── driver/
│   │   ├── camera_driver_desc.[ch]      驱动链接段与发现
│   │   ├── camera_sensor_runtime.[ch]   采集/流公共逻辑
│   │   ├── camera_jpeg_assembler.[ch]   JPEG 组帧
│   │   └── {ov2640,gc032a,bf30a2}/      传感器驱动与寄存器表
│   └── bus/
│       ├── control/sccb.[ch]            控制总线（I2C）
│       └── data/
│           ├── data_bus_adapter.[ch]    注册表与包装 API
│           ├── camera_xclk.[ch]         MCLK（只驱动板级声明的 PWM）
│           ├── camera_xclk_calc.c       MCLK 周期计算
│           ├── dcmi.c                   SDK DCMI：并行 DVP / 2-bit SPI 包模式
│           └── arducam_fifo*.c          SPI FIFO 模组后端
├── software/{isp,jpeg}/    软件 ISP 与 JPEG 编解码（含 vendor 源与 LICENSES）
├── examples/               可运行示例（take_photo / take_photo_to_screen / …）
├── script/                 辅助脚本（如 hex 转 JPEG）
└── tests/                  静态检查脚本与实板测试
```

`conanfile.py` 使用 `exports_sources = "*"`，因此整个目录都会进入包；**只有 `core/` 与
`software/` 参与固件编译**，`examples/`、`script/`、`tests/` 只是文档与检查脚本，
不会被 `SConscript` 加进固件。

## 安装与接入（作为 SF-PKG 依赖）

1. 在工程根目录的 `sf-pkg.yaml` 里声明依赖（revision 由 `sf-pkg build` 的输出给出）：

   ```yaml
   requires:
     - "camera_framework/1.0.0@sifli#<recipe-revision>"
   support_sdk_version: "2.5.0"
   ```

2. 安装依赖，生成 `.sf-pkg/`、`sf-pkgs/`、`sf-pkgs/Kconfig.conandeps` 与
   `sf-pkgs/SConscript_conandeps`：

   ```
   sdk.py sf-pkg install
   ```

3. **去掉工程里对框架源码的直接引用**，否则会和包重复编译 / 重复定义符号：

   - `project/Kconfig.proj` 里的 `rsource "../../../Kconfig"`
   - `project/SConscript` 里 `SConscript(<camera_framework>/SConscript, ...)` 那一段
     （示例工程里是 `camera_root = cwd/../../../` 的写法）

4. 板级仍需提供：`bsp_pinmux.c` 中 `#if defined(BSP_USING_DCMI)` 的 DVP + MCLK 焊盘、
   `CAMERA_I2C_DEVICE_NAME`（SCCB 总线名）、XCLK 使用的 PWM 实例，以及
   `Kconfig.board` 里的 `CAMERA_XCLK_*` / `CAMERA_DCMI_PCLK_FALLING` 等板级符号。

安装后 Kconfig 外部组件菜单会 `orsource` 包内的 `Kconfig`，SDK 构建再按
`sf-pkgs/SConscript_conandeps` 的 `LIBPATH`（包根目录）执行包内的 `SConscript`。

### 从仓库内直接编译迁移过来

包内 Kconfig 的根符号是 `PKG_USING_CAMERA_FRAMEWORK`（默认 `n`，由 SDK 的 Camera
模块在 `BSP_USING_CAMERA1` 打开并选择了型号时 `select`）。

## Kconfig 结构

从本版本起，**型号与通信形式菜单不再放在框架包里**，而是在 SDK 的
`customer/peripherals/camera/Kconfig`：

```text
Board Peripherals -> Camera 1
    Camera model: OV2640 / GC032A / BF30A2
    OV2640: 8-bit DVP / SPI (ArduCam FIFO)
    GC032A: 8-bit DVP / 2-bit SPI
    BF30A2: SPI1 / SPI2
    ArduCam FIFO: SPI1 / SPI2
```

选择型号后，子菜单只显示该型号支持的通信形式；数据总线 backend 由通信形式自动推导
（8-bit DVP → `CAMERA_DVP_BACKEND_DCMI`；2-bit SPI → `CAMERA_SERIAL_SPI_2BIT`，即同一个
DCMI 的 SPI 包模式；ArduCam → `CAMERA_ARDUCAM_FIFO`），不需要手选。

框架包 Kconfig 只保留：

- `PKG_USING_CAMERA_FRAMEWORK`：包使能符号（由 Camera 模块 `select`）
- `CAMERA_BOARD_HAS_MCLK` / `CAMERA_XCLK_REQUIRED`：由板级 XCLK 和传感器派生的内部检查
- `software/Kconfig`：软件 ISP / 软件 JPEG

包源码由 `customer/peripherals/camera/sf-pkg.yaml` 在 `BSP_USING_CAMERA1=y` 时自动拉取；
工程根需要有一个 `sf-pkg.yaml` 才会启用模块级依赖聚合。

除了"选哪条 SPI 实例"这一个二选一，菜单里**没有任何硬件值**：I2C/PWM 设备名、MCLK 归属、引脚、总线时序、缓冲区大小都不在框架 Kconfig 里。

- 板级提供（`customer/boards/<board>/Kconfig.board` + `bsp_pinmux.c`）：`CAMERA_I2C_DEVICE_NAME`、`CAMERA_XCLK_ENABLE`、`CAMERA_XCLK_PWM_DEVICE_NAME/_CHANNEL`、`CAMERA_XCLK_EXTERNAL_OSC`（模组自带晶振时）、`CAMERA_DCMI_PCLK_FALLING`、`CAMERA_BF30A2_RESET_PIN`
- 驱动/后端代码里的常量：SCCB 超时与频率、帧等待超时、SPI 频率、SPI 设备名、FIFO scratch/worker 栈、2-bit 串行包格式（行字节数、包标识、同步字、采样沿、数据线交换）

依赖方向只能**自下而上**：板级只声明"我接了什么、把哪个外设实例分给相机"，框架读这些符号并自行判断"编进来的传感器需不需要"（例如需要 MCLK 却没声明就编译期 `#error`）。因此板级 `Kconfig.board` 里不允许出现任何 `CAMERA_FRAMEWORK_*` / `CAMERA_USING_*` 符号。

完整契约（板级符号清单、`BOARD_CAMERA_HW` 的用法、派生符号 `CAMERA_BOARD_HAS_MCLK` / `CAMERA_XCLK_REQUIRED`、验收 grep）见 `develop.md` §4.1、§4.3（板级 `Kconfig.board` 完整示例）与附录 A。

## 当前能力

- 支持 `PIXFORMAT_JPEG`、`PIXFORMAT_RGB565`、`PIXFORMAT_YUV422`、`PIXFORMAT_RAW8`
- 单帧采集：阻塞 `camera_capture_single`、带超时 `camera_capture_single_timeout`、异步 `camera_capture_single_async`
- JPEG 连拍：`camera_capture_frames_timeout`（一次硬件运行收 count 帧，只保留最后一帧）
- 双缓冲连续流 `camera_start_stream` / `camera_get_stream_frame`，以及由 DMA HT/TC 驱动的半帧模式 `camera_start_stream_mode`
- 帧有效性查询 `camera_stream_frame_is_valid`（判断出队描述符所指缓冲是否仍是这一代数据）
- capability 查询 `camera_get_capabilities`、统一错误码 `camera_handle_status_t`、0/180 度旋转 `camera_set_rotation`
- 图像参数只暴露驱动实现了的部分：`framesize` / `quality` / 旋转；没有全局的镜像/翻转/曝光/白平衡/增益接口

## 分层关系

```text
Application / example
    ↓  只用 camera_handle.h
camera_handle                实例状态机、API 互斥、流帧队列与丢帧计数
    ↓  camera_device_ops_t（函数表，编译期只有一个实现）
传感器驱动 + camera_sensor_runtime
    ↓  bus_adapter_ops_t、sccb.h、camera_xclk.h
数据总线后端（dcmi / arducam_fifo）+ 控制总线（sccb）+ MCLK（camera_xclk）
    ↓  RT-Thread 设备 API：按名字找 dcmi / pwmt2 / i2c1 / spiN
SDK / board（bsp_pinmux.c、drv_dcmi.c、drv_pwm、drv_i2c、drv_spi）
```

依赖是严格单向向下的：下层不回调 handle，框架不反向改板级。

## Driver 发现方式

当前 camera framework 采用和 SDK 中 LCD driver 类似的静态导出方式：

- 每个 sensor driver 通过 `CAMERA_DRIVER_EXPORT(name, ops)` 导出一条描述符
- 描述符进入 `CameraDriverDescTab` section
- `camera_handle` 通过 `camera_driver_get_default_ops()` 扫描这张表，拿到默认 driver 的 `camera_device_ops_t`
- `CAMERA_HANDLE_TESTING` 打开时，测试注入的 fake ops 优先级高于默认 driver

这意味着：

- 新增 sensor driver 时，不需要再改 `camera_handle.c` 里的硬编码 `#ifdef`
- 但链接脚本必须保留 `CameraDriverDescTab` section，否则运行时无法发现 driver

## 公共 API

头文件：`core/handle/camera_handle.h`

主要接口：

- `camera_handler_instance_init()`：初始化 handle 并打开 driver
- `camera_deinit()`：关闭 driver 并释放 handle
- `camera_get_capabilities()`：查询能力表
- `camera_change_settings()`：设置 `pixformat` / `framesize` / `quality`
- `camera_capture_single()`：阻塞单帧采集
- `camera_capture_single_timeout()`：阻塞单帧，指定等待超时
- `camera_capture_frames_timeout()`：连续接收 `count` 帧 JPEG 并只保留最后一帧
- `camera_capture_single_async()`：异步单帧采集
- `camera_stop_capture()`：停止同步/异步采集并确认回调已退出
- `camera_start_stream()`：启动双缓冲流式采集
- `camera_start_stream_mode()`：指定 `CAMERA_STREAM_MODE_FRAME` / `HALF_FRAME`
- `camera_get_stream_frame()`：取下一帧流数据
- `camera_stream_frame_is_valid()`：判断出队描述符所指缓冲是否仍是这一代数据
- `camera_stop_stream()`：停止流式采集
- `camera_get_stream_dropped_count()`：查询丢帧数
- `camera_set_rotation()`：0/180 度翻转（需停止采集）

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

## SF32LB57 硬件 DCMI 后端

GC032A 选择 `8-bit DVP`，或 OV2640 选择 `DVP`（两者都只在 SF32LB57X 上可选）；并行 DVP 只有 SDK DCMI 一个后端，`CAMERA_DVP_BACKEND_DCMI` 隐藏且默认启用，会自动 `select BSP_USING_DCMI`，必须保持启用。此后端通过 SDK 的 `dcmi` 设备驱动使用 DCMI 和 DMA：外设、DMA 通道和中断向量都由 SDK 驱动持有，框架负责采集编程和 JPEG 帧收尾——DMA 半/全传输与错误通过 `rt_device_set_rx_indicate` 上报，DCMI 的帧/错误事件由 SDK 的 `HAL_DCMI_FrameEventCallback` 转成同一个通知，框架只额外重写 `HAL_DCMI_ErrorCallback`。D0-D7、PCLK、HREF、VSYNC 以及 MCLK 的引脚复用由开发板自己的 `bsp_pinmux.c`（`BSP_USING_DCMI` 块）完成，应用不提供 `HAL_DCMI_MspInit()`；SCCB 用板级 `CAMERA_I2C_DEVICE_NAME` 与驱动内的时序常量；默认在 PCLK 上升沿采样（板级可用 `CAMERA_DCMI_PCLK_FALLING` 改成下降沿）。52x/56x 上没有并行 DVP 采集路径，GC032A 请用 `2-bit SPI`，OV2640 请用 `Arducam SPI FIFO`。

OV2640 在硬件 DCMI 后端下默认使用 SVGA/RGB565，并支持传感器原生 JPEG 拍照。ArduCAM FIFO 后端仍使用 JPEG 默认配置。OV2640 的 RGB565 设置将 DSP 寄存器 `IMAGE_MODE` 写为 `0x09`，使每个像素的低字节先输出；在小端 MCU 上按原生 RGB565 使用时，直接复制像素即可。

`camera_start_stream()` 默认使用 `CAMERA_STREAM_MODE_FRAME`，一次提交两块相邻、64 字节对齐的帧缓冲，单帧大小也须为 64 字节的整数倍。DMA 半传输和全传输通知分别对应两个完整帧，后端不会逐帧停止和重新启动 DCMI。

`camera_get_stream_frame()` 返回借用帧描述符。`error` 为负值表示采集失败，`timestamp_ticks` 记录帧事件的 RTOS tick。后端支持 `camera_stream_frame_is_valid()`：消费者在线程中对 DMA 帧执行必要的缓存失效，再于复制前后调用有效性查询；只有两次均有效，复制结果才可以交给显示或编码使用。在 `FRAME` 模式下，该查询检查采集序号和 DMA 的源读取、目标写入进度；队列中的旧描述符不能作为像素仍有效的依据。

半帧采集使用 `camera_start_stream_mode(instance, &config, CAMERA_STREAM_MODE_HALF_FRAME)`。`config.buffers[0]` 指向一块 64 字节对齐的完整原始图像缓冲，`config.buffers[1]` 必须为 `NULL`，`config.buffer_size` 必须等于整帧字节数，图像高度必须为偶数，每半幅字节数也须为 64 字节的整数倍。例如，OV2640 的 800 × 600 RGB565 图像使用一块 960000 字节缓冲，每半幅为 480000 字节。

DMA 半传输（HT）产生 `CAMERA_STREAM_EVENT_HALF_FIRST`，全传输（TC）产生 `CAMERA_STREAM_EVENT_HALF_SECOND`；同一帧的两个事件具有相同的 `sequence`。两个描述符的 `buffer` 和 `frame_size` 始终指向完整图像，消费者根据 `event_type` 选择上半幅或下半幅；成功事件中仅 `HALF_SECOND` 的 `is_complete` 为真。中断回调只入队元数据并唤醒等待者，普通线程通过 `camera_get_stream_frame()` 按 FIFO 顺序消费事件并处理像素。

在 `HALF_FRAME` 模式下，`camera_stream_frame_is_valid()` 只比较事件所指半幅的最新 HT/TC 代次，不轮询 DMA 进度。`HALF_FIRST` 在对应 TC 到来时失效，`HALF_SECOND` 在下一次 HT 到来时失效；停止、错误和重新启动会使旧描述符失效。消费者处理每个半幅前后均须检查有效性，并完成所需缓存维护；只有同一 `sequence` 的两半都处理成功，才可发布拼接后的完整图像。有效性查询不会阻止 DMA 重写缓冲。

停止或关闭失败时，框架保留相应状态和缓冲引用，调用者可重试停止或关闭；仅成功后归还缓冲。单拍完成通知经系统工作队列执行，并在硬件停止后发出。

## 相关文档

- 详细开发说明：`develop.md`
- RGB565 单帧示例：`examples/take_photo/README.md`
- JPEG 预览与拍照示例：`examples/take_photo_to_screen/README.md`
- JPEG 保存到 SD 卡：`examples/take_photo_to_sdcard/README.md`
- JPEG 流式保存到 SD 卡：`examples/take_photo_to_sdcard_streaming/README.md`
- 测试说明：`tests/README.md`
