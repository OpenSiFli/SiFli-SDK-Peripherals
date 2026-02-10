# OV2640 摄像头组件

基于 RT-Thread 的 OV2640 摄像头驱动组件，适用于 Sifli SF32LB52x 平台。通过 GPIO + GPTIM + DMA 软件实现 DVP（Digital Video Port）并行接口采集图像数据，并封装为标准 RT-Thread 设备供应用层调用。

## 目录结构

```
ov2640/
├── dvp.c               # DVP 数据采集层（GPTIM1 + DMA 乒乓缓冲）
├── dvp.h               # DVP 接口定义
├── ov2640.c            # OV2640 传感器驱动 + RT-Thread 设备接口
├── ov2640.h            # 传感器结构体、设备接口及控制命令定义
├── ov2640_regs.h       # OV2640 寄存器地址和位掩码定义
├── ov2640_settings.h   # 寄存器初始化序列及配置查找表
├── sccb.c              # SCCB（I2C）通信层
├── sccb.h              # SCCB 接口定义
├── Kconfig             # 可配置选项菜单
├── SConscript          # SCons 构建脚本
└── README.md           # 本文档
```

## 功能特性

- **JPEG 图像采集**：支持 SOI/EOI 标记自动检测，完整 JPEG 帧提取
- **多种分辨率**：支持从 96×96 到 UXGA（1600×1200）共 16 种分辨率
- **丰富的图像参数调节**：亮度、对比度、饱和度、白平衡、曝光、增益、特效等
- **RT-Thread 设备接口**：标准 `open` / `read` / `close` / `control` 设备操作
- **XCLK 时钟生成**：可选通过 GPTIM2 PWM 输出 6MHz / 12MHz 时钟给传感器
- **线程安全**：传感器操作通过 mutex 保护，帧就绪通过信号量通知

## 硬件连接

| OV2640 引脚 | MCU 引脚 | 功能 | 配置方式 |
|-------------|----------|------|----------|
| D0 ~ D7 | PA0 ~ PA7 | 8-bit 并行数据 | 驱动内部自动配置（固定） |
| PCLK | 用户指定（默认 PA41） | 像素时钟 | 用户配置为 `GPTIM1_ETR` |
| HSYNC/HREF | 用户指定（默认 PA43） | 行同步 | 用户配置为 `GPTIM1_CH1` |
| VSYNC | 用户指定（默认 PA42） | 帧同步 | Kconfig 配置，驱动自动设为 GPIO 中断 |
| SDA | 用户指定（默认 PA39） | I2C 数据 | 用户配置为 `I2C1_SDA` |
| SCL | 用户指定（默认 PA40） | I2C 时钟 | 用户配置为 `I2C1_SCL` |
| XCLK | 用户指定（默认 PA08） | 外部时钟输入 | 用户配置为 `GPTIM2_CH1`（可选） |

> **注意**：PCLK、HSYNC、I2C 和 XCLK 引脚需要用户在调用 `rt_device_open()` **之前**通过 `HAL_PIN_Set()` 手动配置。

## Kconfig 配置

在 menuconfig 中进入 `ov2640 camera` 菜单：

### DVP 配置

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `OV2640_DVP_PINGPONG_BUFFER_SIZE` | 8192 | 乒乓缓冲区大小（1024 ~ 65536） |
| `OV2640_DVP_VSYNC_PIN` | 42 | VSYNC 引脚编号 |

### SCCB 配置

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `OV2640_SCCB_I2C_BUS_NAME` | `"i2c1"` | I2C 总线名称 |
| `OV2640_SCCB_TIMEOUT_MS` | 1000 | I2C 通信超时（ms） |
| `OV2640_SCCB_MAX_HZ` | 100000 | I2C 最大频率（Hz，范围 10000 ~ 400000） |

### 相机配置

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `OV2640_CAMERA_READ_TIMEOUT_MS` | 1000 | 拍照超时时间（ms，范围 100 ~ 10000） |
| `OV2640_DVP_XCLK_PIN` | -1 | XCLK 输出引脚（-1 禁用，使用外部时钟） |
| `OV2640_DVP_XCLK_FREQ` | 12MHz | XCLK 输出频率（6MHz / 12MHz） |

## 快速上手

### 1. 使能组件

在 Kconfig / menuconfig 中开启 `SENSOR_USING_OV2640`。

### 2. 编写应用代码

```c
#include "ov2640.h"
#include "dvp.h"

void capture_photo(void)
{
    /* 第 1 步：配置引脚（在打开设备之前） */
    HAL_PIN_Set(PAD_PA00 + 40, I2C1_SCL, PIN_PULLUP, 1);     // I2C SCL
    HAL_PIN_Set(PAD_PA00 + 39, I2C1_SDA, PIN_PULLUP, 1);     // I2C SDA
    HAL_PIN_Set(PAD_PA00 + 41, GPTIM1_ETR, PIN_PULLDOWN, 1); // PCLK
    HAL_PIN_Set(PAD_PA00 + 43, GPTIM1_CH1, PIN_PULLDOWN, 1); // HSYNC
    HAL_PIN_Set(PAD_PA00 + 8,  GPTIM2_CH1, PIN_NOPULL, 1);   // XCLK

    /* 第 2 步：查找并打开设备 */
    rt_device_t dev = rt_device_find("ov2640");
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR);

    /* 第 3 步：配置拍摄参数 */
    rt_device_control(dev, CAMERA_CMD_SET_FRAMESIZE, (void *)(rt_ubase_t)FRAMESIZE_VGA);
    rt_device_control(dev, CAMERA_CMD_SET_QUALITY,   (void *)(rt_ubase_t)10);
    rt_thread_mdelay(500);  // 等待传感器设置生效

    /* 第 4 步：拍照 - read 会阻塞直到帧就绪或超时 */
    uint8_t *buf = rt_malloc(1024 * 200);  // 分配 JPEG 缓冲区
    rt_size_t size = rt_device_read(dev, 0, buf, 1024 * 200);
    if (size > 0)
    {
        rt_kprintf("Captured JPEG frame: %d bytes\n", size);
        // buf[0..size-1] 即为完整的 JPEG 数据 (FF D8 ... FF D9)
    }

    /* 第 5 步：释放资源 */
    rt_free(buf);
    rt_device_close(dev);
}
```

### 3. MSH 命令

组件示例提供了 `take_photo` MSH 命令：

```
msh> take_photo VGA 10 3
```

参数说明：
- **framesize**：分辨率 — `QQVGA` / `QCIF` / `QVGA` / `CIF` / `VGA` / `SVGA` / `XGA` / `HD` / `SXGA` / `UXGA`
- **quality**：JPEG 压缩质量（0 = 最高质量，63 = 最高压缩）
- **count**：连拍张数

## API 参考

### 设备操作

| 操作 | 函数 | 说明 |
|------|------|------|
| 查找设备 | `rt_device_find("ov2640")` | 设备名固定为 `"ov2640"` |
| 打开设备 | `rt_device_open(dev, RT_DEVICE_OFLAG_RDWR)` | 初始化传感器 + DVP 硬件 |
| 拍照读取 | `rt_device_read(dev, 0, buf, size)` | 阻塞式读取一帧 JPEG 数据 |
| 参数控制 | `rt_device_control(dev, cmd, args)` | 设置各项参数，见下表 |
| 关闭设备 | `rt_device_close(dev)` | 释放硬件资源 |

### 控制命令一览

| 命令 | 值 | 参数 | 说明 |
|------|----|------|------|
| `CAMERA_CMD_SET_PIXFORMAT` | 0x01 | `pixformat_t` | 像素格式（当前仅 JPEG 可用） |
| `CAMERA_CMD_SET_FRAMESIZE` | 0x02 | `framesize_t` | 分辨率 |
| `CAMERA_CMD_SET_BRIGHTNESS` | 0x03 | `int` (-2 ~ 2) | 亮度 |
| `CAMERA_CMD_SET_CONTRAST` | 0x04 | `int` (-2 ~ 2) | 对比度 |
| `CAMERA_CMD_SET_SATURATION` | 0x05 | `int` (-2 ~ 2) | 饱和度 |
| `CAMERA_CMD_SET_QUALITY` | 0x06 | `int` (0 ~ 63) | JPEG 压缩质量 |
| `CAMERA_CMD_SET_HMIRROR` | 0x07 | `int` (0/1) | 水平镜像 |
| `CAMERA_CMD_SET_VFLIP` | 0x08 | `int` (0/1) | 垂直翻转 |
| `CAMERA_CMD_SET_COLORBAR` | 0x09 | `int` (0/1) | 彩条测试图 |
| `CAMERA_CMD_SET_WHITEBAL` | 0x0A | `int` (0/1) | 自动白平衡 |
| `CAMERA_CMD_SET_GAIN_CTRL` | 0x0B | `int` (0/1) | 自动增益控制 |
| `CAMERA_CMD_SET_EXPOSURE_CTRL` | 0x0C | `int` (0/1) | 自动曝光控制 |
| `CAMERA_CMD_SET_AEC2` | 0x0D | `int` (0/1) | 增强自动曝光 |
| `CAMERA_CMD_SET_AWB_GAIN` | 0x0E | `int` (0/1) | AWB 增益 |
| `CAMERA_CMD_SET_AGC_GAIN` | 0x0F | `int` (0 ~ 30) | 手动 AGC 增益值 |
| `CAMERA_CMD_SET_AEC_VALUE` | 0x13 | `int` (0 ~ 1200) | 手动曝光值 |
| `CAMERA_CMD_SET_SPECIAL_EFFECT` | 0x14 | `int` (0 ~ 6) | 特效模式 |
| `CAMERA_CMD_SET_WB_MODE` | 0x15 | `int` (0 ~ 4) | 白平衡模式 |
| `CAMERA_CMD_SET_AE_LEVEL` | 0x16 | `int` (-2 ~ 2) | AE 等级 |
| `CAMERA_CMD_SET_GAINCEILING` | 0x1C | `gainceiling_t` | 增益上限 |
| `CAMERA_CMD_START_CAPTURE` | 0x10 | — | 启动采集 |
| `CAMERA_CMD_STOP_CAPTURE` | 0x11 | — | 停止采集 |
| `CAMERA_CMD_GET_FRAME_SIZE` | 0x12 | `uint32_t *` | 获取当前帧大小 |

**特效模式** (`CAMERA_CMD_SET_SPECIAL_EFFECT`)：0=正常, 1=负片, 2=黑白, 3=偏红, 4=偏绿, 5=偏蓝, 6=复古

**白平衡模式** (`CAMERA_CMD_SET_WB_MODE`)：0=自动, 1=晴天, 2=阴天, 3=办公室, 4=家庭

## 支持的分辨率

| 枚举值 | 分辨率 | 名称 |
|--------|--------|------|
| `FRAMESIZE_96X96` | 96×96 | — |
| `FRAMESIZE_QQVGA` | 160×120 | QQVGA |
| `FRAMESIZE_128X128` | 128×128 | — |
| `FRAMESIZE_QCIF` | 176×144 | QCIF |
| `FRAMESIZE_HQVGA` | 240×176 | HQVGA |
| `FRAMESIZE_240X240` | 240×240 | — |
| `FRAMESIZE_QVGA` | 320×240 | QVGA |
| `FRAMESIZE_320X320` | 320×320 | — |
| `FRAMESIZE_CIF` | 400×296 | CIF |
| `FRAMESIZE_HVGA` | 480×320 | HVGA |
| `FRAMESIZE_VGA` | 640×480 | VGA |
| `FRAMESIZE_SVGA` | 800×600 | SVGA |
| `FRAMESIZE_XGA` | 1024×768 | XGA |
| `FRAMESIZE_HD` | 1280×720 | HD |
| `FRAMESIZE_SXGA` | 1280×1024 | SXGA |
| `FRAMESIZE_UXGA` | 1600×1200 | UXGA |

## 架构说明

```
┌──────────────────────────────────────────────┐
│                应用层 (main.c)                │
│   rt_device_find / open / read / control     │
└──────────────────┬───────────────────────────┘
                   │ RT-Thread Device Interface
┌──────────────────▼───────────────────────────┐
│            ov2640.c  (设备驱动层)              │
│  camera_open / camera_read / camera_control  │
├──────────────┬───────────────────────────────┤
│  OV2640 传感器控制    │    DVP 数据采集 (dvp.c) │
│  · 寄存器配置         │    · GPTIM1 + DMA       │
│  · 分辨率/格式设置    │    · 乒乓缓冲           │
│  · 图像参数调节       │    · JPEG SOI/EOI 检测   │
├──────────────┤        │                         │
│  SCCB (sccb.c)       │                         │
│  · I2C 读写           │                         │
└──────────────┴───────┴─────────────────────────┘
                   │                │
            I2C 总线           GPIO / DMA / Timer
                   │                │
            ┌──────▼────────────────▼──────┐
            │         OV2640 硬件           │
            └──────────────────────────────┘
```

## 注意事项

1. **数据引脚固定**：D0 ~ D7 固定使用 PA0 ~ PA7（GPIO1 低 8 位），硬件设计时不可更改。
2. **引脚配置顺序**：PCLK、HSYNC、I2C、XCLK 引脚必须在 `rt_device_open()` 之前配置好。
3. **缓冲区大小**：`rt_device_read()` 传入的 buffer 需要足够容纳一帧 JPEG 数据。UXGA 分辨率下建议至少 512KB；VGA 建议至少 100KB。
4. **当前仅支持 JPEG 模式**：DVP 采集层目前仅实现了 JPEG 模式（自动检测 SOI/EOI 标记），RAW / RGB565 / YUV422 模式暂未支持。
5. **设备自动注册**：组件通过 `INIT_PREV_EXPORT` 自动注册设备，无需手动调用注册函数。
6. **拍照延迟**：修改分辨率或质量参数后，建议等待约 500ms 再读取，以确保传感器设置生效。

## 依赖项

- RT-Thread（含 I2C 驱动框架、设备框架、PIN 驱动）
- Sifli BSP HAL 库（`bf0_hal.h`、GPIO / DMA / GPTIM 驱动）
- I2C 总线设备（默认 `i2c1`）
