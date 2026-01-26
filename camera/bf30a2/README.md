# BF30A2 SPI Camera Driver

源码路径：example\rt_device\camera\BF30A2
## 支持平台
* sf32lb52-lcd_n16r8

## 概述
* 为 BF30A2 SPI 摄像头提供了符合标准的设备驱动实现，展示对摄像头BF30A2的驱动能力，实现简单的拍照功能。
* 此例程中用到了camera设备，在采用RT-Thread操作系统时，camera外设会虚拟成了一个rt_device来进行数据读取操作，此时需要确认menuconfig中以下三个选项正确开启。
```c
CONFIG_BSP_USING_SPI2=y
CONFIG_BSP_SPI2_RX_USING_DMA=y
CONFIG_BSP_USING_SPI_CAMERA=y
```
* 此驱动的例程用到的LVGL相关宏参照lvgl_v9_examples例程进行开启，此处不再赘述，LVGL在此处仅用于展示显示到屏幕的能力，非驱动摄像头所必需。
* 切换到工程例程的project目录运行scons命令进行代码编译：
```
scons --board=sf32lb52-lcd_n16r8 -j8
```
* 切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：

>`build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`//下载代码

>`Uart Download`

>`please input the serial port num:5`

关于编译、下载的详细步骤，请参考[](/quickstart/get-started.md)的相关介绍。



### 硬件连接
```{eval-rst}
+--------------+----------+---------------+---------------+
| 开发板       | 功能引脚 | 本端设备引脚  | 对端设备引脚  |
+==============+==========+===============+===============+
| sf32lb52-lcd | PA_37    | dio           | SPI_D0        |
|              +----------+---------------+---------------+
|              | PA_39    | clk           | SPI_CLK       |
|              +----------+---------------+---------------+
|              | PA_40    | cs            | GND           |
|              +----------+---------------+---------------+
|              | PA_29    | pwm           | MCLK          |
|              +----------+---------------+---------------+
|              | PA_43    | gpio          | PWDN          |
|              +----------+---------------+---------------+
|              | PA_42    | sda           | IIC_SDA       |
|              +----------+---------------+---------------+
|              | PA_41    | scl           | IIC_SCL       |
+--------------+----------+---------------+---------------+
```
---

## 2. 功能实现方法

### 2.1 硬件接口

```
┌─────────────┐         ┌──────────────┐
│  SF32LB52   │         │   BF30A2     │
│             │         │              │
│  I2C ───────┼────────►│ SCCB (配置)  │
│  PWM ───────┼────────►│ MCLK (时钟)  │
│  GPIO ──────┼────────►│ PWDN (控制)  │
│             │         │              │
│  SPI2 ◄─────┼─────────│ SPI (数据)   │
│  (Slave)    │         │ (Master)     │
└─────────────┘         └──────────────┘
```

- **I2C**: 写入 `bf30a2_spi_config.h` 中的寄存器表完成传感器初始化
- **PWM**: 配置为24MHz输出，作为摄像头主时钟
- **SPI**: 从机模式，3线制，DMA循环接收图像数据

### 2.2 数据解析

SPI数据流采用MTK标识的协议格式，驱动使用状态机逐字节解析，提取每行YUV数据后转换为RGB565。

### 2.3 内存分配

| 缓冲区 | 大小 | 用途 |
|--------|------|------|
| DMA Buffer | ~8KB | SPI循环接收 |
| RGB565 Frame | 150KB | 当前帧存储 (240×320×2) |
| PSRAM Heap | 512KB | 拍照存储 |

---

## 3. 工作流程

![alt text](assets/workflow.png)

**数据采集线程:**
1. DMA中断通知事件
2. 读取环形缓冲区新数据
3. 状态机解析协议
4. 每行YUV→RGB565转换
5. 帧完成时触发回调，更新UI

---

## 4. 功能使用方法

### 设备注册函数

#### bf30a2_device_register

```c
rt_err_t bf30a2_device_register(void);
```

**功能**: 使用默认配置注册 BF30A2 摄像头设备

**参数**: 无

**返回值**:
| 返回值 | 说明 |
|--------|------|
| RT_EOK | 注册成功 |
| -RT_ENOMEM | 内存分配失败 |
| 其他负值 | 注册失败 |

**示例**:
```c
rt_err_t ret = bf30a2_device_register();
if (ret != RT_EOK) {
    LOG_E("摄像头注册失败");
}
```

---

#### bf30a2_device_register_with_config

```c
rt_err_t bf30a2_device_register_with_config(const char *name, const bf30a2_hw_cfg_t *hw_cfg);
```

**功能**: 使用自定义配置注册 BF30A2 摄像头设备

**参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| name | const char * | 设备注册名称 |
| hw_cfg | const bf30a2_hw_cfg_t * | 硬件配置结构体指针,传入 NULL 则使用默认配置 |

**返回值**: 同 `bf30a2_device_register()`

**示例**:
```c
bf30a2_hw_cfg_t cfg;
bf30a2_get_default_config(&cfg);
cfg.i2c_addr = 0x6E;
cfg.pins.pwdn_pin = 43;

rt_err_t ret = bf30a2_device_register_with_config("camera0", &cfg);
```

---

#### bf30a2_get_default_config

```c
void bf30a2_get_default_config(bf30a2_hw_cfg_t *cfg);
```

**功能**: 获取默认硬件配置

**参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| cfg | bf30a2_hw_cfg_t * | 配置结构体指针,用于接收默认配置 |

**返回值**: 无

**默认配置值**:
| 配置项 | 默认值 |
|--------|--------|
| spi_bus_name | "spi2" |
| i2c_bus_name | "i2c2" |
| pwm_dev_name | "pwm2" |
| i2c_addr | 0x6E |
| pwm_channel | 2 |
| spi_clk_pad | PAD_PA39 |
| spi_dio_pad | PAD_PA37 |
| spi_cs_pad | PAD_PA40 |
| i2c_scl_pad | PAD_PA41 |
| i2c_sda_pad | PAD_PA42 |
| pwdn_pin | 43 |
| pwm_pad | PAD_PA20 |

---

### 便捷宏定义

驱动提供以下便捷宏简化设备控制操作:

```c
/* 启动摄像头采集 */
#define bf30a2_start(dev)           rt_device_control(dev, BF30A2_CMD_START, RT_NULL)

/* 停止摄像头采集 */
#define bf30a2_stop(dev)            rt_device_control(dev, BF30A2_CMD_STOP, RT_NULL)

/* 获取摄像头信息 */
#define bf30a2_get_info(dev, info)  rt_device_control(dev, BF30A2_CMD_GET_INFO, info)

/* 获取摄像头状态 */
#define bf30a2_get_status(dev, status)  rt_device_control(dev, BF30A2_CMD_GET_STATUS, status)
```

---

### 帧回调函数类型

```c
typedef void (*bf30a2_frame_callback_t)(rt_device_t dev,
                                        rt_uint32_t frame_num,
                                        rt_uint8_t *buffer,
                                        rt_uint32_t size,
                                        void *user_data);
```

**参数说明**:
| 参数 | 说明 |
|------|------|
| dev | 设备句柄 |
| frame_num | 帧序号 |
| buffer | 帧数据指针 (RGB565 格式) |
| size | 帧数据大小 (字节) |
| user_data | 用户上下文指针 |

---

### 控制命令

所有控制命令通过 `rt_device_control()` 函数调用,命令值定义在 `enum bf30a2_cmd` 中。

### BF30A2_CMD_START (0x100)

**功能**: 启动帧采集

**参数**: 无 (传入 RT_NULL)

**返回值**: RT_EOK 成功,-RT_ERROR 失败

**示例**:
```c
rt_device_control(cam_device, BF30A2_CMD_START, RT_NULL);
```

---

#### BF30A2_CMD_STOP (0x101)

**功能**: 停止帧采集

**参数**: 无 (传入 RT_NULL)

**返回值**: RT_EOK 成功

**示例**:
```c
rt_device_control(cam_device, BF30A2_CMD_STOP, RT_NULL);
```

---

#### BF30A2_CMD_GET_INFO (0x102)

**功能**: 获取摄像头信息

**参数**: `bf30a2_info_t *` 类型指针

**返回值**: RT_EOK 成功

**bf30a2_info_t 结构体**:
```c
typedef struct bf30a2_info {
    rt_uint16_t width;          /* 图像宽度 (像素) */
    rt_uint16_t height;         /* 图像高度 (像素) */
    rt_uint32_t frame_size;     /* 帧缓冲区大小 (字节) */
    bf30a2_format_t format;     /* 输出格式 */
    rt_uint16_t chip_id;        /* 传感器芯片 ID */
} bf30a2_info_t;
```

**示例**:
```c
bf30a2_info_t info;
rt_device_control(cam_device, BF30A2_CMD_GET_INFO, &info);
rt_kprintf("分辨率: %dx%d\n", info.width, info.height);
rt_kprintf("芯片ID: 0x%04X\n", info.chip_id);
```

---

#### BF30A2_CMD_GET_STATUS (0x103)

**功能**: 获取摄像头状态

**参数**: `bf30a2_status_info_t *` 类型指针

**返回值**: RT_EOK 成功

**bf30a2_status_info_t 结构体**:
```c
typedef struct bf30a2_status {
    bf30a2_status_t state;      /* 当前状态: IDLE/RUNNING/ERROR */
    rt_uint32_t frame_count;    /* 总采集帧数 */
    rt_uint32_t complete_frames;/* 成功完成帧数 */
    rt_uint32_t error_count;    /* 错误计数 */
    float fps;                  /* 当前帧率 */
    rt_uint8_t frame_ready;     /* 帧就绪标志 */
} bf30a2_status_info_t;
```

**状态枚举值**:
| 状态 | 值 | 说明 |
|------|-----|------|
| BF30A2_STATUS_IDLE | 0 | 空闲状态 |
| BF30A2_STATUS_RUNNING | 1 | 采集中 |
| BF30A2_STATUS_ERROR | 2 | 错误状态 |

**示例**:
```c
bf30a2_status_info_t status;
rt_device_control(cam_device, BF30A2_CMD_GET_STATUS, &status);
rt_kprintf("状态: %s\n", status.state == BF30A2_STATUS_RUNNING ? "运行中" : "空闲");
rt_kprintf("帧率: %.1f FPS\n", status.fps);
```

---

#### BF30A2_CMD_GET_FPS (0x104)

**功能**: 获取当前帧率

**参数**: `float *` 类型指针

**返回值**: RT_EOK 成功

**示例**:
```c
float fps;
rt_device_control(cam_device, BF30A2_CMD_GET_FPS, &fps);
rt_kprintf("当前帧率: %.1f\n", fps);
```

---

#### BF30A2_CMD_GET_FRAME_COUNT (0x105)

**功能**: 获取已采集的帧总数

**参数**: `rt_uint32_t *` 类型指针

**返回值**: RT_EOK 成功

**示例**:
```c
rt_uint32_t count;
rt_device_control(cam_device, BF30A2_CMD_GET_FRAME_COUNT, &count);
rt_kprintf("已采集帧数: %d\n", count);
```

---

#### BF30A2_CMD_SET_CALLBACK (0x106)

**功能**: 设置帧就绪回调函数

**参数**: `bf30a2_callback_cfg_t *` 类型指针

**返回值**: RT_EOK 成功

**bf30a2_callback_cfg_t 结构体**:
```c
typedef struct bf30a2_callback_cfg {
    bf30a2_frame_callback_t callback;  /* 回调函数指针 */
    void *user_data;                   /* 用户上下文指针 */
} bf30a2_callback_cfg_t;
```

**示例**:
```c
void on_frame_ready(rt_device_t dev, rt_uint32_t frame_num,
                    rt_uint8_t *buffer, rt_uint32_t size, void *user_data)
{
    rt_kprintf("收到帧 %d, 大小 %d 字节\n", frame_num, size);
}

bf30a2_callback_cfg_t cb_cfg = {
    .callback = on_frame_ready,
    .user_data = RT_NULL
};
rt_device_control(cam_device, BF30A2_CMD_SET_CALLBACK, &cb_cfg);
```

---

#### BF30A2_CMD_GET_BUFFER (0x107)

**功能**: 获取帧缓冲区指针

**参数**: `bf30a2_buffer_t *` 类型指针

**返回值**: RT_EOK 成功

**bf30a2_buffer_t 结构体**:
```c
typedef struct bf30a2_buffer {
    rt_uint8_t *data;          /* 帧数据指针 */
    rt_uint32_t size;          /* 缓冲区大小 (字节) */
    rt_uint32_t frame_num;     /* 帧序号 */
    rt_uint32_t timestamp;     /* 采集时间戳 (tick) */
} bf30a2_buffer_t;
```

**示例**:
```c
bf30a2_buffer_t buf;
rt_device_control(cam_device, BF30A2_CMD_GET_BUFFER, &buf);
rt_kprintf("缓冲区地址: 0x%08X, 大小: %d\n", (uint32_t)buf.data, buf.size);
```

---

#### BF30A2_CMD_WAIT_FRAME (0x108)

**功能**: 等待下一帧就绪

**参数**: `bf30a2_wait_cfg_t *` 类型指针 (可选,传入 NULL 使用默认超时 1000ms)

**返回值**: RT_EOK 成功,-RT_ETIMEOUT 超时

**bf30a2_wait_cfg_t 结构体**:
```c
typedef struct bf30a2_wait_cfg {
    rt_uint32_t timeout_ms;    /* 等待超时时间 (毫秒) */
    bf30a2_buffer_t *buffer;   /* 输出缓冲区信息 (可选) */
} bf30a2_wait_cfg_t;
```

**示例**:
```c
bf30a2_buffer_t buf;
bf30a2_wait_cfg_t wait_cfg = {
    .timeout_ms = 500,
    .buffer = &buf
};

rt_err_t ret = rt_device_control(cam_device, BF30A2_CMD_WAIT_FRAME, &wait_cfg);
if (ret == RT_EOK) {
    rt_kprintf("帧就绪, 序号: %d\n", buf.frame_num);
}
```

---

#### BF30A2_CMD_EXPORT_UART (0x109)

**功能**: 通过 UART 导出当前帧数据

**参数**: 无 (传入 RT_NULL)

**返回值**: RT_EOK 成功

**输出格式**:
```
===PHOTO_START===
WIDTH:240
HEIGHT:320
FORMAT:RGB565
SIZE:153600
===DATA_BEGIN===
<十六进制数据>
===DATA_END===
===PHOTO_END===
```

**示例**:
```c
rt_device_control(cam_device, BF30A2_CMD_EXPORT_UART, RT_NULL);
```

---

#### BF30A2_CMD_RESET_STATS (0x10A)

**功能**: 重置统计信息

**参数**: 无 (传入 RT_NULL)

**返回值**: RT_EOK 成功

**重置内容**: 帧计数、完成帧数、错误计数、帧率等统计数据

**示例**:
```c
rt_device_control(cam_device, BF30A2_CMD_RESET_STATS, RT_NULL);
```

---
## Shell 命令

驱动提供以下 MSH Shell 命令用于调试:

| 命令 | 说明 |
|------|------|
| `bf30a2_init` | 初始化摄像头设备 |
| `bf30a2_open` | 打开摄像头设备 |
| `bf30a2_start` | 启动采集 |
| `bf30a2_stop` | 停止采集 |
| `bf30a2_status` | 显示摄像头状态 |
| `bf30a2_export` | 通过 UART 导出帧数据 |

## 典型使用流程

```c
#include "drv_bf30a2.h"

static rt_device_t cam_device = RT_NULL;

/* 帧回调函数 */
static void on_frame_ready(rt_device_t dev, rt_uint32_t frame_num,
                          rt_uint8_t *buffer, rt_uint32_t size, void *user_data)
{
    /* 处理帧数据 */
}

int camera_example(void)
{
    rt_err_t ret;

    /* 1. 注册设备 */
    ret = bf30a2_device_register();
    if (ret != RT_EOK) return ret;

    /* 2. 查找设备 */
    cam_device = rt_device_find(BF30A2_DEVICE_NAME);
    if (cam_device == RT_NULL) return -RT_ENOSYS;

    /* 3. 初始化设备 */
    ret = rt_device_init(cam_device);
    if (ret != RT_EOK) return ret;

    /* 4. 打开设备 */
    ret = rt_device_open(cam_device, RT_DEVICE_FLAG_RDONLY);
    if (ret != RT_EOK) return ret;

    /* 5. 设置帧回调 */
    bf30a2_callback_cfg_t cb_cfg = {
        .callback = on_frame_ready,
        .user_data = RT_NULL
    };
    rt_device_control(cam_device, BF30A2_CMD_SET_CALLBACK, &cb_cfg);

    /* 6. 启动采集 */
    rt_device_control(cam_device, BF30A2_CMD_START, RT_NULL);

    /* ... 运行中 ... */

    /* 7. 停止采集 */
    rt_device_control(cam_device, BF30A2_CMD_STOP, RT_NULL);

    /* 8. 关闭设备 */
    rt_device_close(cam_device);

    return RT_EOK;
}
```

### 4.4 按键操作

| 状态 | KEY1 | KEY2 |
|------|------|------|
| 待机 | 进入预览 | - |
| 预览 | 拍照 | 返回待机 |
| 照片 | - | 返回预览 |

---

## 5. 技术参数

| 参数 | 值 |
|------|-----|
| 图像分辨率 | 240 × 320 |
| 输出格式 | RGB565 |
| SPI时钟 | 24MHz |
| 帧率 | 6~15 FPS |
| 单帧大小 | 153,600 字节 |

