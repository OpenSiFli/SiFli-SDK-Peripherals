# take_photo_to_screen

SF32LB57x（`spi-hdk_lb575ybbn6_n16`）上的摄像头示例：连续预览刷到 LCD，按 KEY1 或
用控制台命令把一张照片写到 SD 卡根目录。

预览固定按 **640×480（30 万像素）** 采集，屏幕上怎么显示由面板尺寸决定，不需要改代码：

- **面板小于 640×480**：拿到整帧后按"填满屏幕"缩放并居中裁剪（例如 CO5300 390×450）。
- **面板大于等于 640×480**：不缩放、不裁剪，帧的左上角抵住屏幕左上角（原点 0,0）显示，
  屏幕其余部分保持原样；每次只刷新这块区域。
- 面板尺寸由 LCD 驱动在 `camera_display_open()` 里读取，不再有"尺寸不匹配就初始化失败"
  的校验；旋转（`camera_preview 1`）会先把帧转成 480×640 再按上面两条规则判断。

摄像头只通过 `camera_framework` 的 handle 层使用：传感器、通信形式、采集方式和
图像后处理全部在 menuconfig 里选择，工程本身不带传感器驱动。可选组合：

- `GC032A`：`8-bit DVP（并行）` 或 `2-bit SPI（串行）`
- `OV2640`：`8-bit DVP（并行）` 或 `SPI（ArduCam FIFO 模组）`；选后者还要挑 `SPI1`/`SPI2`
- `BF30A2`：固定 RGB565 / 240×320（SPI），菜单里挑 `SPI1`/`SPI2`

软件 ISP 与 JPEG 编解码由框架持有（`camera_framework/software/{isp,jpeg}`，两个核
共用同一份源码），可分别配置运行在 HCPU 或 ACPU 上；本工程只保留 HCPU/ACPU 胶水
（`components/camera_software/example/acpu/`）。
```sh
source /path/to/SiFli-SDK/export.sh
scons -C camera_framework/examples/take_photo_to_screen/project \
      --board=spi-hdk_lb575ybbn6_n16 -j8
```
构建成功只说明固件能生成，采集、显示和写卡闭环仍需实板验收。

## 配置

`project/proj.conf` 只固定与开发板相关的项（SDMMC2、ADC 按键、CPU profiler、相机
I2C 设备名）；传感器、通信形式、采集方式、ISP 与 JPEG 的运行核在 menuconfig 中选择，
DVP/2-bit SPI 与 MCLK 的引脚复用由开发板自己的 `bsp_pinmux.c`（`BSP_USING_DCMI` 块）
完成，`project/Kconfig.proj` 不再定义引脚菜单：

- `Camera drivers -> Sensor settings -> Active camera sensor`：选择传感器
- `Camera drivers -> Data bus settings`：通信形式，随传感器变化
  （OV2640/GC032A：`8-bit DVP（并行）` 或 `SPI/2-bit SPI`；BF30A2：`SPI1`/`SPI2`；
  选 ArduCam 时额外挑 `SPI1`/`SPI2`）
- `Camera software ISP` / `Camera software JPEG`：是否使用软件 ISP / 编解码，以及
  各自运行在 HCPU 还是 ACPU
- 开发板的板级配置会启用 SDK 自带的 GC032A 驱动（与本框架争用同一个 DCMI 外设），
  因此 `project/proj.conf` 保留 `# CONFIG_BSP_USING_CAMERA1 is not set`

## 接线（`spi-hdk_lb575ybbn6_n16`）

GC032A 2-bit SPI（DCMI SPI 包模式；引脚由开发板 `bsp_pinmux.c` 的 `BSP_USING_DCMI` 块复用）：

| GC032A 模块引脚 | 开发板 | 说明 |
| --- | --- | --- |
| IIC_SDA | PA20 | SCCB / I2C1 SDA |
| IIC_SCL | PA27 | SCCB / I2C1 SCL |
| MCLK | PA39 | 24 MHz XCLK（GPTIM2_CH1） |
| SPI_CLK | PA40 | DCMI_CLK，串行时钟 |
| SPI_D0 | PA41 | DCMI_DI0 |
| SPI_D1 | PA43 | DCMI_DI1 |

2-bit 串行的包格式（每行字节数、包标识、同步字、采样沿、数据线顺序）是传感器协议，
现在写在 `core/driver/gc032a/gc032a.c` 的 `s_serial_2bit` 常量里，运行时下发给 DCMI
后端；接线顺序不一致时改那个常量，不再有菜单项。

OV2640（DVP，硬件 DCMI）默认引脚：VSYNC=PA37、HSYNC=PA38、PCLK=PA40、
D0..D7=PA41/43/44/42/48/47/46/45、XCLK=PA39，SCCB 同上。OV2640 的 ArduCam FIFO 与
BF30A2 走 SPI，SPI 实例在 `Data bus settings` 里选，引脚由板级复用。
```text
camera: captured=.. displayed=.. dropped=.. failed=.. fps=.. period=.. copy=.. display=.. ms
camera: isp n=.. acpu=.. hcpu=.. ipc=.. ms
camera: stage n=.. dclean=.. blend=.. wait=.. push=.. ms
```
`captured/displayed/dropped/failed` 每 5 秒统计一次；`period` 是采集周期上限，`copy`
是每帧复制耗时，`display` 是刷屏耗时（软件 ISP 下 `display == isp + write`，`write`
为 EPIC 等待加面板推送）。`stage` 行把刷屏拆成 dclean / blend / wait / push，用来定位
EPIC 的旋转、双层混合与面板推送开销；EPIC 的输入 clean、输出 invalidate 由 EPIC 驱动
自己做，预览路径不再重复维护，`dclean` 只在 CPU 写过预览缓冲（清屏、叠加文字）时才非
零。`camera: isp n=..` 把一次 ISP 的 HCPU 墙钟、ACPU 侧耗时和 IPC 差值分开给出。

## 按键与命令

- KEY1（PA34）：拍照
- ADC key0：拍照（`camera_keys.c` 里把 `CAMERA_KEYS_USE_ADC_KEYS` 改成 1 才启用，
  见下一节）
- `camera_photo`：排队拍一张，预览未运行时打印 `photo ignored, preview is not running`
- `camera_stats [window_ms]`：按窗口统计帧数、ISP/拍照耗时与双核占用
- `camera_preview [rotate 0|1] [background 0|1] [swap 0|1]`：运行时改 EPIC 旋转、
  背景层与显示字节序，用于新摄像头在实板上确认方向与颜色

### 关于 ADC 按键

`button_bind_adc_button()` 会把绑定按钮自己的 GPIO 回调替换成 ADC 组的回调，所以
一条按键不能同时走 GPIO 和 ADC 两条路。`spi-hdk` 板配置的 ADC 按键组用的是电池
检测通道（`bat1` / channel 6），读到的电压是电池电压而不是按键分压，按下 KEY1 时
只会打印 `Unknown pin:0,34,<action>` 而不会拍照。因此本示例默认只把 KEY1（PA34）
当普通 GPIO 按键用；只有按键载板真的把 ADC 通道接到分压电阻上时，才把
`src/input/camera_keys.c` 里的 `CAMERA_KEYS_USE_ADC_KEYS` 置 1。

## 拍照与文件

- 传感器自带 JPEG 编码器（OV2640）：拍照时把传感器切到它支持的最大 JPEG 分辨率
  （**UXGA 1600×1200**，全 2 MP），存的就是传感器自己压出来的那张 JPEG。
- 保留的那一帧来自**一次连续接收**：先收 9 帧预热丢掉，第 10 帧留下写卡，所以拍照缓冲
  要能放下 10 帧。DCMI 一置位就开始搬数据，缓冲区开头可能是置位前就在传的那一帧的尾巴，
  因此后端不是从 offset 0 数帧，而是在整段接收数据里找**最后一个完整的 JPEG**（SOI…EOI
  能完整解析的那一张），把它搬到缓冲区开头再返回。
- 因为拍照缓冲借用了预览的双 DMA 缓冲，**拍照前必须把 stream 和 capture 两条通道都停掉**
  （`camera_stop_stream()` + `camera_stop_capture()` 都返回成功），否则 DCMI 可能仍按上次
  预览的 DMA 目标写入：看起来收满了、字节数也正常，但缓冲里其实还是预览像素。
- 只有 RGB565 的传感器（GC032A、BF30A2）：编码当前预览帧，分辨率与预览相同。

照片写到 SD 卡根目录：`/IMG_0001.JPG`、`/IMG_0002.JPG`……（SDMMC2 枚举为 `sd1`，
挂载在 `/`），写卡前后校验 JPEG 的 SOI/EOI 标记。每张照片打印两行，软件编码路径的
格式为：

```text
camera: photo /IMG_0001.JPG <bytes> B <w>x<h> encode=<ms> write=<ms> ms
camera: photo cpu hcpu=<%> acpu=<%> codec=<hcpu|acpu|sensor>
```

传感器 JPEG 路径把中间的耗时项换成 `setup=` / `capture=` / `frames=` / `write=`，
`frames=` 是这次连续接收的帧数（预热 + 保留），`capture=` 是整段接收的时长。整条路径
失败时打印 `camera: still capture failed status=… bytes=…`。
```sh
source /path/to/SiFli-SDK/export.sh
scons -C camera_framework/examples/isp/project \
      --board=spi-hdk_lb573ub7n6_hcpu
scons -C camera_framework/examples/jpeg/project \
      --board=spi-hdk_lb573ub7n6_hcpu
scons -C camera_framework/examples/acpu/project/hcpu \
      --board=spi-hdk_lb573ub7n6_hcpu
```
ISP 和 JPEG 例程烧录后分别输出 `status=PASS`；ACPU 例程需要双核板，并同时生成 HCPU
与 ACPU 镜像。详细 API、Kconfig 选项和内存约束见
`camera_framework/software/isp/README.md` 与
`camera_framework/software/jpeg/README.md`。

## 颜色与方向

`RGB565_BE` 是后处理与显示使用的 16-bit 像素格式。每个摄像头的字节序、显示交换与
EPIC 旋转/背景层集中在 `src/camera/camera_profile.h`：OV2640 低字节在前（面板原生），
GC032A 与 2-bit SPI 高字节在前（软件 ISP 的 gamma 曲线也按它标定），旋转与背景层默认
关闭（EPIC 旋转 + 双层混合约 245 ms/帧，单层无旋转约 5 ms）。换摄像头后先用
`camera_preview` 在实板上确认颜色和方向，再把结论记回该头文件。

## 屏幕尺寸

预览缓冲按"一帧 VGA"申请（`CAMERA_DISPLAY_PREVIEW_WIDTH/HEIGHT` = 640×480，见
`src/display/camera_display.h`），面板尺寸在 `camera_display_open()` 里从 LCD 驱动读：

| 面板 | 行为 | 每次刷新的区域 |
| --- | --- | --- |
| ≥ 640×480（如 1024×600） | 不缩放、不裁剪，帧左上角贴屏幕左上角，其余保持原样 | 640×480，位于 (0,0) |
| < 640×480（如 390×450、240×320） | 缩放到"填满"屏幕 + 居中裁剪 | 整屏 |

启动日志：

```text
camera: panel 1024x600 fmt=rgb888 align=1, buffer 614400 bytes
camera: panel larger than buffer, preview window 640x480 at (0,0)
camera: cfg 640x480 rgb565 ...
camera: push 640x480 queued (window from layer area)
camera: first push 640x480 at (0,0) done        ← 这条路通了才会出现
```

推送是异步的（`draw_rect_async()` + `tx_complete` 回调等待，200 ms 超时；超时打印
`camera: push WxH timed out` 并跳帧，不会卡死系统）。DPI 屏（`fmt=rgb888`）跳过同步的
`set_window()`，QSPI 屏（`fmt=rgb565`）仍会调用。

### 内存约束（为什么不能盲目加大缓冲）

预览缓冲在 `.RW_PSRAM_NON_RET`：PSRAM 区域共 4 MB，这个段已经用到 `0x3E4000`
（3,072,000 B 相机帧 + 393,216 B 图像处理 + 614,400 B JPEG 解码），**只剩 114,688 B**。
所以：

- 把预览缓冲改成整屏 1024×600×2（1 228 800 B）会溢出 499,712 B，链接直接失败；
- 若将来需要 RGB888 输出（这块 DPI 面板的层是 RGB888），640×480×3 = 921,600 B，
  还需要额外 307,200 B —— 必须先腾出 `.RW_PSRAM_NON_RET` 里的空间
  （例如 `camera_display_buffers` 的 JPEG 解码缓冲 614,400 B，或 `camera_frames.stable[]`）。