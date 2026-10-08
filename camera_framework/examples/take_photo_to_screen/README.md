# take_photo_to_screen

SF32LB57x（`spi-hdk_lb575ybbn6_n16`）上的摄像头示例：连续预览刷到 CO5300 屏
（390×450），按 KEY1 或用控制台命令把一张照片写到 SD 卡根目录。

摄像头只通过 `camera_framework` 的 handle 层使用：传感器、输出接口、采集方式和
图像后处理全部在 menuconfig 里选择，工程本身不带传感器驱动。可选组合：

- `GC032A`：`8-bit DVP`（硬件 DCMI）或 `2-bit SPI`（57x 上走 DCMI 的 SPI 包模式）
- `OV2640`：`DVP`（硬件 DCMI）或 `Arducam SPI FIFO`（JPEG）
- `BF30A2`：固定 RGB565 / 240×320（SPI）

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

`project/proj.conf` 只固定与开发板相关的项（SDMMC2、ADC 按键、CPU profiler、
SCCB/XCLK 引脚）；传感器、输出接口、采集方式、ISP 与 JPEG 的运行核在 menuconfig
中选择，板级引脚在 `project/Kconfig.proj` 中：

- `Camera drivers -> Sensor settings -> Active camera sensor`：选择传感器
- 同级的输出接口：`8-bit DVP` / `2-bit SPI` / `Arducam SPI FIFO`
- `Data bus settings`：DVP 选 `Hardware DCMI (HAL)`（同时关闭 `BSP_USING_DCMI`）；
  2-bit SPI 在 57x 上默认 `2-bit capture mode -> DCMI SPI 2-bit packet mode`
- `Camera software ISP` / `Camera software JPEG`：是否使用软件 ISP / 编解码，以及
  各自运行在 HCPU 还是 ACPU

## 接线（`spi-hdk_lb575ybbn6_n16`）

GC032A 2-bit SPI（DCMI SPI 包模式；引脚可在 `project/Kconfig.proj` 中修改）：

| GC032A 模块引脚 | 开发板 | 说明 |
| --- | --- | --- |
| IIC_SDA | PA20 | SCCB / I2C1 SDA |
| IIC_SCL | PA27 | SCCB / I2C1 SCL |
| MCLK | PA39 | 24 MHz XCLK（GPTIM2_CH1） |
| SPI_CLK | PA40 | DCMI_CLK，串行时钟 |
| SPI_D0 | PA41 | DCMI_DI0 |
| SPI_D1 | PA43 | DCMI_DI1 |

数据线接反时打开 `Swap the two data lines` 即可，不必改飞线；采样沿、每行
字节数（默认 1280）、包标识（`0x9d80b6ca`）和同步字（`0x00ffffff`）同样在
`Data bus settings` 中配置。

OV2640（DVP，硬件 DCMI）默认引脚：VSYNC=PA37、HSYNC=PA38、PCLK=PA40、
D0..D7=PA41/43/44/42/48/47/46/45、XCLK=PA39，SCCB 同上。OV2640 的 Arducam FIFO 与
BF30A2 走 SPI，引脚见各自的 Kconfig 菜单。
```text
camera: captured=.. displayed=.. dropped=.. failed=.. fps=.. period=.. copy=.. display=.. ms
camera: isp n=.. acpu=.. hcpu=.. ipc=.. ms
camera: stage n=.. clean=.. blend=.. wait=.. inv=.. dclean=.. push=.. ms
```
`captured/displayed/dropped/failed` 每 5 秒统计一次；`period` 是采集周期上限，`copy`
是每帧复制耗时，`display` 是刷屏耗时（软件 ISP 下 `display == isp + write`，`write`
为 EPIC 等待加面板推送）。`stage` 行把刷屏拆成 clean / blend / wait / inv / dclean /
push，用来定位 EPIC 的旋转、双层混合与缓存维护开销；`camera: isp n=..` 把一次 ISP
的 HCPU 墙钟、ACPU 侧耗时和 IPC 差值分开给出。

## 按键与命令

- KEY1（PA34）或 ADC key0：拍照（ADC key1/key2 保留给相册与下一张，本示例未实现）
- `camera_photo`：排队拍一张，预览未运行时打印 `photo ignored, preview is not running`
- `camera_stats [window_ms]`：按窗口统计帧数、ISP/拍照耗时与双核占用
- `camera_preview [rotate 0|1] [background 0|1] [swap 0|1]`：运行时改 EPIC 旋转、
  背景层与显示字节序，用于新摄像头在实板上确认方向与颜色
- `camera_sccb_scan [sda scl]`：扫描 SCCB 总线（可先改 I2C1 引脚复用），确认传感器
  地址与接线

## 拍照与文件

- 传感器自带 JPEG 编码器（OV2640）：拍照时把传感器切到它支持的最大 JPEG 分辨率
  （1600×1200），一次连续接收 9 帧预热 + 1 帧保留，取最后一帧写卡；随后恢复预览
  配置并丢弃 2 帧（曝光重新收敛）。
- 只有 RGB565 的传感器（GC032A、BF30A2）：编码当前预览帧，分辨率与预览相同。

照片写到 SD 卡根目录：`/IMG_0001.JPG`、`/IMG_0002.JPG`……（SDMMC2 枚举为 `sd1`，
挂载在 `/`），写卡前后校验 JPEG 的 SOI/EOI 标记。每张照片打印两行，软件编码路径的
格式为：

```text
camera: photo /IMG_0001.JPG <bytes> B <w>x<h> encode=<ms> write=<ms> ms
camera: photo cpu hcpu=<%> acpu=<%> codec=<hcpu|acpu|sensor>
```

传感器 JPEG 路径把中间的耗时项换成 `setup=` / `capture=` / `frames=` / `write=`。
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