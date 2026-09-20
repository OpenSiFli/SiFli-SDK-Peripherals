/**
 * @file bf30a2.c
 * @brief BF30A2 SPI camera sensor driver for camera_handle.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#include "bf30a2.h"

#include "../camera_driver_desc.h"
#include "bf30a2_regs.h"
#include "bf0_hal.h"
#include "camera_xclk.h"
#include "drv_io.h"
#include "drv_spi.h"
#include "rtdevice.h"
#include "rthw.h"
#include <string.h>

#define DBG_TAG "bf30a2"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define BF30A2_SPI_BUS_NAME       "spi2"
#define BF30A2_SPI_DEVICE_NAME    "bf30a2"
#define BF30A2_LINE_BYTES         (BF30A2_WIDTH * 2U)
#define BF30A2_CAPTURE_EVENT      0x01U
#define BF30A2_FRAME_TIMEOUT_MS   CAMERA_READ_TIMEOUT_MS

typedef enum
{
    PARSE_FIND_SYNC,
    PARSE_GET_TYPE,
    PARSE_FRAME_FORMAT,
    PARSE_FRAME_WIDTH_H,
    PARSE_FRAME_WIDTH_L,
    PARSE_FRAME_HEIGHT_H,
    PARSE_FRAME_HEIGHT_L,
    PARSE_LINE_NUM_H,
    PARSE_LINE_NUM_L,
    PARSE_DATA_SYNC_1,
    PARSE_DATA_SYNC_2,
    PARSE_DATA_SYNC_3,
    PARSE_DATA_TYPE,
    PARSE_DATA_SIZE_H,
    PARSE_DATA_SIZE_L,
    PARSE_PIXEL_DATA,
} bf30a2_parse_state_t;

typedef struct
{
    struct rt_spi_device *spi_dev;
    SPI_HandleTypeDef *hspi;
    struct rt_i2c_bus_device *i2c_bus;
    rt_device_t gpio_dev;
    rt_event_t event;
    struct rt_semaphore frame_sem;
    rt_mutex_t lock;
    rt_thread_t thread;
    uint8_t *dma_buf;
    uint32_t dma_size;
    uint8_t line_yuv[BF30A2_LINE_BYTES];
    bf30a2_parse_state_t state;
    uint8_t ff_count;
    uint16_t frame_width;
    uint16_t frame_height;
    uint16_t line_num;
    uint16_t data_size;
    uint16_t data_pos;
    uint16_t lines_received;
    uint16_t chip_id;
    uint8_t *frame_buffer;
    rt_size_t frame_capacity;
    rt_size_t frame_size;
    rt_bool_t in_frame;
    rt_bool_t frame_ready;
    rt_bool_t initialized;
    rt_bool_t open;
    rt_bool_t running;
    rt_bool_t stop_request;
    rt_bool_t async_mode;
    camera_capture_done_callback_t async_callback;
    void *async_context;
    camera_stream_frame_callback_t stream_callback;
    void *stream_context;
    uint8_t *stream_buffers[2];
    rt_size_t stream_buffer_size;
    uint8_t stream_buffer_index;
    uint32_t sequence;
} bf30a2_device_t;

static bf30a2_device_t s_device;

typedef struct
{
    uint32_t xclk_frequency_hz;
} bf30a2_hw_config_t;

static const bf30a2_hw_config_t g_hw_config =
{
    .xclk_frequency_hz = 24000000U,
};

static const pixformat_t s_pixformats[] =
{
    PIXFORMAT_RGB565,
};

static const framesize_t s_framesizes[] =
{
    FRAMESIZE_240X320,
};

static const camera_capabilities_t s_capabilities =
{
    .pixformats = s_pixformats,
    .num_pixformats = sizeof(s_pixformats) / sizeof(s_pixformats[0]),
    .framesizes = s_framesizes,
    .num_framesizes = sizeof(s_framesizes) / sizeof(s_framesizes[0]),
    .max_buffer_size = BF30A2_FRAME_SIZE,
};

static const camera_capture_config_t s_default_config =
{
    .pixformat = PIXFORMAT_RGB565,
    .framesize = FRAMESIZE_240X320,
    .quality = 0,
};

static int bf30a2_open(void);
static int bf30a2_close(void);
static int bf30a2_set_pixformat(pixformat_t pixformat);
static int bf30a2_set_framesize(framesize_t framesize);
static rt_size_t bf30a2_capture(void *buffer, rt_size_t buffer_size);
static int bf30a2_capture_async(void *buffer,
                                rt_size_t buffer_size,
                                camera_capture_done_callback_t callback,
                                void *context);
static int bf30a2_start_stream(const camera_stream_start_args_t *args);
static int bf30a2_stop_stream(void);
static void stop_dma(void);

const camera_device_ops_t bf30a2_ops =
{
    .capabilities = &s_capabilities,
    .default_config = &s_default_config,
    .open = bf30a2_open,
    .close = bf30a2_close,
    .set_pixformat = bf30a2_set_pixformat,
    .set_framesize = bf30a2_set_framesize,
    .set_quality = RT_NULL,
    .capture = bf30a2_capture,
    .capture_async = bf30a2_capture_async,
    .start_stream = bf30a2_start_stream,
    .stop_stream = bf30a2_stop_stream,
};

CAMERA_DRIVER_EXPORT(bf30a2, &bf30a2_ops);

static uint8_t clamp8(int value)
{
    if (value < 0)
    {
        return 0U;
    }
    if (value > 255)
    {
        return 255U;
    }
    return (uint8_t)value;
}

static void yuv422_line_to_rgb565(const uint8_t *yuv, uint8_t *rgb)
{
    uint32_t x;

    for (x = 0; x < BF30A2_WIDTH; x += 2U)
    {
        int y0 = yuv[0];
        int cb = yuv[1] - 128;
        int y1 = yuv[2];
        int cr = yuv[3] - 128;
        uint16_t p0;
        uint16_t p1;

        yuv += 4;
        p0 = (uint16_t)(((clamp8(y0 + ((359 * cr) >> 8)) & 0xf8U) << 8) |
                        ((clamp8(y0 - ((88 * cb + 183 * cr) >> 8)) & 0xfcU) << 3) |
                        (clamp8(y0 + ((454 * cb) >> 8)) >> 3));
        p1 = (uint16_t)(((clamp8(y1 + ((359 * cr) >> 8)) & 0xf8U) << 8) |
                        ((clamp8(y1 - ((88 * cb + 183 * cr) >> 8)) & 0xfcU) << 3) |
                        (clamp8(y1 + ((454 * cb) >> 8)) >> 3));
        *rgb++ = (uint8_t)(p0 >> 8);
        *rgb++ = (uint8_t)p0;
        *rgb++ = (uint8_t)(p1 >> 8);
        *rgb++ = (uint8_t)p1;
    }
}

static void parse_reset(bf30a2_device_t *dev)
{
    dev->state = PARSE_FIND_SYNC;
    dev->ff_count = 0;
    dev->frame_width = 0;
    dev->frame_height = 0;
    dev->line_num = 0;
    dev->data_size = 0;
    dev->data_pos = 0;
    dev->lines_received = 0;
    dev->in_frame = RT_FALSE;
    dev->frame_ready = RT_FALSE;
    dev->frame_size = 0;
}

static void frame_start(bf30a2_device_t *dev)
{
    dev->in_frame = RT_TRUE;
    dev->lines_received = 0;
}

static void frame_complete(bf30a2_device_t *dev)
{
    camera_capture_done_callback_t async_callback;
    void *async_context;

    if (!dev->in_frame || dev->lines_received < (BF30A2_HEIGHT * 8U / 10U))
    {
        dev->in_frame = RT_FALSE;
        return;
    }

    dev->frame_ready = RT_TRUE;
    dev->frame_size = BF30A2_FRAME_SIZE;
    dev->sequence++;

    if (dev->stream_callback != RT_NULL)
    {
        camera_stream_frame_t frame = {0};

        frame.buffer_index = dev->stream_buffer_index;
        frame.buffer = dev->frame_buffer;
        frame.buffer_size = dev->stream_buffer_size;
        frame.frame_size = BF30A2_FRAME_SIZE;
        frame.sequence = dev->sequence;
        frame.is_complete = RT_TRUE;
        dev->stream_callback(dev->stream_context, &frame);
        dev->stream_buffer_index ^= 1U;
        dev->frame_buffer = dev->stream_buffers[dev->stream_buffer_index];
        dev->frame_capacity = dev->stream_buffer_size;
        parse_reset(dev);
        return;
    }

    async_callback = dev->async_callback;
    async_context = dev->async_context;
    dev->async_callback = RT_NULL;
    dev->async_context = RT_NULL;
    dev->async_mode = RT_FALSE;
    if (async_callback != RT_NULL)
    {
        async_callback(async_context, CAMERA_OK, BF30A2_FRAME_SIZE);
    }
    rt_sem_release(&dev->frame_sem);
    dev->in_frame = RT_FALSE;
}

static void line_complete(bf30a2_device_t *dev)
{
    if (dev->in_frame && dev->line_num < BF30A2_HEIGHT &&
        dev->frame_buffer != RT_NULL &&
        dev->frame_capacity >= BF30A2_FRAME_SIZE)
    {
        yuv422_line_to_rgb565(dev->line_yuv,
                              dev->frame_buffer +
                              ((uint32_t)dev->line_num * BF30A2_LINE_BYTES));
        dev->lines_received++;
    }
}

static void parse_byte(bf30a2_device_t *dev, uint8_t value)
{
    switch (dev->state)
    {
    case PARSE_FIND_SYNC:
        if (value == 0xffU)
        {
            dev->ff_count++;
            if (dev->ff_count >= 3U)
            {
                dev->state = PARSE_GET_TYPE;
                dev->ff_count = 0;
            }
        }
        else
        {
            dev->ff_count = 0;
        }
        break;
    case PARSE_GET_TYPE:
        if (value == 0x01U)
        {
            dev->state = PARSE_FRAME_FORMAT;
        }
        else if (value == 0x02U)
        {
            dev->state = PARSE_LINE_NUM_H;
        }
        else if (value == 0x00U)
        {
            frame_complete(dev);
            dev->state = PARSE_FIND_SYNC;
        }
        else
        {
            dev->state = PARSE_FIND_SYNC;
            dev->ff_count = value == 0xffU ? 1U : 0U;
        }
        break;
    case PARSE_FRAME_FORMAT:
        dev->state = value == 0x00U ? PARSE_FRAME_WIDTH_H : PARSE_FIND_SYNC;
        break;
    case PARSE_FRAME_WIDTH_H:
        dev->frame_width = (uint16_t)value << 8;
        dev->state = PARSE_FRAME_WIDTH_L;
        break;
    case PARSE_FRAME_WIDTH_L:
        dev->frame_width |= value;
        dev->state = PARSE_FRAME_HEIGHT_H;
        break;
    case PARSE_FRAME_HEIGHT_H:
        dev->frame_height = (uint16_t)value << 8;
        dev->state = PARSE_FRAME_HEIGHT_L;
        break;
    case PARSE_FRAME_HEIGHT_L:
        dev->frame_height |= value;
        if (dev->frame_width == BF30A2_WIDTH &&
            dev->frame_height == BF30A2_HEIGHT)
        {
            frame_start(dev);
        }
        dev->state = PARSE_FIND_SYNC;
        dev->ff_count = 0;
        break;
    case PARSE_LINE_NUM_H:
        dev->line_num = (uint16_t)value << 8;
        dev->state = PARSE_LINE_NUM_L;
        break;
    case PARSE_LINE_NUM_L:
        dev->line_num |= value;
        dev->state = PARSE_DATA_SYNC_1;
        break;
    case PARSE_DATA_SYNC_1:
        dev->state = value == 0xffU ? PARSE_DATA_SYNC_2 : PARSE_FIND_SYNC;
        break;
    case PARSE_DATA_SYNC_2:
        dev->state = value == 0xffU ? PARSE_DATA_SYNC_3 : PARSE_FIND_SYNC;
        break;
    case PARSE_DATA_SYNC_3:
        dev->state = value == 0xffU ? PARSE_DATA_TYPE : PARSE_FIND_SYNC;
        break;
    case PARSE_DATA_TYPE:
        dev->state = value == 0x40U ? PARSE_DATA_SIZE_H : PARSE_FIND_SYNC;
        break;
    case PARSE_DATA_SIZE_H:
        dev->data_size = (uint16_t)value << 8;
        dev->state = PARSE_DATA_SIZE_L;
        break;
    case PARSE_DATA_SIZE_L:
        dev->data_size |= value;
        if (dev->data_size == BF30A2_LINE_BYTES)
        {
            dev->data_pos = 0;
            dev->state = PARSE_PIXEL_DATA;
        }
        else
        {
            dev->state = PARSE_FIND_SYNC;
        }
        break;
    case PARSE_PIXEL_DATA:
        dev->line_yuv[dev->data_pos++] = value;
        if (dev->data_pos >= BF30A2_LINE_BYTES)
        {
            line_complete(dev);
            dev->state = PARSE_FIND_SYNC;
            dev->ff_count = 0;
        }
        break;
    default:
        parse_reset(dev);
        break;
    }
}

static uint32_t dma_position(const bf30a2_device_t *dev)
{
    uint32_t remaining;

    if (dev->hspi == RT_NULL || dev->hspi->hdmarx == RT_NULL)
    {
        return 0U;
    }
    remaining = dev->hspi->hdmarx->Instance->CNDTR;
    if (remaining > dev->dma_size)
    {
        return 0U;
    }
    return dev->dma_size - remaining;
}

static void capture_thread(void *arg)
{
    bf30a2_device_t *dev = (bf30a2_device_t *)arg;
    uint32_t last_pos = dma_position(dev);

    while (!dev->stop_request)
    {
        rt_uint32_t event;
        uint32_t pos;

        rt_event_recv(dev->event,
                      BF30A2_CAPTURE_EVENT,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      50,
                      &event);
        pos = dma_position(dev);
        while (last_pos != pos && !dev->stop_request)
        {
#ifdef PSRAM_CACHE_WB
            mpu_dcache_invalidate(dev->dma_buf + last_pos, 1);
#endif
            parse_byte(dev, dev->dma_buf[last_pos]);
            last_pos++;
            if (last_pos >= dev->dma_size)
            {
                last_pos = 0;
            }
        }
        if (dev->frame_ready && dev->stream_callback == RT_NULL)
        {
            dev->stop_request = RT_TRUE;
        }
    }
    if (dev->frame_ready && dev->stream_callback == RT_NULL)
    {
        stop_dma();
        dev->running = RT_FALSE;
    }
    dev->thread = RT_NULL;
}

static rt_err_t rx_indicate(rt_device_t device, rt_size_t offset)
{
    (void)offset;
    if (device == (rt_device_t)s_device.spi_dev && s_device.event != RT_NULL)
    {
        rt_event_send(s_device.event, BF30A2_CAPTURE_EVENT);
    }
    return RT_EOK;
}

static int i2c_write_reg(uint8_t reg, uint8_t value)
{
    struct rt_i2c_msg msg;
    uint8_t buffer[2] = {reg, value};

    msg.addr = BF30A2_ADDR;
    msg.flags = RT_I2C_WR;
    msg.buf = buffer;
    msg.len = sizeof(buffer);
    return rt_i2c_transfer(s_device.i2c_bus, &msg, 1) == 1 ?
           RT_EOK : -RT_ERROR;
}

static int i2c_read_reg(uint8_t reg, uint8_t *value)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = BF30A2_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg;
    msgs[0].len = 1;
    msgs[1].addr = BF30A2_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = value;
    msgs[1].len = 1;
    return rt_i2c_transfer(s_device.i2c_bus, msgs, 2) == 2 ?
           RT_EOK : -RT_ERROR;
}

static int sensor_check_id(void)
{
    uint8_t high;
    uint8_t low;
    int ret;

    ret = i2c_read_reg(0xfc, &high);
    if (ret == RT_EOK)
    {
        ret = i2c_read_reg(0xfd, &low);
    }
    if (ret != RT_EOK)
    {
        return ret;
    }

    s_device.chip_id = (uint16_t)(((uint16_t)high << 8) | low);
    if (s_device.chip_id != BF30A2_CHIP_ID)
    {
        LOG_E("unexpected chip ID: 0x%04x", s_device.chip_id);
        return -RT_ERROR;
    }
    LOG_I("Detected BF30A2, ID=0x%04x", s_device.chip_id);
    return RT_EOK;
}

static int sensor_load_regs(void)
{
    uint32_t i;
    uint8_t pda;
    int ret = RT_EOK;

    for (i = 0; bf30a2_default_regs[i].reg != BF30A2_REGLIST_TAIL; i++)
    {
        if (bf30a2_default_regs[i].reg == BF30A2_REG_DELAY)
        {
            rt_thread_mdelay(bf30a2_default_regs[i].value);
            continue;
        }
        ret = i2c_write_reg((uint8_t)bf30a2_default_regs[i].reg,
                            bf30a2_default_regs[i].value);
        if (ret != RT_EOK)
        {
            return ret;
        }
        if (bf30a2_default_regs[i].reg == 0xf2U)
        {
            rt_thread_mdelay(10);
        }
    }

    if (i2c_read_reg(0xcf, &pda) == RT_EOK && (pda & 0x01U) != 0U)
    {
        ret = i2c_write_reg(0xcf, 0xb0);
        rt_thread_mdelay(10);
    }
    return ret;
}

static int xclk_start(void)
{
    if (CAMERA_XCLK_PIN < 0)
    {
        return RT_EOK;
    }
    return camera_xclk_start(CAMERA_XCLK_PIN, g_hw_config.xclk_frequency_hz) ==
           CAMERA_XCLK_OK ? RT_EOK : -RT_ERROR;
}

static void xclk_stop(void)
{
    if (CAMERA_XCLK_PIN >= 0)
    {
        camera_xclk_stop(CAMERA_XCLK_PIN);
    }
}

static int power_on(void)
{
    struct rt_device_pin_mode mode;
    struct rt_device_pin_status status;

    HAL_PIN_Set(PAD_PA43, GPIO_A0 + 43, PIN_NOPULL, 1);
    s_device.gpio_dev = rt_device_find("pin");
    if (s_device.gpio_dev == RT_NULL)
    {
        return -RT_ERROR;
    }
    rt_device_open(s_device.gpio_dev, RT_DEVICE_OFLAG_RDWR);
    mode.pin = 43;
    mode.mode = PIN_MODE_OUTPUT;
    rt_device_control(s_device.gpio_dev, 0, &mode);
    status.pin = 43;
    status.status = 1;
    rt_device_write(s_device.gpio_dev, 0, &status, sizeof(status));
    rt_thread_mdelay(1);
    status.status = 0;
    rt_device_write(s_device.gpio_dev, 0, &status, sizeof(status));
    rt_thread_mdelay(10);
    return RT_EOK;
}

static int i2c_init(void)
{
    struct rt_i2c_configuration config =
    {
        .mode = 0,
        .addr = 0,
        .timeout = 1000,
        .max_hz = 100000,
    };

    if (strcmp(CAMERA_SCCB_I2C_BUS_NAME, "i2c1") == 0)
    {
        HAL_PIN_Set(PAD_PA00 + CAMERA_SCCB_SCL_PIN, I2C1_SCL, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA00 + CAMERA_SCCB_SDA_PIN, I2C1_SDA, PIN_PULLUP, 1);
    }
    else
    {
        HAL_PIN_Set(PAD_PA00 + CAMERA_SCCB_SCL_PIN, I2C2_SCL, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA00 + CAMERA_SCCB_SDA_PIN, I2C2_SDA, PIN_PULLUP, 1);
    }
    s_device.i2c_bus = rt_i2c_bus_device_find(CAMERA_SCCB_I2C_BUS_NAME);
    if (s_device.i2c_bus == RT_NULL)
    {
        return -RT_ERROR;
    }
    if (rt_device_open((rt_device_t)s_device.i2c_bus,
                       RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        return -RT_ERROR;
    }
    rt_i2c_configure(s_device.i2c_bus, &config);
    return RT_EOK;
}

static int spi_init(void)
{
    struct rt_spi_configuration config;
    struct rt_spi_dma_circular_config circular;
    struct sifli_spi *driver;
    rt_err_t ret;

    HAL_PIN_Set(PAD_PA39, SPI2_CLK, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA37, SPI2_DIO, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA40, SPI2_CS, PIN_PULLDOWN, 1);
    if (rt_device_find(BF30A2_SPI_DEVICE_NAME) == RT_NULL)
    {
        ret = rt_hw_spi_device_attach(BF30A2_SPI_BUS_NAME,
                                      BF30A2_SPI_DEVICE_NAME);
        if (ret != RT_EOK)
        {
            return ret;
        }
    }
    s_device.spi_dev = (struct rt_spi_device *)
        rt_device_find(BF30A2_SPI_DEVICE_NAME);
    if (s_device.spi_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    ret = rt_device_open((rt_device_t)s_device.spi_dev,
                         RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_DMA_RX);
    if (ret != RT_EOK)
    {
        return ret;
    }

    rt_memset(&config, 0, sizeof(config));
    config.data_width = 8;
    config.max_hz = 24000000;
    config.frameMode = RT_SPI_MOTO;
    config.mode = RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_SLAVE | RT_SPI_3WIRE;
    ret = rt_spi_configure(s_device.spi_dev, &config);
    if (ret != RT_EOK)
    {
        return ret;
    }

    rt_device_set_rx_indicate((rt_device_t)s_device.spi_dev, rx_indicate);
    rt_memset(&circular, 0, sizeof(circular));
    circular.enable = 1U;
    circular.direction = RT_SPI_DMA_CIRCULAR_DIR_RX;
    ret = rt_device_control((rt_device_t)s_device.spi_dev,
                            RT_SPI_CTRL_CONFIG_DMA_CIRCULAR,
                            &circular);
    if (ret != RT_EOK)
    {
        rt_device_set_rx_indicate((rt_device_t)s_device.spi_dev, RT_NULL);
        return ret;
    }

    driver = rt_container_of(s_device.spi_dev->bus, struct sifli_spi, spi_bus);
    s_device.hspi = &driver->handle;
    return RT_EOK;
}

static int allocate_runtime(void)
{
    int ret;

    if (s_device.initialized)
    {
        return RT_EOK;
    }
    s_device.dma_size = CAMERA_BF30A2_DMA_BUFFER_SIZE;
    if (s_device.dma_size < 1024U || (s_device.dma_size & 1U) != 0U)
    {
        return -RT_EINVAL;
    }
    s_device.dma_buf = rt_malloc_align(s_device.dma_size, 32);
    if (s_device.dma_buf == RT_NULL)
    {
        return -RT_ENOMEM;
    }
    s_device.event = rt_event_create("bf30a2", RT_IPC_FLAG_FIFO);
    if (s_device.event == RT_NULL)
    {
        rt_free_align(s_device.dma_buf);
        s_device.dma_buf = RT_NULL;
        return -RT_ENOMEM;
    }
    ret = rt_sem_init(&s_device.frame_sem, "bf30_frm", 0, RT_IPC_FLAG_FIFO);
    if (ret != RT_EOK)
    {
        rt_event_delete(s_device.event);
        rt_free_align(s_device.dma_buf);
        s_device.event = RT_NULL;
        s_device.dma_buf = RT_NULL;
        return ret;
    }
    s_device.lock = rt_mutex_create("bf30a2", RT_IPC_FLAG_PRIO);
    if (s_device.lock == RT_NULL)
    {
        rt_sem_detach(&s_device.frame_sem);
        rt_event_delete(s_device.event);
        rt_free_align(s_device.dma_buf);
        s_device.event = RT_NULL;
        s_device.dma_buf = RT_NULL;
        return -RT_ENOMEM;
    }
    s_device.initialized = RT_TRUE;
    return RT_EOK;
}

static int start_dma(void)
{
    rt_size_t transferred;

    rt_memset(s_device.dma_buf, 0, s_device.dma_size);
    transferred = rt_spi_transfer(s_device.spi_dev,
                                  RT_NULL,
                                  s_device.dma_buf,
                                  s_device.dma_size);
    return transferred == s_device.dma_size ? RT_EOK : -RT_ERROR;
}

static void stop_dma(void)
{
    if (s_device.spi_dev != RT_NULL)
    {
        rt_device_control((rt_device_t)s_device.spi_dev,
                          RT_SPI_CTRL_STOP_DMA_CIRCULAR,
                          RT_NULL);
    }
}

static int start_capture_locked(uint8_t *buffer, rt_size_t size)
{
    if (!s_device.open || s_device.running ||
        buffer == RT_NULL || size < BF30A2_FRAME_SIZE)
    {
        return -RT_EINVAL;
    }

    parse_reset(&s_device);
    s_device.frame_buffer = buffer;
    s_device.frame_capacity = size;
    s_device.stop_request = RT_FALSE;
    s_device.running = RT_TRUE;
    if (start_dma() != RT_EOK)
    {
        s_device.running = RT_FALSE;
        return -RT_ERROR;
    }
    s_device.thread = rt_thread_create("bf30a2",
                                      capture_thread,
                                      &s_device,
                                      2048,
                                      RT_THREAD_PRIORITY_HIGH,
                                      10);
    if (s_device.thread == RT_NULL)
    {
        stop_dma();
        s_device.running = RT_FALSE;
        return -RT_ENOMEM;
    }
    rt_thread_startup(s_device.thread);
    return RT_EOK;
}

static void stop_capture_locked(void)
{
    if (!s_device.running)
    {
        return;
    }
    s_device.stop_request = RT_TRUE;
    if (s_device.event != RT_NULL)
    {
        rt_event_send(s_device.event, BF30A2_CAPTURE_EVENT);
    }
    rt_thread_mdelay(50);
    stop_dma();
    s_device.running = RT_FALSE;
    s_device.thread = RT_NULL;
}

static int bf30a2_open(void)
{
    int ret;

    ret = allocate_runtime();
    if (ret != RT_EOK)
    {
        return ret;
    }
    if (s_device.open)
    {
        return RT_EOK;
    }

    ret = xclk_start();
    if (ret == RT_EOK)
    {
        ret = power_on();
    }
    if (ret == RT_EOK)
    {
        ret = i2c_init();
    }
    if (ret == RT_EOK)
    {
        ret = sensor_check_id();
    }
    if (ret == RT_EOK)
    {
        ret = sensor_load_regs();
    }
    if (ret == RT_EOK)
    {
        ret = spi_init();
    }
    if (ret != RT_EOK)
    {
        bf30a2_close();
        return ret;
    }

    s_device.open = RT_TRUE;
    return RT_EOK;
}

static int bf30a2_close(void)
{
    if (s_device.lock != RT_NULL)
    {
        rt_mutex_take(s_device.lock, RT_WAITING_FOREVER);
        stop_capture_locked();
        rt_mutex_release(s_device.lock);
    }
    if (s_device.spi_dev != RT_NULL)
    {
        rt_device_set_rx_indicate((rt_device_t)s_device.spi_dev, RT_NULL);
        rt_device_close((rt_device_t)s_device.spi_dev);
        s_device.spi_dev = RT_NULL;
        s_device.hspi = RT_NULL;
    }
    if (s_device.i2c_bus != RT_NULL)
    {
        rt_device_close((rt_device_t)s_device.i2c_bus);
        s_device.i2c_bus = RT_NULL;
    }
    xclk_stop();
    s_device.open = RT_FALSE;
    return RT_EOK;
}

static int bf30a2_set_pixformat(pixformat_t pixformat)
{
    return pixformat == PIXFORMAT_RGB565 ? RT_EOK : -RT_EINVAL;
}

static int bf30a2_set_framesize(framesize_t framesize)
{
    return framesize == FRAMESIZE_240X320 ? RT_EOK : -RT_EINVAL;
}

static rt_size_t bf30a2_capture(void *buffer, rt_size_t buffer_size)
{
    rt_int32_t timeout = (rt_int32_t)
        ((BF30A2_FRAME_TIMEOUT_MS * RT_TICK_PER_SECOND + 999U) / 1000U);
    rt_size_t frame_size = 0;

    if (s_device.lock == RT_NULL)
    {
        return 0;
    }
    rt_mutex_take(s_device.lock, RT_WAITING_FOREVER);
    if (start_capture_locked((uint8_t *)buffer, buffer_size) == RT_EOK)
    {
        rt_mutex_release(s_device.lock);
        if (rt_sem_take(&s_device.frame_sem, timeout) == RT_EOK)
        {
            frame_size = s_device.frame_size;
        }
        rt_mutex_take(s_device.lock, RT_WAITING_FOREVER);
        stop_capture_locked();
    }
    rt_mutex_release(s_device.lock);
    return frame_size;
}

static int bf30a2_capture_async(void *buffer,
                                rt_size_t buffer_size,
                                camera_capture_done_callback_t callback,
                                void *context)
{
    int ret;

    if (callback == RT_NULL || s_device.lock == RT_NULL)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(s_device.lock, RT_WAITING_FOREVER);
    s_device.async_callback = callback;
    s_device.async_context = context;
    s_device.async_mode = RT_TRUE;
    ret = start_capture_locked((uint8_t *)buffer, buffer_size);
    if (ret != RT_EOK)
    {
        s_device.async_callback = RT_NULL;
        s_device.async_context = RT_NULL;
        s_device.async_mode = RT_FALSE;
    }
    rt_mutex_release(s_device.lock);
    return ret;
}

static int bf30a2_start_stream(const camera_stream_start_args_t *args)
{
    int ret;

    if (args == RT_NULL || args->mode != CAMERA_STREAM_MODE_FRAME ||
        args->buffers[0] == RT_NULL ||
        args->buffers[1] == RT_NULL || args->buffer_size < BF30A2_FRAME_SIZE ||
        s_device.lock == RT_NULL)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(s_device.lock, RT_WAITING_FOREVER);
    s_device.stream_buffers[0] = (uint8_t *)args->buffers[0];
    s_device.stream_buffers[1] = (uint8_t *)args->buffers[1];
    s_device.stream_buffer_size = args->buffer_size;
    s_device.stream_buffer_index = 0;
    s_device.stream_callback = args->frame_callback;
    s_device.stream_context = args->callback_context;
    ret = start_capture_locked(s_device.stream_buffers[0], args->buffer_size);
    rt_mutex_release(s_device.lock);
    return ret;
}

static int bf30a2_stop_stream(void)
{
    if (s_device.lock == RT_NULL)
    {
        return RT_EOK;
    }
    rt_mutex_take(s_device.lock, RT_WAITING_FOREVER);
    stop_capture_locked();
    s_device.stream_callback = RT_NULL;
    s_device.stream_context = RT_NULL;
    s_device.stream_buffers[0] = RT_NULL;
    s_device.stream_buffers[1] = RT_NULL;
    rt_mutex_release(s_device.lock);
    return RT_EOK;
}
