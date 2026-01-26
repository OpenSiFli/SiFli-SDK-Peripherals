/**
 * @file    drv_bf30a2.c
 * @brief   BF30A2 SPI Camera RT-Thread Device Driver Implementation
 *
 * This file implements the RT-Thread standard device driver for the
 * BF30A2 SPI camera sensor. It provides:
 * - Standard device interface (init, open, close, read, control)
 * - Frame capture with DMA
 * - YUV to RGB565 color conversion
 * - Statistics and FPS calculation
 * - UART export functionality
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2024
 */

/*============================================================================*/
/*                              INCLUDES                                      */
/*============================================================================*/

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>

#include "drv_bf30a2.h"
#include "bf0_hal.h"
#include "drv_spi.h"

#define DBG_TAG "drv.bf30a2"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/*============================================================================*/
/*                         CONFIGURATION DEFAULTS                             */
/*============================================================================*/

/* Default SPI Configuration */
#ifndef BF30A2_SPI_BUS
#define BF30A2_SPI_BUS              "spi2"
#endif

#ifndef BF30A2_SPI_DEV
#define BF30A2_SPI_DEV              "camera"
#endif

/* Default I2C Configuration */
#ifndef BF30A2_I2C_BUS
#define BF30A2_I2C_BUS              "i2c2"
#endif

#ifndef BF30A2_I2C_ADDR
#define BF30A2_I2C_ADDR             0x6E
#endif

/* Default PWM Configuration */
#ifndef BF30A2_PWM_DEV
#define BF30A2_PWM_DEV              "pwm2"
#endif

#ifndef BF30A2_PWM_CHANNEL
#define BF30A2_PWM_CHANNEL          2
#endif

/* Default Pin Configuration */
#ifndef BF30A2_SPI_CLK_PAD
#define BF30A2_SPI_CLK_PAD          PAD_PA39
#endif

#ifndef BF30A2_SPI_DIO_PAD
#define BF30A2_SPI_DIO_PAD          PAD_PA37
#endif

#ifndef BF30A2_SPI_CS_PAD
#define BF30A2_SPI_CS_PAD           PAD_PA40
#endif

#ifndef BF30A2_I2C_SCL_PAD
#define BF30A2_I2C_SCL_PAD          PAD_PA41
#endif

#ifndef BF30A2_I2C_SDA_PAD
#define BF30A2_I2C_SDA_PAD          PAD_PA42
#endif

#ifndef BF30A2_PWDN_PIN
#define BF30A2_PWDN_PIN             43
#endif

#ifndef BF30A2_PWM_PAD
#define BF30A2_PWM_PAD              PAD_PA20
#endif

/* Image Parameters */
#define IMG_WIDTH                   BF30A2_DEFAULT_WIDTH
#define IMG_HEIGHT                  BF30A2_DEFAULT_HEIGHT
#define BYTES_PER_LINE              (IMG_WIDTH * 2)

/* Protocol Frame Sizes */
#define FRAME_HEADER_SIZE           9
#define LINE_HEADER_SIZE            6
#define DATA_HEADER_SIZE            6
#define ONE_LINE_TOTAL              (LINE_HEADER_SIZE + DATA_HEADER_SIZE + BYTES_PER_LINE)
#define ONE_FRAME_SIZE              (IMG_WIDTH * IMG_HEIGHT * 2)

/* DMA Configuration */
#define DMA_BUFFER_SIZE             (ONE_LINE_TOTAL * 16)

/*============================================================================*/
/*                          EXTERNAL DECLARATIONS                             */
/*============================================================================*/

extern int  camera_start_dma(SPI_HandleTypeDef *hspi, uint8_t *buffer, uint32_t size);
extern void camera_stop_dma(SPI_HandleTypeDef *hspi);

/*============================================================================*/
/*                            TYPE DEFINITIONS                                */
/*============================================================================*/

/**
 * @brief Parse state machine states
 */
typedef enum
{
    STATE_FIND_SYNC,
    STATE_GET_TYPE,
    STATE_FRAME_FORMAT,
    STATE_FRAME_WIDTH_H,
    STATE_FRAME_WIDTH_L,
    STATE_FRAME_HEIGHT_H,
    STATE_FRAME_HEIGHT_L,
    STATE_LINE_NUM_H,
    STATE_LINE_NUM_L,
    STATE_DATA_SYNC_1,
    STATE_DATA_SYNC_2,
    STATE_DATA_SYNC_3,
    STATE_DATA_TYPE,
    STATE_DATA_SIZE_H,
    STATE_DATA_SIZE_L,
    STATE_PIXEL_DATA,
} parse_state_t;

/**
 * @brief BF30A2 camera device structure (extends rt_device)
 */
typedef struct bf30a2_device
{
    struct rt_device parent;            /**< RT-Thread device base class */

    /* Hardware configuration */
    bf30a2_hw_cfg_t hw_cfg;             /**< Hardware configuration */

    /* Device handles */
    struct rt_spi_device *spi_dev;      /**< SPI device handle */
    SPI_HandleTypeDef *hspi;            /**< Low-level SPI handle */
    struct rt_i2c_bus_device *i2c_bus;  /**< I2C bus handle */
    rt_device_t pwm_device;             /**< PWM device handle */
    rt_device_t gpio_device;            /**< GPIO device handle */

    /* DMA buffers */
    rt_uint8_t *dma_buf;                /**< DMA receive buffer */
    rt_uint32_t dma_size;               /**< DMA buffer size */

    /* Parse state machine */
    parse_state_t state;                /**< Current parse state */
    rt_uint8_t ff_count;                /**< 0xFF byte count */
    rt_uint16_t frame_width;            /**< Detected frame width */
    rt_uint16_t frame_height;           /**< Detected frame height */
    rt_uint16_t line_num;               /**< Current line number */
    rt_uint16_t data_size;              /**< Data size for current line */
    rt_uint16_t data_pos;               /**< Position in line data */

    /* Frame buffers */
    rt_uint8_t *frame_rgb565;           /**< RGB565 frame buffer */
    rt_uint8_t line_yuv[BYTES_PER_LINE];/**< YUV line buffer */
    rt_uint16_t lines_received;         /**< Lines received in current frame */
    rt_uint16_t max_line_seen;          /**< Maximum line number seen */
    rt_uint8_t frame_ready;             /**< Frame ready flag */
    rt_uint8_t in_frame;                /**< Currently receiving frame flag */

    /* Statistics */
    rt_uint32_t frame_count;            /**< Total frame count */
    rt_uint32_t complete_frames;        /**< Complete frames count */
    rt_uint32_t frame_start_count;      /**< Frame start count */
    rt_uint32_t frame_end_count;        /**< Frame end count */
    rt_uint32_t line_count;             /**< Total line count */
    rt_uint32_t errors;                 /**< Error count */
    rt_uint32_t rx_count;               /**< DMA receive count */
    rt_uint32_t total_bytes;            /**< Total bytes received */
    rt_uint32_t last_time;              /**< Last FPS calculation time */
    rt_uint32_t last_frames;            /**< Last frame count */
    float fps;                          /**< Current FPS */

    /* Thread management */
    rt_thread_t thread;                 /**< Processing thread */
    rt_event_t event;                   /**< Synchronization event */
    volatile rt_uint8_t running;        /**< Running flag */
    volatile rt_uint8_t stop_flag;      /**< Stop request flag */

    /* Callback */
    bf30a2_frame_callback_t callback;   /**< Frame callback function */
    void *user_data;                    /**< User callback context */

    /* Device info */
    rt_uint16_t chip_id;                /**< Sensor chip ID */
    rt_uint8_t hw_initialized;          /**< Hardware initialized flag */
    rt_uint8_t opened;                  /**< Device opened flag */

    /* Mutex for thread safety */
    rt_mutex_t lock;                    /**< Device lock mutex */
} bf30a2_device_t;

/*============================================================================*/
/*                     REGISTER CONFIGURATION                                 */
/*============================================================================*/

/**
 * @brief Register address-value pair structure
 */
typedef struct
{
    rt_uint16_t reg;
    rt_uint8_t val;
} bf30a2_reg_t;

#define REGLIST_TAIL    0xFFFE
#define REG_DLY         0xFFFF

/**
 * @brief BF30A2 SPI initialization register sequence
 */
static const bf30a2_reg_t bf30a2_init_regs[] =
{
    {0xf2, 0x01},
    {0xcf, 0xf0},
    {0x12, 0x40},
    {0x15, 0x80},
    {0x6b, 0x71},
    {0x00, 0x40},
    {0x04, 0x00},
    {0x06, 0x26},
    {0x08, 0x07},
    {0x1c, 0x12},
    {0x20, 0x20},
    {0x21, 0x20},
    {0x34, 0x02},
    {0x35, 0x02},
    {0x36, 0x21},
    {0x37, 0x13},
    {0xca, 0x23},
    {0xcb, 0x22},
    {0xcc, 0x89},
    {0xcd, 0x4c},
    {0xce, 0x6b},
    {0xa0, 0x8e},
    {0x01, 0x1b},
    {0x02, 0x1d},
    {0x13, 0x08},
    {0x87, 0x13},
    {0x8b, 0x08},
    {0x70, 0x17},
    {0x71, 0x43},
    {0x72, 0x0a},
    {0x73, 0x62},
    {0x74, 0xa2},
    {0x75, 0xbf},
    {0x76, 0x00},
    {0x77, 0xcc},
    {0x40, 0x32},
    {0x41, 0x28},
    {0x42, 0x26},
    {0x43, 0x1d},
    {0x44, 0x1a},
    {0x45, 0x14},
    {0x46, 0x11},
    {0x47, 0x0f},
    {0x48, 0x0e},
    {0x49, 0x0d},
    {0x4B, 0x0c},
    {0x4C, 0x0b},
    {0x4E, 0x0a},
    {0x4F, 0x09},
    {0x50, 0x09},
    {0x24, 0x30},
    {0x25, 0x36},
    {0x80, 0x00},
    {0x81, 0x20},
    {0x82, 0x40},
    {0x83, 0x30},
    {0x84, 0x50},
    {0x85, 0x30},
    {0x86, 0xd8},
    {0x89, 0x45},
    {0x8a, 0x33},
    {0x8f, 0x81},
    {0x91, 0xff},
    {0x92, 0x08},
    {0x94, 0x82},
    {0x95, 0xfd},
    {0x9a, 0x20},
    {0x9e, 0xbc},
    {0xf0, 0x8f},
    {0x51, 0x06},
    {0x52, 0x25},
    {0x53, 0x2b},
    {0x54, 0x0f},
    {0x57, 0x2a},
    {0x58, 0x22},
    {0x59, 0x2c},
    {0x23, 0x33},
    {0xa1, 0x93},
    {0xa2, 0x0f},
    {0xa3, 0x2a},
    {0xa4, 0x08},
    {0xa5, 0x26},
    {0xa7, 0x80},
    {0xa8, 0x80},
    {0xa9, 0x1e},
    {0xaa, 0x19},
    {0xab, 0x18},
    {0xae, 0x50},
    {0xaf, 0x04},
    {0xc8, 0x10},
    {0xc9, 0x15},
    {0xd3, 0x0c},
    {0xd4, 0x16},
    {0xee, 0x06},
    {0xef, 0x04},
    {0x55, 0x34},
    {0x56, 0x9c},
    {0xb1, 0x98},
    {0xb2, 0x98},
    {0xb3, 0xc4},
    {0xb4, 0x0c},
    {0xa0, 0x8f},
    {0x13, 0x07},
    {REGLIST_TAIL, 0x00}
};

/*============================================================================*/
/*                          STATIC VARIABLES                                  */
/*============================================================================*/

static bf30a2_device_t *g_bf30a2_dev = RT_NULL;

/*============================================================================*/
/*                     FORWARD DECLARATIONS                                   */
/*============================================================================*/

static rt_err_t bf30a2_dev_init(rt_device_t dev);
static rt_err_t bf30a2_dev_open(rt_device_t dev, rt_uint16_t oflag);
static rt_err_t bf30a2_dev_close(rt_device_t dev);
static rt_size_t bf30a2_dev_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size);
static rt_err_t bf30a2_dev_control(rt_device_t dev, int cmd, void *args);

/*============================================================================*/
/*                     DMA CALLBACK                                           */
/*============================================================================*/

/**
 * @brief DMA receive indication callback
 * @param p Pointer to received data (unused)
 */
void camera_rx_ind(rt_uint8_t *p)
{
    if ((g_bf30a2_dev == RT_NULL) || (!g_bf30a2_dev->running))
    {
        return;
    }

    g_bf30a2_dev->rx_count++;
    g_bf30a2_dev->total_bytes += DMA_BUFFER_SIZE / 2;

    if (g_bf30a2_dev->event != RT_NULL)
    {
        rt_event_send(g_bf30a2_dev->event, 0x01);
    }
}

/*============================================================================*/
/*                     COLOR CONVERSION                                       */
/*============================================================================*/

/**
 * @brief Clamp integer value to 8-bit range
 */
static inline rt_uint8_t clamp8(int v)
{
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (rt_uint8_t)v;
}

/**
 * @brief Convert YUV422 line to RGB565 format
 */
static void yuv_line_to_rgb565(rt_uint8_t *yuv, rt_uint8_t *rgb, int width)
{
    int x;
    int y0, cb, y1, cr;
    int cb_off, cr_off;
    int r0, g0, b0, r1, g1, b1;
    rt_uint16_t p0, p1;

    for (x = 0; x < width; x += 2)
    {
        y0 = yuv[0];
        cb = yuv[1];
        y1 = yuv[2];
        cr = yuv[3];
        yuv += 4;

        cb_off = cb - 128;
        cr_off = cr - 128;

        r0 = clamp8(y0 + ((359 * cr_off) >> 8));
        g0 = clamp8(y0 - ((88 * cb_off + 183 * cr_off) >> 8));
        b0 = clamp8(y0 + ((454 * cb_off) >> 8));

        r1 = clamp8(y1 + ((359 * cr_off) >> 8));
        g1 = clamp8(y1 - ((88 * cb_off + 183 * cr_off) >> 8));
        b1 = clamp8(y1 + ((454 * cb_off) >> 8));

        p0 = ((r0 & 0xF8) << 8) | ((g0 & 0xFC) << 3) | (b0 >> 3);
        p1 = ((r1 & 0xF8) << 8) | ((g1 & 0xFC) << 3) | (b1 >> 3);

        *rgb++ = p0 & 0xFF;
        *rgb++ = p0 >> 8;
        *rgb++ = p1 & 0xFF;
        *rgb++ = p1 >> 8;
    }
}

/*============================================================================*/
/*                     PARSE STATE MACHINE                                    */
/*============================================================================*/

static void reset_parse(bf30a2_device_t *dev)
{
    dev->state = STATE_FIND_SYNC;
    dev->ff_count = 0;
    dev->lines_received = 0;
    dev->max_line_seen = 0;
    dev->data_pos = 0;
    dev->in_frame = 0;
    dev->frame_ready = 0;  /* 重要：重置frame_ready标志，确保重新启动时状态正确 */
}

static void on_frame_start(bf30a2_device_t *dev)
{
    dev->frame_start_count++;
    dev->in_frame = 1;
    dev->lines_received = 0;
    dev->max_line_seen = 0;
}

static void on_frame_end(bf30a2_device_t *dev)
{
    dev->frame_end_count++;

    if (dev->in_frame && (dev->lines_received >= (IMG_HEIGHT * 8 / 10)))
    {
        dev->frame_ready = 1;
        dev->complete_frames++;

        if (dev->callback != RT_NULL)
        {
            dev->callback(&dev->parent, dev->frame_count,
                         dev->frame_rgb565, ONE_FRAME_SIZE, dev->user_data);
        }
        dev->frame_count++;
    }

    dev->in_frame = 0;
}

static void on_line_complete(bf30a2_device_t *dev)
{
    rt_uint16_t line = dev->line_num;

    dev->line_count++;

    if ((line < IMG_HEIGHT) && (dev->frame_rgb565 != RT_NULL))
    {
        yuv_line_to_rgb565(dev->line_yuv,
                          dev->frame_rgb565 + (line * BYTES_PER_LINE),
                          IMG_WIDTH);
        dev->lines_received++;
        if (line > dev->max_line_seen)
        {
            dev->max_line_seen = line;
        }
    }
    else
    {
        dev->errors++;
    }
}

static void parse_byte(bf30a2_device_t *dev, rt_uint8_t b)
{
    switch (dev->state)
    {
    case STATE_FIND_SYNC:
        if (b == 0xFF)
        {
            dev->ff_count++;
            if (dev->ff_count >= 3)
            {
                dev->state = STATE_GET_TYPE;
                dev->ff_count = 0;
            }
        }
        else
        {
            dev->ff_count = 0;
        }
        break;

    case STATE_GET_TYPE:
        switch (b)
        {
        case 0x01:
            dev->state = STATE_FRAME_FORMAT;
            break;
        case 0x02:
            dev->state = STATE_LINE_NUM_H;
            break;
        case 0x00:
            on_frame_end(dev);
            dev->state = STATE_FIND_SYNC;
            break;
        case 0xFF:
            dev->ff_count = 1;
            break;
        default:
            dev->state = STATE_FIND_SYNC;
            dev->ff_count = 0;
            break;
        }
        break;

    case STATE_FRAME_FORMAT:
        if (b == 0x00)
        {
            dev->state = STATE_FRAME_WIDTH_H;
        }
        else
        {
            dev->state = STATE_FIND_SYNC;
            dev->ff_count = 0;
        }
        break;

    case STATE_FRAME_WIDTH_H:
        dev->frame_width = (rt_uint16_t)b << 8;
        dev->state = STATE_FRAME_WIDTH_L;
        break;

    case STATE_FRAME_WIDTH_L:
        dev->frame_width |= b;
        dev->state = STATE_FRAME_HEIGHT_H;
        break;

    case STATE_FRAME_HEIGHT_H:
        dev->frame_height = (rt_uint16_t)b << 8;
        dev->state = STATE_FRAME_HEIGHT_L;
        break;

    case STATE_FRAME_HEIGHT_L:
        dev->frame_height |= b;
        if ((dev->frame_width == IMG_WIDTH) && (dev->frame_height == IMG_HEIGHT))
        {
            on_frame_start(dev);
        }
        else
        {
            dev->errors++;
        }
        dev->state = STATE_FIND_SYNC;
        dev->ff_count = 0;
        break;

    case STATE_LINE_NUM_H:
        dev->line_num = (rt_uint16_t)b << 8;
        dev->state = STATE_LINE_NUM_L;
        break;

    case STATE_LINE_NUM_L:
        dev->line_num |= b;
        dev->state = STATE_DATA_SYNC_1;
        dev->ff_count = 0;
        break;

    case STATE_DATA_SYNC_1:
        dev->state = (b == 0xFF) ? STATE_DATA_SYNC_2 : STATE_FIND_SYNC;
        if (b != 0xFF) dev->ff_count = 0;
        break;

    case STATE_DATA_SYNC_2:
        dev->state = (b == 0xFF) ? STATE_DATA_SYNC_3 : STATE_FIND_SYNC;
        if (b != 0xFF) dev->ff_count = 0;
        break;

    case STATE_DATA_SYNC_3:
        dev->state = (b == 0xFF) ? STATE_DATA_TYPE : STATE_FIND_SYNC;
        if (b != 0xFF) dev->ff_count = 0;
        break;

    case STATE_DATA_TYPE:
        if (b == 0x40)
        {
            dev->state = STATE_DATA_SIZE_H;
        }
        else
        {
            dev->state = STATE_FIND_SYNC;
            dev->ff_count = 0;
        }
        break;

    case STATE_DATA_SIZE_H:
        dev->data_size = (rt_uint16_t)b << 8;
        dev->state = STATE_DATA_SIZE_L;
        break;

    case STATE_DATA_SIZE_L:
        dev->data_size |= b;
        if (dev->data_size == BYTES_PER_LINE)
        {
            dev->data_pos = 0;
            dev->state = STATE_PIXEL_DATA;
        }
        else
        {
            dev->errors++;
            dev->state = STATE_FIND_SYNC;
            dev->ff_count = 0;
        }
        break;

    case STATE_PIXEL_DATA:
        dev->line_yuv[dev->data_pos++] = b;
        if (dev->data_pos >= BYTES_PER_LINE)
        {
            on_line_complete(dev);
            dev->state = STATE_FIND_SYNC;
            dev->ff_count = 0;
        }
        break;

    default:
        dev->state = STATE_FIND_SYNC;
        dev->ff_count = 0;
        break;
    }
}

/*============================================================================*/
/*                     CAMERA THREAD                                          */
/*============================================================================*/

static void cam_thread_entry(void *arg)
{
    bf30a2_device_t *dev = (bf30a2_device_t *)arg;
    rt_uint32_t evt;
    rt_uint32_t last_pos;
    rt_uint32_t dma_pos;
    rt_uint32_t remain;
    rt_uint32_t now;

    LOG_I("Camera thread started");

    /* 关键修复：启动时同步到当前DMA位置，跳过可能的旧数据 */
    last_pos = 0;
    if ((dev->hspi != RT_NULL) && (dev->hspi->hdmarx != RT_NULL))
    {
        remain = dev->hspi->hdmarx->Instance->CNDTR;
        last_pos = dev->dma_size - remain;
        if (last_pos >= dev->dma_size)
        {
            last_pos = 0;
        }
    }
    LOG_D("Thread sync: last_pos=%d, dma_size=%d", last_pos, dev->dma_size);

    while (!dev->stop_flag)
    {
        rt_event_recv(dev->event, 0x01,
                     RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 50, &evt);

        if (dev->stop_flag)
        {
            break;
        }

        /* Calculate current DMA position */
        dma_pos = 0;
        if ((dev->hspi != RT_NULL) && (dev->hspi->hdmarx != RT_NULL))
        {
            remain = dev->hspi->hdmarx->Instance->CNDTR;
            dma_pos = dev->dma_size - remain;
            if (dma_pos >= dev->dma_size)
            {
                dma_pos = 0;
            }
        }

        /* Process received bytes */
        while (last_pos != dma_pos)
        {
            parse_byte(dev, dev->dma_buf[last_pos]);
            last_pos++;
            if (last_pos >= dev->dma_size)
            {
                last_pos = 0;
            }
        }

        /* Calculate FPS every second */
        now = rt_tick_get_millisecond();
        if ((now - dev->last_time) >= 1000)
        {
            dev->fps = (dev->complete_frames - dev->last_frames) * 1000.0f /
                      (now - dev->last_time);
            dev->last_time = now;
            dev->last_frames = dev->complete_frames;
        }
    }

    LOG_I("Camera thread exited");
}

/*============================================================================*/
/*                     I2C OPERATIONS                                         */
/*============================================================================*/

static rt_err_t bf30a2_i2c_write_reg(bf30a2_device_t *dev, rt_uint8_t reg, rt_uint8_t val)
{
    struct rt_i2c_msg msg;
    rt_uint8_t buf[2] = {reg, val};

    msg.addr = dev->hw_cfg.i2c_addr;
    msg.flags = RT_I2C_WR;
    msg.buf = buf;
    msg.len = 2;

    if (rt_i2c_transfer(dev->i2c_bus, &msg, 1) != 1)
    {
        LOG_E("I2C write failed: reg=0x%02X", reg);
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t bf30a2_i2c_read_reg(bf30a2_device_t *dev, rt_uint8_t reg, rt_uint8_t *val)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = dev->hw_cfg.i2c_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = dev->hw_cfg.i2c_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = val;
    msgs[1].len = 1;

    if (rt_i2c_transfer(dev->i2c_bus, msgs, 2) != 2)
    {
        LOG_E("I2C read failed: reg=0x%02X", reg);
        return -RT_ERROR;
    }

    return RT_EOK;
}

/*============================================================================*/
/*                     GPIO/PWM OPERATIONS                                    */
/*============================================================================*/

static rt_err_t bf30a2_gpio_init(bf30a2_device_t *dev)
{
    struct rt_device_pin_mode mode;

    /* Use macro directly: PWDN pin is PA43 */
    HAL_PIN_Set(PAD_PA00 + 43, GPIO_A0 + 43, PIN_NOPULL, 1);

    dev->gpio_device = rt_device_find("pin");
    if (!dev->gpio_device)
    {
        LOG_E("GPIO device not found");
        return -RT_ERROR;
    }

    rt_device_open(dev->gpio_device, RT_DEVICE_OFLAG_RDWR);

    mode.pin = dev->hw_cfg.pins.pwdn_pin;
    mode.mode = PIN_MODE_OUTPUT;
    rt_device_control(dev->gpio_device, 0, &mode);

    return RT_EOK;
}

static void bf30a2_pwdn_set(bf30a2_device_t *dev, rt_uint8_t level)
{
    struct rt_device_pin_status st;

    st.pin = dev->hw_cfg.pins.pwdn_pin;
    st.status = level;
    rt_device_write(dev->gpio_device, 0, &st, sizeof(st));
}

static void bf30a2_pwm_config_24mhz(struct rt_device_pwm *device, rt_uint8_t channel)
{
    struct bf0_pwm
    {
        struct rt_device_pwm pwm_device;
        GPT_HandleTypeDef tim_handle;
        rt_uint8_t channel;
        char *name;
        void *pwm_cc_dma[4];
        void *pwm_update_dma;
    };

    struct bf0_pwm *pwm = (struct bf0_pwm *)device->parent.user_data;
    GPT_HandleTypeDef *htim = &(pwm->tim_handle);
    GPT_OC_InitTypeDef oc_config = {0};

    rt_uint32_t pclk = (htim->Instance == hwp_gptim2) ?
                       24000000 : HAL_RCC_GetPCLKFreq(htim->core, 1);
    rt_uint32_t ch = 0x04 * (channel - 1);
    rt_uint32_t divider = pclk / 24000000;
    rt_uint32_t prescaler = 0;
    rt_uint32_t period, pulse;

    if (divider == 0) divider = 1;

    if (divider % 2 == 0)
    {
        period = divider - 1;
        pulse = (divider / 2) - 1;
    }
    else
    {
        period = divider - 1;
        pulse = divider / 2;
    }

    if (period > 65535)
    {
        prescaler = (period / 65535) + 1;
        period = (divider / (prescaler + 1)) - 1;
        pulse = ((divider / (prescaler + 1)) / 2) - 1;
    }

    HAL_GPT_PWM_Stop(htim, ch);
    __HAL_GPT_SET_PRESCALER(htim, prescaler);
    __HAL_GPT_SET_AUTORELOAD(htim, period);
    __HAL_GPT_SET_COMPARE(htim, ch, pulse);

    oc_config.OCMode = GPT_OCMODE_PWM1;
    oc_config.Pulse = pulse;
    oc_config.OCPolarity = GPT_OCPOLARITY_HIGH;
    oc_config.OCFastMode = GPT_OCFAST_DISABLE;

    HAL_GPT_PWM_ConfigChannel(htim, &oc_config, ch);
    HAL_GPT_GenerateEvent(htim, GPT_EVENTSOURCE_UPDATE);
    HAL_GPT_PWM_Start(htim, ch);
}

static rt_err_t bf30a2_pwm_init(bf30a2_device_t *dev)
{
    /* Use macro directly: PWM pin is PA20 */
    HAL_PIN_Set(PAD_PA20, GPTIM1_CH2, PIN_NOPULL, 1);

    dev->pwm_device = rt_device_find(dev->hw_cfg.pwm_dev_name);
    if (!dev->pwm_device)
    {
        LOG_E("PWM device '%s' not found", dev->hw_cfg.pwm_dev_name);
        return -RT_ERROR;
    }

    if (rt_device_open(dev->pwm_device, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        LOG_E("PWM open failed");
        return -RT_ERROR;
    }

    rt_pwm_set((struct rt_device_pwm *)dev->pwm_device,
               dev->hw_cfg.pwm_channel, 1000000, 500000);
    rt_pwm_enable((struct rt_device_pwm *)dev->pwm_device, dev->hw_cfg.pwm_channel);
    rt_thread_mdelay(10);
    bf30a2_pwm_config_24mhz((struct rt_device_pwm *)dev->pwm_device,
                            dev->hw_cfg.pwm_channel);

    return RT_EOK;
}

/*============================================================================*/
/*                     I2C/SPI INITIALIZATION                                 */
/*============================================================================*/

static rt_err_t bf30a2_i2c_init(bf30a2_device_t *dev)
{
    struct rt_i2c_configuration cfg =
    {
        .mode = 0,
        .addr = 0,
        .timeout = 1000,
        .max_hz = 100000
    };

    /* Use macro directly to avoid type truncation issues */
    HAL_PIN_Set(PAD_PA41, I2C2_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA42, I2C2_SDA, PIN_PULLUP, 1);

    dev->i2c_bus = rt_i2c_bus_device_find(dev->hw_cfg.i2c_bus_name);
    if (!dev->i2c_bus)
    {
        LOG_E("I2C bus '%s' not found", dev->hw_cfg.i2c_bus_name);
        return -RT_ERROR;
    }

    if (rt_device_open((rt_device_t)dev->i2c_bus, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        LOG_E("I2C open failed");
        return -RT_ERROR;
    }

    rt_i2c_configure(dev->i2c_bus, &cfg);

    return RT_EOK;
}

static rt_err_t bf30a2_spi_init(bf30a2_device_t *dev)
{
    rt_err_t ret;
    struct rt_spi_configuration cfg = {0};
    struct sifli_spi *drv;

    /* Use macro directly: SPI pins are PA39(CLK), PA37(DIO), PA40(CS) */
    HAL_PIN_Set(PAD_PA39, SPI2_CLK, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA37, SPI2_DIO, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA40, SPI2_CS, PIN_PULLDOWN, 1);

    dev->spi_dev = (struct rt_spi_device *)rt_device_find(BF30A2_SPI_DEV);
    if (dev->spi_dev == RT_NULL)
    {
        ret = rt_hw_spi_device_attach(dev->hw_cfg.spi_bus_name, BF30A2_SPI_DEV);
        if (ret != RT_EOK)
        {
            return ret;
        }
        dev->spi_dev = (struct rt_spi_device *)rt_device_find(BF30A2_SPI_DEV);
    }

    if (dev->spi_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    ret = rt_device_open(&dev->spi_dev->parent,
                        RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_DMA_RX);
    if (ret != RT_EOK)
    {
        return ret;
    }

    cfg.data_width = 8;
    cfg.max_hz = 24000000;
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_SLAVE | RT_SPI_3WIRE;

    ret = rt_spi_configure(dev->spi_dev, &cfg);
    if (ret != RT_EOK)
    {
        return ret;
    }

    ret = rt_spi_take_bus(dev->spi_dev);
    if (ret != RT_EOK)
    {
        return ret;
    }

    drv = rt_container_of(dev->spi_dev->bus, struct sifli_spi, spi_bus);
    dev->hspi = &drv->handle;

    return RT_EOK;
}

/*============================================================================*/
/*                     SENSOR CONFIGURATION                                   */
/*============================================================================*/

static rt_err_t bf30a2_sensor_check_id(bf30a2_device_t *dev)
{
    rt_uint8_t id_h, id_l;

    if (bf30a2_i2c_read_reg(dev, 0xFC, &id_h) != RT_EOK)
    {
        return -RT_ERROR;
    }
    if (bf30a2_i2c_read_reg(dev, 0xFD, &id_l) != RT_EOK)
    {
        return -RT_ERROR;
    }

    dev->chip_id = ((rt_uint16_t)id_h << 8) | id_l;
    LOG_I("Chip ID: 0x%04X", dev->chip_id);

    if (id_h != 0x3B || id_l != 0x02)
    {
        LOG_E("Unexpected Chip ID");
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t bf30a2_sensor_load_config(bf30a2_device_t *dev)
{
    rt_uint8_t pda_val;
    int i;

    for (i = 0; bf30a2_init_regs[i].reg != REGLIST_TAIL; i++)
    {
        if (bf30a2_init_regs[i].reg == REG_DLY)
        {
            rt_thread_mdelay(bf30a2_init_regs[i].val);
        }
        else
        {
            if (bf30a2_i2c_write_reg(dev, bf30a2_init_regs[i].reg,
                                     bf30a2_init_regs[i].val) != RT_EOK)
            {
                return -RT_ERROR;
            }
            if (bf30a2_init_regs[i].reg == 0xF2)
            {
                rt_thread_mdelay(10);
            }
        }
    }

    bf30a2_i2c_read_reg(dev, 0xCF, &pda_val);
    if (pda_val & 0x01)
    {
        bf30a2_i2c_write_reg(dev, 0xCF, 0xB0);
        rt_thread_mdelay(10);
    }

    return RT_EOK;
}

/*============================================================================*/
/*                     UART EXPORT                                            */
/*============================================================================*/

static void bf30a2_export_uart(bf30a2_device_t *dev)
{
    rt_uint32_t i;
    rt_uint8_t *data;

    if (!dev->frame_rgb565 || !dev->frame_ready)
    {
        LOG_E("No frame data to export");
        return;
    }

    data = dev->frame_rgb565;

    LOG_I("========================================");
    LOG_I("Exporting frame via UART...");
    LOG_I("Format: RGB565, Size: %dx%d", IMG_WIDTH, IMG_HEIGHT);
    LOG_I("Total bytes: %d", ONE_FRAME_SIZE);
    LOG_I("========================================");

    rt_kprintf("\n===PHOTO_START===\n");
    rt_kprintf("WIDTH:%d\n", IMG_WIDTH);
    rt_kprintf("HEIGHT:%d\n", IMG_HEIGHT);
    rt_kprintf("FORMAT:RGB565\n");
    rt_kprintf("SIZE:%d\n", ONE_FRAME_SIZE);
    rt_kprintf("SOURCE:BF30A2\n");
    rt_kprintf("===DATA_BEGIN===\n");

    for (i = 0; i < ONE_FRAME_SIZE; i++)
    {
        rt_kprintf("%02X", data[i]);
        if ((i + 1) % 32 == 0)
        {
            rt_kprintf("\n");
            if ((i + 1) % 1024 == 0)
            {
                rt_thread_mdelay(5);
            }
        }
    }

    rt_kprintf("\n===DATA_END===\n");
    rt_kprintf("===PHOTO_END===\n\n");

    LOG_I("Export completed!");
}

/*============================================================================*/
/*                     RT-THREAD DEVICE OPERATIONS                            */
/*============================================================================*/

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops bf30a2_ops =
{
    .init    = bf30a2_dev_init,
    .open    = bf30a2_dev_open,
    .close   = bf30a2_dev_close,
    .read    = bf30a2_dev_read,
    .write   = RT_NULL,
    .control = bf30a2_dev_control,
};
#endif

/**
 * @brief Device init operation
 */
static rt_err_t bf30a2_dev_init(rt_device_t dev)
{
    bf30a2_device_t *cam = (bf30a2_device_t *)dev;

    if (cam->hw_initialized)
    {
        return RT_EOK;
    }

    LOG_I("BF30A2 device initializing...");

    /* Allocate DMA buffer */
    cam->dma_size = DMA_BUFFER_SIZE;
    cam->dma_buf = rt_malloc_align(cam->dma_size, 32);
    if (cam->dma_buf == RT_NULL)
    {
        LOG_E("Alloc DMA buffer failed (%d bytes)", cam->dma_size);
        return -RT_ENOMEM;
    }

    /* Allocate frame buffer */
    cam->frame_rgb565 = rt_malloc(ONE_FRAME_SIZE);
    if (cam->frame_rgb565 == RT_NULL)
    {
        LOG_E("Alloc frame buffer failed (%d bytes)", ONE_FRAME_SIZE);
        rt_free_align(cam->dma_buf);
        cam->dma_buf = RT_NULL;
        return -RT_ENOMEM;
    }

    /* Create event object */
    cam->event = rt_event_create("bf30a2", RT_IPC_FLAG_FIFO);
    if (cam->event == RT_NULL)
    {
        LOG_E("Create event failed");
        rt_free(cam->frame_rgb565);
        rt_free_align(cam->dma_buf);
        cam->frame_rgb565 = RT_NULL;
        cam->dma_buf = RT_NULL;
        return -RT_ENOMEM;
    }

    /* Create mutex */
    cam->lock = rt_mutex_create("bf30a2", RT_IPC_FLAG_PRIO);
    if (cam->lock == RT_NULL)
    {
        LOG_E("Create mutex failed");
        rt_event_delete(cam->event);
        rt_free(cam->frame_rgb565);
        rt_free_align(cam->dma_buf);
        cam->event = RT_NULL;
        cam->frame_rgb565 = RT_NULL;
        cam->dma_buf = RT_NULL;
        return -RT_ENOMEM;
    }

    LOG_I("BF30A2 init OK");
    LOG_I("  DMA buffer: %d bytes", cam->dma_size);
    LOG_I("  Frame buffer: %d bytes", ONE_FRAME_SIZE);

    cam->hw_initialized = 1;

    return RT_EOK;
}

/**
 * @brief Device open operation
 */
static rt_err_t bf30a2_dev_open(rt_device_t dev, rt_uint16_t oflag)
{
    bf30a2_device_t *cam = (bf30a2_device_t *)dev;
    rt_err_t ret;

    if (cam->opened)
    {
        return RT_EOK;
    }

    rt_mutex_take(cam->lock, RT_WAITING_FOREVER);

    LOG_I("Opening BF30A2 device...");

    /* Step 1: Initialize GPIO for PWDN pin */
    ret = bf30a2_gpio_init(cam);
    if (ret != RT_EOK)
    {
        LOG_E("GPIO init failed");
        rt_mutex_release(cam->lock);
        return ret;
    }

    /* Step 2: Initialize PWM for 24MHz MCLK */
    ret = bf30a2_pwm_init(cam);
    if (ret != RT_EOK)
    {
        LOG_E("PWM init failed");
        rt_mutex_release(cam->lock);
        return ret;
    }
    rt_thread_mdelay(10);

    /* Step 3: Power sequence - reset sensor */
    bf30a2_pwdn_set(cam, 1);
    rt_thread_mdelay(1);
    bf30a2_pwdn_set(cam, 0);
    rt_thread_mdelay(10);

    /* Step 4: Initialize I2C */
    ret = bf30a2_i2c_init(cam);
    if (ret != RT_EOK)
    {
        LOG_E("I2C init failed");
        rt_mutex_release(cam->lock);
        return ret;
    }

    /* Step 5: Check sensor chip ID */
    ret = bf30a2_sensor_check_id(cam);
    if (ret != RT_EOK)
    {
        LOG_E("Sensor check ID failed");
        rt_mutex_release(cam->lock);
        return ret;
    }

    /* Step 6: Load sensor configuration */
    ret = bf30a2_sensor_load_config(cam);
    if (ret != RT_EOK)
    {
        LOG_E("Sensor config failed");
        rt_mutex_release(cam->lock);
        return ret;
    }
    rt_thread_mdelay(100);

    /* Step 7: Initialize SPI for data capture */
    ret = bf30a2_spi_init(cam);
    if (ret != RT_EOK)
    {
        LOG_E("SPI init failed");
        rt_mutex_release(cam->lock);
        return ret;
    }

    cam->opened = 1;
    LOG_I("BF30A2 device opened");

    rt_mutex_release(cam->lock);

    return RT_EOK;
}

/**
 * @brief Device close operation
 */
static rt_err_t bf30a2_dev_close(rt_device_t dev)
{
    bf30a2_device_t *cam = (bf30a2_device_t *)dev;

    if (!cam->opened)
    {
        return RT_EOK;
    }

    rt_mutex_take(cam->lock, RT_WAITING_FOREVER);

    /* Stop capture if running */
    if (cam->running)
    {
        cam->stop_flag = 1;
        rt_event_send(cam->event, 0x01);
        rt_thread_mdelay(100);
        camera_stop_dma(cam->hspi);
        cam->running = 0;
    }

    /* Release SPI resources */
    if (cam->spi_dev != RT_NULL)
    {
        rt_spi_release(cam->spi_dev);
        rt_spi_release_bus(cam->spi_dev);
        rt_device_close(&cam->spi_dev->parent);
        cam->spi_dev = RT_NULL;
    }

    cam->opened = 0;
    LOG_I("BF30A2 device closed");

    rt_mutex_release(cam->lock);

    return RT_EOK;
}

/**
 * @brief Device read operation - read frame data
 */
static rt_size_t bf30a2_dev_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    bf30a2_device_t *cam = (bf30a2_device_t *)dev;
    rt_size_t copy_size;

    if ((cam->frame_rgb565 == RT_NULL) || (!cam->frame_ready) || (buffer == RT_NULL))
    {
        return 0;
    }

    rt_mutex_take(cam->lock, RT_WAITING_FOREVER);

    copy_size = (size < ONE_FRAME_SIZE) ? size : ONE_FRAME_SIZE;
    rt_memcpy(buffer, cam->frame_rgb565, copy_size);
    cam->frame_ready = 0;

    rt_mutex_release(cam->lock);

    return copy_size;
}

/**
 * @brief Device control operation
 */
static rt_err_t bf30a2_dev_control(rt_device_t dev, int cmd, void *args)
{
    bf30a2_device_t *cam = (bf30a2_device_t *)dev;
    rt_err_t ret = RT_EOK;

    switch (cmd)
    {
    case BF30A2_CMD_START:
    {
        if (cam->running)
        {
            return RT_EOK;
        }

        /* 确保旧线程已经完全清理 */
        if (cam->thread != RT_NULL)
        {
            LOG_W("Old thread exists, cleaning up...");
            cam->stop_flag = 1;
            rt_event_send(cam->event, 0x01);
            rt_thread_mdelay(150);
            cam->thread = RT_NULL;
        }

        rt_mutex_take(cam->lock, RT_WAITING_FOREVER);

        /* Initialize buffers and state */
        rt_memset(cam->dma_buf, 0xAA, cam->dma_size);
        reset_parse(cam);

        /* Reset statistics */
        cam->frame_count = 0;
        cam->complete_frames = 0;
        cam->frame_start_count = 0;
        cam->frame_end_count = 0;
        cam->line_count = 0;
        cam->errors = 0;
        cam->rx_count = 0;
        cam->total_bytes = 0;
        cam->last_time = rt_tick_get_millisecond();
        cam->last_frames = 0;
        cam->fps = 0;
        cam->stop_flag = 0;
        cam->frame_ready = 0;  /* 确保frame_ready在启动时被重置 */
        cam->running = 1;

        /* Start DMA reception */
        ret = camera_start_dma(cam->hspi, cam->dma_buf, cam->dma_size);
        if (ret != 0)
        {
            LOG_E("camera_start_dma failed: %d", ret);
            cam->running = 0;
            rt_mutex_release(cam->lock);
            return -RT_ERROR;
        }

        /* 短暂延时让DMA开始接收一些数据 */
        rt_thread_mdelay(10);

        /* Create camera processing thread */
        cam->thread = rt_thread_create("bf30a2", cam_thread_entry, cam,
                                       2048, RT_THREAD_PRIORITY_HIGH, 10);
        if (cam->thread != RT_NULL)
        {
            rt_thread_startup(cam->thread);
        }
        else
        {
            LOG_E("Failed to create camera thread");
            camera_stop_dma(cam->hspi);
            cam->running = 0;
            rt_mutex_release(cam->lock);
            return -RT_ENOMEM;
        }

        LOG_I("Capture started");
        rt_mutex_release(cam->lock);
        break;
    }

    case BF30A2_CMD_STOP:
    {
        if (!cam->running)
        {
            return RT_EOK;
        }

        /* 设置停止标志 */
        cam->stop_flag = 1;
        rt_event_send(cam->event, 0x01);
        
        /* 等待线程退出 - 在获取锁之前等待，避免死锁 */
        rt_thread_mdelay(150);
        
        rt_mutex_take(cam->lock, RT_WAITING_FOREVER);

        /* 停止DMA */
        camera_stop_dma(cam->hspi);
        cam->running = 0;
        
        /* 清理线程句柄 */
        cam->thread = RT_NULL;

        LOG_I("Stopped: %d complete frames, %d errors",
              cam->complete_frames, cam->errors);
        rt_mutex_release(cam->lock);
        break;
    }

    case BF30A2_CMD_GET_INFO:
    {
        bf30a2_info_t *info = (bf30a2_info_t *)args;
        if (info != RT_NULL)
        {
            info->width = IMG_WIDTH;
            info->height = IMG_HEIGHT;
            info->frame_size = ONE_FRAME_SIZE;
            info->format = BF30A2_FORMAT_RGB565;
            info->chip_id = cam->chip_id;
        }
        break;
    }

    case BF30A2_CMD_GET_STATUS:
    {
        bf30a2_status_info_t *status = (bf30a2_status_info_t *)args;
        if (status != RT_NULL)
        {
            status->state = cam->running ? BF30A2_STATUS_RUNNING : BF30A2_STATUS_IDLE;
            status->frame_count = cam->frame_count;
            status->complete_frames = cam->complete_frames;
            status->error_count = cam->errors;
            status->fps = cam->fps;
            status->frame_ready = cam->frame_ready;
        }
        break;
    }

    case BF30A2_CMD_GET_FPS:
    {
        float *fps = (float *)args;
        if (fps != RT_NULL)
        {
            *fps = cam->fps;
        }
        break;
    }

    case BF30A2_CMD_GET_FRAME_COUNT:
    {
        rt_uint32_t *count = (rt_uint32_t *)args;
        if (count != RT_NULL)
        {
            *count = cam->complete_frames;
        }
        break;
    }

    case BF30A2_CMD_SET_CALLBACK:
    {
        bf30a2_callback_cfg_t *cfg = (bf30a2_callback_cfg_t *)args;
        if (cfg != RT_NULL)
        {
            rt_mutex_take(cam->lock, RT_WAITING_FOREVER);
            cam->callback = cfg->callback;
            cam->user_data = cfg->user_data;
            rt_mutex_release(cam->lock);
        }
        break;
    }

    case BF30A2_CMD_GET_BUFFER:
    {
        bf30a2_buffer_t *buf = (bf30a2_buffer_t *)args;
        if (buf != RT_NULL)
        {
            buf->data = cam->frame_rgb565;
            buf->size = ONE_FRAME_SIZE;
            buf->frame_num = cam->frame_count;
            buf->timestamp = rt_tick_get();
        }
        break;
    }

    case BF30A2_CMD_WAIT_FRAME:
    {
        bf30a2_wait_cfg_t *cfg = (bf30a2_wait_cfg_t *)args;
        rt_uint32_t timeout = (cfg != RT_NULL) ? cfg->timeout_ms : 1000;
        rt_uint32_t start = rt_tick_get_millisecond();

        while (!cam->frame_ready)
        {
            rt_thread_mdelay(10);
            if ((rt_tick_get_millisecond() - start) > timeout)
            {
                return -RT_ETIMEOUT;
            }
        }

        if (cfg != RT_NULL && cfg->buffer != RT_NULL)
        {
            cfg->buffer->data = cam->frame_rgb565;
            cfg->buffer->size = ONE_FRAME_SIZE;
            cfg->buffer->frame_num = cam->frame_count;
            cfg->buffer->timestamp = rt_tick_get();
        }
        break;
    }

    case BF30A2_CMD_EXPORT_UART:
    {
        bf30a2_export_uart(cam);
        break;
    }

    case BF30A2_CMD_RESET_STATS:
    {
        rt_mutex_take(cam->lock, RT_WAITING_FOREVER);
        cam->frame_count = 0;
        cam->complete_frames = 0;
        cam->errors = 0;
        cam->line_count = 0;
        cam->last_time = rt_tick_get_millisecond();
        cam->last_frames = 0;
        cam->fps = 0;
        rt_mutex_release(cam->lock);
        break;
    }

    default:
        ret = -RT_EINVAL;
        break;
    }

    return ret;
}

/*============================================================================*/
/*                     PUBLIC API                                             */
/*============================================================================*/

void bf30a2_get_default_config(bf30a2_hw_cfg_t *cfg)
{
    if (cfg == RT_NULL)
    {
        return;
    }

    cfg->spi_bus_name = BF30A2_SPI_BUS;
    cfg->i2c_bus_name = BF30A2_I2C_BUS;
    cfg->pwm_dev_name = BF30A2_PWM_DEV;
    cfg->i2c_addr = BF30A2_I2C_ADDR;
    cfg->pwm_channel = BF30A2_PWM_CHANNEL;

    cfg->pins.spi_clk_pad = BF30A2_SPI_CLK_PAD;
    cfg->pins.spi_dio_pad = BF30A2_SPI_DIO_PAD;
    cfg->pins.spi_cs_pad = BF30A2_SPI_CS_PAD;
    cfg->pins.i2c_scl_pad = BF30A2_I2C_SCL_PAD;
    cfg->pins.i2c_sda_pad = BF30A2_I2C_SDA_PAD;
    cfg->pins.pwdn_pin = BF30A2_PWDN_PIN;
    cfg->pins.pwm_pad = BF30A2_PWM_PAD;
}

rt_err_t bf30a2_device_register_with_config(const char *name, const bf30a2_hw_cfg_t *hw_cfg)
{
    bf30a2_device_t *dev;
    rt_err_t ret;

    dev = rt_malloc(sizeof(bf30a2_device_t));
    if (dev == RT_NULL)
    {
        LOG_E("Failed to allocate device structure");
        return -RT_ENOMEM;
    }

    rt_memset(dev, 0, sizeof(bf30a2_device_t));

    /* Copy hardware configuration */
    if (hw_cfg != RT_NULL)
    {
        rt_memcpy(&dev->hw_cfg, hw_cfg, sizeof(bf30a2_hw_cfg_t));
    }
    else
    {
        bf30a2_get_default_config(&dev->hw_cfg);
    }

    /* Initialize device structure */
    dev->parent.type = RT_Device_Class_Miscellaneous;

#ifdef RT_USING_DEVICE_OPS
    dev->parent.ops = &bf30a2_ops;
#else
    dev->parent.init    = bf30a2_dev_init;
    dev->parent.open    = bf30a2_dev_open;
    dev->parent.close   = bf30a2_dev_close;
    dev->parent.read    = bf30a2_dev_read;
    dev->parent.write   = RT_NULL;
    dev->parent.control = bf30a2_dev_control;
#endif

    dev->parent.user_data = dev;

    /* Register device */
    ret = rt_device_register(&dev->parent, name,
                            RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_STANDALONE);
    if (ret != RT_EOK)
    {
        LOG_E("Failed to register device");
        rt_free(dev);
        return ret;
    }

    /* Save global pointer for DMA callback */
    g_bf30a2_dev = dev;

    LOG_I("BF30A2 device '%s' registered", name);

    return RT_EOK;
}

rt_err_t bf30a2_device_register(void)
{
    return bf30a2_device_register_with_config(BF30A2_DEVICE_NAME, RT_NULL);
}

/*============================================================================*/
/*                     SHELL COMMANDS                                         */
/*============================================================================*/

static void cmd_bf30a2_init(int argc, char **argv)
{
    rt_device_t dev = rt_device_find(BF30A2_DEVICE_NAME);
    if (dev == RT_NULL)
    {
        bf30a2_device_register();
        dev = rt_device_find(BF30A2_DEVICE_NAME);
    }
    if (dev != RT_NULL)
    {
        rt_device_init(dev);
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_bf30a2_init, bf30a2_init, Init BF30A2 camera device);

static void cmd_bf30a2_open(int argc, char **argv)
{
    rt_device_t dev = rt_device_find(BF30A2_DEVICE_NAME);
    if (dev != RT_NULL)
    {
        rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
    }
    else
    {
        rt_kprintf("Device not found, run bf30a2_init first\n");
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_bf30a2_open, bf30a2_open, Open BF30A2 camera device);

static void cmd_bf30a2_start(int argc, char **argv)
{
    rt_device_t dev = rt_device_find(BF30A2_DEVICE_NAME);
    if (dev != RT_NULL)
    {
        rt_device_control(dev, BF30A2_CMD_START, RT_NULL);
    }
    else
    {
        rt_kprintf("Device not found\n");
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_bf30a2_start, bf30a2_start, Start capture);

static void cmd_bf30a2_stop(int argc, char **argv)
{
    rt_device_t dev = rt_device_find(BF30A2_DEVICE_NAME);
    if (dev != RT_NULL)
    {
        rt_device_control(dev, BF30A2_CMD_STOP, RT_NULL);
    }
    else
    {
        rt_kprintf("Device not found\n");
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_bf30a2_stop, bf30a2_stop, Stop capture);

static void cmd_bf30a2_status(int argc, char **argv)
{
    rt_device_t dev = rt_device_find(BF30A2_DEVICE_NAME);
    if (dev != RT_NULL)
    {
        bf30a2_status_info_t status;
        bf30a2_info_t info;

        rt_device_control(dev, BF30A2_CMD_GET_INFO, &info);
        rt_device_control(dev, BF30A2_CMD_GET_STATUS, &status);

        rt_kprintf("=== BF30A2 Status ===\n");
        rt_kprintf("Chip ID: 0x%04X\n", info.chip_id);
        rt_kprintf("Resolution: %dx%d\n", info.width, info.height);
        rt_kprintf("State: %s\n", status.state == BF30A2_STATUS_RUNNING ? "Running" : "Idle");
        rt_kprintf("Frames: %d complete\n", status.complete_frames);
        rt_kprintf("Errors: %d\n", status.error_count);
        rt_kprintf("FPS: %.1f\n", status.fps);
        rt_kprintf("Frame ready: %d\n", status.frame_ready);
        rt_kprintf("=====================\n");
    }
    else
    {
        rt_kprintf("Device not found\n");
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_bf30a2_status, bf30a2_status, Show camera status);

static void cmd_bf30a2_export(int argc, char **argv)
{
    rt_device_t dev = rt_device_find(BF30A2_DEVICE_NAME);
    if (dev != RT_NULL)
    {
        rt_device_control(dev, BF30A2_CMD_EXPORT_UART, RT_NULL);
    }
    else
    {
        rt_kprintf("Device not found\n");
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_bf30a2_export, bf30a2_export, Export frame via UART);

/*============================================================================*/
/*                     AUTO INITIALIZATION                                    */
/*============================================================================*/

#ifdef BF30A2_AUTO_REGISTER
static int bf30a2_auto_register(void)
{
    return bf30a2_device_register();
}
INIT_DEVICE_EXPORT(bf30a2_auto_register);
#endif

/*============================================================================*/
/*                     END OF FILE                                            */
/*============================================================================*/