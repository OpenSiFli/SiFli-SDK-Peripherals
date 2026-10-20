/**
 * @file gc032a.c
 * @brief GC032A 8-bit DVP camera sensor driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#include "gc032a.h"

#include "../camera_driver_desc.h"
#include "../camera_sensor_runtime.h"
#include "camera_xclk.h"
#include "gc032a_regs.h"
#if defined(CAMERA_GC032A_INTERFACE_SERIAL_2BIT)
#include "gc032a_serial_settings.h"
#else
#include "gc032a_dvp_settings.h"
#endif
#include "sccb.h"

#define DBG_TAG "gc032a"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* Sensor-owned constants: the control-bus speed/timeout, the frame wait and the
 * 2-bit serial packet format are properties of this sensor, not menu options.
 * The I2C bus itself (and therefore its device name) belongs to the board. */
#define GC032A_SCCB_TIMEOUT_MS   1000U
#define GC032A_SCCB_MAX_HZ       100000U
#define GC032A_FRAME_TIMEOUT_MS  1000U

/* GC032A has no oscillator: the missing-MCLK case is a build error raised by
 * camera_xclk.c (CAMERA_XCLK_REQUIRED / CAMERA_XCLK_ENABLE). */

#if defined(CAMERA_GC032A_INTERFACE_SERIAL_2BIT)
/* One VGA line of RGB565 payload. The DCMI packet mode uses it to tell a
 * complete packet from a truncated one, and the marker bytes come from the
 * sensor's serial packet format: frame start 0xca, frame end 0xb6, line start
 * 0x80, line end 0x9d, packed low to high.
 * The data lines are wired in the sensor's order, and GC032A clocks the payload
 * out on the falling serial clock edge. */
static const bus_serial_config_t s_serial_2bit =
{
    .packet_size = 1280U,
    .packet_ids = 0x9d80b6caU,
    .sync_word = 0x00ffffffU,
    .sample_falling = 1U,
    .data_line_swap = 0U,
};
#endif

typedef struct
{
    camera_sensor_runtime_t runtime;
} gc032a_device_t;

typedef struct
{
    sccb_config_t sccb;
    camera_sensor_runtime_config_t runtime;
    uint32_t xclk_frequency_hz;
} gc032a_hw_config_t;

static gc032a_device_t s_device;

static const pixformat_t s_pixformats[] =
{
    PIXFORMAT_RGB565,
#if !defined(CAMERA_GC032A_INTERFACE_SERIAL_2BIT)
    PIXFORMAT_YUV422,
#endif
};

static const framesize_t s_framesizes[] =
{
#if defined(CAMERA_GC032A_INTERFACE_SERIAL_2BIT)
    FRAMESIZE_VGA,
#else
    FRAMESIZE_96X96,
    FRAMESIZE_QQVGA,
    FRAMESIZE_128X128,
    FRAMESIZE_QCIF,
    FRAMESIZE_HQVGA,
    FRAMESIZE_240X240,
    FRAMESIZE_QVGA,
    FRAMESIZE_320X320,
    FRAMESIZE_CIF,
    FRAMESIZE_HVGA,
    FRAMESIZE_VGA,
#endif
};

static const camera_capabilities_t s_capabilities =
{
    .pixformats = s_pixformats,
    .num_pixformats = sizeof(s_pixformats) / sizeof(s_pixformats[0]),
    .framesizes = s_framesizes,
    .num_framesizes = sizeof(s_framesizes) / sizeof(s_framesizes[0]),
    .max_buffer_size = 640U * 480U * 2U,
};

static const camera_capture_config_t s_default_config =
{
    .pixformat = PIXFORMAT_RGB565,
#if defined(CAMERA_GC032A_INTERFACE_SERIAL_2BIT)
    .framesize = FRAMESIZE_VGA,
#else
    .framesize = FRAMESIZE_QVGA,
#endif
    .quality = 0,
};

static const gc032a_hw_config_t s_hw_config =
{
    .sccb =
    {
        .bus_name = SCCB_I2C_BUS_NAME,
        .timeout_ms = GC032A_SCCB_TIMEOUT_MS,
        .max_hz = GC032A_SCCB_MAX_HZ,
    },
    .runtime =
    {
        .adapter_name = CAMERA_DATA_BUS_ADAPTER_NAME,
#if defined(CAMERA_GC032A_INTERFACE_SERIAL_2BIT)
        .adapter_type = BUS_TYPE_SPI,
        .serial = &s_serial_2bit,
#else
        .adapter_type = BUS_TYPE_DVP,
#endif
        .frame_timeout_ms = GC032A_FRAME_TIMEOUT_MS,
        .default_config = &s_default_config,
    },
#if defined(CAMERA_GC032A_INTERFACE_SERIAL_2BIT) && \
    !defined(CAMERA_SERIAL_SPI_2BIT)
    .xclk_frequency_hz = 6000000U,
#else
    .xclk_frequency_hz = 24000000U,
#endif
};

static int gc032a_open(void);
static int gc032a_close(void);
static int gc032a_set_pixformat(pixformat_t pixformat);
static int gc032a_set_framesize(framesize_t framesize);
static rt_size_t gc032a_capture(void *buffer, rt_size_t buffer_size);
static int gc032a_capture_async(void *buffer,
                                rt_size_t buffer_size,
                                camera_capture_done_callback_t callback,
                                void *context);
static int gc032a_start_stream(const camera_stream_start_args_t *args);
static int gc032a_stop_stream(void);
static rt_bool_t gc032a_stream_frame_is_valid(const camera_stream_frame_t *frame);

const camera_device_ops_t gc032a_ops =
{
    .capabilities = &s_capabilities,
    .default_config = &s_default_config,
    .open = gc032a_open,
    .close = gc032a_close,
    .set_pixformat = gc032a_set_pixformat,
    .set_framesize = gc032a_set_framesize,
    .set_quality = RT_NULL,
    .capture = gc032a_capture,
    .capture_async = gc032a_capture_async,
    .start_stream = gc032a_start_stream,
    .stop_stream = gc032a_stop_stream,
    .is_stream_frame_valid = gc032a_stream_frame_is_valid,
};

CAMERA_DRIVER_EXPORT(gc032a, &gc032a_ops);

static int gc032a_write_reg(uint8_t reg, uint8_t value)
{
    return sccb_write(GC032A_ADDR, reg, value);
}

static int gc032a_read_reg(uint8_t reg, uint8_t *value)
{
    return sccb_read_bytes(GC032A_ADDR, &reg, 1, value, 1);
}

static int gc032a_write_table(const uint8_t (*regs)[2], rt_size_t count)
{
    rt_size_t i;

    for (i = 0; i < count; i++)
    {
        int ret = gc032a_write_reg(regs[i][0], regs[i][1]);
        if (ret != RT_EOK)
        {
            return ret;
        }
    }

    return RT_EOK;
}

static int gc032a_update_reg(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current;
    int ret;

    ret = gc032a_read_reg(reg, &current);
    if (ret != RT_EOK)
    {
        return ret;
    }

    current = (uint8_t)((current & (uint8_t)~mask) | (value & mask));
    return gc032a_write_reg(reg, current);
}

static int gc032a_check_id(void)
{
    uint8_t id_high;
    uint8_t id_low;
    uint16_t chip_id;
    int ret;

    ret = gc032a_read_reg(GC032A_REG_CHIP_ID_HIGH, &id_high);
    if (ret == RT_EOK)
    {
        ret = gc032a_read_reg(GC032A_REG_CHIP_ID_LOW, &id_low);
    }
    if (ret != RT_EOK)
    {
        return ret;
    }

    chip_id = (uint16_t)(((uint16_t)id_high << 8) | id_low);
    if (chip_id != GC032A_CHIP_ID)
    {
        LOG_E("Unexpected chip ID: 0x%04x", chip_id);
        return -RT_ERROR;
    }

    LOG_I("Detected GC032A, ID=0x%04x", chip_id);
    return RT_EOK;
}

static int gc032a_apply_pixformat(pixformat_t pixformat)
{
#if defined(CAMERA_GC032A_INTERFACE_SERIAL_2BIT)
    if (pixformat != PIXFORMAT_RGB565)
    {
        return -RT_EINVAL;
    }
#endif
    uint8_t value;
    int ret;

    switch (pixformat)
    {
        case PIXFORMAT_RGB565:
            value = GC032A_OUTPUT_FORMAT_RGB565;
            break;
        case PIXFORMAT_YUV422:
            value = GC032A_OUTPUT_FORMAT_YUV422;
            break;
        default:
            return -RT_EINVAL;
    }

    ret = gc032a_write_reg(GC032A_REG_PAGE_SELECT, GC032A_PAGE_0);
    if (ret == RT_EOK)
    {
        ret = gc032a_update_reg(GC032A_REG_OUTPUT_FORMAT,
                                GC032A_OUTPUT_FORMAT_MASK,
                                value);
    }
    return ret;
}

static int gc032a_apply_framesize(framesize_t framesize)
{
#if defined(CAMERA_GC032A_INTERFACE_SERIAL_2BIT)
    return framesize == FRAMESIZE_VGA ? RT_EOK : -RT_EINVAL;
#else
    uint8_t regs[13][2];
    uint16_t width;
    uint16_t height;
    uint16_t row_start;
    uint16_t column_start;
    int ret;

    if (framesize > FRAMESIZE_VGA ||
        camera_sensor_get_resolution(framesize, &width, &height) != RT_EOK)
    {
        return -RT_EINVAL;
    }

    row_start = (uint16_t)((480U - height) / 2U);
    column_start = (uint16_t)((640U - width) / 2U);

    regs[0][0] = GC032A_REG_ROW_START_HIGH;
    regs[0][1] = (uint8_t)(row_start >> 8);
    regs[1][0] = GC032A_REG_ROW_START_LOW;
    regs[1][1] = (uint8_t)row_start;
    regs[2][0] = GC032A_REG_COLUMN_START_HIGH;
    regs[2][1] = (uint8_t)(column_start >> 8);
    regs[3][0] = GC032A_REG_COLUMN_START_LOW;
    regs[3][1] = (uint8_t)column_start;
    regs[4][0] = GC032A_REG_WINDOW_HEIGHT_HIGH;
    regs[4][1] = (uint8_t)((height + 8U) >> 8);
    regs[5][0] = GC032A_REG_WINDOW_HEIGHT_LOW;
    regs[5][1] = (uint8_t)(height + 8U);
    regs[6][0] = GC032A_REG_WINDOW_WIDTH_HIGH;
    regs[6][1] = (uint8_t)((width + 8U) >> 8);
    regs[7][0] = GC032A_REG_WINDOW_WIDTH_LOW;
    regs[7][1] = (uint8_t)(width + 8U);
    regs[8][0] = GC032A_REG_WINDOW_MODE;
    regs[8][1] = 0x01;
    regs[9][0] = GC032A_REG_OUTPUT_HEIGHT_HIGH;
    regs[9][1] = (uint8_t)(height >> 8);
    regs[10][0] = GC032A_REG_OUTPUT_HEIGHT_LOW;
    regs[10][1] = (uint8_t)height;
    regs[11][0] = GC032A_REG_OUTPUT_WIDTH_HIGH;
    regs[11][1] = (uint8_t)(width >> 8);
    regs[12][0] = GC032A_REG_OUTPUT_WIDTH_LOW;
    regs[12][1] = (uint8_t)width;

    ret = gc032a_write_reg(GC032A_REG_PAGE_SELECT, GC032A_PAGE_0);
    if (ret == RT_EOK)
    {
        ret = gc032a_write_table(regs, sizeof(regs) / sizeof(regs[0]));
    }
    return ret;
#endif
}

static int gc032a_sensor_init(void)
{
    int ret;

    ret = gc032a_check_id();
    if (ret != RT_EOK)
    {
        return ret;
    }

    ret = gc032a_write_reg(GC032A_REG_PAGE_SELECT, GC032A_SOFTWARE_RESET);
    if (ret != RT_EOK)
    {
        return ret;
    }
    rt_thread_mdelay(100);

#if defined(CAMERA_GC032A_INTERFACE_SERIAL_2BIT)
    ret = gc032a_write_table(gc032a_serial_default_regs,
                             sizeof(gc032a_serial_default_regs) /
                             sizeof(gc032a_serial_default_regs[0]));
#else
    ret = gc032a_write_table(gc032a_dvp_default_regs,
                             sizeof(gc032a_dvp_default_regs) /
                             sizeof(gc032a_dvp_default_regs[0]));
#endif
    if (ret == RT_EOK)
    {
        rt_thread_mdelay(100);
    }
    if (ret == RT_EOK)
    {
        ret = gc032a_apply_pixformat(s_default_config.pixformat);
    }
    if (ret == RT_EOK)
    {
        ret = gc032a_apply_framesize(s_default_config.framesize);
    }
    return ret;
}

static int gc032a_open(void)
{
    int ret;

    if (s_device.runtime.is_open)
    {
        return RT_EOK;
    }

    rt_memset(&s_device, 0, sizeof(s_device));
    if (camera_xclk_start(s_hw_config.xclk_frequency_hz) != CAMERA_XCLK_OK)
    {
        LOG_E("XCLK start failed");
        return -RT_ERROR;
    }

    ret = sccb_init(&s_hw_config.sccb);
    if (ret != RT_EOK)
    {
        LOG_E("SCCB init failed: %d", ret);
        camera_xclk_stop();
        return ret;
    }

    ret = gc032a_sensor_init();
    if (ret != RT_EOK)
    {
        LOG_E("GC032A init failed: %d", ret);
        sccb_deinit();
        camera_xclk_stop();
        return ret;
    }

    ret = camera_sensor_runtime_open(&s_device.runtime, &s_hw_config.runtime);
    if (ret != RT_EOK)
    {
        LOG_E("Camera runtime init failed: %d", ret);
        sccb_deinit();
        camera_xclk_stop();
    }
    return ret;
}

static int gc032a_close(void)
{
    int ret;

    if (!s_device.runtime.is_open)
    {
        return RT_EOK;
    }

    ret = camera_sensor_runtime_close(&s_device.runtime);
    if (ret == RT_EOK)
    {
        sccb_deinit();
        camera_xclk_stop();
    }
    return ret;
}

static int gc032a_set_pixformat(pixformat_t pixformat)
{
    int ret;

    if (!s_device.runtime.is_open)
    {
        return -RT_ERROR;
    }
    if (camera_sensor_runtime_busy(&s_device.runtime))
    {
        return -RT_EBUSY;
    }

    ret = gc032a_apply_pixformat(pixformat);
    if (ret == RT_EOK)
    {
        ret = camera_sensor_runtime_set_pixformat(&s_device.runtime, pixformat);
    }
    return ret;
}

static int gc032a_set_framesize(framesize_t framesize)
{
    int ret;

    if (!s_device.runtime.is_open)
    {
        return -RT_ERROR;
    }
    if (camera_sensor_runtime_busy(&s_device.runtime))
    {
        return -RT_EBUSY;
    }

    ret = gc032a_apply_framesize(framesize);
    if (ret == RT_EOK)
    {
        ret = camera_sensor_runtime_set_framesize(&s_device.runtime, framesize);
    }
    return ret;
}

static rt_size_t gc032a_capture(void *buffer, rt_size_t buffer_size)
{
    return camera_sensor_runtime_capture(&s_device.runtime, buffer, buffer_size);
}

static int gc032a_capture_async(void *buffer,
                                rt_size_t buffer_size,
                                camera_capture_done_callback_t callback,
                                void *context)
{
    return camera_sensor_runtime_capture_async(&s_device.runtime,
                                               buffer,
                                               buffer_size,
                                               callback,
                                               context);
}

static int gc032a_start_stream(const camera_stream_start_args_t *args)
{
    return camera_sensor_runtime_start_stream(&s_device.runtime, args);
}

static int gc032a_stop_stream(void)
{
    return camera_sensor_runtime_stop_stream(&s_device.runtime);
}

static rt_bool_t gc032a_stream_frame_is_valid(const camera_stream_frame_t *frame)
{
    return camera_sensor_runtime_stream_frame_is_valid(&s_device.runtime, frame);
}
