/**
 * @file    ov2640.c
 * @brief   OV2640 camera sensor driver implementation
 *
 * This module implements the OV2640 sensor initialization, register
 * configuration, image parameter control, single-shot capture, and streaming
 * operations exposed through the camera handle's driver ops interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#include "ov2640.h"
#include "../camera_driver_desc.h"
#include "ov2640_regs.h"
#include "ov2640_settings.h"
#if defined(CAMERA_USING_ARDUCAM_FIFO)
/* Sensor-side JPEG tables for the ArduCAM SPI FIFO module. */
#include "ov2640_arducam_settings.h"
#endif
#include "camera_xclk.h"
#include "bf0_hal.h"
#include "rtthread.h"

#define DBG_TAG "ov2640"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <string.h>

static sensor_device_t  *s_active_device = RT_NULL;
static sensor_device_t  s_device;

static const pixformat_t g_camera_pixformats[] = {
    PIXFORMAT_JPEG,
#if defined(CAMERA_USING_ARDUCAM_FIFO)
    /* The FIFO module carries the OV2640's JPEG output only: it neither exposes
     * the sensor's parallel path nor accepts a per-register RGB565 profile.
     * Report JPEG alone so a caller cannot ask for something the module cannot
     * deliver. */
#else
    PIXFORMAT_RGB565,
    PIXFORMAT_YUV422,
#endif
};

static const framesize_t g_camera_framesizes[] = {
#if defined(CAMERA_USING_ARDUCAM_FIFO)
    /* The module knows two JPEG modes: 640x480 for preview and 1600x1200 for
     * stills. Each one is a complete register table, see
     * sensor_set_framesize(). */
    FRAMESIZE_VGA,
    FRAMESIZE_UXGA,
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
    FRAMESIZE_SVGA,
    FRAMESIZE_XGA,
    FRAMESIZE_HD,
    FRAMESIZE_SXGA,
    FRAMESIZE_UXGA,
#endif /* CAMERA_USING_ARDUCAM_FIFO */
};

static const camera_capabilities_t g_camera_caps = {
    .pixformats      = g_camera_pixformats,
    .num_pixformats  = sizeof(g_camera_pixformats) / sizeof(g_camera_pixformats[0]),
    .framesizes      = g_camera_framesizes,
    .num_framesizes  = sizeof(g_camera_framesizes) / sizeof(g_camera_framesizes[0]),
    .max_buffer_size = 1600U * 1200U * 2U,
};

static const camera_capture_config_t g_default_config = {
#if defined(CAMERA_DVP_BACKEND_DCMI)
    .pixformat = PIXFORMAT_RGB565,
#else
    .pixformat = PIXFORMAT_JPEG,
#endif
    .framesize = FRAMESIZE_SVGA,
    .quality = 10,
};

typedef struct {
    sccb_config_t sccb;
    camera_sensor_runtime_config_t runtime;
    uint32_t xclk_frequency_hz;
} sensor_hw_config_t;

static const sensor_hw_config_t g_hw_config = {
    .sccb = {
        .bus_name               = CAMERA_SCCB_I2C_BUS_NAME,
        .timeout_ms             = CAMERA_SCCB_TIMEOUT_MS,
        .max_hz                 = CAMERA_SCCB_MAX_HZ,
    },
    .runtime = {
        .adapter_name = CAMERA_DATA_BUS_ADAPTER_NAME,
#if defined(CAMERA_USING_ARDUCAM_FIFO)
        .adapter_type = BUS_TYPE_SPI,
#else
        .adapter_type = BUS_TYPE_DVP,
#endif
        .frame_timeout_ms = CAMERA_READ_TIMEOUT_MS,
        .default_config = &g_default_config,
    },
    .xclk_frequency_hz = 24000000U,
};

static int sensor_open(void);
static int sensor_close(void);
static int sensor_apply_pixformat(pixformat_t pixformat);
static int sensor_apply_framesize(framesize_t framesize);
static int sensor_apply_quality(uint8_t quality);
static int sensor_apply_rotation(uint16_t degrees);
static rt_size_t sensor_capture(void *buffer, rt_size_t buffer_size);
static rt_size_t sensor_capture_timeout(void *buffer, rt_size_t buffer_size,
                                        uint32_t timeout_ms);
static rt_size_t sensor_capture_frames_timeout(void *buffer, rt_size_t buffer_size,
                                               uint32_t frame_count, uint32_t timeout_ms);
static int sensor_capture_async(void *buffer,
                                rt_size_t buffer_size,
                                camera_capture_done_callback_t callback,
                                void *context);
static int sensor_start_stream(const camera_stream_start_args_t *args);
static int sensor_stop_stream(void);
static rt_bool_t sensor_stream_frame_is_valid(const camera_stream_frame_t *frame);

static int sensor_configure_arducam_sccb_pins(void)
{
#if defined(CAMERA_USING_ARDUCAM_FIFO)
    if (strcmp(CAMERA_SCCB_I2C_BUS_NAME, "i2c1") == 0)
    {
        HAL_PIN_Set(PAD_PA00 + CAMERA_SCCB_SCL_PIN, I2C1_SCL, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA00 + CAMERA_SCCB_SDA_PIN, I2C1_SDA, PIN_PULLUP, 1);
        return RT_EOK;
    }
    if (strcmp(CAMERA_SCCB_I2C_BUS_NAME, "i2c2") == 0)
    {
        HAL_PIN_Set(PAD_PA00 + CAMERA_SCCB_SCL_PIN, I2C2_SCL, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA00 + CAMERA_SCCB_SDA_PIN, I2C2_SDA, PIN_PULLUP, 1);
        return RT_EOK;
    }
    return -RT_EINVAL;
#else
    return RT_EOK;
#endif
}

const camera_device_ops_t ov2640_ops = {
    .capabilities   = &g_camera_caps,
    .default_config = &g_default_config,
    .open           = sensor_open,
    .close          = sensor_close,
    .set_pixformat  = sensor_apply_pixformat,
    .set_framesize  = sensor_apply_framesize,
    .set_quality    = sensor_apply_quality,
    .capture        = sensor_capture,
    .capture_timeout = sensor_capture_timeout,
    .capture_frames_timeout = sensor_capture_frames_timeout,
    .capture_async  = sensor_capture_async,
    .start_stream   = sensor_start_stream,
    .stop_stream    = sensor_stop_stream,
    .is_stream_frame_valid = sensor_stream_frame_is_valid,
    .set_rotation   = sensor_apply_rotation,
};

CAMERA_DRIVER_EXPORT(ov2640, &ov2640_ops);


typedef enum {
    ASPECT_RATIO_4X3,
    ASPECT_RATIO_3X2,
    ASPECT_RATIO_16X10,
    ASPECT_RATIO_5X3,
    ASPECT_RATIO_16X9,
    ASPECT_RATIO_21X9,
    ASPECT_RATIO_5X4,
    ASPECT_RATIO_1X1,
    ASPECT_RATIO_9X16
} aspect_ratio_t;

#if !defined(CAMERA_USING_ARDUCAM_FIFO)
/* Crop/scale tables for the direct-sensor paths. The ArduCAM module uploads one
 * complete JPEG table per resolution instead (see sensor_set_framesize()). */
static const aspect_ratio_t s_aspect_ratios[FRAMESIZE_INVALID] = {
    ASPECT_RATIO_1X1,  /* 96x96 */
    ASPECT_RATIO_4X3,  /* QQVGA */
    ASPECT_RATIO_1X1,  /* 128x128 */
    ASPECT_RATIO_5X4,  /* QCIF  */
    ASPECT_RATIO_4X3,  /* HQVGA */
    ASPECT_RATIO_1X1,  /* 240x240 */
    ASPECT_RATIO_4X3,  /* QVGA  */
    ASPECT_RATIO_1X1,  /* 320x320 */
    ASPECT_RATIO_4X3,  /* CIF   */
    ASPECT_RATIO_3X2,  /* HVGA  */
    ASPECT_RATIO_4X3,  /* VGA   */
    ASPECT_RATIO_4X3,  /* SVGA  */
    ASPECT_RATIO_4X3,  /* XGA   */
    ASPECT_RATIO_16X9, /* HD    */
    ASPECT_RATIO_5X4,  /* SXGA  */
    ASPECT_RATIO_4X3,  /* UXGA  */
};
#endif /* CAMERA_USING_ARDUCAM_FIFO */




/**
 * @brief Invalidate the cached bank so the next access forces a BANK_SEL write.
 *
 * Call on deinit to ensure the driver state is consistent after re-open.
 */
static void sensor_reset_bank_state(void)
{
    if (s_active_device != RT_NULL)
    {
        s_active_device->current_bank = (rt_uint8_t)BANK_MAX;
    }
}

/** @brief Write a register and track explicit bank selections and resets. */
static int sensor_write_raw(uint8_t reg, uint8_t data)
{
    sensor_device_t *dev = s_active_device;
    int ret = sccb_write(OV2640_ADDR, reg, data);

    if (ret != RT_EOK)
    {
        unsigned int bank = (dev != RT_NULL && dev->current_bank < BANK_MAX) ?
                            dev->current_bank : 0xffU;

        LOG_E("SCCB write failed: bank=%02x reg=%02x val=%02x rc=%d",
              bank, reg, data, ret);
        sensor_reset_bank_state();
        return ret;
    }
    if (dev != RT_NULL)
    {
        if (reg == BANK_SEL)
        {
            dev->current_bank = (data < BANK_MAX) ? data : (rt_uint8_t)BANK_MAX;
        }
        else if (dev->current_bank == BANK_SENSOR &&
                 reg == COM7 && (data & COM7_SRST) != 0U)
        {
            sensor_reset_bank_state();
        }
    }
    return RT_EOK;
}

/** @brief Switch the active register bank if needed. */
static int sensor_set_bank(ov2640_bank_t bank)
{
    if ((unsigned int)bank >= BANK_MAX)
    {
        return -RT_EINVAL;
    }
    if (s_active_device != RT_NULL && s_active_device->current_bank == bank)
    {
        return RT_EOK;
    }
    return sensor_write_raw(BANK_SEL, (uint8_t)bank);
}

/** @brief Write one register in the selected bank. */
static int sensor_write_reg(ov2640_bank_t bank, uint8_t reg, uint8_t data)
{
    int ret = sensor_set_bank(bank);

    if (ret != RT_EOK)
    {
        return ret;
    }
    return sensor_write_raw(reg, data);
}

/** @brief Write a register table ending with {0, 0}. */
static int sensor_write_regs(const uint8_t (*regs)[2])
{
    int ret;

    while (regs[0][0] != 0 || regs[0][1] != 0)
    {
        ret = sensor_write_raw(regs[0][0], regs[0][1]);
        if (ret != RT_EOK)
        {
            return ret;
        }
        regs++;
    }
    return RT_EOK;
}


/** @brief Read one register without hiding SCCB errors. */
static int sensor_read_reg(ov2640_bank_t bank, uint8_t reg, uint8_t *data)
{
    int ret = sensor_set_bank(bank);

    if (ret != RT_EOK)
    {
        return ret;
    }
    return sccb_read_bytes(OV2640_ADDR, &reg, 1, data, 1);
}


#if defined(CAMERA_USING_ARDUCAM_FIFO)
/*
 * Write one ArduCAM JPEG table. These tables select their own register bank and
 * end with {0xFF, 0xFF} instead of the {0x00, 0x00} terminator expected by
 * sensor_write_regs(), so they need their own walker.
 */
static int sensor_write_arducam_table(const arducam_sensor_reg_t *table)
{
    rt_size_t index;
    int ret = RT_EOK;

    for (index = 0U; !((table[index].reg == 0xFFU) &&
                       (table[index].value == 0xFFU)); index++)
    {
        ret = sensor_write_raw(table[index].reg, table[index].value);
        if (ret != RT_EOK)
        {
            break;
        }
    }
    /* The tables switch banks themselves, so the cached bank is stale. */
    if (s_active_device != RT_NULL)
        s_active_device->current_bank = (rt_uint8_t)BANK_MAX;
    return ret;
}
#endif /* CAMERA_USING_ARDUCAM_FIFO */

/** @brief Reset OV2640 and load the selected initialization profile. */
static int sensor_reset(void)
{
#if defined(CAMERA_USING_ARDUCAM_FIFO)
    int ret = sensor_write_reg(BANK_SENSOR, COM7, COM7_SRST);

    if (ret != RT_EOK)
    {
        return ret;
    }
    rt_thread_mdelay(100);
    return RT_EOK;
#else
    int ret = sensor_write_reg(BANK_SENSOR, COM7, COM7_SRST);

    if (ret != RT_EOK)
    {
        return ret;
    }
    rt_thread_mdelay(10);
    return sensor_write_regs(ov2640_settings_cif);
#endif /* CAMERA_USING_ARDUCAM_FIFO */
}

/** @brief Read the initial sensor status after initialization. */
static int sensor_init_status(sensor_device_t *dev)
{
    int ret = sensor_read_reg(BANK_DSP, QS, &dev->runtime.quality);

    if (ret != RT_EOK)
    {
        return ret;
    }
    dev->runtime.framesize = FRAMESIZE_UXGA;
    return RT_EOK;
}

/**
 * @brief Set pixel output format (RGB565, YUV422, JPEG, RAW8).
 *
 * @param dev      is a pointer to the sensor handle.
 * @param pixformat is the desired output format.
 *
 * @return RT_EOK on success; negative error code on failure.
 */
static int sensor_set_pixformat(sensor_device_t *dev, pixformat_t pixformat)
{
    (void)dev;

#if defined(CAMERA_USING_ARDUCAM_FIFO)
    /* The module only carries JPEG. Its sensor tables are written once by
     * sensor_init() and again by every sensor_set_framesize() call, so there is
     * nothing left to rewrite here. */
    return pixformat == PIXFORMAT_JPEG ? RT_EOK : -RT_EINVAL;
#else
    switch (pixformat)
    {
        case PIXFORMAT_RGB565:
            return sensor_write_regs(ov2640_settings_rgb565);
        case PIXFORMAT_YUV422:
            return sensor_write_regs(ov2640_settings_yuv422);
        case PIXFORMAT_JPEG:
            return sensor_write_regs(ov2640_settings_jpeg3);
        case PIXFORMAT_RAW8:
            return sensor_write_regs(ov2640_settings_raw8);
        default:
            return -RT_EINVAL;
    }
#endif /* CAMERA_USING_ARDUCAM_FIFO */
}

#if !defined(CAMERA_USING_ARDUCAM_FIFO)
/** @brief Read back the stable registers in the 24 MHz SVGA timing profile. */
static int sensor_verify_svga_30fps(void)
{
    /* Bank, register, defined-field mask, expected field values. */
    static const uint8_t expected[][4] = {
        {BANK_SENSOR, CLKRC, 0xbf, 0x00},
        {BANK_SENSOR, COM7, 0x76, 0x40},
        {BANK_SENSOR, COM1, 0xcf, 0x0a},
        {BANK_SENSOR, REG2A, 0xf0, 0x00},
        {BANK_SENSOR, FRARL, 0xff, 0x00},
        {BANK_SENSOR, ADDVSL, 0xff, 0x00},
        {BANK_SENSOR, ADDVSH, 0xff, 0x00},
        {BANK_SENSOR, FLL, 0xff, 0x00},
        {BANK_SENSOR, FLH, 0xff, 0x00},
        {BANK_DSP, R_DVP_SP, 0xff, 0x82},
        {BANK_DSP, IMAGE_MODE, 0x5f, 0x09},
    };
    rt_size_t i;

    for (i = 0; i < sizeof(expected) / sizeof(expected[0]); i++)
    {
        uint8_t value;
        int ret = sensor_read_reg((ov2640_bank_t)expected[i][0], expected[i][1], &value);

        if (ret != RT_EOK)
        {
            LOG_E("SVGA timing read failed: bank=%02x reg=%02x rc=%d",
                  expected[i][0], expected[i][1], ret);
            return ret;
        }
        if ((value & expected[i][2]) != expected[i][3])
        {
            LOG_E("SVGA timing mismatch: bank=%02x reg=%02x mask=%02x expected=%02x actual=%02x",
                  expected[i][0], expected[i][1], expected[i][2], expected[i][3], value);
            return -RT_ERROR;
        }
    }
    LOG_I("SVGA 30fps profile readback OK (24MHz, RGB565)");
    return RT_EOK;
}

static int sensor_verify_uxga_rgb565(const uint8_t (*window)[2])
{
    static const uint8_t expected[][4] = {
        {BANK_SENSOR, CLKRC, 0xbf, 0x01},
        {BANK_SENSOR, COM7, 0x76, 0x00},
        {BANK_SENSOR, COM1, 0xcf, 0x0f},
        {BANK_SENSOR, COM2, 0x13, 0x02},
        {BANK_SENSOR, HSTART, 0xff, 0x11},
        {BANK_SENSOR, HSTOP, 0xff, 0x75},
        {BANK_SENSOR, VSTART, 0xff, 0x01},
        {BANK_SENSOR, VSTOP, 0xff, 0x97},
        {BANK_SENSOR, FLL, 0xff, 0x00},
        {BANK_SENSOR, FLH, 0xff, 0x00},
        {BANK_DSP, HSIZE8, 0xff, 0xc8},
        {BANK_DSP, VSIZE8, 0xff, 0x96},
        {BANK_DSP, SIZEL, 0xff, 0x00},
        {BANK_DSP, R_DVP_SP, 0xff, 0x82},
        {BANK_DSP, IMAGE_MODE, 0x5f, 0x09},
        {BANK_DSP, R_BYPASS, 0x01, 0x00},
        {BANK_DSP, OV2640_REG_RESET, 0xff, 0x00},
    };
    rt_size_t i;
    uint8_t value;
    int ret;

    for (i = 0; i < sizeof(expected) / sizeof(expected[0]); ++i)
    {
        ret = sensor_read_reg((ov2640_bank_t)expected[i][0], expected[i][1], &value);
        if (ret != RT_EOK || (value & expected[i][2]) != expected[i][3])
        {
            LOG_E("UXGA readback failed: bank=%02x reg=%02x rc=%d expected=%02x actual=%02x",
                  expected[i][0], expected[i][1], ret, expected[i][3],
                  ret == RT_EOK ? value : 0);
            return ret != RT_EOK ? ret : -RT_ERROR;
        }
    }
    /* Skip the bank selection; compare the calculated crop/scale registers. */
    for (i = 1; window[i][0] != 0 || window[i][1] != 0; ++i)
    {
        ret = sensor_read_reg(BANK_DSP, window[i][0], &value);
        if (ret != RT_EOK || value != window[i][1])
        {
            LOG_E("UXGA window readback failed: reg=%02x rc=%d expected=%02x actual=%02x",
                  window[i][0], ret, window[i][1], ret == RT_EOK ? value : 0);
            return ret != RT_EOK ? ret : -RT_ERROR;
        }
    }
    LOG_I("UXGA RGB565 readback OK (24MHz, CLKRC=01 DVP=82 drive=02)");
    return RT_EOK;
}

/**
 * @brief Configure sensor window (crop and scale) settings
 * @param dev Pointer to sensor_device_t structure
 * @param mode Sensor mode (CIF, SVGA, UXGA)
 * @param offset_x Horizontal offset
 * @param offset_y Vertical offset
 * @param max_x Maximum horizontal size
 * @param max_y Maximum vertical size
 * @param w Output width
 * @param h Output height
 * @return 0 on success, negative error code on failure
 */
static int sensor_set_window(sensor_device_t *dev, ov2640_sensor_mode_t mode,
                             int offset_x, int offset_y, int max_x, int max_y,
                             int w, int h)
{
    int ret;
    const uint8_t (*regs)[2];
    ov2640_clk_t c;
    c.reserved = 0;

    max_x /= 4;
    max_y /= 4;
    w /= 4;
    h /= 4;
    uint8_t win_regs[][2] =
    {
        {BANK_SEL, BANK_DSP},
        {HSIZE, max_x & 0xFF},
        {VSIZE, max_y & 0xFF},
        {XOFFL, offset_x & 0xFF},
        {YOFFL, offset_y & 0xFF},
        {VHYX, ((max_y >> 1) & 0X80) | ((offset_y >> 4) & 0X70) | ((max_x >> 5) & 0X08) | ((offset_x >> 8) & 0X07)},
        {TEST, (max_x >> 2) & 0X80},
        {ZMOW, (w) & 0xFF},
        {ZMOH, (h) & 0xFF},
        {ZMHH, ((h >> 6) & 0x04) | ((w >> 8) & 0x03)},
        {0, 0}
    };

    if (dev->runtime.pixformat == PIXFORMAT_JPEG)
    {
        c.clk_2x = 1;
        c.clk_div = 0;
        c.pclk_auto = 0;
        c.pclk_div = 6;
        if (mode == OV2640_MODE_UXGA)
        {
            c.pclk_div = 24;
#if defined(CAMERA_DVP_BACKEND_DCMI)
            if (g_hw_config.xclk_frequency_hz == 24000000U)
            {
                /* Retain the verified UXGA sensor clock; JPEG uses a fixed
                 * DVP output divider rather than RGB565 auto division. */
                c.clk_2x = 0;
                c.clk_div = 1;
                c.pclk_div = 8;
            }
#endif
        }
    }
    else
    {
        c.clk_2x = 1;
        c.clk_div = 3;
        c.pclk_auto = 1;
        c.pclk_div = 4;
        if (mode == OV2640_MODE_CIF)
        {
            c.clk_div = 3;
        }
        else if (mode == OV2640_MODE_SVGA)
        {
            /* OV2640 application notes: SVGA 30 fps with a 24 MHz XCLK. */
            c.clk_2x = 0;
            c.clk_div = 0;
            c.pclk_div = 2;
        }
        else if (mode == OV2640_MODE_UXGA)
        {
            if (dev->runtime.pixformat == PIXFORMAT_RGB565 &&
                g_hw_config.xclk_frequency_hz == 24000000U)
            {
                c.clk_2x = 0;
                c.clk_div = 1;
                c.pclk_div = 2;
            }
            else
            {
                c.pclk_div = 12;
            }
        }
    }

    if (mode == OV2640_MODE_CIF)
    {
        regs = ov2640_settings_to_cif;
    }
    else if (mode == OV2640_MODE_SVGA)
    {
        regs = ov2640_settings_to_svga;
    }
    else
    {
        regs = ov2640_settings_to_uxga;
    }

    ret = sensor_write_reg(BANK_DSP, R_BYPASS, R_BYPASS_DSP_BYPAS);
    if (ret != RT_EOK)
    {
        return ret;
    }
    ret = sensor_write_regs(regs);
    if (ret != RT_EOK)
    {
        return ret;
    }
    ret = sensor_write_regs(win_regs);
    if (ret != RT_EOK)
    {
        return ret;
    }
    ret = sensor_write_reg(BANK_SENSOR, CLKRC, c.clk);
    if (ret != RT_EOK)
    {
        return ret;
    }
    ret = sensor_write_reg(BANK_DSP, R_DVP_SP, c.pclk);
    if (ret != RT_EOK)
    {
        return ret;
    }
    ret = sensor_write_reg(BANK_SENSOR, FLH, 0x00);
    if (ret != RT_EOK)
    {
        return ret;
    }
    ret = sensor_write_reg(BANK_SENSOR, FLL, 0x00);
    if (ret != RT_EOK)
    {
        return ret;
    }
    ret = sensor_write_reg(BANK_DSP, R_BYPASS, R_BYPASS_DSP_EN);
    if (ret != RT_EOK)
    {
        return ret;
    }

    rt_thread_mdelay(10);
    /* Resolution tables overwrite the output format. */
    ret = sensor_set_pixformat(dev, dev->runtime.pixformat);
    if (ret != RT_EOK)
    {
        return ret;
    }
#if defined(CAMERA_DVP_BACKEND_DCMI)
    if (mode == OV2640_MODE_UXGA && dev->runtime.pixformat == PIXFORMAT_JPEG &&
        g_hw_config.xclk_frequency_hz == 24000000U)
    {
        uint8_t clock, dvp, format;

        if (sensor_read_reg(BANK_SENSOR, CLKRC, &clock) != RT_EOK ||
            sensor_read_reg(BANK_DSP, R_DVP_SP, &dvp) != RT_EOK ||
            sensor_read_reg(BANK_DSP, IMAGE_MODE, &format) != RT_EOK ||
            (clock & 0xbfU) != 0x01U || dvp != 0x08U || (format & 0x5fU) != 0x12U)
            return -RT_EIO;
        LOG_I("UXGA JPEG readback OK (24MHz, CLKRC=01 DVP=08)");
    }
#endif
#if !defined(CAMERA_USING_ARDUCAM_FIFO)
    /* These profile checks read the sensor's register file back through SCCB.
     * Behind the ArduCAM module the plain OV2640 register map is not exposed
     * (reads answer with the module's own values), so the checks are only valid
     * for the direct-sensor backends. */
    if (mode == OV2640_MODE_SVGA && dev->runtime.pixformat == PIXFORMAT_RGB565 &&
        g_hw_config.xclk_frequency_hz == 24000000U)
    {
        return sensor_verify_svga_30fps();
    }
    if (mode == OV2640_MODE_UXGA && dev->runtime.pixformat == PIXFORMAT_RGB565 &&
        g_hw_config.xclk_frequency_hz == 24000000U)
    {
        return sensor_verify_uxga_rgb565(win_regs);
    }
#endif /* CAMERA_USING_ARDUCAM_FIFO */
    return RT_EOK;
}
#endif /* CAMERA_USING_ARDUCAM_FIFO */

/**
 * @brief Set frame size/resolution
 * @param dev Pointer to sensor_device_t structure
 * @param framesize Desired frame size (QVGA, VGA, SVGA, UXGA, etc.)
 * @return 0 on success, negative error code on failure
 */
static int sensor_set_framesize(sensor_device_t *dev, framesize_t framesize)
{
#if defined(CAMERA_USING_ARDUCAM_FIFO)
    /* One complete JPEG table per mode; no crop/scale registers are involved. */
    int ret;

    if (framesize == FRAMESIZE_UXGA)
        ret = sensor_write_arducam_table(OV2640_1600x1200_JPEG);
    else if (framesize == FRAMESIZE_VGA)
        ret = sensor_write_arducam_table(OV2640_640x480_JPEG);
    else
        return -RT_EINVAL;
    if (ret != RT_EOK)
        return ret;
    rt_thread_mdelay(100);
    dev->runtime.framesize = framesize;
    return RT_EOK;
#else
    if ((unsigned int)framesize > FRAMESIZE_UXGA)
    {
        return -RT_EINVAL;
    }

    int ret = 0;
    uint16_t w;
    uint16_t h;
    aspect_ratio_t ratio = s_aspect_ratios[framesize];
    uint16_t max_x = ratio_table[ratio].max_x;
    uint16_t max_y = ratio_table[ratio].max_y;
    uint16_t offset_x = ratio_table[ratio].offset_x;
    uint16_t offset_y = ratio_table[ratio].offset_y;
    ov2640_sensor_mode_t mode = OV2640_MODE_UXGA;

    if (camera_sensor_get_resolution(framesize, &w, &h) != RT_EOK || w == 0 || h == 0)
    {
        return -RT_EINVAL;
    }

    if (framesize <= FRAMESIZE_CIF) {
        mode = OV2640_MODE_CIF;
        max_x /= 4;
        max_y /= 4;
        offset_x /= 4;
        offset_y /= 4;
        if(max_y > 296){
            max_y = 296;
        }
    } else if (framesize <= FRAMESIZE_SVGA) {
        mode = OV2640_MODE_SVGA;
        max_x /= 2;
        max_y /= 2;
        offset_x /= 2;
        offset_y /= 2;
    }

    ret = sensor_set_window(dev, mode, offset_x, offset_y, max_x, max_y, w, h);
    return ret;
#endif /* CAMERA_USING_ARDUCAM_FIFO */
}

/**
 * @brief Set JPEG compression quality
 * @param dev Pointer to sensor_device_t structure
 * @param quality Quality scale (0-63, lower = better quality, higher compression)
 * @return 0 on success, negative error code on failure
 */
static int sensor_set_quality(sensor_device_t *dev, int quality)
{
    int ret;

    if(quality < 0) quality = 0;
    if(quality > 63) quality = 63;
    ret = sensor_write_reg(BANK_DSP, QS, (uint8_t)quality);
    if(ret != 0) {
        return ret;
    }
    dev->runtime.quality = quality;
    return 0;
}



/** @brief Reset OV2640, load its initialization profile and read status. */
static int sensor_init(sensor_device_t *dev)
{
    int ret = sensor_reset();

    if (ret != RT_EOK)
    {
        return ret;
    }

#if defined(CAMERA_USING_ARDUCAM_FIFO)
    /* Reset, then the three common JPEG tables. The resolution table follows
     * from sensor_set_framesize(). */
    if ((sensor_write_arducam_table(OV2640_JPEG_INIT) != RT_EOK) ||
        (sensor_write_arducam_table(OV2640_YUV422) != RT_EOK) ||
        (sensor_write_arducam_table(OV2640_JPEG) != RT_EOK))
    {
        return -RT_EIO;
    }
#endif /* CAMERA_USING_ARDUCAM_FIFO */

    return sensor_init_status(dev);
}

/**
 * @brief Open the single OV2640 driver instance.
 *
 * Powers up SCCB + sensor + data bus and prepares single-shot/stream state.
 * The driver is currently single-instance, so repeated opens simply reuse the
 * same static state object.
 */
static int sensor_open(void)
{
    sensor_device_t *cam_dev = &s_device;
    int ret;

    if (cam_dev->runtime.is_open)
    {
        return RT_EOK;
    }

    rt_memset(cam_dev, 0, sizeof(*cam_dev));
    cam_dev->current_bank = (rt_uint8_t)BANK_MAX;
    s_active_device = cam_dev;

    if (CAMERA_XCLK_PIN >= 0 &&
        camera_xclk_start(CAMERA_XCLK_PIN,
                          g_hw_config.xclk_frequency_hz) != CAMERA_XCLK_OK)
    {
        LOG_E("XCLK start failed");
        s_active_device = RT_NULL;
        return -RT_ERROR;
    }

    ret = sensor_configure_arducam_sccb_pins();
    if (ret != RT_EOK)
    {
        LOG_E("unsupported SCCB bus: %s", CAMERA_SCCB_I2C_BUS_NAME);
        camera_xclk_stop(CAMERA_XCLK_PIN);
        s_active_device = RT_NULL;
        return ret;
    }

    ret = sccb_init(&g_hw_config.sccb);
    if (ret != RT_EOK)
    {
        LOG_E("SCCB init failed: %d", ret);
        camera_xclk_stop(CAMERA_XCLK_PIN);
        s_active_device = RT_NULL;
        return ret;
    }

    /*
     * The ArduCAM module only passes the camera's SCCB through once its
     * controller has been reset and probed, so the data bus (which performs
     * that handshake) has to come up before any sensor register access.
     */
    ret = camera_sensor_runtime_open(&cam_dev->runtime, &g_hw_config.runtime);
    if (ret != RT_EOK)
    {
        LOG_E("Camera runtime init failed: %d", ret);
        sccb_deinit();
        camera_xclk_stop(CAMERA_XCLK_PIN);
        s_active_device = RT_NULL;
        return ret;
    }

#if defined(CAMERA_USING_ARDUCAM_FIFO)
    /* After the handshake the OV2640 has to answer on SCCB; rejecting the module
     * here is clearer than failing later at capture time. */
    {
        uint8_t pid = 0U;
        uint8_t ver = 0U;

        if (sensor_read_reg(BANK_SENSOR, REG_PID, &pid) != RT_EOK ||
            sensor_read_reg(BANK_SENSOR, REG_VER, &ver) != RT_EOK ||
            (pid != 0x26U) || ((ver != 0x41U) && (ver != 0x42U)))
        {
            LOG_E("ArduCAM OV2640 id check failed (pid=0x%02x ver=0x%02x)", pid,
                  ver);
            camera_sensor_runtime_close(&cam_dev->runtime);
            sccb_deinit();
            camera_xclk_stop(CAMERA_XCLK_PIN);
            s_active_device = RT_NULL;
            return -RT_EIO;
        }
    }
#endif /* CAMERA_USING_ARDUCAM_FIFO */

    ret = sensor_init(cam_dev);
    if (ret != 0)
    {
        LOG_E("OV2640 init failed: %d", ret);
        camera_sensor_runtime_close(&cam_dev->runtime);
        sccb_deinit();
        camera_xclk_stop(CAMERA_XCLK_PIN);
        s_active_device = RT_NULL;
        return ret;
    }

    return RT_EOK;
}

/**
 * @brief Close the single OV2640 driver instance.
 *
 * Stops any ongoing transfer, deinitialises the active data bus backend and
 * SCCB, then destroys the frame semaphore.
 */
static int sensor_close(void)
{
    sensor_device_t *cam_dev = &s_device;
    int ret;

    if (!cam_dev->runtime.is_open)
    {
        return RT_EOK;
    }

    ret = camera_sensor_runtime_close(&cam_dev->runtime);
    if (ret != RT_EOK)
    {
        return ret;
    }
    sccb_deinit();
    camera_xclk_stop(CAMERA_XCLK_PIN);
    sensor_reset_bank_state();
    s_active_device = RT_NULL;

    LOG_I("Camera device closed");
    return RT_EOK;
}

/**
 * @brief Trigger a single-shot capture and block until one frame is ready.
 *
 * Starts a data bus transfer into @p buffer then waits on @c frame_sem.
 * The wait is bounded by the shared runtime timeout configuration; a timeout
 * returns 0.
 *
 * @param buffer is the destination frame buffer (must be DMA-accessible).
 * @param size   is the buffer size in bytes.
 *
 * @return Return the number of bytes captured; 0 on timeout or error.
 */
static rt_size_t sensor_capture(void *buffer, rt_size_t size)
{
    return camera_sensor_runtime_capture(&s_device.runtime, buffer, size);
}

static rt_size_t sensor_capture_timeout(void *buffer, rt_size_t size,
                                        uint32_t timeout_ms)
{
    return camera_sensor_runtime_capture_timeout(&s_device.runtime,
                                                 buffer, size, timeout_ms);
}

static rt_size_t sensor_capture_frames_timeout(void *buffer, rt_size_t size,
                                               uint32_t frame_count, uint32_t timeout_ms)
{
    return camera_sensor_runtime_capture_frames_timeout(&s_device.runtime,
                                                        buffer, size, frame_count, timeout_ms);
}

static int sensor_capture_async(void *buffer,
                                rt_size_t size,
                                camera_capture_done_callback_t callback,
                                void *context)
{
    return camera_sensor_runtime_capture_async(&s_device.runtime,
                                               buffer,
                                               size,
                                               callback,
                                               context);
}

/**
 * @brief Dispatch one internal @c OV2640_CMD_* request.
 *
 * Routes @p cmd to the appropriate sensor helper or bus-adapter operation.
 * This remains an internal implementation helper: the public integration
 * surface of the driver is the strong-typed @ref camera_device_ops_t table.
 *
 * @param cam_dev  Active OV2640 runtime instance.
 * @param cmd      One of the @c OV2640_CMD_* constants declared in ov2640.h.
 * @param args     Command argument; concrete type varies per command.
 *
 * @return RT_EOK on success; negative validation, sensor, or bus error on failure.
 */
static rt_err_t sensor_control(sensor_device_t *cam_dev, int cmd, void *args)
{
    int ret;
    
    if (cam_dev == RT_NULL)
    {
        return -RT_ERROR;
    }
    
    switch (cmd)
    {
        case CMD_SET_PIXFORMAT:
        {
            pixformat_t format = (pixformat_t)(rt_ubase_t)args;

            if (camera_sensor_runtime_busy(&cam_dev->runtime))
            {
                return -RT_EBUSY;
            }
            ret = sensor_set_pixformat(cam_dev, format);
            if (ret != RT_EOK)
            {
                return ret;
            }
            return camera_sensor_runtime_set_pixformat(&cam_dev->runtime, format);
        }
        
        case CMD_SET_FRAMESIZE:
        {
            framesize_t framesize = (framesize_t)(rt_ubase_t)args;

            if (camera_sensor_runtime_busy(&cam_dev->runtime))
            {
                return -RT_EBUSY;
            }
            if ((unsigned int)framesize > FRAMESIZE_UXGA)
            {
                return -RT_EINVAL;
            }
            ret = sensor_set_framesize(cam_dev, framesize);
            if (ret != RT_EOK)
            {
                return ret;
            }
            return camera_sensor_runtime_set_framesize(&cam_dev->runtime, framesize);
        }
        
        case CMD_SET_QUALITY:
        {
            if (camera_sensor_runtime_busy(&cam_dev->runtime))
            {
                return -RT_EBUSY;
            }
            int quality = (int)(rt_base_t)args;
            ret = sensor_set_quality(cam_dev, quality);
            return ret;
        }

        default:
            return -RT_EINVAL;
    }
}

/**
 * @brief Apply a new pixel format through the internal control helper.
 */
static int sensor_apply_pixformat(pixformat_t pixformat)
{
    int ret;

    if (!s_device.runtime.is_open)
    {
        return -RT_ERROR;
    }

    ret = sensor_control(&s_device, CMD_SET_PIXFORMAT,
                         (void *)(rt_ubase_t)pixformat);
    if (ret != RT_EOK)
    {
        LOG_E("Set pixel format %u failed: %d", (unsigned int)pixformat, ret);
    }
    return ret;
}

/**
 * @brief Apply a new frame size through the internal control helper.
 */
static int sensor_apply_framesize(framesize_t framesize)
{
    int ret;

    if (!s_device.runtime.is_open)
    {
        return -RT_ERROR;
    }

    ret = sensor_control(&s_device,
                         CMD_SET_FRAMESIZE,
                         (void *)(rt_ubase_t)framesize);
    if (ret != RT_EOK)
    {
        LOG_E("Set frame size %u failed: %d", (unsigned int)framesize, ret);
        return ret;
    }

    /* OV2640-specific settle window after format/size path updates. */
    rt_thread_mdelay(200);
    ret = sensor_set_bank(BANK_DSP);
    return ret;
}

/**
 * @brief Apply a new JPEG quality level through the internal control helper.
 */
static int sensor_apply_quality(uint8_t quality)
{
    if (!s_device.runtime.is_open)
    {
        return -RT_ERROR;
    }

    return sensor_control(&s_device,CMD_SET_QUALITY,(void *)(rt_ubase_t)quality);
}

/** @brief Mirror and flip together, preserving exposure and unrelated REG04 bits. */
static int sensor_apply_rotation(uint16_t degrees)
{
    const uint8_t mask = REG04_HFLIP_IMG | REG04_VFLIP_IMG | REG04_VREF_EN;
    uint8_t value;
    uint8_t actual;
    int ret;

    if (degrees != 0U && degrees != 180U)
    {
        return -RT_EINVAL;
    }
    if (!s_device.runtime.is_open)
    {
        return -RT_ERROR;
    }
    if (camera_sensor_runtime_busy(&s_device.runtime))
    {
        return -RT_EBUSY;
    }
    ret = sensor_read_reg(BANK_SENSOR, REG04, &value);
    if (ret != RT_EOK)
    {
        return ret;
    }
    /* VREF bit 0 follows vertical flip. AEC[1:0] shares this register. */
    value = (uint8_t)((value & (uint8_t)~mask) | (degrees == 180U ? mask : 0U));
    ret = sensor_write_raw(REG04, value);
    if (ret != RT_EOK)
    {
        return ret;
    }
    ret = sensor_read_reg(BANK_SENSOR, REG04, &actual);
    if (ret != RT_EOK)
    {
        return ret;
    }
    return (actual & mask) == (value & mask) ? RT_EOK : -RT_ERROR;
}

/**
 * @brief Start continuous streaming using handle-supplied buffers.
 */
static int sensor_start_stream(const camera_stream_start_args_t *args)
{
    return camera_sensor_runtime_start_stream(&s_device.runtime, args);
}

/**
 * @brief Stop continuous streaming and clear driver stream state.
 */
static int sensor_stop_stream(void)
{
    return camera_sensor_runtime_stop_stream(&s_device.runtime);
}

static rt_bool_t sensor_stream_frame_is_valid(const camera_stream_frame_t *frame)
{
    return camera_sensor_runtime_stream_frame_is_valid(&s_device.runtime, frame);
}
