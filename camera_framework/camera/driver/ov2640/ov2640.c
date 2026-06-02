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
#include "rtthread.h"
#include "rthw.h"
#include "drivers/pin.h"

#define DBG_TAG "ov2640"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <string.h>

static sensor_device_t  *s_active_device = RT_NULL;
static sensor_device_t  s_device;
static bus_adapter_t    *s_data_bus = RT_NULL;
static rt_bool_t        s_is_open = RT_FALSE;

static const pixformat_t g_camera_pixformats[] = {
    PIXFORMAT_JPEG,
    PIXFORMAT_RGB565,
    PIXFORMAT_YUV422,
};

static const framesize_t g_camera_framesizes[] = {
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
};

/* max_buffer_size: UXGA (1600×1200) in RGB565 = 1600*1200*2 bytes */
static const camera_capabilities_t g_camera_caps = {
    .pixformats      = g_camera_pixformats,
    .num_pixformats  = sizeof(g_camera_pixformats) / sizeof(g_camera_pixformats[0]),
    .framesizes      = g_camera_framesizes,
    .num_framesizes  = sizeof(g_camera_framesizes) / sizeof(g_camera_framesizes[0]),
    .max_buffer_size = 1600U * 1200U * 2U,
};

typedef struct {
    const char          *name;
    bus_capture_mode_t  default_mode;
    uint32_t            frame_timeout_ms;
} sensor_data_bus_config_t;

typedef struct {
    sccb_config_t            sccb;
    sensor_data_bus_config_t data_bus;
} sensor_hw_config_t;

static const sensor_hw_config_t g_hw_config = {
    .sccb = {
        .bus_name               = CAMERA_SCCB_I2C_BUS_NAME,
        .timeout_ms             = CAMERA_SCCB_TIMEOUT_MS,
        .max_hz                 = CAMERA_SCCB_MAX_HZ,
    },
    .data_bus = {
        .name                   = CAMERA_DATA_BUS_ADAPTER_NAME,
        .default_mode           = BUS_CAPTURE_MODE_JPEG,
        .frame_timeout_ms       = CAMERA_READ_TIMEOUT_MS,
    },
};

static rt_int32_t sensor_get_frame_timeout_ticks(void)
{
    uint32_t timeout_ms = g_hw_config.data_bus.frame_timeout_ms;

    if (timeout_ms == 0U)
    {
        return 0;
    }

    return (rt_int32_t)((timeout_ms * RT_TICK_PER_SECOND + 999U) / 1000U);
}

static void sensor_stop_active_capture(void)
{
    if (s_data_bus == RT_NULL)
    {
        return;
    }

    bus_adapter_stop(s_data_bus);
    bus_adapter_abort_capture(s_data_bus);
}

static int sensor_open(void);
static int sensor_close(void);
static int sensor_apply_pixformat(pixformat_t pixformat);
static int sensor_apply_framesize(framesize_t framesize);
static int sensor_apply_quality(uint8_t quality);
static rt_size_t sensor_capture(void *buffer, rt_size_t buffer_size);
static int sensor_capture_async(void *buffer,
                                rt_size_t buffer_size,
                                camera_capture_done_callback_t callback,
                                void *context);
static int sensor_start_stream(const camera_stream_start_args_t *args);
static int sensor_stop_stream(void);

const camera_device_ops_t ov2640_ops = {
    .capabilities   = &g_camera_caps,
    .open           = sensor_open,
    .close          = sensor_close,
    .set_pixformat  = sensor_apply_pixformat,
    .set_framesize  = sensor_apply_framesize,
    .set_quality    = sensor_apply_quality,
    .capture        = sensor_capture,
    .capture_async  = sensor_capture_async,
    .start_stream   = sensor_start_stream,
    .stop_stream    = sensor_stop_stream,
};

CAMERA_DRIVER_EXPORT(ov2640, &ov2640_ops);


/**
 * @brief Reset streaming state to inactive / all-zeroes.
 *
 * @param stream is a pointer to the stream state to reset.
 */
static void sensor_stream_reset(sensor_stream_state_t *stream)
{
    if (stream == RT_NULL)
    {
        return;
    }

    rt_memset(stream, 0, sizeof(*stream));
}


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

typedef struct {
        const uint16_t width;
        const uint16_t height;
        const aspect_ratio_t aspect_ratio;
} resolution_info_t;

static const resolution_info_t resolution[FRAMESIZE_INVALID] = {
    {   96,   96, ASPECT_RATIO_1X1   }, /* 96x96 */
    {  160,  120, ASPECT_RATIO_4X3   }, /* QQVGA */
    {  128,  128, ASPECT_RATIO_1X1   }, /* 128x128 */
    {  176,  144, ASPECT_RATIO_5X4   }, /* QCIF  */
    {  240,  176, ASPECT_RATIO_4X3   }, /* HQVGA */
    {  240,  240, ASPECT_RATIO_1X1   }, /* 240x240 */
    {  320,  240, ASPECT_RATIO_4X3   }, /* QVGA  */
    {  320,  320, ASPECT_RATIO_1X1   }, /* 320x320 */
    {  400,  296, ASPECT_RATIO_4X3   }, /* CIF   */
    {  480,  320, ASPECT_RATIO_3X2   }, /* HVGA  */
    {  640,  480, ASPECT_RATIO_4X3   }, /* VGA   */
    {  800,  600, ASPECT_RATIO_4X3   }, /* SVGA  */
    { 1024,  768, ASPECT_RATIO_4X3   }, /* XGA   */
    { 1280,  720, ASPECT_RATIO_16X9  }, /* HD    */
    { 1280, 1024, ASPECT_RATIO_5X4   }, /* SXGA  */
    { 1600, 1200, ASPECT_RATIO_4X3   }, /* UXGA  */
};




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

/**
 * @brief Switch the active register bank if needed.
 *
 * Skips the SCCB write when @p bank already matches the cached value.
 *
 * @param bank is the target bank (BANK_SENSOR or BANK_DSP).
 *
 * @return Return 0 on success; negative SCCB error on failure.
 */
static int sensor_set_bank(ov2640_bank_t bank)
{
    sensor_device_t *dev = s_active_device;
    rt_uint8_t cached = (dev != RT_NULL) ? dev->current_bank : (rt_uint8_t)BANK_MAX;
    if ((rt_uint8_t)bank == cached) return 0;
    int res = sccb_write(OV2640_ADDR, BANK_SEL, (uint8_t)bank);
    if(res) return res;
    if (dev != RT_NULL)
    {
        dev->current_bank = (rt_uint8_t)bank;
    }
    return 0;
}

/**
 * @brief  Write OV2640 single register
 * @param  bank: Bank ID
 * @param  reg: Register address
 * @param  data: Data to write
 * @return 0 on success, negative error code on failure
 */
static int sensor_write_reg(ov2640_bank_t bank, uint8_t reg, uint8_t data)
{
    int ret;
    ret = sensor_set_bank(bank);
    if(ret) return ret;
    ret = sccb_write(OV2640_ADDR, reg, data);
    if(ret) return ret;
    return 0;
}

/**
 * @brief  Write OV2640 multiple registers
 * @param  regs: Pointer to register array, ending with {0, 0}
 * @return RT_EOK on success, negative error code on failure
 */
static int sensor_write_regs(const uint8_t (*regs)[2])
{
    int ret;
    const uint8_t (*reg_ptr)[2] = regs;

    while(((*reg_ptr)[0] != 0) || ((*reg_ptr)[1] != 0))
    {
        ret = sccb_write(OV2640_ADDR, (*reg_ptr)[0], (*reg_ptr)[1]);
        if(ret) return ret;
        reg_ptr++;
    }
    return 0;
}

/**
 * @brief  Read OV2640 single register
 * @param  data: Pointer to store read data
 * @return register value, if read fails, return 0
 */
static uint8_t sensor_read_reg(uint8_t reg)
{
    return sccb_read(OV2640_ADDR, reg);
}

/**
 * @brief Get specific bits from OV2640 register, align to LSB by offset
 * @param reg: Register address
 * @param bank: Register bank
 * @param mask: Bit mask
 * @param offset: Bit offset
 * @param value: Pointer to store the extracted bits
 * @return aligned register bits, if read fails, return 0
 */
static uint8_t sensor_get_bits(uint8_t reg,ov2640_bank_t bank, uint8_t mask, uint32_t offset)
{
    int ret;
    uint8_t reg_value;
    ret = sensor_set_bank(bank);
    if(ret) return 0;
    reg_value = sensor_read_reg(reg);
    return (reg_value >> offset) & mask;

}


/**
 * @brief Reset OV2640 sensor to default settings.
 *
 * Sends the software reset command then writes the base CIF initialisation
 * register table.
 *
 * @param dev is a pointer to the sensor handle.
 *
 * @return Return 0 on success; negative error code on failure.
 */
static int sensor_reset(void)
{
    sensor_write_reg(BANK_SENSOR, COM7, COM7_SRST);
    rt_thread_mdelay(10);
    sensor_write_regs(ov2640_settings_cif);
    return 0;
}

/**
 * @brief Read all sensor registers and populate @p dev->status.
 *
 * Queries AEC, AGC, AWB, gain ceiling, gamma, lens correction and geometry
 * settings over SCCB and mirrors them into the in-memory status struct so
 * callers can inspect parameters without further I2C transactions.
 *
 * @param dev is a pointer to the sensor handle.
 *
 * @return Return 0 on success; -1 if a bank switch fails.
 */
static int sensor_init_status(sensor_device_t *dev)
{
    int ret = sensor_set_bank(BANK_DSP);
    if (ret != 0) return -1;
    dev->quality = sensor_read_reg(QS);
    dev->framesize = FRAMESIZE_UXGA;
    return 0;
}

/**
 * @brief Set pixel output format (RGB565, YUV422, JPEG, RAW8).
 *
 * @param dev      is a pointer to the sensor handle.
 * @param pixformat is the desired output format.
 *
 * @return Return 0 on success; -1 for an unsupported format.
 */
static int sensor_set_pixformat(sensor_device_t *dev, pixformat_t pixformat)
{
    int ret = 0;

    dev->pixformat = pixformat;

    switch(pixformat)
    {
        case PIXFORMAT_RGB565:
            sensor_write_regs(ov2640_settings_rgb565);
            break;
        case PIXFORMAT_YUV422:
            sensor_write_regs(ov2640_settings_yuv422);
            break;
        case PIXFORMAT_JPEG:
            sensor_write_regs(ov2640_settings_jpeg3);
            break;
        case PIXFORMAT_RAW8:
            sensor_write_regs(ov2640_settings_raw8);
            break;
        default:
            ret = -1;
    }

    return ret;
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
static int sensor_set_window(sensor_device_t *dev, ov2640_sensor_mode_t mode,int offset_x, int offset_y, int max_x, int max_y, int w, int h)
{
    int ret;
    const uint8_t (*regs)[2];
    ov2640_clk_t c;
    c.reserved = 0;

    max_x /= 4;
    max_y /= 4;
    w /= 4;
    h /= 4;
    uint8_t win_regs[][2] = {
        {BANK_SEL, BANK_DSP},
        {HSIZE, max_x & 0xFF},
        {VSIZE, max_y & 0xFF},
        {XOFFL, offset_x & 0xFF},
        {YOFFL, offset_y & 0xFF},
        {VHYX, ((max_y >> 1) & 0X80) | ((offset_y >> 4) & 0X70) | ((max_x >> 5) & 0X08) | ((offset_x >> 8) & 0X07)},
        {TEST, (max_x >> 2) & 0X80},
        {ZMOW, (w)&0xFF},
        {ZMOH, (h)&0xFF},
        {ZMHH, ((h>>6)&0x04)|((w>>8)&0x03)},
        {0, 0}
    };

    if (dev->pixformat == PIXFORMAT_JPEG) {
        c.clk_2x = 1;
        c.clk_div = 0;
        c.pclk_auto = 0;
        c.pclk_div =6;
        if(mode == OV2640_MODE_UXGA) {
            c.pclk_div = 24;
        }
    } else {
        c.clk_2x = 1;
        c.clk_div =3;
        c.pclk_auto = 1;
        c.pclk_div = 4;
        if (mode == OV2640_MODE_CIF) {
            c.clk_div = 3;
        } else if(mode == OV2640_MODE_UXGA) {
            c.pclk_div = 12;
        }
    }

    if (mode == OV2640_MODE_CIF) {
        regs = ov2640_settings_to_cif;
    } else if (mode == OV2640_MODE_SVGA) {
        regs = ov2640_settings_to_svga;
    } else {
        regs = ov2640_settings_to_uxga;
    }

    sensor_set_bank(BANK_DSP);
    sensor_write_reg(BANK_DSP, R_BYPASS, R_BYPASS_DSP_BYPAS);
    sensor_write_regs(regs);
    sensor_write_regs(win_regs);
    sensor_set_bank(BANK_SENSOR);
    sensor_write_reg(BANK_SENSOR, CLKRC, c.clk);
    sensor_set_bank(BANK_DSP);
    sensor_write_reg(BANK_DSP, R_DVP_SP, c.pclk);
    sensor_set_bank(BANK_DSP);
    sensor_write_reg(BANK_DSP, R_BYPASS, R_BYPASS_DSP_EN);

    rt_thread_mdelay(10);
    //required when changing resolution
    sensor_set_pixformat(dev, dev->pixformat);

    return 0;
}

/**
 * @brief Set frame size/resolution
 * @param dev Pointer to sensor_device_t structure
 * @param framesize Desired frame size (QVGA, VGA, SVGA, UXGA, etc.)
 * @return 0 on success, negative error code on failure
 */
static int sensor_set_framesize(sensor_device_t *dev, framesize_t framesize)
{
    if(framesize >= FRAMESIZE_INVALID) {
        return -1;
    }

    int ret = 0;
    uint16_t w = resolution[framesize].width;
    uint16_t h = resolution[framesize].height;
    aspect_ratio_t ratio = resolution[framesize].aspect_ratio;
    uint16_t max_x = ratio_table[ratio].max_x;
    uint16_t max_y = ratio_table[ratio].max_y;
    uint16_t offset_x = ratio_table[ratio].offset_x;
    uint16_t offset_y = ratio_table[ratio].offset_y;
    ov2640_sensor_mode_t mode = OV2640_MODE_UXGA;

    dev->framesize = framesize;

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
}

/**
 * @brief Set JPEG compression quality
 * @param dev Pointer to sensor_device_t structure
 * @param quality Quality scale (0-63, lower = better quality, higher compression)
 * @return 0 on success, negative error code on failure
 */
static int sensor_set_quality(sensor_device_t *dev, int quality)
{
    int ret = 0;

    if(quality < 0) quality = 0;
    if(quality > 63) quality = 63;
    ret = sensor_write_reg(BANK_DSP, QS, (uint8_t)quality);
    if(ret != 0) {
        return ret;
    }
    dev->quality = quality;
    return 0;
}



/**
 * @brief Reset OV2640 and load the initial register state into the handle.
 *
 * Issues a software reset (CIF init table) and reads the configuration
 * registers into @p dev->status. The sensor is left in JPEG/UXGA defaults.
 *
 * @param dev is a pointer to the sensor handle to initialise.
 *
 * @return Return 0 on success; negative error code on failure.
 */
static int sensor_init(sensor_device_t *dev)
{
    int ret;

    ret = sensor_reset();
    if (ret != 0)
    {
        return ret;
    }

    ret = sensor_init_status(dev);
    if (ret != 0)
    {
        return ret;
    }

    return 0;
}


/**
 * @brief Frame-ready callback registered with the bus adapter.
 *
 * For streaming mode, rotates ping-pong buffers and invokes the user
 * frame callback. For single-shot mode, releases @c frame_sem and halts
 * the bus. Always called from ISR / timer-thread context.
 *
 * @param buffer    is the frame buffer pointer.
 * @param length    is the captured frame length.
 * @param user      is the owning @ref sensor_device_t instance cast to @c void*.
 */
static void sensor_frame_ready_callback(void *buffer, uint32_t length, void *user)
{
    sensor_device_t *cam_dev = (sensor_device_t *)user;
    sensor_stream_state_t *stream = &cam_dev->stream;
    rt_size_t frame_size = (buffer != RT_NULL) ? length : 0;
    camera_capture_done_callback_t async_callback = RT_NULL;
    void *async_context = RT_NULL;

    rt_base_t level = rt_hw_interrupt_disable();
    cam_dev->last_frame_size = frame_size;
    camera_stream_frame_callback_t frame_callback = stream->frame_callback;
    void *callback_context = stream->callback_context;
    rt_hw_interrupt_enable(level);

    if (frame_callback != RT_NULL)
    {
        if (cam_dev->pixformat == PIXFORMAT_JPEG)
        {
            camera_stream_frame_t segment;
            segment.buffer = buffer;
            segment.buffer_size = stream->buffer_size;
            segment.frame_size = frame_size;
            segment.sequence = ++stream->sequence;
            segment.buffer_index = 0;
            frame_callback(callback_context, &segment);
            return;
        }

        rt_uint8_t completed_index = stream->active_buffer_index;
        camera_stream_frame_t frame;

        frame.buffer = stream->buffers[completed_index];
        frame.buffer_size = stream->buffer_size;
        frame.frame_size = frame_size;
        frame.sequence = ++stream->sequence;
        frame.buffer_index = completed_index;

        stream->active_buffer_index ^= 1U;

        if (bus_adapter_rearm_capture(s_data_bus,
                                      stream->buffers[stream->active_buffer_index],
                                      stream->buffer_size) != RT_EOK)
        {

            rt_base_t lock = rt_hw_interrupt_disable();
            sensor_stream_reset(stream);
            rt_hw_interrupt_enable(lock);
        }

        frame_callback(callback_context, &frame);

        return;
    }

    
    level = rt_hw_interrupt_disable();
    async_callback = cam_dev->async_capture.callback;
    async_context = cam_dev->async_capture.context;
    cam_dev->async_capture.callback = RT_NULL;
    cam_dev->async_capture.context = RT_NULL;
    cam_dev->async_capture.in_flight = RT_FALSE;
    rt_hw_interrupt_enable(level);
    

    if (async_callback != RT_NULL)
    {
        camera_handle_status_t status =
            (frame_size > 0) ? CAMERA_OK : CAMERA_ERRORTIMEOUT;
        async_callback(async_context, status, frame_size);
        sensor_stop_active_capture();
        return;
    }

    /* Notify blocking single-shot capture path via semaphore. */
    rt_sem_release(&cam_dev->frame_sem);
    sensor_stop_active_capture();
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

    if (s_is_open)
    {
        return RT_EOK;
    }

    rt_memset(cam_dev, 0, sizeof(*cam_dev));
    cam_dev->current_bank = (rt_uint8_t)BANK_MAX;
    s_active_device = cam_dev;
    sensor_stream_reset(&cam_dev->stream);


    rt_sem_init(&cam_dev->frame_sem, "cam_frm", 0, RT_IPC_FLAG_FIFO);

    ret = sccb_init(&g_hw_config.sccb);
    if (ret != RT_EOK)
    {
        LOG_E("SCCB init failed: %d", ret);
        return -RT_ERROR;
    }


    ret = sensor_init(cam_dev);
    if (ret != 0)
    {
        LOG_E("OV2640 init failed: %d", ret);
        sccb_deinit();
        return -RT_ERROR;
    }
    



    s_data_bus = RT_NULL;
    s_data_bus = bus_adapter_find(g_hw_config.data_bus.name);
    if (s_data_bus == RT_NULL)
    {
        LOG_E("Data bus adapter '%s' not registered", g_hw_config.data_bus.name);
        sccb_deinit();
        return -RT_ERROR;
    }
    if (s_data_bus->type != BUS_TYPE_DVP)
    {
        LOG_E("Data bus adapter '%s' type=%d is not supported by OV2640",
              g_hw_config.data_bus.name,
              (int)s_data_bus->type);
        sccb_deinit();
        s_data_bus = RT_NULL;
        return -RT_ERROR;
    }

  
    bus_adapter_set_frame_notify_callback(s_data_bus, sensor_frame_ready_callback, cam_dev);

    ret = bus_adapter_init(s_data_bus);
    if (ret != 0)
    {
        LOG_E("Bus adapter init failed: %d", ret);
        sccb_deinit();
        s_data_bus = RT_NULL;
        return -RT_ERROR;
    }

    s_is_open = RT_TRUE;
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

    if (!s_is_open)
    {
        return RT_EOK;
    }

    sensor_stream_reset(&cam_dev->stream);
    
    bus_adapter_abort_capture(s_data_bus);
    
    bus_adapter_stop(s_data_bus);
    
    bus_adapter_deinit(s_data_bus);
    s_data_bus = RT_NULL;
    
    sccb_deinit();

    sensor_reset_bank_state();
    
    rt_sem_detach(&cam_dev->frame_sem);
    s_active_device = RT_NULL;
    s_is_open = RT_FALSE;
    
    LOG_I("Camera device closed");
    return RT_EOK;
}

#ifndef OV2640_ENABLE_CAPTURE_TIMEOUT
#define OV2640_ENABLE_CAPTURE_TIMEOUT 1
#endif

#if OV2640_ENABLE_CAPTURE_TIMEOUT
static void sensor_dump_timeout_state(sensor_device_t *cam_dev,
                                      rt_size_t request_size,
                                      rt_err_t wait_result,
                                      rt_tick_t start_tick)
{
    rt_tick_t elapsed_ticks = rt_tick_get() - start_tick;
    unsigned long elapsed_ms = (unsigned long)elapsed_ticks * 1000UL / RT_TICK_PER_SECOND;

    LOG_W("Camera read timeout");
    LOG_W("  wait: result=%d elapsed=%lu ms req=%u",
        wait_result, elapsed_ms, (unsigned int)request_size);
    LOG_W("  sensor: pix=%d frame=%d quality=%u",
        (int)cam_dev->pixformat,
        (int)cam_dev->framesize,
        (unsigned int)cam_dev->quality);
    bus_adapter_dump_state(s_data_bus);
}
#endif /* OV2640_ENABLE_CAPTURE_TIMEOUT */

/**
 * @brief Trigger a single-shot capture and block until one frame is ready.
 *
 * Starts a data bus transfer into @p buffer then waits on @c frame_sem.
 * If @c OV2640_ENABLE_CAPTURE_TIMEOUT is set the wait is bounded by the
 * driver-owned frame timeout configuration; a timeout dumps diagnostic state
 * and returns 0.
 *
 * @param buffer is the destination frame buffer (must be DMA-accessible).
 * @param size   is the buffer size in bytes.
 *
 * @return Return the number of bytes captured; 0 on timeout or error.
 */
static rt_size_t sensor_capture(void *buffer, rt_size_t size)
{
    sensor_device_t *cam_dev = &s_device;
    rt_size_t frame_size = 0;
    rt_err_t result;
#if OV2640_ENABLE_CAPTURE_TIMEOUT
    rt_tick_t start_tick;
#endif

    if (!s_is_open)
    {
        return 0;
    }

    if (cam_dev->async_capture.in_flight)
    {
        return 0;
    }


    while (rt_sem_trytake(&cam_dev->frame_sem) == RT_EOK);
    cam_dev->last_frame_size = 0;
    
#if OV2640_ENABLE_CAPTURE_TIMEOUT
    start_tick = rt_tick_get();
#endif
    result = (rt_err_t)bus_adapter_start_capture(s_data_bus, buffer, size);
    if (result != RT_EOK)
    {
        LOG_E("Failed to start capture: %d", result);
        return 0;
    }

#if OV2640_ENABLE_CAPTURE_TIMEOUT

    result = rt_sem_take(&cam_dev->frame_sem, sensor_get_frame_timeout_ticks());
    
    if (result != RT_EOK)
    {
        sensor_dump_timeout_state(cam_dev, size, result, start_tick);
        sensor_stop_active_capture();
        return 0;
    }
#else
    /* Block until frame ready indefinitely */
    result = rt_sem_take(&cam_dev->frame_sem, RT_WAITING_FOREVER);
#endif
         



    rt_base_t level = rt_hw_interrupt_disable();
    frame_size = cam_dev->last_frame_size;
    rt_hw_interrupt_enable(level);
    
    
    return frame_size;
}

static int sensor_capture_async(void *buffer,
                                rt_size_t size,
                                camera_capture_done_callback_t callback,
                                void *context)
{
    sensor_device_t *cam_dev = &s_device;
    rt_err_t result;

    if (!s_is_open)
    {
        return -RT_ERROR;
    }

    if (buffer == RT_NULL || size == 0 || callback == RT_NULL)
    {
        return -RT_EINVAL;
    }

 
    rt_base_t level = rt_hw_interrupt_disable();
    if (cam_dev->stream.frame_callback != RT_NULL || cam_dev->async_capture.in_flight)
    {
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }
    cam_dev->async_capture.callback = callback;
    cam_dev->async_capture.context = context;
    cam_dev->async_capture.in_flight = RT_TRUE;
    rt_hw_interrupt_enable(level);

    result = (rt_err_t)bus_adapter_start_capture(s_data_bus, buffer, size);
    if (result != RT_EOK)
    {
        rt_base_t level = rt_hw_interrupt_disable();
        cam_dev->async_capture.callback = RT_NULL;
        cam_dev->async_capture.context = RT_NULL;
        cam_dev->async_capture.in_flight = RT_FALSE;
        rt_hw_interrupt_enable(level);
        return result;
    }

    return RT_EOK;
}

/**
 * @brief Map a sensor-level @c pixformat_t to a generic @c bus_capture_mode_t.
 *
 * Centralizes the pixel-format → bus-capture-mode translation so that the
 * mapping lives in exactly one place. Returns @c RT_TRUE on success and
 * writes the resolved mode through @p out_mode; returns @c RT_FALSE for
 * formats that have no bus equivalent.
 */
static rt_bool_t sensor_pixformat_to_bus_mode(pixformat_t format, bus_capture_mode_t *out_mode)
{
    switch (format)
    {
        case PIXFORMAT_JPEG:   *out_mode = BUS_CAPTURE_MODE_JPEG;   return RT_TRUE;
        case PIXFORMAT_RGB565: *out_mode = BUS_CAPTURE_MODE_RGB565; return RT_TRUE;
        case PIXFORMAT_YUV422: *out_mode = BUS_CAPTURE_MODE_YUV422; return RT_TRUE;
        case PIXFORMAT_RAW8:   *out_mode = BUS_CAPTURE_MODE_RAW;    return RT_TRUE;
        default:                                                    return RT_FALSE;
    }
}

/** @brief Return bytes per pixel for fixed-size non-JPEG formats. */
static uint32_t sensor_bytes_per_pixel(pixformat_t format)
{
    switch (format)
    {
        case PIXFORMAT_RGB565:
        case PIXFORMAT_YUV422:
            return 2U;
        case PIXFORMAT_RAW8:
            return 1U;
        default:
            return 0U;
    }
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
 * @return Return RT_EOK on success; @c -RT_EINVAL for unknown commands or
 *         invalid arguments; @c -RT_ERROR if the underlying operation fails.
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
            if (cam_dev->async_capture.in_flight || cam_dev->stream.frame_callback != RT_NULL)
            {
                return -RT_EBUSY;
            }
            pixformat_t format = (pixformat_t)(rt_ubase_t)args;
            pixformat_t old_format = cam_dev->pixformat;
            ret = sensor_set_pixformat(cam_dev, format);
            if (ret != 0)
            {
                return -RT_ERROR;
            }


            bus_capture_mode_t old_mode;
            bus_capture_mode_t new_mode;
            if (!sensor_pixformat_to_bus_mode(old_format, &old_mode))
            {
                old_mode = g_hw_config.data_bus.default_mode;
            }
            if (!sensor_pixformat_to_bus_mode(format, &new_mode))
            {
                return -RT_EINVAL;
            }


            if (old_mode != new_mode)
            {
                bus_adapter_stop(s_data_bus);
                if (bus_adapter_set_mode(s_data_bus, new_mode) != BUS_OK)
                {
                    LOG_E("Failed to set bus mode");
                    return -RT_ERROR;
                }
                ret = bus_adapter_start(s_data_bus);
                if (ret != 0)
                {
                    LOG_E("Failed to restart bus after mode change");
                    return -RT_ERROR;
                }
            }

            return RT_EOK;
        }
        
        case CMD_SET_FRAMESIZE:
        {
            uint32_t line_bytes;
            uint32_t pingpong_size;
            uint32_t bytes_per_pixel;
            if (cam_dev->async_capture.in_flight || cam_dev->stream.frame_callback != RT_NULL)
            {
                return -RT_EBUSY;
            }
            if((framesize_t)(rt_ubase_t)args >= FRAMESIZE_INVALID)
            {
                return -RT_EINVAL;
            }
            framesize_t framesize = (framesize_t)(rt_ubase_t)args;
            ret = sensor_set_framesize(cam_dev, framesize);
            if (ret != 0)
            {
                return -RT_ERROR;
            }

            if (cam_dev->pixformat == PIXFORMAT_JPEG)
            {
                return RT_EOK;
            }

            bytes_per_pixel = sensor_bytes_per_pixel(cam_dev->pixformat);
            if (bytes_per_pixel == 0U)
            {
                return -RT_EINVAL;
            }

            line_bytes = (uint32_t)resolution[framesize].width * bytes_per_pixel;
            pingpong_size = line_bytes * 2U;
            ret = bus_adapter_set_pingpong_size(s_data_bus, pingpong_size);
            return (ret == BUS_OK || ret == RT_EOK) ? RT_EOK : ret;
        }
        
        case CMD_SET_QUALITY:
        {
            if (cam_dev->async_capture.in_flight || cam_dev->stream.frame_callback != RT_NULL)
            {
                return -RT_EBUSY;
            }
            int quality = (int)(rt_base_t)args;
            ret = sensor_set_quality(cam_dev, quality);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }

        case CMD_START_STREAM:
        {
            camera_stream_start_args_t *stream_args = (camera_stream_start_args_t *)args;

            if (stream_args == RT_NULL ||
                stream_args->buffers[0] == RT_NULL ||
                stream_args->buffers[1] == RT_NULL ||
                stream_args->buffer_size == 0)
            {
                return -RT_EINVAL;
            }

            if (cam_dev->async_capture.in_flight)
            {
                return -RT_EBUSY;
            }

            cam_dev->stream.buffers[0] = stream_args->buffers[0];
            cam_dev->stream.buffers[1] = stream_args->buffers[1];
            cam_dev->stream.buffer_size = stream_args->buffer_size;
            cam_dev->stream.active_buffer_index = 0;
            cam_dev->stream.sequence = 0;

            {
                rt_base_t level = rt_hw_interrupt_disable();
                cam_dev->stream.callback_context = stream_args->callback_context;
                cam_dev->stream.frame_callback   = stream_args->frame_callback;
                rt_hw_interrupt_enable(level);
            }

            ret = bus_adapter_start_capture(s_data_bus,
                                            cam_dev->stream.buffers[cam_dev->stream.active_buffer_index],
                                            cam_dev->stream.buffer_size);
            if (ret != RT_EOK)
            {
                rt_base_t level = rt_hw_interrupt_disable();
                sensor_stream_reset(&cam_dev->stream);
                rt_hw_interrupt_enable(level);
                return -RT_ERROR;
            }

            return RT_EOK;
        }

        case CMD_STOP_STREAM:
        {
            {
                rt_base_t level = rt_hw_interrupt_disable();
                cam_dev->stream.frame_callback = RT_NULL;
                cam_dev->stream.callback_context = RT_NULL;
                sensor_stream_reset(&cam_dev->stream);
                rt_hw_interrupt_enable(level);
            }
            sensor_stop_active_capture();
            return RT_EOK;
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
    if (!s_is_open)
    {
        return -RT_ERROR;
    }

    return sensor_control(&s_device,
                          CMD_SET_PIXFORMAT,
                          (void *)(rt_ubase_t)pixformat);
}

/**
 * @brief Apply a new frame size through the internal control helper.
 */
static int sensor_apply_framesize(framesize_t framesize)
{
    int ret;

    if (!s_is_open)
    {
        return -RT_ERROR;
    }

    ret = sensor_control(&s_device,
                         CMD_SET_FRAMESIZE,
                         (void *)(rt_ubase_t)framesize);
    if (ret != RT_EOK)
    {
        return ret;
    }

    /* OV2640-specific settle window after format/size path updates. */
    rt_thread_mdelay(200);
    return RT_EOK;
}

/**
 * @brief Apply a new JPEG quality level through the internal control helper.
 */
static int sensor_apply_quality(uint8_t quality)
{
    if (!s_is_open)
    {
        return -RT_ERROR;
    }

    return sensor_control(&s_device,CMD_SET_QUALITY,(void *)(rt_ubase_t)quality);
}

/**
 * @brief Start continuous streaming using handle-supplied buffers.
 */
static int sensor_start_stream(const camera_stream_start_args_t *args)
{
    if (!s_is_open)
    {
        return -RT_ERROR;
    }
    return sensor_control(&s_device,CMD_START_STREAM,(void *)args);
}

/**
 * @brief Stop continuous streaming and clear driver stream state.
 */
static int sensor_stop_stream(void)
{
    if (!s_is_open)
    {
        return -RT_ERROR;
    }

    return sensor_control(&s_device,CMD_STOP_STREAM,RT_NULL);
}

