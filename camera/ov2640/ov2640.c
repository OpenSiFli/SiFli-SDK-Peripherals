/**
 * @file    ov2640.c
 * @brief   OV2640 camera sensor driver and RT-Thread device interface
 *
 * This module implements the OV2640 sensor initialization, register
 * configuration, image parameter control, and RT-Thread standard
 * device driver interface (open/read/close/control).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#include "ov2640.h"
#include "ov2640_regs.h"
#include "ov2640_settings.h"
#include "rtthread.h"

#define DBG_TAG "ov2640"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* Thread safety: Mutex for protecting sensor operations */
static struct rt_mutex ov2640_mutex;
static rt_bool_t ov2640_mutex_initialized = RT_FALSE;

/* GPTIM2 handle for XCLK generation */
static GPT_HandleTypeDef xclk_gptim;
static rt_bool_t xclk_initialized = RT_FALSE;

/**
 * @brief Initialize OV2640 mutex (called once)
 */
static void ov2640_mutex_init(void)
{
    if (!ov2640_mutex_initialized)
    {
        rt_mutex_init(&ov2640_mutex, "ov2640_lock", RT_IPC_FLAG_PRIO);
        ov2640_mutex_initialized = RT_TRUE;
    }
}

/**
 * @brief Lock OV2640 mutex for thread-safe access
 * @return RT_EOK on success
 */
static rt_err_t ov2640_lock(void)
{
    if (!ov2640_mutex_initialized)
    {
        ov2640_mutex_init();
    }
    return rt_mutex_take(&ov2640_mutex, RT_WAITING_FOREVER);
}

/**
 * @brief Unlock OV2640 mutex
 */
static void ov2640_unlock(void)
{
    if (ov2640_mutex_initialized)
    {
        rt_mutex_release(&ov2640_mutex);
    }
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

/* Register W/R operation */

/**
 * @brief  Set OV2640 register bank
 * @param  bank: Bank ID
 * @return 0 on success, negative error code on failure
 */
static volatile ov2640_bank_t current_bank = BANK_MAX;

/**
 * @brief Reset sensor bank state (call during deinit)
 */
static void ov2640_reset_bank_state(void)
{
    current_bank = BANK_MAX;
}

static int ov2640_set_bank(ov2640_bank_t bank)
{
    if(bank == current_bank) return 0;
    int res = sccb_write(OV2640_ADDR, BANK_SEL, (uint8_t)bank);
    if(res) return res;
    current_bank = bank;
    return 0;
}

/**
 * @brief  Write OV2640 single register
 * @param  bank: Bank ID
 * @param  reg: Register address
 * @param  data: Data to write
 * @return 0 on success, negative error code on failure
 */
static int ov2640_write_reg(ov2640_bank_t bank, uint8_t reg, uint8_t data)
{
    int ret;
    ret = ov2640_set_bank(bank);
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
static int ov2640_write_regs(const uint8_t (*regs)[2])
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
static uint8_t ov2640_read_reg(uint8_t reg)
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
static uint8_t ov2640_get_bits(uint8_t reg,ov2640_bank_t bank, uint8_t mask, uint32_t offset)
{
    int ret;
    uint8_t reg_value;
    ret = ov2640_set_bank(bank);
    if(ret) return 0;
    reg_value = ov2640_read_reg(reg);
    return (reg_value >> offset) & mask;

}

/* Sensor functions implementation */

/**
 * @brief Reset OV2640 sensor to default settings
 * @param dev Pointer to ov2640_t structure
 * @return 0 on success, negative error code on failure
 */
static int ov2640_reset(ov2640_t *dev)
{
    int ret;
    
    ov2640_lock();
    // ov2640_set_bank(BANK_SENSOR);
    ov2640_write_reg(BANK_SENSOR, COM7, COM7_SRST);
    rt_thread_mdelay(10);
    ov2640_write_regs(ov2640_settings_cif);
    ov2640_unlock();
    return 0;
}

/**
 * @brief Initialize camera status by reading all configuration registers
 * @param dev Pointer to ov2640_t structure
 * @return 0 on success, negative error code on failure
 */
static int ov2640_init_status(ov2640_t *dev)
{
    int ret;
    uint8_t reg_value_h, reg_value_m, reg_value_l;
    uint8_t temp_value;
    
    ov2640_lock();
    
    dev->status.brightness = 0;
    dev->status.contrast = 0;
    dev->status.saturation = 0;
    dev->status.ae_level = 0;
    dev->status.special_effect = 0;
    dev->status.wb_mode = 0;

    // Read AEC value (16-bit value split across 3 registers)
    // AEC[15:10] from REG45, AEC[9:2] from AEC, AEC[1:0] from REG04
    reg_value_h = ov2640_get_bits(REG45, BANK_SENSOR, REG45_AEC_MASK, REG45_AEC_OFFSET);
    ret = ov2640_set_bank(BANK_SENSOR);
    if(ret != 0) return -1;
    reg_value_m = ov2640_read_reg(AEC);
    reg_value_l = ov2640_get_bits(REG04, BANK_SENSOR, REG04_AEC_MASK, REG04_AEC_OFFSET);
    dev->status.aec_value = ((uint16_t)reg_value_h << 10) | ((uint16_t)reg_value_m << 2) | reg_value_l; // 0 - 1200
    
    // Read quality
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) return -1;
    temp_value = ov2640_read_reg(QS);
    dev->status.quality = temp_value;
    
    // Read gain ceiling
    dev->status.gainceiling = ov2640_get_bits(COM9, BANK_SENSOR, COM9_GAINCEILING_MASK, COM9_GAINCEILING_OFFSET);
    
    // Read AWB
    dev->status.awb = ov2640_get_bits(CTRL1, BANK_DSP, CTRL1_AWB_MASK, CTRL1_AWB_OFFSET);
    
    // Read AWB gain
    dev->status.awb_gain = ov2640_get_bits(CTRL1, BANK_DSP, CTRL1_AWB_GAIN_MASK, CTRL1_AWB_GAIN_OFFSET);
    
    // Read AEC
    dev->status.aec = ov2640_get_bits(COM8, BANK_SENSOR, COM8_AEC_MASK, COM8_AEC_OFFSET);
    
    // Read AEC2
    dev->status.aec2 = ov2640_get_bits(CTRL0, BANK_DSP, CTRL0_AEC2_MASK, CTRL0_AEC2_OFFSET);
    
    // Read AGC
    dev->status.agc = ov2640_get_bits(COM8, BANK_SENSOR, COM8_AGC_MASK, COM8_AGC_OFFSET);
    
    // Read BPC
    dev->status.bpc = ov2640_get_bits(CTRL3, BANK_DSP, CTRL3_BPC_MASK, CTRL3_BPC_OFFSET);
    
    // Read WPC
    dev->status.wpc = ov2640_get_bits(CTRL3, BANK_DSP, CTRL3_WPC_MASK, CTRL3_WPC_OFFSET);
    
    // Read RAW GMA
    dev->status.raw_gma = ov2640_get_bits(CTRL1, BANK_DSP, CTRL1_RAW_GMA_MASK, CTRL1_RAW_GMA_OFFSET);
    
    // Read LENC
    dev->status.lenc = ov2640_get_bits(CTRL1, BANK_DSP, CTRL1_LENC_MASK, CTRL1_LENC_OFFSET);
    
    // Read horizontal mirror
    dev->status.hmirror = ov2640_get_bits(REG04, BANK_SENSOR, REG04_HMIRROR_MASK, REG04_HMIRROR_OFFSET);
    
    // Read vertical flip
    dev->status.vflip = ov2640_get_bits(REG04, BANK_SENSOR, REG04_VFLIP_MASK, REG04_VFLIP_OFFSET);
    
    // Read DCW
    dev->status.dcw = ov2640_get_bits(CTRL2, BANK_DSP, CTRL2_DCW_MASK, CTRL2_DCW_OFFSET);
    
    // Read color bar
    dev->status.colorbar = ov2640_get_bits(COM7, BANK_SENSOR, COM7_COLORBAR_MASK, COM7_COLORBAR_OFFSET);
    
    // Sharpness and denoise are not supported
    dev->status.sharpness = 0;
    dev->status.denoise = 0;
    
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set pixel format (RGB565, YUV422, JPEG, etc.)
 * @param dev Pointer to ov2640_t structure
 * @param pixformat Desired pixel format
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_pixformat(ov2640_t *dev, pixformat_t pixformat)
{
    int ret = 0;
    
    ov2640_lock();
    dev->pixformat = pixformat;

    switch(pixformat)
    {
        case PIXFORMAT_RGB565:
            ov2640_write_regs(ov2640_settings_rgb565);
            break;
        case PIXFORMAT_YUV422:
            ov2640_write_regs(ov2640_settings_yuv422);
            break;
        case PIXFORMAT_JPEG:
            ov2640_write_regs(ov2640_settings_jpeg3);
            break;
        case PIXFORMAT_RAW8:
            // Unsupported Now
            ret = -1;
            break;
        default:
            ret = -1;
    }
    ov2640_unlock();

    return ret;
}

/**
 * @brief Configure sensor window (crop and scale) settings
 * @param dev Pointer to ov2640_t structure
 * @param mode Sensor mode (CIF, SVGA, UXGA)
 * @param offset_x Horizontal offset
 * @param offset_y Vertical offset
 * @param max_x Maximum horizontal size
 * @param max_y Maximum vertical size
 * @param w Output width
 * @param h Output height
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_window(ov2640_t *dev, ov2640_sensor_mode_t mode,int offset_x, int offset_y, int max_x, int max_y, int w, int h)
{
    int ret;
    const uint8_t (*regs)[2];
    ov2640_clk_t c;
    c.reserved = 0;
    
    ov2640_lock();

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
        c.clk_2x = 0;
        c.clk_div = 0;
        c.pclk_auto = 0;
        c.pclk_div = 12;
        if(mode == OV2640_MODE_UXGA) {
            c.pclk_div = 24;
        }
    } else {
        c.clk_2x = 1;
        c.clk_div = 7;
        c.pclk_auto = 1;
        c.pclk_div = 8;
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

    ov2640_set_bank(BANK_DSP);
    ov2640_write_reg(BANK_DSP, R_BYPASS, R_BYPASS_DSP_BYPAS);
    ov2640_write_regs(regs);
    ov2640_write_regs(win_regs);
    ov2640_set_bank(BANK_SENSOR);
    ov2640_write_reg(BANK_SENSOR, CLKRC, c.clk);
    ov2640_set_bank(BANK_DSP);
    ov2640_write_reg(BANK_DSP, R_DVP_SP, c.pclk);
    ov2640_set_bank(BANK_DSP);
    ov2640_write_reg(BANK_DSP, R_BYPASS, R_BYPASS_DSP_EN);

    rt_thread_mdelay(10);
    //required when changing resolution
    /* Note: ov2640_set_pixformat will acquire its own lock, so we need to release first */
    ov2640_unlock();
    ov2640_set_pixformat(dev, dev->pixformat);

    return 0;
}

/**
 * @brief Set frame size/resolution
 * @param dev Pointer to ov2640_t structure
 * @param framesize Desired frame size (QVGA, VGA, SVGA, UXGA, etc.)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_framesize(ov2640_t *dev, framesize_t framesize)
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

    dev->status.framesize = framesize;

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

    ret = ov2640_set_window(dev, mode, offset_x, offset_y, max_x, max_y, w, h);
    return ret;
}

/**
 * @brief Set image contrast level
 * @param dev Pointer to ov2640_t structure
 * @param level Contrast level (-2 to +2, 0 is default)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_contrast(ov2640_t *dev, int level)
{
    int ret = 0;
    
    ov2640_lock();
    level += 2; // Map -2~2 to 0~4
    ret = ov2640_write_reg(BANK_SENSOR, 0x81, (uint8_t)(0x14 + level * 4));
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.contrast = level - 2;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set image brightness level
 * @param dev Pointer to ov2640_t structure
 * @param level Brightness level (-2 to +2, 0 is default)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_brightness(ov2640_t *dev, int level)
{
    int ret = 0;
    
    ov2640_lock();
    if(level < -2) level = -2;
    if(level > 2) level = 2;
    level += 2; // Map -2~2 to 0~4
    ret = ov2640_write_reg(BANK_SENSOR, 0x9B, (uint8_t)(level * 0x20));
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.brightness = level - 2;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set image color saturation level
 * @param dev Pointer to ov2640_t structure
 * @param level Saturation level (-2 to +2, 0 is default)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_saturation(ov2640_t *dev, int level)
{
    int ret = 0;
    
    ov2640_lock();
    if(level < -2) level = -2;
    if(level > 2) level = 2;
    level += 2; // Map -2~2 to 0~4
    ret = ov2640_write_reg(BANK_DSP, 0xD9, (uint8_t)(0x40 + level * 0x10));
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.saturation = level - 2;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set image sharpness (not supported by OV2640)
 * @param dev Pointer to ov2640_t structure
 * @param level Sharpness level (ignored)
 * @return 0 (always success, feature not supported)
 */
static int ov2640_set_sharpness(ov2640_t *dev, int level)
{
    // OV2640 doesn't support sharpness adjustment
    dev->status.sharpness = 0;
    return 0;
}

/**
 * @brief Set denoise level (not supported by OV2640)
 * @param dev Pointer to ov2640_t structure
 * @param level Denoise level (ignored)
 * @return 0 (always success, feature not supported)
 */
static int ov2640_set_denoise(ov2640_t *dev, int level)
{
    // OV2640 doesn't support direct denoise level adjustment
    dev->status.denoise = 0;
    return 0;
}

/**
 * @brief Set AGC (Automatic Gain Control) gain ceiling
 * @param dev Pointer to ov2640_t structure
 * @param gainceiling Gain ceiling value (2x, 4x, 8x, 16x, 32x, 64x, 128x)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_gainceiling(ov2640_t *dev, gainceiling_t gainceiling)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(COM9);
    reg_value = (reg_value & ~COM9_GAINCEILING_MASK) | ((gainceiling << COM9_GAINCEILING_OFFSET) & COM9_GAINCEILING_MASK);
    ret = ov2640_write_reg(BANK_SENSOR, COM9, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.gainceiling = gainceiling;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set JPEG compression quality
 * @param dev Pointer to ov2640_t structure
 * @param quality Quality scale (0-63, lower = better quality, higher compression)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_quality(ov2640_t *dev, int quality)
{
    int ret = 0;
    
    ov2640_lock();
    // Quality scale: 0-63 (lower = better quality, higher compression)
    if(quality < 0) quality = 0;
    if(quality > 63) quality = 63;
    ret = ov2640_write_reg(BANK_DSP, QS, (uint8_t)quality);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.quality = quality;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable color bar test pattern
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_colorbar(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(COM7);
    if(enable) {
        reg_value |= COM7_COLORBAR_MASK;
    } else {
        reg_value &= ~COM7_COLORBAR_MASK;
    }
    ret = ov2640_write_reg(BANK_SENSOR, COM7, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.colorbar = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable automatic white balance (AWB)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable AWB, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_whitebal(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL1);
    if(enable) {
        reg_value |= CTRL1_AWB_MASK;
    } else {
        reg_value &= ~CTRL1_AWB_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL1, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.awb = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable automatic gain control (AGC)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable AGC, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_gain_ctrl(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(COM8);
    if(enable) {
        reg_value |= COM8_AGC_MASK;
    } else {
        reg_value &= ~COM8_AGC_MASK;
    }
    ret = ov2640_write_reg(BANK_SENSOR, COM8, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.agc = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable automatic exposure control (AEC)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable AEC, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_exposure_ctrl(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(COM8);
    if(enable) {
        reg_value |= COM8_AEC_MASK;
    } else {
        reg_value &= ~COM8_AEC_MASK;
    }
    ret = ov2640_write_reg(BANK_SENSOR, COM8, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.aec = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable horizontal mirror (flip image horizontally)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable horizontal mirror, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_hmirror(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(REG04);
    if(enable) {
        reg_value |= REG04_HMIRROR_MASK;
    } else {
        reg_value &= ~REG04_HMIRROR_MASK;
    }
    ret = ov2640_write_reg(BANK_SENSOR, REG04, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.hmirror = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable vertical flip (flip image vertically)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable vertical flip, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_vflip(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(REG04);
    if(enable) {
        reg_value |= REG04_VFLIP_MASK;
    } else {
        reg_value &= ~REG04_VFLIP_MASK;
    }
    ret = ov2640_write_reg(BANK_SENSOR, REG04, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.vflip = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable AEC2 (enhanced automatic exposure control)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable AEC2, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_aec2(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL0);
    if(enable) {
        reg_value |= CTRL0_AEC2_MASK;
    } else {
        reg_value &= ~CTRL0_AEC2_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL0, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.aec2 = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable AWB gain control
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable AWB gain, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_awb_gain(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL1);
    if(enable) {
        reg_value |= CTRL1_AWB_GAIN_MASK;
    } else {
        reg_value &= ~CTRL1_AWB_GAIN_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL1, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.awb_gain = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set manual AGC gain value
 * @param dev Pointer to ov2640_t structure
 * @param gain Gain value (0-30)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_agc_gain(ov2640_t *dev, int gain)
{
    int ret = 0;
    
    ov2640_lock();
    // AGC gain: 0-30 maps to register value
    if(gain < 0) gain = 0;
    if(gain > 30) gain = 30;
    ret = ov2640_write_reg(BANK_SENSOR, GAIN, (uint8_t)gain);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.agc_gain = gain;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set manual AEC exposure value
 * @param dev Pointer to ov2640_t structure
 * @param value Exposure value (0-1200)
 * @return 0 on success, negative error code on failure
 * @note Value is split across 3 registers: REG45[5:0], AEC[7:0], REG04[1:0]
 */
static int ov2640_set_aec_value(ov2640_t *dev, int value)
{
    int ret = 0;
    // AEC value: 0-1200, split across 3 registers
    // AEC[15:10] -> REG45[5:0]
    // AEC[9:2] -> AEC[7:0]
    // AEC[1:0] -> REG04[1:0]
    if(value < 0) value = 0;
    if(value > 1200) value = 1200;
    
    ov2640_lock();
    
    uint8_t reg45_val = ov2640_read_reg(REG45);
    reg45_val = (reg45_val & ~REG45_AEC_MASK) | ((value >> 10) & REG45_AEC_MASK);
    ret = ov2640_write_reg(BANK_SENSOR, REG45, reg45_val);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    
    ret = ov2640_write_reg(BANK_SENSOR, AEC, (uint8_t)((value >> 2) & 0xFF));
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    
    uint8_t reg04_val = ov2640_read_reg(REG04);
    reg04_val = (reg04_val & ~REG04_AEC_MASK) | (value & REG04_AEC_MASK);
    ret = ov2640_write_reg(BANK_SENSOR, REG04, reg04_val);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    
    dev->status.aec_value = value;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set special image effect
 * @param dev Pointer to ov2640_t structure
 * @param effect Effect mode:
 *        - 0: Normal
 *        - 1: Negative
 *        - 2: Black & White
 *        - 3: Reddish
 *        - 4: Greenish
 *        - 5: Bluish
 *        - 6: Sepia
 * @return 0 on success, -1 if effect is out of range
 */
static int ov2640_set_special_effect(ov2640_t *dev, int effect)
{
    int ret = 0;
    // Special effects: 0=Normal, 1=Negative, 2=B&W, 3=Reddish, 4=Greenish, 5=Bluish, 6=Sepia
    const uint8_t effects[][3] = {
        {0x00, 0x80, 0x80}, // Normal
        {0x40, 0x80, 0x80}, // Negative
        {0x18, 0x80, 0x80}, // B&W
        {0x18, 0x40, 0xC0}, // Reddish
        {0x18, 0x40, 0x40}, // Greenish
        {0x18, 0xA0, 0x40}, // Bluish
        {0x18, 0x40, 0xA0}, // Sepia
    };
    
    if(effect < 0 || effect > 6) return -1;
    
    ov2640_lock();
    
    ret = ov2640_write_reg(BANK_DSP, 0xD7, effects[effect][0]);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, 0xD8, effects[effect][1]);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, 0xD9, effects[effect][2]);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    
    dev->status.special_effect = effect;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Set white balance mode
 * @param dev Pointer to ov2640_t structure
 * @param mode White balance mode:
 *        - 0: Auto (AWB enabled)
 *        - 1: Sunny
 *        - 2: Cloudy
 *        - 3: Office
 *        - 4: Home
 * @return 0 on success, -1 if mode is out of range
 */
static int ov2640_set_wb_mode(ov2640_t *dev, int mode)
{
    int ret = 0;
    // WB modes: 0=Auto, 1=Sunny, 2=Cloudy, 3=Office, 4=Home
    const uint8_t wb_modes[][3] = {
        {0x00, 0x00, 0x00}, // Auto (AWB enabled)
        {0x52, 0x41, 0x00}, // Sunny
        {0x65, 0x41, 0x00}, // Cloudy
        {0x45, 0x51, 0x00}, // Office
        {0x42, 0x51, 0x00}, // Home
    };
    
    if(mode < 0 || mode > 4) return -1;
    
    if(mode == 0) {
        // Enable AWB (set_whitebal will acquire its own lock)
        ret = ov2640_set_whitebal(dev, 1);
    } else {
        // Disable AWB and set manual WB
        ret = ov2640_set_whitebal(dev, 0);
        if(ret != 0) return ret;
        
        ov2640_lock();
        ret = ov2640_write_reg(BANK_DSP, 0xCC, wb_modes[mode][0]);
        if(ret != 0) {
            ov2640_unlock();
            return ret;
        }
        ret = ov2640_write_reg(BANK_DSP, 0xCD, wb_modes[mode][1]);
        if(ret != 0) {
            ov2640_unlock();
            return ret;
        }
        ret = ov2640_write_reg(BANK_DSP, 0xCE, wb_modes[mode][2]);
        if(ret != 0) {
            ov2640_unlock();
            return ret;
        }
        ov2640_unlock();
    }
    
    dev->status.wb_mode = mode;
    return 0;
}

/**
 * @brief Set automatic exposure level
 * @param dev Pointer to ov2640_t structure
 * @param level AE level (-2 to +2, 0 is default)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_ae_level(ov2640_t *dev, int level)
{
    int ret = 0;
    // AE level: -2 to +2
    level += 2; // Map to 0~4
    if(level < 0) level = 0;
    if(level > 4) level = 4;
    
    const uint8_t ae_levels[5] = {0x40, 0x30, 0x24, 0x18, 0x10};
    
    ov2640_lock();
    ret = ov2640_write_reg(BANK_SENSOR, AEW, ae_levels[level]);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_SENSOR, AEB, ae_levels[level]);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    
    dev->status.ae_level = level - 2;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable DCW (Downsize Clock)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable DCW, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_dcw(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL2);
    if(enable) {
        reg_value |= CTRL2_DCW_MASK;
    } else {
        reg_value &= ~CTRL2_DCW_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL2, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.dcw = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable BPC (Black Pixel Correction)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable BPC, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_bpc(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL3);
    if(enable) {
        reg_value |= CTRL3_BPC_MASK;
    } else {
        reg_value &= ~CTRL3_BPC_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL3, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.bpc = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable WPC (White Pixel Correction)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable WPC, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_wpc(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL3);
    if(enable) {
        reg_value |= CTRL3_WPC_MASK;
    } else {
        reg_value &= ~CTRL3_WPC_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL3, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.wpc = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable RAW Gamma correction
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable gamma correction, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_raw_gma(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL1);
    if(enable) {
        reg_value |= CTRL1_RAW_GMA_MASK;
    } else {
        reg_value &= ~CTRL1_RAW_GMA_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL1, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.raw_gma = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Enable or disable LENC (Lens Correction)
 * @param dev Pointer to ov2640_t structure
 * @param enable 1 to enable lens correction, 0 to disable
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_lenc(ov2640_t *dev, int enable)
{
    int ret = 0;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg(CTRL1);
    if(enable) {
        reg_value |= CTRL1_LENC_MASK;
    } else {
        reg_value &= ~CTRL1_LENC_MASK;
    }
    ret = ov2640_set_bank(BANK_DSP);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    ret = ov2640_write_reg(BANK_DSP, CTRL1, reg_value);
    if(ret != 0) {
        ov2640_unlock();
        return ret;
    }
    dev->status.lenc = enable ? 1 : 0;
    ov2640_unlock();
    return 0;
}

/**
 * @brief Read register value and apply bit mask
 * @param dev Pointer to ov2640_t structure
 * @param reg Register address
 * @param mask Bit mask to apply
 * @return Masked register value
 */
static int ov2640_get_reg(ov2640_t *dev, int reg, int mask)
{
    uint8_t value;
    
    ov2640_lock();
    value = ov2640_read_reg((uint8_t)reg);
    ov2640_unlock();
    return (int)(value & mask);
}

/**
 * @brief Set specific bits in a register
 * @param dev Pointer to ov2640_t structure
 * @param reg Register address
 * @param mask Bit mask for bits to modify
 * @param value Value to set (will be masked)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_reg(ov2640_t *dev, int reg, int mask, int value)
{
    int ret;
    uint8_t reg_value;
    
    ov2640_lock();
    reg_value = ov2640_read_reg((uint8_t)reg);
    reg_value = (reg_value & ~mask) | (value & mask);
    ret = ov2640_write_reg(current_bank, (uint8_t)reg, reg_value);
    ov2640_unlock();
    return ret;
}

/**
 * @brief Set raw resolution parameters (not supported)
 * @param dev Pointer to ov2640_t structure
 * @param startX Start X coordinate
 * @param startY Start Y coordinate
 * @param endX End X coordinate
 * @param endY End Y coordinate
 * @param offsetX X offset
 * @param offsetY Y offset
 * @param totalX Total X size
 * @param totalY Total Y size
 * @param outputX Output X size
 * @param outputY Output Y size
 * @param scale Enable scaling
 * @param binning Enable binning
 * @return -1 (not supported)
 */
static int ov2640_set_res_raw(ov2640_t *dev, int startX, int startY, int endX, int endY, 
                               int offsetX, int offsetY, int totalX, int totalY, 
                               int outputX, int outputY, bool scale, bool binning)
{
    // Complex raw resolution setting - not commonly used
    // Placeholder implementation
    return -1;
}

/**
 * @brief Configure PLL settings (not supported)
 * @param dev Pointer to ov2640_t structure
 * @param bypass PLL bypass flag
 * @param mul PLL multiplier
 * @param sys System clock divider
 * @param root Root divider
 * @param pre Pre-divider
 * @param seld5 Divider select 5
 * @param pclken Pixel clock enable
 * @param pclk Pixel clock divider
 * @return -1 (not supported)
 */
static int ov2640_set_pll(ov2640_t *dev, int bypass, int mul, int sys, int root, int pre, int seld5, int pclken, int pclk)
{
    // PLL configuration not supported
    return -1;
}

/**
 * @brief Configure external clock (XCLK) using GPTIM2 PWM output
 * @param dev Pointer to ov2640_t structure
 * @param timer Timer selection (ignored, always uses GPTIM2)
 * @param xclk XCLK frequency in Hz (e.g., 24000000 for 24MHz)
 * @return 0 on success, negative error code on failure
 */
static int ov2640_set_xclk(ov2640_t *dev, int timer, int xclk)
{
    HAL_StatusTypeDef status;
    uint32_t timer_clk;
    uint32_t period;
    GPT_OC_InitTypeDef sConfigOC = {0};
    
    (void)timer;  // Timer parameter ignored, always use GPTIM2
    
    if (xclk <= 0)
    {
        // Stop XCLK if frequency is 0 or negative
        if (xclk_initialized)
        {
            HAL_GPT_PWM_Stop(&xclk_gptim, GPT_CHANNEL_1);
            HAL_GPT_Base_DeInit(&xclk_gptim);
            xclk_initialized = RT_FALSE;
            
            // Restore XCLK pin to GPIO function
            HAL_PIN_Set(PAD_PA00 + OV2640_DVP_XCLK_PIN, GPIO_A0 + OV2640_DVP_XCLK_PIN, PIN_NOPULL, 1);
            
            LOG_I("XCLK stopped");
        }
        return 0;
    }
    
    // Configure XCLK pin as GPTIM2_CH1
    HAL_PIN_Set(PAD_PA00 + OV2640_DVP_XCLK_PIN, GPTIM2_CH1, PIN_NOPULL, 1);
    
    // Enable GPTIM2 clock
    HAL_RCC_EnableModule(RCC_MOD_GPTIM2);

#if defined(SOC_SF32LB52X) && SOC_SF32LB52X == 1
    timer_clk = 24000000;
#else
    timer_clk = HAL_RCC_GetPCLKFreq(xclk_gptim.core, 1);
    rt_kprintf("GPTIM2 clock frequency: %d Hz\n", timer_clk);
#endif

    // Calculate period for desired frequency
    // PWM frequency = timer_clk / (prescaler + 1) / (period + 1)
    // For 50% duty cycle, compare value = period / 2
    // Use prescaler = 0 for highest resolution
    period = (timer_clk / xclk) - 1;
    
    if (period < 1 || period > 0xFFFF)
    {
        LOG_E("XCLK frequency %d Hz out of range (timer_clk=%d Hz)", xclk, timer_clk);
        return -1;
    }
    
    // Stop previous configuration if running
    if (xclk_initialized)
    {
        HAL_GPT_PWM_Stop(&xclk_gptim, GPT_CHANNEL_1);
        HAL_GPT_Base_DeInit(&xclk_gptim);
    }
    
    // Initialize GPTIM2 for PWM
    xclk_gptim.Instance = hwp_gptim2;
    xclk_gptim.Init.Prescaler = 0;
    xclk_gptim.Init.CounterMode = GPT_COUNTERMODE_UP;
    xclk_gptim.Init.Period = period;
    
    status = HAL_GPT_Base_Init(&xclk_gptim);
    if (status != HAL_OK)
    {
        LOG_E("GPTIM2 base init failed: %d", status);
        return -1;
    }
    
    // Configure PWM channel 1 (50% duty cycle)
    sConfigOC.OCMode = GPT_OCMODE_PWM1;
    sConfigOC.Pulse = period / 2 + 1;  // 50% duty cycle
    sConfigOC.OCPolarity = GPT_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = GPT_OCFAST_DISABLE;
    
    status = HAL_GPT_PWM_ConfigChannel(&xclk_gptim, &sConfigOC, GPT_CHANNEL_1);
    if (status != HAL_OK)
    {
        LOG_E("GPTIM2 PWM config failed: %d", status);
        HAL_GPT_Base_DeInit(&xclk_gptim);
        return -1;
    }
    
    // Start PWM output
    status = HAL_GPT_PWM_Start(&xclk_gptim, GPT_CHANNEL_1);
    if (status != HAL_OK)
    {
        LOG_E("GPTIM2 PWM start failed: %d", status);
        HAL_GPT_Base_DeInit(&xclk_gptim);
        return -1;
    }
    
    xclk_initialized = RT_TRUE;
    LOG_I("XCLK configured: %d Hz (period=%d, timer_clk=%d Hz)", xclk, period, timer_clk);
    
    return 0;
}

/* Public API */

/**
 * @brief  Initialize OV2640 sensor
 * @param  dev: Pointer to ov2640_t structure
 * @return 0 on success, negative error code on failure
 */
int ov2640_init(ov2640_t *dev)
{
    int ret;

    dev->init_status = ov2640_init_status;
    dev->reset = ov2640_reset;
    dev->set_pixformat = ov2640_set_pixformat;
    dev->set_framesize = ov2640_set_framesize;
    dev->set_contrast = ov2640_set_contrast;
    dev->set_brightness = ov2640_set_brightness;
    dev->set_saturation = ov2640_set_saturation;
    dev->set_sharpness = ov2640_set_sharpness;
    dev->set_denoise = ov2640_set_denoise;
    dev->set_gainceiling = ov2640_set_gainceiling;
    dev->set_quality = ov2640_set_quality;
    dev->set_colorbar = ov2640_set_colorbar;
    dev->set_whitebal = ov2640_set_whitebal;
    dev->set_gain_ctrl = ov2640_set_gain_ctrl;
    dev->set_exposure_ctrl = ov2640_set_exposure_ctrl;
    dev->set_hmirror = ov2640_set_hmirror;
    dev->set_vflip = ov2640_set_vflip;
    dev->set_aec2 = ov2640_set_aec2;
    dev->set_awb_gain = ov2640_set_awb_gain;
    dev->set_agc_gain = ov2640_set_agc_gain;
    dev->set_aec_value = ov2640_set_aec_value;
    dev->set_special_effect = ov2640_set_special_effect;
    dev->set_wb_mode = ov2640_set_wb_mode;
    dev->set_ae_level = ov2640_set_ae_level;
    dev->set_dcw = ov2640_set_dcw;
    dev->set_bpc = ov2640_set_bpc;
    dev->set_wpc = ov2640_set_wpc;
    dev->set_raw_gma = ov2640_set_raw_gma;
    dev->set_lenc = ov2640_set_lenc;
    dev->get_reg = ov2640_get_reg;
    dev->set_reg = ov2640_set_reg;
    dev->set_res_raw = ov2640_set_res_raw;
    dev->set_pll = ov2640_set_pll;
    dev->set_xclk = ov2640_set_xclk;

    ret = dev->reset(dev);
    if(ret != 0)
    {
        return ret;
    }

    ret = dev->init_status(dev);
    if(ret != 0)
    {
        return ret;
    }

    return 0;
}

/* Device registration and other functions */

/*****************************************************************************
 * RT-Thread Device Interface Implementation
 *****************************************************************************/

/* Control command definitions are in ov2640.h */

/* Frame ready callback for continuous capture */
static void camera_frame_ready_callback(dvp_handle_t *handle, void *ctx)
{
    rt_device_t dev = (rt_device_t)ctx;
    camera_device_t *cam_dev = (camera_device_t *)dev->user_data;

    /* Notify blocking camera_read() via semaphore */
    rt_sem_release(&cam_dev->frame_sem);

    if(dev && dev->rx_indicate)
    {
        dev->rx_indicate(dev, dvp_get_frame_size(handle));
    }
}

/* RT-Device init - called only once during first open */
static rt_err_t camera_init(rt_device_t dev)
{
    /* Device init is now handled in camera_open for proper re-initialization */
    return RT_EOK;
}

/* RT-Device open */
static rt_err_t camera_open(rt_device_t dev, rt_uint16_t oflag)
{
    camera_device_t *cam_dev = (camera_device_t *)dev->user_data;
    int ret;

    /* Initialize frame ready semaphore */
    rt_sem_init(&cam_dev->frame_sem, "cam_frm", 0, RT_IPC_FLAG_FIFO);

#if (OV2640_DVP_XCLK_PIN >= 0)
    /* Start XCLK first - sensor needs clock to respond */
    ret = ov2640_set_xclk(&cam_dev->sensor, 0, OV2640_DVP_XCLK_FREQ);
    if (ret != 0)
    {
        LOG_E("XCLK start failed: %d", ret);
        return -RT_ERROR;
    }
    
    /* Small delay for clock to stabilize */
    rt_thread_mdelay(10);
#endif
    
    sccb_init(SCCB_USE_IIC);  // Initialize SCCB/I2C interface
    
    /* Initialize OV2640 sensor */
    ret = ov2640_init(&cam_dev->sensor);
    if (ret != 0)
    {
        LOG_E("OV2640 init failed: %d", ret);
        sccb_deinit();
#if (OV2640_DVP_XCLK_PIN >= 0)
        ov2640_set_xclk(&cam_dev->sensor, 0, 0);  // Stop XCLK on failure
#endif
        return -RT_ERROR;
    }
    
    /* Set default pixel format and frame size */
    cam_dev->sensor.set_pixformat(&cam_dev->sensor, PIXFORMAT_JPEG);
    cam_dev->sensor.set_framesize(&cam_dev->sensor, FRAMESIZE_UXGA);
    cam_dev->sensor.set_quality(&cam_dev->sensor, 12);
    
    /* Initialize DVP interface */
    dvp_config_t dvp_config = {
        .mode = DVP_MODE_JPEG,
        .buffer_size = 0,
        .frame_buffer = NULL,
        .pingpong_buffer_size = OV2640_DVP_PINGPONG_BUFFER_SIZE, // e.g., 8192 for JPEG
        .vsync_pin = OV2640_DVP_VSYNC_PIN,          // PA42
        .frame_callback = camera_frame_ready_callback,
        .callback_ctx = dev,  // Pass camera device as context
    };
    
    ret = dvp_init(&cam_dev->dvp, &dvp_config);
    if (ret != 0)
    {
        LOG_E("DVP init failed: %d", ret);
        sccb_deinit();
        return -RT_ERROR;
    }
    
    /* Start DVP hardware */
    ret = dvp_start(&cam_dev->dvp);
    if (ret != 0)
    {
        LOG_E("DVP start failed: %d", ret);
        dvp_deinit(&cam_dev->dvp);
        sccb_deinit();
        return -RT_ERROR;
    }
    
    return RT_EOK;
}

/* RT-Device close */
static rt_err_t camera_close(rt_device_t dev)
{
    camera_device_t *cam_dev = (camera_device_t *)dev->user_data;
    
    /* Stop any ongoing capture */
    dvp_abort_capture(&cam_dev->dvp);
    
    /* Stop DVP hardware */
    dvp_stop(&cam_dev->dvp);
    
    /* Deinitialize DVP interface */
    dvp_deinit(&cam_dev->dvp);
    
    /* Deinitialize SCCB/I2C interface */
    sccb_deinit();
    
#if (OV2640_DVP_XCLK_PIN >= 0)
    /* Stop XCLK */
    ov2640_set_xclk(&cam_dev->sensor, 0, 0);
#endif
    
    /* Reset sensor bank state */
    ov2640_reset_bank_state();
    
    /* Destroy frame ready semaphore */
    rt_sem_detach(&cam_dev->frame_sem);
    
    LOG_I("Camera device closed");
    return RT_EOK;
}

#ifndef OV2640_CAMERA_READ_TIMEOUT_MS
#define CAMERA_READ_TIMEOUT_MS    1000  /* 1 second timeout */
#else
#define CAMERA_READ_TIMEOUT_MS    OV2640_CAMERA_READ_TIMEOUT_MS
#endif
#define CAMERA_READ_TIMEOUT_TICKS    (CAMERA_READ_TIMEOUT_MS * RT_TICK_PER_SECOND / 1000)
/* RT-Device read - for single/continuous capture */
static rt_size_t camera_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    camera_device_t *cam_dev = (camera_device_t *)dev->user_data;
    rt_size_t frame_size = 0;
    rt_err_t result;

    /* Drain any stale semaphore tokens before starting capture */
    while (rt_sem_trytake(&cam_dev->frame_sem) == RT_EOK);

    dvp_start_capture(&cam_dev->dvp, buffer, size);

    /* Block until frame ready or timeout */
    result = rt_sem_take(&cam_dev->frame_sem, CAMERA_READ_TIMEOUT_TICKS);
    if (result != RT_EOK)
    {
        LOG_W("Camera read timeout");
        dvp_abort_capture(&cam_dev->dvp);
        return 0;
    }

    frame_size = dvp_get_frame_size(&cam_dev->dvp);
    
    return frame_size;
}

/* RT-Device write - not used for camera */
static rt_size_t camera_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    return 0;  /* Not supported */
}

/* RT-Device control */
static rt_err_t camera_control(rt_device_t dev, int cmd, void *args)
{
    camera_device_t *cam_dev = (camera_device_t *)dev->user_data;
    int ret;
    
    if (cam_dev == RT_NULL)
    {
        return -RT_ERROR;
    }
    
    switch (cmd)
    {
        case CAMERA_CMD_SET_PIXFORMAT:
        {
            pixformat_t format = (pixformat_t)(rt_ubase_t)args;
            ret = cam_dev->sensor.set_pixformat(&cam_dev->sensor, format);
            if (ret != 0)
            {
                return -RT_ERROR;
            }
            
            /* Update DVP mode accordingly and restart hardware */
            dvp_mode_t new_mode;
            switch (format)
            {
                case PIXFORMAT_JPEG:
                    new_mode = DVP_MODE_JPEG;
                    break;
                case PIXFORMAT_RGB565:
                    new_mode = DVP_MODE_RGB565;
                    break;
                case PIXFORMAT_YUV422:
                    new_mode = DVP_MODE_YUV422;
                    break;
                case PIXFORMAT_RAW8:
                    new_mode = DVP_MODE_RAW;
                    break;
                default:
                    return -RT_EINVAL;
            }
            
            /* If mode changed, restart DVP hardware */
            if (cam_dev->dvp.config.mode != new_mode)
            {
                dvp_stop(&cam_dev->dvp);
                cam_dev->dvp.config.mode = new_mode;
                ret = dvp_start(&cam_dev->dvp);
                if (ret != 0)
                {
                    LOG_E("Failed to restart DVP after mode change");
                    return -RT_ERROR;
                }
            }
            
            return RT_EOK;
        }
        
        case CAMERA_CMD_SET_FRAMESIZE:
        {
            if((framesize_t)(rt_ubase_t)args >= FRAMESIZE_INVALID)
            {
                return -RT_EINVAL;
            }
            framesize_t framesize = (framesize_t)(rt_ubase_t)args;
            ret = cam_dev->sensor.set_framesize(&cam_dev->sensor, framesize);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_BRIGHTNESS:
        {
            int level = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_brightness(&cam_dev->sensor, level);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_CONTRAST:
        {
            int level = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_contrast(&cam_dev->sensor, level);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_SATURATION:
        {
            int level = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_saturation(&cam_dev->sensor, level);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_QUALITY:
        {
            int quality = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_quality(&cam_dev->sensor, quality);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_HMIRROR:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_hmirror(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_VFLIP:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_vflip(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_COLORBAR:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_colorbar(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_WHITEBAL:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_whitebal(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_GAIN_CTRL:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_gain_ctrl(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_EXPOSURE_CTRL:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_exposure_ctrl(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_AEC2:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_aec2(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_AWB_GAIN:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_awb_gain(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_AGC_GAIN:
        {
            int gain = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_agc_gain(&cam_dev->sensor, gain);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_AEC_VALUE:
        {
            int value = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_aec_value(&cam_dev->sensor, value);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_SPECIAL_EFFECT:
        {
            int effect = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_special_effect(&cam_dev->sensor, effect);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_WB_MODE:
        {
            int mode = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_wb_mode(&cam_dev->sensor, mode);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_AE_LEVEL:
        {
            int level = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_ae_level(&cam_dev->sensor, level);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_DCW:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_dcw(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_BPC:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_bpc(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_WPC:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_wpc(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_RAW_GMA:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_raw_gma(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_LENC:
        {
            int enable = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_lenc(&cam_dev->sensor, enable);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_GAINCEILING:
        {
            gainceiling_t gainceiling = (gainceiling_t)(rt_ubase_t)args;
            ret = cam_dev->sensor.set_gainceiling(&cam_dev->sensor, gainceiling);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_SHARPNESS:
        {
            int level = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_sharpness(&cam_dev->sensor, level);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_SET_DENOISE:
        {
            int level = (int)(rt_base_t)args;
            ret = cam_dev->sensor.set_denoise(&cam_dev->sensor, level);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }
        
        case CAMERA_CMD_START_CAPTURE:
        {
            /* args not used for start capture */
            (void)args;
            dvp_start_capture(&cam_dev->dvp, RT_NULL, 0);
            return RT_EOK;
        }
        
        case CAMERA_CMD_STOP_CAPTURE:
        {
            /* args not used for stop capture */
            (void)args;
            dvp_abort_capture(&cam_dev->dvp);
            return RT_EOK;
        }
        
        case CAMERA_CMD_GET_FRAME_SIZE:
        {
            uint32_t *size_ptr = (uint32_t *)args;
            if (size_ptr != RT_NULL)
            {
                *size_ptr = dvp_get_frame_size(&cam_dev->dvp);
                return RT_EOK;
            }
            return -RT_EINVAL;
        }
        
        case CAMERA_CMD_SET_DVP_BUFFER:
        {
            uint8_t *buffer = (uint8_t *)args;
            if (buffer != RT_NULL)
            {
                cam_dev->dvp.config.frame_buffer = buffer;
                LOG_I("DVP buffer set to 0x%08X", (uint32_t)buffer);
                return RT_EOK;
            }
            return -RT_EINVAL;
        }
        
        case CAMERA_CMD_SET_DVP_BUFFER_SIZE:
        {
            uint32_t size = (uint32_t)(rt_ubase_t)args;
            if (size > 0)
            {
                cam_dev->dvp.config.buffer_size = size;
                LOG_I("DVP buffer size set to %d bytes", size);
                return RT_EOK;
            }
            return -RT_EINVAL;
        }
        
        default:
            return -RT_EINVAL;
    }
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops camera_ops =
{
    camera_init,
    camera_open,
    camera_close,
    camera_read,
    camera_write,
    camera_control
};
#endif

/**
 * @brief Register camera device to RT-Thread
 * @param name Device name (e.g., "camera0")
 * @return RT_EOK on success, error code on failure
 */
int ov2640_device_register()
{
    rt_device_t dev;
    int ret;
    const char *name = "ov2640";

    /* Allocate camera device structure */
    dev = rt_malloc(sizeof(struct rt_device));
    if (dev == RT_NULL)
    {
        LOG_E("Failed to allocate ov2640 device memory");
        return -RT_ENOMEM;
    }
    
    rt_memset(dev, 0, sizeof(struct rt_device));
    
    /* Initialize device structure */
    dev->type = RT_Device_Class_Miscellaneous;
    
#ifdef RT_USING_DEVICE_OPS
    dev->ops = &camera_ops;
#else
    dev->init = camera_init;
    dev->open = camera_open;
    dev->close = camera_close;
    dev->read = camera_read;
    dev->write = camera_write;
    dev->control = camera_control;
#endif
    dev->user_data = rt_malloc(sizeof(camera_device_t));
    if (dev->user_data == RT_NULL)
    {
        LOG_E("Failed to allocate ov2640 device user data memory");
        rt_free(dev);
        return -RT_ENOMEM;
    }
    rt_memset(dev->user_data, 0, sizeof(camera_device_t));
    
    /* Register device */
    ret = rt_device_register(dev, name, RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK)
    {
        LOG_E("Failed to register ov2640 device: %d", ret);
        rt_free(dev->user_data);
        rt_free(dev);
        return ret;
    }
    
    LOG_I("ov2640 device '%s' registered successfully", name);
    return RT_EOK;
}

/**
 * @brief Unregister camera device from RT-Thread
 * @return RT_EOK on success, error code on failure
 */
int ov2640_device_unregister(void)
{
    rt_device_t dev;
    const char *name = "ov2640";
    
    dev = rt_device_find(name);
    if (dev == RT_NULL)
    {
        LOG_W("ov2640 device '%s' not found", name);
        return -RT_ENOSYS;
    }
    
    /* Close device if still open */
    if (dev->ref_count > 0)
    {
        rt_device_close(dev);
    }
    
    /* Unregister device */
    rt_err_t ret = rt_device_unregister(dev);
    if (ret != RT_EOK)
    {
        LOG_E("Failed to unregister ov2640 device: %d", ret);
        return ret;
    }
    
    /* Free user data */
    if (dev->user_data != RT_NULL)
    {
        rt_free(dev->user_data);
        dev->user_data = RT_NULL;
    }
    
    /* Free device structure */
    rt_free(dev);
    
    LOG_I("ov2640 device '%s' unregistered successfully", name);
    return RT_EOK;
}

INIT_DEVICE_EXPORT(ov2640_device_register);