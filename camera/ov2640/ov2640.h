/**
 * @file    ov2640.h
 * @brief   OV2640 camera sensor RT-Thread device driver interface
 *
 * This header provides the OV2640 sensor data structures, camera
 * status definitions, RT-Thread device interface, and control
 * command definitions.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#ifndef OV2640_H_
#define OV2640_H_

#include <stdint.h>
#include <stdbool.h>
#include "sccb.h"
#include "rtthread.h"
#include "rtconfig.h"

#define OV2640_ADDR 0x30

typedef enum {
    PIXFORMAT_RGB565,    // 2BPP/RGB565
    PIXFORMAT_YUV422,    // 2BPP/YUV422
    PIXFORMAT_JPEG,      // JPEG/COMPRESSED
    PIXFORMAT_RAW8,      // RAW 8-bit
    PIXFORMAT_INVALID
} pixformat_t;

typedef enum {
    FRAMESIZE_96X96,    // 96x96
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_128X128,    // 128x128
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_HQVGA,    // 240x176
    FRAMESIZE_240X240,  // 240x240
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_320X320,  // 320x320
    FRAMESIZE_CIF,      // 400x296
    FRAMESIZE_HVGA,     // 480x320
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_XGA,      // 1024x768
    FRAMESIZE_HD,       // 1280x720
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200
    FRAMESIZE_INVALID
} framesize_t;

typedef struct {
    framesize_t framesize;//0 - 10
    bool scale;
    bool binning;
    uint8_t quality;//0 - 63
    int8_t brightness;//-2 - 2
    int8_t contrast;//-2 - 2
    int8_t saturation;//-2 - 2
    int8_t sharpness;//-2 - 2
    uint8_t denoise;
    uint8_t special_effect;//0 - 6
    uint8_t wb_mode;//0 - 4
    uint8_t awb;
    uint8_t awb_gain;
    uint8_t aec;
    uint8_t aec2;
    int8_t ae_level;//-2 - 2
    uint16_t aec_value;//0 - 1200
    uint8_t agc;
    uint8_t agc_gain;//0 - 30
    uint8_t gainceiling;//0 - 6
    uint8_t bpc;
    uint8_t wpc;
    uint8_t raw_gma;
    uint8_t lenc;
    uint8_t hmirror;
    uint8_t vflip;
    uint8_t dcw;
    uint8_t colorbar;
} camera_status_t;

typedef enum {
    GAINCEILING_2X,
    GAINCEILING_4X,
    GAINCEILING_8X,
    GAINCEILING_16X,
    GAINCEILING_32X,
    GAINCEILING_64X,
    GAINCEILING_128X,
} gainceiling_t;

typedef struct ov2640
{
    uint8_t MIDH;
    uint8_t MIDL;
    uint16_t PID;
    uint8_t VER;

    pixformat_t pixformat;
    camera_status_t status;

    // Sensor function pointers
    int  (*init_status)         (struct ov2640 *sensor);
    int  (*reset)               (struct ov2640 *sensor); 
    int  (*set_pixformat)       (struct ov2640 *sensor, pixformat_t pixformat);
    int  (*set_framesize)       (struct ov2640 *sensor, framesize_t framesize);
    int  (*set_contrast)        (struct ov2640 *sensor, int level);
    int  (*set_brightness)      (struct ov2640 *sensor, int level);
    int  (*set_saturation)      (struct ov2640 *sensor, int level);
    int  (*set_sharpness)       (struct ov2640 *sensor, int level);
    int  (*set_denoise)         (struct ov2640 *sensor, int level);
    int  (*set_gainceiling)     (struct ov2640 *sensor, gainceiling_t gainceiling);
    int  (*set_quality)         (struct ov2640 *sensor, int quality);
    int  (*set_colorbar)        (struct ov2640 *sensor, int enable);
    int  (*set_whitebal)        (struct ov2640 *sensor, int enable);
    int  (*set_gain_ctrl)       (struct ov2640 *sensor, int enable);
    int  (*set_exposure_ctrl)   (struct ov2640 *sensor, int enable);
    int  (*set_hmirror)         (struct ov2640 *sensor, int enable);
    int  (*set_vflip)           (struct ov2640 *sensor, int enable);

    int  (*set_aec2)            (struct ov2640 *sensor, int enable);
    int  (*set_awb_gain)        (struct ov2640 *sensor, int enable);
    int  (*set_agc_gain)        (struct ov2640 *sensor, int gain);
    int  (*set_aec_value)       (struct ov2640 *sensor, int gain);

    int  (*set_special_effect)  (struct ov2640 *sensor, int effect);
    int  (*set_wb_mode)         (struct ov2640 *sensor, int mode);
    int  (*set_ae_level)        (struct ov2640 *sensor, int level);

    int  (*set_dcw)             (struct ov2640 *sensor, int enable);
    int  (*set_bpc)             (struct ov2640 *sensor, int enable);
    int  (*set_wpc)             (struct ov2640 *sensor, int enable);

    int  (*set_raw_gma)         (struct ov2640 *sensor, int enable);
    int  (*set_lenc)            (struct ov2640 *sensor, int enable);

    int  (*get_reg)             (struct ov2640 *sensor, int reg, int mask);
    int  (*set_reg)             (struct ov2640 *sensor, int reg, int mask, int value);
    int  (*set_res_raw)         (struct ov2640 *sensor, int startX, int startY, int endX, int endY, int offsetX, int offsetY, int totalX, int totalY, int outputX, int outputY, bool scale, bool binning);
    int  (*set_pll)             (struct ov2640 *sensor, int bypass, int mul, int sys, int root, int pre, int seld5, int pclken, int pclk);
    int  (*set_xclk)            (struct ov2640 *sensor, int timer, int xclk);
} ov2640_t;

int ov2640_init(ov2640_t *dev);

/* RT-Thread Device Interface */

#include "dvp.h"

/* Camera device structure */
typedef struct {
    ov2640_t sensor;
    dvp_handle_t dvp;
    struct rt_semaphore frame_sem;  /* Semaphore for frame ready notification */
} camera_device_t;

/* Camera control commands */
#define CAMERA_CMD_SET_PIXFORMAT    0x01
#define CAMERA_CMD_SET_FRAMESIZE    0x02
#define CAMERA_CMD_SET_BRIGHTNESS   0x03
#define CAMERA_CMD_SET_CONTRAST     0x04
#define CAMERA_CMD_SET_SATURATION   0x05
#define CAMERA_CMD_SET_QUALITY      0x06
#define CAMERA_CMD_SET_HMIRROR      0x07
#define CAMERA_CMD_SET_VFLIP        0x08
#define CAMERA_CMD_SET_COLORBAR     0x09
#define CAMERA_CMD_SET_WHITEBAL     0x0A
#define CAMERA_CMD_SET_GAIN_CTRL    0x0B
#define CAMERA_CMD_SET_EXPOSURE_CTRL 0x0C
#define CAMERA_CMD_SET_AEC2         0x0D
#define CAMERA_CMD_SET_AWB_GAIN     0x0E
#define CAMERA_CMD_SET_AGC_GAIN     0x0F
#define CAMERA_CMD_START_CAPTURE    0x10
#define CAMERA_CMD_STOP_CAPTURE     0x11
#define CAMERA_CMD_GET_FRAME_SIZE   0x12
#define CAMERA_CMD_SET_AEC_VALUE    0x13
#define CAMERA_CMD_SET_SPECIAL_EFFECT 0x14
#define CAMERA_CMD_SET_WB_MODE      0x15
#define CAMERA_CMD_SET_AE_LEVEL     0x16
#define CAMERA_CMD_SET_DCW          0x17
#define CAMERA_CMD_SET_BPC          0x18
#define CAMERA_CMD_SET_WPC          0x19
#define CAMERA_CMD_SET_RAW_GMA      0x1A
#define CAMERA_CMD_SET_LENC         0x1B
#define CAMERA_CMD_SET_GAINCEILING  0x1C
#define CAMERA_CMD_SET_SHARPNESS    0x1D
#define CAMERA_CMD_SET_DENOISE      0x1E
#define CAMERA_CMD_SET_DVP_BUFFER   0x1F
#define CAMERA_CMD_SET_DVP_BUFFER_SIZE 0x20

/**
 * @brief Register camera device to RT-Thread
 * @return RT_EOK on success, error code on failure
 */
int ov2640_device_register(void);

/**
 * @brief Unregister camera device from RT-Thread
 * @return RT_EOK on success, error code on failure
 */
int ov2640_device_unregister(void);

/**
 * @brief Set receive callback for camera device
 * @param dev Camera device pointer
 * @param rx_ind Callback function
 * @return RT_EOK on success
 */
rt_err_t camera_set_rx_indicate(rt_device_t dev, rt_err_t (*rx_ind)(rt_device_t dev, rt_size_t size));

#endif /* OV2640_H_ */