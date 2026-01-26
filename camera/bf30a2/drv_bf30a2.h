/**
 * @file    drv_bf30a2.h
 * @brief   BF30A2 SPI Camera RT-Thread Device Driver Interface
 *
 * This header provides the RT-Thread standard device driver interface
 * for the BF30A2 SPI camera sensor.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2024
 */

#ifndef __DRV_BF30A2_H__
#define __DRV_BF30A2_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/* Sensor Specifications                                                     */
/*===========================================================================*/

/** @brief Default image width in pixels */
#define BF30A2_DEFAULT_WIDTH        240

/** @brief Default image height in pixels */
#define BF30A2_DEFAULT_HEIGHT       320

/** @brief Bytes per pixel for RGB565 format */
#define BF30A2_BYTES_PER_PIXEL      2

/** @brief Default frame buffer size in bytes (RGB565: 2 bytes per pixel) */
#define BF30A2_DEFAULT_FRAME_SIZE   (BF30A2_DEFAULT_WIDTH * BF30A2_DEFAULT_HEIGHT * BF30A2_BYTES_PER_PIXEL)

/** @brief Device name for registration */
#define BF30A2_DEVICE_NAME          "bf30a2"

/*===========================================================================*/
/* Device Control Commands                                                   */
/*===========================================================================*/

/**
 * @brief Camera device control commands
 */
enum bf30a2_cmd
{
    BF30A2_CMD_START = 0x100,       /**< Start frame capture */
    BF30A2_CMD_STOP,                /**< Stop frame capture */
    BF30A2_CMD_GET_INFO,            /**< Get camera information */
    BF30A2_CMD_GET_STATUS,          /**< Get camera status */
    BF30A2_CMD_GET_FPS,             /**< Get current frame rate */
    BF30A2_CMD_GET_FRAME_COUNT,     /**< Get total captured frame count */
    BF30A2_CMD_SET_CALLBACK,        /**< Set frame ready callback */
    BF30A2_CMD_GET_BUFFER,          /**< Get frame buffer pointer */
    BF30A2_CMD_WAIT_FRAME,          /**< Wait for next frame */
    BF30A2_CMD_EXPORT_UART,         /**< Export frame via UART */
    BF30A2_CMD_RESET_STATS,         /**< Reset statistics */
};

/*===========================================================================*/
/* Data Structures                                                           */
/*===========================================================================*/

/**
 * @brief Camera status enumeration
 */
typedef enum
{
    BF30A2_STATUS_IDLE = 0,         /**< Camera is idle */
    BF30A2_STATUS_RUNNING,          /**< Camera is capturing */
    BF30A2_STATUS_ERROR,            /**< Camera error state */
} bf30a2_status_t;

/**
 * @brief Camera output format enumeration
 */
typedef enum
{
    BF30A2_FORMAT_RGB565 = 0,       /**< RGB565 format (default) */
    BF30A2_FORMAT_YUV422,           /**< YUV422 format */
} bf30a2_format_t;

/**
 * @brief Camera information structure
 */
typedef struct bf30a2_info
{
    rt_uint16_t width;              /**< Image width in pixels */
    rt_uint16_t height;             /**< Image height in pixels */
    rt_uint32_t frame_size;         /**< Frame buffer size in bytes */
    bf30a2_format_t format;         /**< Output format */
    rt_uint16_t chip_id;            /**< Sensor chip ID */
} bf30a2_info_t;

/**
 * @brief Camera status structure
 */
typedef struct bf30a2_status
{
    bf30a2_status_t state;          /**< Current state */
    rt_uint32_t frame_count;        /**< Total frames captured */
    rt_uint32_t complete_frames;    /**< Successfully completed frames */
    rt_uint32_t error_count;        /**< Error count */
    float fps;                      /**< Current frame rate */
    rt_uint8_t frame_ready;         /**< Frame ready flag */
} bf30a2_status_info_t;

/**
 * @brief Frame ready callback function type
 *
 * @param dev       Device handle
 * @param frame_num Frame sequence number
 * @param buffer    Pointer to frame data (RGB565 format)
 * @param size      Frame data size in bytes
 * @param user_data User-provided context pointer
 */
typedef void (*bf30a2_frame_callback_t)(rt_device_t dev,
                                        rt_uint32_t frame_num,
                                        rt_uint8_t *buffer,
                                        rt_uint32_t size,
                                        void *user_data);

/**
 * @brief Callback configuration structure
 */
typedef struct bf30a2_callback_cfg
{
    bf30a2_frame_callback_t callback;   /**< Callback function pointer */
    void *user_data;                    /**< User context pointer */
} bf30a2_callback_cfg_t;

/**
 * @brief Frame buffer information structure
 */
typedef struct bf30a2_buffer
{
    rt_uint8_t *data;               /**< Pointer to frame data */
    rt_uint32_t size;               /**< Buffer size in bytes */
    rt_uint32_t frame_num;          /**< Frame sequence number */
    rt_uint32_t timestamp;          /**< Capture timestamp (tick) */
} bf30a2_buffer_t;

/**
 * @brief Wait frame configuration structure
 */
typedef struct bf30a2_wait_cfg
{
    rt_uint32_t timeout_ms;         /**< Wait timeout in milliseconds */
    bf30a2_buffer_t *buffer;        /**< Output buffer info (optional) */
} bf30a2_wait_cfg_t;

/*===========================================================================*/
/* Hardware Configuration Structure                                          */
/*===========================================================================*/

/**
 * @brief Hardware pin configuration
 */
typedef struct bf30a2_pin_cfg
{
    rt_uint8_t spi_clk_pad;         /**< SPI clock pad number */
    rt_uint8_t spi_dio_pad;         /**< SPI data IO pad number */
    rt_uint8_t spi_cs_pad;          /**< SPI chip select pad number */
    rt_uint8_t i2c_scl_pad;         /**< I2C clock pad number */
    rt_uint8_t i2c_sda_pad;         /**< I2C data pad number */
    rt_uint8_t pwdn_pin;            /**< Power down GPIO pin number */
    rt_uint8_t pwm_pad;             /**< PWM output pad number */
} bf30a2_pin_cfg_t;

/**
 * @brief Hardware configuration structure
 */
typedef struct bf30a2_hw_cfg
{
    const char *spi_bus_name;       /**< SPI bus device name */
    const char *i2c_bus_name;       /**< I2C bus device name */
    const char *pwm_dev_name;       /**< PWM device name */
    rt_uint8_t i2c_addr;            /**< I2C slave address */
    rt_uint8_t pwm_channel;         /**< PWM channel number */
    bf30a2_pin_cfg_t pins;          /**< Pin configuration */
} bf30a2_hw_cfg_t;

/*===========================================================================*/
/* Public API Functions                                                      */
/*===========================================================================*/

/**
 * @brief Register BF30A2 camera device with default configuration
 *
 * This function registers the BF30A2 camera as an RT-Thread device
 * with default hardware configuration.
 *
 * @return RT_EOK on success, error code on failure
 */
rt_err_t bf30a2_device_register(void);

/**
 * @brief Register BF30A2 camera device with custom configuration
 *
 * This function registers the BF30A2 camera as an RT-Thread device
 * with user-specified hardware configuration.
 *
 * @param name      Device name for registration
 * @param hw_cfg    Hardware configuration structure
 *
 * @return RT_EOK on success, error code on failure
 */
rt_err_t bf30a2_device_register_with_config(const char *name,
                                            const bf30a2_hw_cfg_t *hw_cfg);

/**
 * @brief Get default hardware configuration
 *
 * @param cfg   Pointer to configuration structure to fill
 */
void bf30a2_get_default_config(bf30a2_hw_cfg_t *cfg);

/*===========================================================================*/
/* Convenience Macros for Device Control                                     */
/*===========================================================================*/

/**
 * @brief Start camera capture
 * @param dev Device handle
 * @return RT_EOK on success
 */
#define bf30a2_start(dev) \
    rt_device_control(dev, BF30A2_CMD_START, RT_NULL)

/**
 * @brief Stop camera capture
 * @param dev Device handle
 * @return RT_EOK on success
 */
#define bf30a2_stop(dev) \
    rt_device_control(dev, BF30A2_CMD_STOP, RT_NULL)

/**
 * @brief Get camera information
 * @param dev Device handle
 * @param info Pointer to bf30a2_info_t structure
 * @return RT_EOK on success
 */
#define bf30a2_get_info(dev, info) \
    rt_device_control(dev, BF30A2_CMD_GET_INFO, info)

/**
 * @brief Get camera status
 * @param dev Device handle
 * @param status Pointer to bf30a2_status_info_t structure
 * @return RT_EOK on success
 */
#define bf30a2_get_status(dev, status) \
    rt_device_control(dev, BF30A2_CMD_GET_STATUS, status)

#ifdef __cplusplus
}
#endif

#endif /* __DRV_BF30A2_H__ */
