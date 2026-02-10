/**
 * @file    dvp.h
 * @brief   Software DVP (Digital Video Port) driver interface
 *
 * This header defines the DVP configuration, handle structures
 * and public API for parallel camera data capture.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#ifndef __DVP_H__
#define __DVP_H__

#include "rtthread.h"
#include "bf0_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DVP capture mode */
typedef enum {
    DVP_MODE_JPEG,      // JPEG mode - scan for SOI/EOI markers
    DVP_MODE_RAW,       // RAW mode - fixed size capture
    DVP_MODE_YUV422,    // YUV422 mode - fixed size capture
    DVP_MODE_RGB565,    // RGB565 mode - fixed size capture
} dvp_mode_t;

/* Forward declaration */
typedef struct dvp_handle dvp_handle_t;

/* DVP callback function type definition */
typedef void (*dvp_frame_ready_callback_t)(dvp_handle_t *handle, void *ctx);

/* DVP configuration structure */
typedef struct {
    dvp_mode_t mode;                // DVP capture mode
    uint32_t buffer_size;           // Image buffer size
    uint8_t *frame_buffer;          // Frame buffer pointer (user provided)
    uint32_t pingpong_buffer_size;  // Ping-pong buffer size
    
    /* Pin number configuration */
    uint8_t vsync_pin;              // VSYNC pin number (GPIO interrupt)
    /* Note: PCLK and HSYNC pins must be configured by user before calling dvp_init()
     *   - PCLK: Configure as GPTIM1_ETR (e.g., HAL_PIN_Set(PAD_PA00 + pin, GPTIM1_ETR, PIN_PULLDOWN, 1))
     *   - HSYNC: Configure as GPTIM1_CH1 (e.g., HAL_PIN_Set(PAD_PA00 + pin, GPTIM1_CH1, PIN_PULLDOWN, 1))
     */
    
    dvp_frame_ready_callback_t frame_callback; // Frame ready callback
    void *callback_ctx;                        // User context for callback
} dvp_config_t;

/* DVP handle structure */
struct dvp_handle {
    dvp_config_t config;
    GPT_HandleTypeDef gptim;
    DMA_HandleTypeDef dma;
    uint8_t *pingpong_buffer;
    volatile uint8_t frame_ready;
    volatile uint32_t current_size;
    volatile uint8_t soi_found;
    volatile uint8_t enable_capture;
};

/* Function declarations */

/**
 * @brief Initialize DVP interface
 * @param handle DVP handle pointer
 * @param config DVP configuration structure pointer
 * @return 0 on success, other values on failure
 * @note PCLK is hardcoded to GPTIM1_ETR, HSYNC to GPTIM1_CH1, VSYNC auto-calculated from pin number
 */
int dvp_init(dvp_handle_t *handle, const dvp_config_t *config);

/**
 * @brief Deinitialize DVP interface
 * @param handle DVP handle pointer
 * @return 0 on success, other values on failure
 */
int dvp_deinit(dvp_handle_t *handle);

/**
 * @brief Start DVP hardware (enable timer and DMA)
 * @param handle DVP handle pointer
 * @return 0 on success, other values on failure
 */
int dvp_start(dvp_handle_t *handle);

/**
 * @brief Stop DVP hardware (stop timer and DMA)
 * @param handle DVP handle pointer
 * @return 0 on success, other values on failure
 */
int dvp_stop(dvp_handle_t *handle);

/**
 * @brief Start capturing one frame (reset state, wait for VSYNC trigger)
 * @param handle DVP handle pointer
 * @param new_buffer New frame buffer pointer, if NULL use original buffer
 * @param buffer_size Buffer size (bytes), must be provided if new_buffer is not NULL
 * @return 0 on success, other values on failure
 */
int dvp_start_capture(dvp_handle_t *handle, uint8_t *new_buffer, uint32_t buffer_size);

/**
 * @brief Abort current capture (keep hardware running)
 * @param handle DVP handle pointer
 * @return 0 on success, other values on failure
 */
int dvp_abort_capture(dvp_handle_t *handle);

/**
 * @brief Check if new frame is ready
 * @param handle DVP handle pointer
 * @return 1 if new frame available, 0 if no new frame
 */
int dvp_is_frame_ready(dvp_handle_t *handle);

/**
 * @brief Get current captured frame size
 * @param handle DVP handle pointer
 * @return Frame size (bytes)
 */
uint32_t dvp_get_frame_size(dvp_handle_t *handle);

/**
 * @brief Reset capture state, prepare for next frame
 * @param handle DVP handle pointer
 */
void dvp_reset_capture(dvp_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* __DVP_H__ */
