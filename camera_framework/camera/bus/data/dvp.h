/******************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file dvp.h
 * 
 * @par dependencies 
 * - rtthread.h
 * - bf0_hal.h
 * - stdint.h
 * 
 * @author SiFli 思澈科技
 * 
 * @brief DVP data-bus backend public interface.
 *
 * This header declares the DVP implementation of the generic data-bus
 * adapter contract (`bus_adapter_ops_t`) and exposes DVP-private helper
 * interfaces used by internal debug/tuning flows.
 * 
 * @version V1.0 2026-4-3
 *
 * @note 1 tab == 4 spaces!
 * 
 *****************************************************************************/

#ifndef __DVP_H__
#define __DVP_H__

//******************************** Includes *********************************//
#include "rtthread.h"
#include "bf0_hal.h"
#include <stdint.h>
//******************************** Includes *********************************//

//******************************** Defines **********************************//
#ifdef __cplusplus
extern "C" {
#endif

/* PCLK and HSYNC pin PAD numbers are provided by the sensor driver.
 * The alternate function is GPTIM1_ETR / GPTIM1_CH1 on all supported platforms. */
#define DVP_PCLK_PIN_PAD(pin_idx_)   (PAD_PA00 + (pin_idx_))
#define DVP_PCLK_PIN_FUNC            GPTIM1_ETR
#define DVP_HSYNC_PIN_PAD(pin_idx_)  (PAD_PA00 + (pin_idx_))
#define DVP_HSYNC_PIN_FUNC           GPTIM1_CH1

/* DATA bus pins and their GPIO/DMA source address are platform-specific. */
#ifdef SF32LB52X
#define DVP_DATA_PIN_PAD_BASE      PAD_PA00
#define DVP_DATA_PIN_FUNC_BASE     GPIO_A0
#define DVP_DATA_GPIO_PIN_BASE     0
#define DVP_DATA_PIN_SOURCE_ADDR   ((uint32_t)&hwp_gpio1->DIR)

#elif defined(SF32LB56X)
#define DVP_DATA_PIN_PAD_BASE      (PAD_PA00 + 64)
#define DVP_DATA_PIN_FUNC_BASE     (GPIO_A0 + 64)
#define DVP_DATA_GPIO_PIN_BASE     64
#define DVP_DATA_PIN_SOURCE_ADDR   ((uint32_t)&((GPIO1_TypeDef *)GPIO1_BASE)->DIR2)

#else
    #error "DVP default resource macros are not defined for this platform"
#endif

#define DVP_DATA_GPIO_INSTANCE     hwp_gpio1
#define DVP_TIMER_INSTANCE         hwp_gptim1
#define DVP_TIMER_RCC_MODULE       RCC_MOD_GPTIM1
#define DVP_DMA_INSTANCE           DMA1_Channel1
#define DVP_DMA_REQUEST            8
#define DVP_DMA_IRQn               DMAC1_CH1_IRQn

//******************************** Defines **********************************//

//******************************** Typedefs *********************************//
/* DVP capture mode is the same concept as bus_capture_mode_t — use it directly
 * so the two layers share one enum and no conversion/assert is needed. */
#include "data_bus_adapter.h"

/* Forward declaration */
typedef struct dvp_handle dvp_handle_t;

/* DVP configuration structure */
typedef struct {
    bus_capture_mode_t  mode;                   /* DVP capture mode                          */
    uint32_t            buffer_size;            /* Image buffer size in bytes                */
    uint8_t             *frame_buffer;          /* Frame buffer pointer (user provided)      */
    uint8_t             *pingpong_buffer;       /* External ping-pong DMA buffer pool        */
    uint32_t            pingpong_pool_size;     /* Capacity of pingpong_buffer in bytes      */
    uint32_t            pingpong_buffer_size;   /* Ping-pong DMA buffer size in bytes        */
    uint32_t            pclk_pin_pad;
    uint32_t            pclk_pin_func;
    uint32_t            hsync_pin_pad;
    uint32_t            hsync_pin_func;
    uint8_t             vsync_pin;
    uint32_t            data_pin_pad_base;
    uint32_t            data_pin_func_base;
    uint32_t            data_gpio_pin_base;
    uint32_t            data_pin_source_addr;
    void                *data_gpio_instance;
    void                *timer_instance;
    uint32_t            timer_rcc_module;
    void                *dma_instance;
    uint32_t            dma_request;
    int32_t             dma_irqn;
    int                 xclk_pin;
    uint32_t            xclk_freq;
} dvp_config_t;

typedef struct {
    volatile uint8_t    frame_ready;
    volatile uint32_t   current_size;
    volatile uint8_t    enable_capture;
    volatile uint8_t    capture_started;
    volatile uint8_t    soi_found;
} dvp_capture_state_t;

typedef struct {
    volatile uint32_t   ring_write_pos;
} dvp_jpeg_parser_state_t;

/* DVP handle structure */
struct dvp_handle{
    dvp_config_t            config;
    GPT_HandleTypeDef       gptim;
    DMA_HandleTypeDef       dma;
    uint8_t                 *pingpong_buffer;
    dvp_capture_state_t     capture;
    dvp_jpeg_parser_state_t jpeg;
};
/* Function declarations */

#define DVP_BUS_ADAPTER_NAME "dvp"

//******************************** Typedefs *********************************//

//******************************** Function *********************************//
/*
 * Public DVP API. Core capture-control functions match `bus_adapter_ops_t`
 * entries directly; `self->priv` points at the DVP singleton handle.
 * Backend-specific tuning helpers remain DVP-private and are not exposed
 * through `bus_adapter_ops_t`.
 */

/** @brief Initialize DVP backend. */
int dvp_init(bus_adapter_t *self);

/** @brief Apply DVP config to singleton handle. */
int dvp_apply_config(bus_adapter_t *self, const dvp_config_t *config);

/** @brief Deinitialize DVP interface. */
int dvp_deinit(bus_adapter_t *self);

/** @brief Start DVP hardware (timer + DMA). */
int dvp_start(bus_adapter_t *self);

/** @brief Stop DVP hardware (timer + DMA). */
int dvp_stop(bus_adapter_t *self);

/** @brief Register a lightweight frame-ready callback. */
int dvp_set_frame_notify_callback(bus_adapter_t *self,
                                  bus_frame_notify_callback_t callback,
                                  void *user_data);

/** @brief Arm the next frame capture, optionally swapping the user buffer. */
int dvp_start_capture(bus_adapter_t *self, void *new_buffer, uint32_t buffer_size);

/** @brief Re-arm capture from inside the frame callback (no HW restart). */
int dvp_rearm_capture(bus_adapter_t *self, void *new_buffer, uint32_t buffer_size);

/** @brief Abort current capture (HW keeps running). */
int dvp_abort_capture(bus_adapter_t *self);

/** @brief Resize ping-pong DMA buffer. */
int dvp_set_pingpong_size(bus_adapter_t *self, uint32_t new_size);

/** @brief Set DVP capture mode. */
int dvp_set_mode(bus_adapter_t *self, bus_capture_mode_t mode);

/** @brief Dump DVP diagnostic registers to log. */
void dvp_dump_state(bus_adapter_t *self);

#ifdef __cplusplus
}
#endif

#endif /* __DVP_H__ */
