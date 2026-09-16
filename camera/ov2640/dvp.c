/**
 * @file    dvp.c
 * @brief   Software DVP (Digital Video Port) driver using GPTIM + DMA
 *
 * This module implements a software DVP interface for parallel camera
 * data capture using GPTIM1 as pixel/line clock and DMA with
 * ping-pong buffering for continuous JPEG frame acquisition.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#include "dvp.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"
#include "rthw.h"
#include <rtdevice.h>

#define DBG_TAG "dvp"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* Global DVP handle pointer (for interrupt callbacks) */
static dvp_handle_t *g_dvp_handle = NULL;

/* Search for JPEG SOI marker */
static int search_for_SOI(uint8_t *buffer, size_t length)
{
    if (length < 2) return -1;
    for (size_t i = 0; i < length - 1; i++)
    {
        if ((buffer[i] == (uint8_t)0xFF) && (buffer[i + 1] == (uint8_t)0xD8))
        {
            return i;
        }
    }
    return -1;
}

/* Search for JPEG EOI marker */
static int search_for_EOI(uint8_t *buffer, size_t length)
{
    if (length < 2) return -1;
    for (size_t i = 0; i < length - 1; i++)
    {
        if ((buffer[i] == (uint8_t)0xFF) && (buffer[i + 1] == (uint8_t)0xD9))
        {
            return i + 1;
        }
    }
    return -1;
}

/**
 * @brief Process JPEG data (generic function)
 * @param handle DVP handle pointer
 * @param buffer_offset Ping-pong buffer offset (0 for first half, half_size for second half)
 */
static void process_jpeg_data(dvp_handle_t *handle, uint32_t buffer_offset)
{
    uint8_t *full_buffer = handle->config.frame_buffer;
    uint8_t *pingpong_buffer = handle->pingpong_buffer;
    uint32_t half_size = handle->config.pingpong_buffer_size / 2;
    uint8_t *source_ptr = &pingpong_buffer[buffer_offset];
    uint32_t buffer_size = handle->config.buffer_size;

    if(full_buffer == NULL || buffer_size == 0)
        return;

    int index = 0;
    while(index<half_size && handle->enable_capture)
    {
        if(!handle->soi_found)
        {
            int soi_index = search_for_SOI(&source_ptr[index], half_size - index);
            if (soi_index >= 0)
            {
                rt_base_t level = rt_hw_interrupt_disable();
                handle->soi_found = 1;
                rt_hw_interrupt_enable(level);
                index += soi_index;
                int eoi_index = search_for_EOI(&source_ptr[index], half_size - index);
                if (eoi_index >= 0)
                {
                    // Found complete JPEG frame
                    uint32_t copy_size = eoi_index + 1;
                    if (handle->current_size + copy_size > buffer_size)
                    {
                        LOG_E("DVP buffer overflow: current=%d, copy=%d, buffer=%d", 
                                   handle->current_size, copy_size, buffer_size);
                        level = rt_hw_interrupt_disable();
                        handle->enable_capture = 0;
                        handle->soi_found = 0;
                        handle->frame_ready = 0;
                        handle->current_size = 0;
                        rt_hw_interrupt_enable(level);
                        break;
                    }
                    memcpy(&full_buffer[handle->current_size], &source_ptr[index], copy_size);
                    level = rt_hw_interrupt_disable();
                    handle->current_size += copy_size;
                    handle->frame_ready = 1;
                    handle->soi_found = 0;
                    handle->enable_capture = 0; // Stop capture, wait for processing
                    rt_hw_interrupt_enable(level);
                    index += copy_size;
                }
                else
                {
                    // Found SOI only, copy remaining data
                    uint32_t copy_size = half_size - index;
                    if (handle->current_size + copy_size > buffer_size)
                    {
                        LOG_E("DVP buffer overflow: current=%d, copy=%d, buffer=%d", 
                                   handle->current_size, copy_size, buffer_size);
                        level = rt_hw_interrupt_disable();
                        handle->enable_capture = 0;
                        handle->soi_found = 0;
                        handle->frame_ready = 0;
                        handle->current_size = 0;
                        rt_hw_interrupt_enable(level);
                        break;
                    }
                    memcpy(&full_buffer[handle->current_size], &source_ptr[index], copy_size);
                    level = rt_hw_interrupt_disable();
                    handle->current_size += copy_size;
                    rt_hw_interrupt_enable(level);
                    break;
                }

            }
            else
            {
                // SOI not found in this half, skip all
                break;
            }
        }

        if(handle->soi_found)
        {
            int eoi_index = search_for_EOI(&source_ptr[index], half_size - index);
            if (eoi_index >= 0)
            {
                // Found EOI, copy data and mark frame ready
                uint32_t copy_size = eoi_index + 1;
                if (handle->current_size + copy_size > buffer_size)
                {
                    LOG_E("DVP buffer overflow: current=%d, copy=%d, buffer=%d", 
                               handle->current_size, copy_size, buffer_size);
                    rt_base_t level = rt_hw_interrupt_disable();
                    handle->enable_capture = 0;
                    handle->soi_found = 0;
                    handle->frame_ready = 0;
                    handle->current_size = 0;
                    rt_hw_interrupt_enable(level);
                    break;
                }
                memcpy(&full_buffer[handle->current_size], &source_ptr[index], copy_size);
                rt_base_t level = rt_hw_interrupt_disable();
                handle->current_size += copy_size;
                handle->frame_ready = 1;
                handle->soi_found = 0;
                handle->enable_capture = 0; // Stop capture, wait for processing
                rt_hw_interrupt_enable(level);
                index += copy_size;
            }
            else
            {
                // EOI not found, copy remaining data
                uint32_t copy_size = half_size - index;
                if (handle->current_size + copy_size > buffer_size)
                {
                    LOG_E("DVP buffer overflow: current=%d, copy=%d, buffer=%d", 
                               handle->current_size, copy_size, buffer_size);
                    rt_base_t level = rt_hw_interrupt_disable();
                    handle->enable_capture = 0;
                    handle->soi_found = 0;
                    handle->frame_ready = 0;
                    handle->current_size = 0;
                    rt_hw_interrupt_enable(level);
                    break;
                }
                memcpy(&full_buffer[handle->current_size], &source_ptr[index], copy_size);
                rt_base_t level = rt_hw_interrupt_disable();
                handle->current_size += copy_size;
                rt_hw_interrupt_enable(level);
                break;
            }
        }

        if(handle->config.frame_callback && handle->frame_ready)
        {
            handle->config.frame_callback(handle, handle->config.callback_ctx);
            // If next buffer not updated, exit processing
            if(!handle->enable_capture){
                break;
            }
            full_buffer = handle->config.frame_buffer;
            rt_base_t level = rt_hw_interrupt_disable();
            handle->current_size = 0;
            rt_hw_interrupt_enable(level);
        }
    }
}

/* DMA transfer complete callback */
void dvp_dma_xfer_cplt_callback(DMA_HandleTypeDef *hdma)
{
    if (g_dvp_handle == NULL || !g_dvp_handle->enable_capture)
        return;

    dvp_handle_t *handle = g_dvp_handle;
    uint32_t half_size = handle->config.pingpong_buffer_size / 2;

    // JPEG mode: process second half of buffer
    if (handle->config.mode == DVP_MODE_JPEG)
    {
        process_jpeg_data(handle, half_size);
    }
    // Other modes not yet handled
}

/* DMA half transfer complete callback */
void dvp_dma_half_xfer_cplt_callback(DMA_HandleTypeDef *hdma)
{
    if (g_dvp_handle == NULL || !g_dvp_handle->enable_capture)
        return;

    dvp_handle_t *handle = g_dvp_handle;

    // JPEG mode: process first half of buffer
    if (handle->config.mode == DVP_MODE_JPEG)
    {
        process_jpeg_data(handle, 0);
    }
    // Other modes not yet handled
}

/* DMA error callback */
static void dvp_dma_error_callback(DMA_HandleTypeDef *hdma)
{
    LOG_E("DVP DMA Error occurred!");
}

/* VSYNC interrupt handler */
static void dvp_vsync_irq_handler(void *args)
{
    if (g_dvp_handle == NULL)
        return;

    dvp_handle_t *handle = g_dvp_handle;

    // JPEG mode: rely on SOI/EOI detection for frame boundaries
    if (handle->config.mode == DVP_MODE_JPEG)
    {
        // In JPEG mode, subsequent VSYNCs are not responded to, rely on EOI detection, just return.
        return;
    }
    // Other modes not yet handled
    // Check if capture is enabled
    if (!handle->enable_capture)
    {
        return;
    }
    // If previous frame not yet processed, don't start new capture
    if (handle->frame_ready)
    {
        return;
    }
}

/* Configure DVP data pins (D0-D7) */
static int dvp_config_data_pins(dvp_handle_t *handle)
{
    // Data pins must be mapped to GPIO1 bits 0-7 (because DMA reads low 8 bits of DIR register)
    for (int i = 0; i < 8; i++)
    {
        // Fixed mapping to GPIO_A0 ~ GPIO_A7
        HAL_PIN_Set(PAD_PA00 + i, GPIO_A0 + i, PIN_PULLUP, 1);
        
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = i;  // GPIO1 bits 0-7
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);
    }
    
    return RT_EOK;
}

/* Configure DVP control pins */
static int dvp_config_control_pins(dvp_handle_t *handle)
{
    /* Note: PCLK (GPTIM1_ETR) and HSYNC (GPTIM1_CH1) pins must be configured by user
     *       before calling dvp_init(). Example:
     *       HAL_PIN_Set(PAD_PA00 + pclk_pin, GPTIM1_ETR, PIN_PULLDOWN, 1);
     *       HAL_PIN_Set(PAD_PA00 + hsync_pin, GPTIM1_CH1, PIN_PULLDOWN, 1);
     */
    
    // Configure VSYNC interrupt
    rt_pin_attach_irq(handle->config.vsync_pin, PIN_IRQ_MODE_RISING, dvp_vsync_irq_handler, RT_NULL);
    rt_pin_irq_enable(handle->config.vsync_pin, PIN_IRQ_ENABLE);
    
    return RT_EOK;
}

/* Configure DMA */
static int dvp_config_dma(dvp_handle_t *handle)
{
    handle->dma.Instance = DMA1_Channel1;
    handle->dma.Init.Request = 8;
    handle->dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    handle->dma.Init.PeriphInc = DMA_PINC_DISABLE;
    handle->dma.Init.MemInc = DMA_MINC_ENABLE;
    handle->dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    handle->dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    handle->dma.Init.Mode = DMA_CIRCULAR;
    handle->dma.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    handle->dma.Init.BurstSize = 0;
    handle->dma.XferCpltCallback = dvp_dma_xfer_cplt_callback;
    handle->dma.XferHalfCpltCallback = dvp_dma_half_xfer_cplt_callback;
    handle->dma.XferErrorCallback = dvp_dma_error_callback;
    
    if (HAL_DMA_Init(&handle->dma) != HAL_OK)
    {
        LOG_E("DVP DMA init failed!");
        return -RT_ERROR;
    }
    
    __HAL_LINKDMA(&handle->gptim, hdma[GPT_DMA_ID_UPDATE], handle->dma);
    __HAL_DMA_ENABLE_IT(&handle->dma, DMA_IT_TC);
    HAL_NVIC_SetPriority(DMAC1_CH1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMAC1_CH1_IRQn);
    
    return RT_EOK;
}

/* Configure timer */
static int dvp_config_timer(dvp_handle_t *handle)
{
    HAL_RCC_EnableModule(RCC_MOD_GPTIM1);
    
    // Initialize GPTIM1
    handle->gptim.Instance = hwp_gptim1;
    handle->gptim.Init.Prescaler = 0;
    handle->gptim.Init.CounterMode = GPT_COUNTERMODE_DOWN;
    handle->gptim.Init.Period = 0x0;
    
    if (HAL_GPT_Base_Init(&handle->gptim) != HAL_OK)
    {
        LOG_E("DVP GPTIM1 init failed!");
        return -RT_ERROR;
    }
    
    __HAL_GPT_ENABLE_IT(&handle->gptim, GPT_IT_UPDATE);
    
    // Configure external clock source (PCLK via ETR)
    GPT_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = GPT_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = GPT_TRIGGERPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = GPT_CLOCKPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0;
    
    if (HAL_GPT_ConfigClockSource(&handle->gptim, &sClockSourceConfig) != HAL_OK)
    {
        LOG_E("DVP GPTIM1 clock config failed!");
        return -RT_ERROR;
    }
    
    // Configure gate mode (HSYNC as gate signal)
    GPT_SlaveConfigTypeDef sSlaveConfig = {0};
    sSlaveConfig.SlaveMode = GPT_SLAVEMODE_GATED;
    sSlaveConfig.InputTrigger = GPT_TS_TI1FP1;
    sSlaveConfig.TriggerPolarity = GPT_INPUTCHANNELPOLARITY_FALLING;
    sSlaveConfig.TriggerFilter = 0;
    
    if (HAL_GPT_SlaveConfigSynchronization(&handle->gptim, &sSlaveConfig) != HAL_OK)
    {
        LOG_E("DVP GPTIM1 slave config failed!");
        return -RT_ERROR;
    }
    
    // Configure input capture channel 1 (HSYNC)
    GPT_IC_InitTypeDef sConfigIC = {0};
    sConfigIC.ICPolarity = GPT_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = GPT_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = GPT_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    
    if (HAL_GPT_IC_ConfigChannel(&handle->gptim, &sConfigIC, GPT_CHANNEL_1) != HAL_OK)
    {
        LOG_E("DVP GPTIM1 IC config failed!");
        return -RT_ERROR;
    }
    
    // Enable DMA request
    __HAL_GPT_ENABLE_DMA(&handle->gptim, GPT_DMA_UPDATE);
    
    return RT_EOK;
}

/* Public function implementations */

int dvp_init(dvp_handle_t *handle, const dvp_config_t *config)
{
    if (handle == NULL || config == NULL )
    {
        LOG_E("DVP init: invalid parameters!");
        return -RT_EINVAL;
    }
    
    // Save configuration
    memcpy(&handle->config, config, sizeof(dvp_config_t));
    
    // Allocate ping-pong buffer
    if(handle->config.pingpong_buffer_size % 2 != 0)
    {
        LOG_W("DVP init: pingpong_buffer_size not even, rounding up");
        handle->config.pingpong_buffer_size += 1;
    }
    handle->pingpong_buffer = (uint8_t *)rt_malloc(handle->config.pingpong_buffer_size);
    if (handle->pingpong_buffer == NULL)
    {
        LOG_E("DVP init: failed to allocate pingpong buffer!");
        return -RT_ENOMEM;
    }
    
    // Initialize state
    handle->frame_ready = 0;
    handle->current_size = 0;
    handle->soi_found = 0;
    handle->enable_capture = 1;
    
    // Set global handle
    g_dvp_handle = handle;
    
    // Configure hardware
    if (dvp_config_data_pins(handle) != 0)
    {
        rt_free(handle->pingpong_buffer);
        return -RT_ERROR;
    }
    
    if (dvp_config_control_pins(handle) != 0)
    {
        rt_free(handle->pingpong_buffer);
        return -RT_ERROR;
    }
    
    if (dvp_config_dma(handle) != 0)
    {
        rt_free(handle->pingpong_buffer);
        return -RT_ERROR;
    }
    
    if (dvp_config_timer(handle) != 0)
    {
        rt_free(handle->pingpong_buffer);
        return -RT_ERROR;
    }
    
    LOG_I("DVP initialized successfully");
    LOG_I("  Mode: %s", 
               config->mode == DVP_MODE_JPEG ? "JPEG" :
               config->mode == DVP_MODE_RAW ? "RAW" :
               config->mode == DVP_MODE_YUV422 ? "YUV422" : "RGB565");
    LOG_I("  Buffer size: %d bytes", config->buffer_size);
    LOG_I("  Pingpong buffer: %d bytes", config->pingpong_buffer_size);
    LOG_I("  VSYNC: PA%d (GPIO interrupt)", config->vsync_pin);
    
    return RT_EOK;
}

int dvp_deinit(dvp_handle_t *handle)
{
    if (handle == NULL)
        return -RT_EINVAL;
    
    // Stop hardware
    dvp_stop(handle);
    
    // Deinitialize DMA
    HAL_NVIC_DisableIRQ(DMAC1_CH1_IRQn);
    HAL_DMA_DeInit(&handle->dma);
    
    // Deinitialize Timer
    __HAL_GPT_DISABLE_DMA(&handle->gptim, GPT_DMA_UPDATE);
    __HAL_GPT_DISABLE_IT(&handle->gptim, GPT_IT_UPDATE);
    HAL_GPT_Base_DeInit(&handle->gptim);
    
    // Release resources
    if (handle->pingpong_buffer)
    {
        rt_free(handle->pingpong_buffer);
        handle->pingpong_buffer = NULL;
    }
    
    // Disable VSYNC interrupt
    rt_pin_irq_enable(handle->config.vsync_pin, PIN_IRQ_DISABLE);
    rt_pin_detach_irq(handle->config.vsync_pin);
    
    // Reset state
    handle->frame_ready = 0;
    handle->current_size = 0;
    handle->soi_found = 0;
    handle->enable_capture = 0;
    
    // Clear global handle
    g_dvp_handle = NULL;
    
    LOG_I("DVP deinitialized");
    return RT_EOK;
}

int dvp_start(dvp_handle_t *handle)
{
    if (handle == NULL)
        return -RT_EINVAL;
    
    // Start DMA circular transfer (JPEG mode requires continuous SOI detection)
    if (HAL_DMA_Start_IT(&handle->dma,
                        (uint32_t)&hwp_gpio1->DIR,
                        (uint32_t)handle->pingpong_buffer,
                        handle->config.pingpong_buffer_size) != HAL_OK)
    {
        LOG_E("DVP start DMA failed!");
        return -RT_ERROR;
    }
    
    // Start timer (driven by PCLK, gated by HSYNC)
    if (HAL_GPT_Base_Start(&handle->gptim) != HAL_OK)
    {
        LOG_E("DVP start timer failed!");
        HAL_DMA_Abort(&handle->dma);
        return -RT_ERROR;
    }
    
    LOG_D("DVP hardware started (DMA and Timer running)");
    return RT_EOK;
}

int dvp_stop(dvp_handle_t *handle)
{
    if (handle == NULL)
        return -RT_EINVAL;
    
    // Stop timer and DMA
    HAL_GPT_Base_Stop(&handle->gptim);
    HAL_DMA_Abort(&handle->dma);
    
    // Clear capture flag
    handle->enable_capture = 0;
    
    LOG_D("DVP hardware stopped");
    return RT_EOK;
}

int dvp_start_capture(dvp_handle_t *handle, uint8_t *new_buffer, uint32_t buffer_size)
{
    if (handle == NULL)
        return -RT_EINVAL;
    
    // If new buffer specified, update configuration
    if (new_buffer != NULL)
    {
        if (buffer_size == 0)
        {
            LOG_E("DVP error: buffer_size must be provided when new_buffer is not NULL");
            return -RT_EINVAL;
        }
        
        handle->config.frame_buffer = new_buffer;
        handle->config.buffer_size = buffer_size;
        LOG_D("DVP capture buffer updated to 0x%08X, size: %d bytes", 
                   (uint32_t)new_buffer, buffer_size);
    }
    
    // Only set flags, don't operate hardware (hardware started by dvp_start)
    rt_base_t level = rt_hw_interrupt_disable();
    handle->frame_ready = 0;
    handle->current_size = 0;
    handle->soi_found = 0;
    handle->enable_capture = 1;
    rt_hw_interrupt_enable(level);
    
    LOG_D("DVP capture enabled.");
    return RT_EOK;
}

int dvp_abort_capture(dvp_handle_t *handle)
{
    if (handle == NULL)
        return -RT_EINVAL;
    
    // Only clear flags, hardware continues running (continue detecting SOI)
    rt_base_t level = rt_hw_interrupt_disable();
    handle->enable_capture = 0;
    handle->frame_ready = 0;
    rt_hw_interrupt_enable(level);
    
    LOG_D("DVP capture aborted.");
    return RT_EOK;
}

int dvp_is_frame_ready(dvp_handle_t *handle)
{
    if (handle == NULL)
        return 0;
    
    return handle->frame_ready;
}

uint32_t dvp_get_frame_size(dvp_handle_t *handle)
{
    if (handle == NULL)
        return 0;
    
    return handle->current_size;
}

void dvp_reset_capture(dvp_handle_t *handle)
{
    if (handle == NULL)
        return;
    
    rt_base_t level = rt_hw_interrupt_disable();
    handle->frame_ready = 0;
    handle->current_size = 0;
    handle->soi_found = 0;
    handle->enable_capture = 1;
    rt_hw_interrupt_enable(level);
}
