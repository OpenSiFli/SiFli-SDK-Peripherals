/********************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file dvp.c
 *
 * @par dependencies
 * - rtthread.h
 * - dvp.h
 * - stdint.h
 * - stdio.h
 * - string.h
 * - rthw.h
 * - rtdevice.h
 *
 * @author SiFli 思澈科技
 *
 * @brief DVP (Digital Video Port) software driver implementation.
 *
 * This module implements a software DVP interface used to capture camera
 * parallel data (D0..D7) using a GPTIM as pixel/line clock and DMA with
 * ping-pong buffering. It handles DMA callbacks, VSYNC interrupts and offers
 * a small API to start/stop capture, configure ping-pong buffers and query
 * capture status.
 *
 * Processing flow:
 * - Initialize with `dvp_init()` which allocates buffers and configures
 *   pins, DMA and timer resources.
 * - Hardware delivers pixels into a ping-pong DMA buffer. DMA callbacks
 *   call into this module to process halves or full transfers.
 * - For JPEG mode the driver searches SOI/EOI markers to form frames;
 *   for raw/pixel modes it copies fixed-size chunks until the user buffer
 *   is filled.
 * - The driver notifies the upper layer via the user callback registered with
 *   `bus_adapter_set_frame_notify_callback()`.
 *
 * Notes:
 * - This implementation binds to board-specific timer/DMA instances and
 *   pin mappings; platform abstraction is minimal for performance reasons.
 * - Callers should treat these APIs as non-ISR (except callbacks) and
 *   avoid long-running work in callback contexts.
 *
 * @version V1.0 2026-4-3
 *
 * @note 1 tab == 4 spaces!
 *
 ******************************************************************************/
#include "dvp.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"
#include "rthw.h"
#include <stddef.h>
#include <rtdevice.h>

#define DBG_TAG "dvp"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define DEBUG_DVP 0

#if DEBUG_DVP
#define DVP_DEBUG_SNAPSHOT_SIZE 6000U
static uint8_t g_dvp_debug_snapshot[DVP_DEBUG_SNAPSHOT_SIZE];
#endif

static dvp_handle_t s_dvp_handle;
static bus_frame_notify_callback_t s_user_frame_callback;
static void *s_user_frame_callback_data;
static bus_adapter_t s_dvp_bus_adapter;

#ifdef CAMERA_DVP_PINGPONG_USE_SECTION
static uint8_t s_dvp_pingpong_pool[CAMERA_DVP_PINGPONG_POOL_SIZE]
    __attribute__((section(".dvp_pingpong")));
#else
static uint8_t s_dvp_pingpong_pool[CAMERA_DVP_PINGPONG_POOL_SIZE];
#endif

static GPT_HandleTypeDef s_xclk_gptim;
static rt_bool_t         s_xclk_initialized = RT_FALSE;

/**
 * @brief Validate DVP configuration integrity before initialization.
 *
 * @param config Configuration to validate.
 * @return RT_EOK when valid, otherwise -RT_EINVAL.
 */
static int dvp_validate_config(const dvp_config_t *config)
{
    if (config == RT_NULL)
        return -RT_EINVAL;

    if (config->pingpong_buffer_size == 0)
        return -RT_EINVAL;

    if (config->pingpong_buffer == RT_NULL ||
        config->pingpong_pool_size == 0 ||
        config->pingpong_buffer_size > config->pingpong_pool_size)
    {
        return -RT_EINVAL;
    }

    if (config->timer_instance == RT_NULL ||
        config->dma_instance == RT_NULL ||
        config->data_gpio_instance == RT_NULL ||
        config->data_pin_source_addr == 0)
    {
        return -RT_EINVAL;
    }

    return RT_EOK;
}

static void dvp_dispatch_frame(dvp_handle_t *handle, uint8_t *buffer, uint32_t size)
{
    if (s_user_frame_callback == RT_NULL)
        return;

    s_user_frame_callback(buffer,
                          size,
                          s_user_frame_callback_data);
}


/**
 * @brief Resolve parent DVP handle from embedded DMA handle pointer.
 *
 * @param hdma DMA handle address passed by HAL callback.
 * @return Owning DVP handle, or RT_NULL when @p hdma is RT_NULL.
 */
static dvp_handle_t *dvp_get_handle_from_dma(DMA_HandleTypeDef *hdma)
{
    if (hdma == RT_NULL)
    {
        return RT_NULL;
    }

    return (dvp_handle_t *)((uint8_t *)hdma - offsetof(dvp_handle_t, dma));
}

/**
 * @brief Get typed GPT register instance from DVP handle resources.
 */
static GPT_TypeDef *dvp_get_timer_instance(const dvp_handle_t *handle)
{
    return (GPT_TypeDef *)handle->config.timer_instance;
}

/**
 * @brief Get typed GPIO register instance for DVP data bus.
 */
static GPIO_TypeDef *dvp_get_data_gpio_instance(const dvp_handle_t *handle)
{
    return (GPIO_TypeDef *)handle->config.data_gpio_instance;
}

/** @brief Fill default DVP resource fields from board macros. */
static void dvp_apply_default_resource_config(dvp_config_t *config)
{
    config->pclk_pin_pad            = DVP_PCLK_PIN_PAD(CAMERA_DVP_PCLK_PIN);
    config->pclk_pin_func           = DVP_PCLK_PIN_FUNC;
    config->hsync_pin_pad           = DVP_HSYNC_PIN_PAD(CAMERA_DVP_HSYNC_PIN);
    config->hsync_pin_func          = DVP_HSYNC_PIN_FUNC;
    config->vsync_pin               = (uint8_t)CAMERA_DVP_VSYNC_PIN;
    config->data_pin_pad_base       = DVP_DATA_PIN_PAD_BASE;
    config->data_pin_func_base      = DVP_DATA_PIN_FUNC_BASE;
    config->data_gpio_pin_base      = DVP_DATA_GPIO_PIN_BASE;
    config->data_pin_source_addr    = DVP_DATA_PIN_SOURCE_ADDR;
    config->data_gpio_instance      = DVP_DATA_GPIO_INSTANCE;
    config->timer_instance          = DVP_TIMER_INSTANCE;
    config->timer_rcc_module        = DVP_TIMER_RCC_MODULE;
    config->dma_instance            = DVP_DMA_INSTANCE;
    config->dma_request             = DVP_DMA_REQUEST;
    config->dma_irqn                = DVP_DMA_IRQn;
}

#if DEBUG_DVP
static void dump_buffer_hex(const uint8_t *buffer,
                            uint32_t length,
                            uint32_t pingpong_offset,
                            uint32_t frame_offset)
{
    uint32_t index = 0;
    char line[80];

    rt_kprintf("DVP raw chunk: pingpong_offset=%u frame_offset=%u bytes=%u\n",
               (unsigned int)pingpong_offset,
               (unsigned int)frame_offset,
               (unsigned int)length);
    while (index < length)
    {
        uint32_t line_bytes = length - index;
        if (line_bytes > 16U)
        {
            line_bytes = 16U;
        }
        int pos = rt_snprintf(line,
                              sizeof(line),
                              "F%04u P%04u: ",
                              (unsigned int)(frame_offset + index),
                              (unsigned int)(pingpong_offset + index));

        for (uint32_t i = 0; i < line_bytes; i++)
        {
            pos += rt_snprintf(&line[pos], sizeof(line) - pos, "%02X ", buffer[index + i]);
        }

        rt_kprintf("%s\n", line);
        index += line_bytes;
    }
}

static void snapshot_and_dump_buffer(const uint8_t *buffer,
                                     uint32_t length,
                                     uint32_t pingpong_offset,
                                     uint32_t frame_offset)
{
    uint32_t snapshot_length = DVP_DEBUG_SNAPSHOT_SIZE;
    if (length <= DVP_DEBUG_SNAPSHOT_SIZE)
    {
        snapshot_length = length;
    }

    rt_memcpy(g_dvp_debug_snapshot, buffer, snapshot_length);
    if (snapshot_length < length)
    {
        rt_kprintf("DVP raw chunk truncated: requested=%u snapshot=%u\n",
                   (unsigned int)length,
                   (unsigned int)snapshot_length);
    }

    dump_buffer_hex(g_dvp_debug_snapshot, snapshot_length, pingpong_offset, frame_offset);
}
#endif /* DEBUG_DVP */

/**
 * @brief Clear JPEG boundary tracking state.
 */
static void dvp_reset_jpeg_boundary_state(dvp_handle_t *handle)
{
    handle->jpeg.ring_write_pos = 0;
}

/** @brief Process one RAW/RGB/YUV ping-pong half-buffer. */
static void process_raw_data(dvp_handle_t *handle, uint32_t buffer_offset)
{
#if DEBUG_DVP
    /* Debug placeholder. */
#endif

    uint8_t *full_buffer = handle->config.frame_buffer;
    uint8_t *pingpong_buffer = handle->pingpong_buffer;
    uint32_t half_size = handle->config.pingpong_buffer_size / 2;
    uint8_t *source_ptr = &pingpong_buffer[buffer_offset];
    uint32_t buffer_size = handle->config.buffer_size;

    if (full_buffer == NULL || buffer_size == 0)
    {
        rt_base_t level = rt_hw_interrupt_disable();
        handle->capture.enable_capture = 0;
        handle->capture.capture_started = 0;
        handle->capture.frame_ready = 0;
        rt_hw_interrupt_enable(level);
        return;
    }

    if (!handle->capture.soi_found)
        return;

    uint32_t remaining = buffer_size - handle->capture.current_size;
    if (remaining == 0)
        return;

    uint32_t copy_size = remaining;
    if (half_size <= remaining)
    {
        copy_size = half_size;
    }

#if DEBUG_DVP
    if (handle->capture.current_size >= (614400 / 8)*3)
    {
        __HAL_DMA_DISABLE(&handle->dma);
        rt_kprintf("\n[DEBUG] --- SOURCE DUMP (Pingpong Buffer) ---\n");
        snapshot_and_dump_buffer(source_ptr, 1400, buffer_offset, handle->capture.current_size);
        RT_ASSERT(0);
    }
#endif
#ifdef PSRAM_CACHE_WB
    mpu_dcache_invalidate((uint32_t *)source_ptr, copy_size);
#endif
    memcpy(full_buffer + handle->capture.current_size, source_ptr, copy_size);

#if DEBUG_DVP
    if (handle->capture.current_size >= (614400 / 8)*3)
    {
        rt_kprintf("\n[DEBUG] --- DESTINATION DUMP (PSRAM FB) ---\n");
        snapshot_and_dump_buffer(full_buffer + handle->capture.current_size, copy_size, buffer_offset, handle->capture.current_size);
        RT_ASSERT(0);
    }
#endif
    
    rt_base_t level = rt_hw_interrupt_disable();
    handle->capture.current_size += copy_size;

    if (handle->capture.current_size >= buffer_size)
    {     
        handle->capture.frame_ready = 1;
        handle->capture.soi_found = 0;
        handle->capture.enable_capture = 0;        
        handle->capture.capture_started = 0;
    }
    rt_hw_interrupt_enable(level);

    if (handle->capture.frame_ready)
    {
#ifdef PSRAM_CACHE_WB
        mpu_dcache_clean(full_buffer, buffer_size);
#endif
        dvp_dispatch_frame(handle, full_buffer, handle->capture.current_size);
    }
}

/** @brief Process one JPEG ping-pong half-buffer. */
static void process_jpeg_data(dvp_handle_t *handle, uint32_t buffer_offset)
{
    uint8_t *ring_buffer = handle->config.frame_buffer;
    uint8_t *pingpong_buffer = handle->pingpong_buffer;
    uint32_t half_size = handle->config.pingpong_buffer_size / 2;
    uint8_t *source_ptr = &pingpong_buffer[buffer_offset];
    uint32_t ring_size = handle->config.buffer_size;
    uint32_t write_pos;

    if (ring_buffer == NULL || ring_size == 0 || half_size == 0)
    {
        rt_base_t level = rt_hw_interrupt_disable();
        handle->capture.enable_capture = 0;
        handle->capture.capture_started = 0;
        handle->capture.frame_ready = 0;
        rt_hw_interrupt_enable(level);
        return;
    }

#ifdef PSRAM_CACHE_WB
    mpu_dcache_invalidate((uint32_t *)source_ptr, half_size);
#endif

    write_pos = handle->jpeg.ring_write_pos;
    if (write_pos >= ring_size)
    {
        write_pos = 0;
    }

    if (write_pos + half_size <= ring_size)
    {
        memcpy(&ring_buffer[write_pos], source_ptr, half_size);
        dvp_dispatch_frame(handle, &ring_buffer[write_pos], half_size);
        write_pos += half_size;
        if (write_pos >= ring_size)
        {
            write_pos = 0;
        }
    }
    else
    {
        uint32_t tail = ring_size - write_pos;
        uint32_t head = half_size - tail;
        memcpy(&ring_buffer[write_pos], source_ptr, tail);
        dvp_dispatch_frame(handle, &ring_buffer[write_pos], tail);
        memcpy(&ring_buffer[0], &source_ptr[tail], head);
        dvp_dispatch_frame(handle, &ring_buffer[0], head);
        write_pos = head;
    }

    handle->capture.current_size = half_size;
    handle->jpeg.ring_write_pos = write_pos;
}



/** @brief DMA full-transfer callback. */
void dvp_dma_xfer_cplt_callback(DMA_HandleTypeDef *hdma)
{

    dvp_handle_t *handle = dvp_get_handle_from_dma(hdma);

    if (handle == RT_NULL || !handle->capture.enable_capture)
        return;

    uint32_t half_size = handle->config.pingpong_buffer_size / 2;

    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        process_jpeg_data(handle, half_size);
    }
    else
    { 
        process_raw_data(handle, half_size);
    }
}

/** @brief DMA half-transfer callback. */
void dvp_dma_half_xfer_cplt_callback(DMA_HandleTypeDef *hdma)
{
    dvp_handle_t *handle = dvp_get_handle_from_dma(hdma);

    if (handle == RT_NULL || !handle->capture.enable_capture)
        return;

    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        process_jpeg_data(handle, 0);
    }
    else
    {
        process_raw_data(handle, 0);
    }
}

/** @brief DMA error callback. */
static void dvp_dma_error_callback(DMA_HandleTypeDef *hdma)
{
    LOG_E("DVP DMA Error occurred! hdma=%p", hdma);
}

/** @brief VSYNC rising-edge IRQ handler. */
static void dvp_vsync_irq_handler(void *args)
{
    bus_adapter_t *self = (bus_adapter_t *)args;
    if (self == RT_NULL)
        return;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    if (handle == RT_NULL)
        return;

    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        return;
    }

    if (!handle->capture.enable_capture)
    {
        return;
    }
    if (handle->capture.capture_started)
    {
        return;
    }
    if (handle->capture.frame_ready)
    {
        return;
    }
#if DEBUG_DVP
// For debugging purposes, we can toggle a GPIO pin here to measure VSYNC timing or trigger an oscilloscope capture.
    GPIO_TypeDef *gpio = hwp_gpio1;
    GPIO_InitTypeDef GPIO_InitStruct;
    /* set GPIO1 pin10 to output mode */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = 74;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);
    /* set pin to high */
    HAL_GPIO_WritePin(gpio, 74, GPIO_PIN_SET);
#endif
    dvp_start(self);
}
/** @brief Configure DVP data pins. */
static int dvp_config_data_pins(dvp_handle_t *handle)
{
    GPIO_TypeDef *data_gpio = dvp_get_data_gpio_instance(handle);

    for (int i = 0; i < 8; i++)
    {
        HAL_PIN_Set(handle->config.data_pin_pad_base + i,
                    handle->config.data_pin_func_base + i,
                    PIN_PULLUP,
                    1);
        
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = handle->config.data_gpio_pin_base + i;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(data_gpio, &GPIO_InitStruct);
    }
    
    return RT_EOK;
}

/** @brief Configure VSYNC pin and IRQ. */
static int dvp_config_control_pins(dvp_handle_t *handle)
{
    rt_pin_attach_irq(handle->config.vsync_pin, PIN_IRQ_MODE_RISING, dvp_vsync_irq_handler, &s_dvp_bus_adapter);
    rt_pin_irq_enable(handle->config.vsync_pin, PIN_IRQ_ENABLE);

    return RT_EOK;
}

/** @brief Configure DVP DMA channel and callbacks. */
static int dvp_config_dma(dvp_handle_t *handle)
{
    handle->dma.Instance = handle->config.dma_instance;
    handle->dma.Init.Request = handle->config.dma_request;
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
    HAL_NVIC_SetPriority(handle->config.dma_irqn, 1, 0);
    HAL_NVIC_EnableIRQ(handle->config.dma_irqn);
    
    return RT_EOK;
}
/** @brief Configure DVP capture timer. */
static int dvp_config_timer(dvp_handle_t *handle)
{
    HAL_PIN_Set(handle->config.pclk_pin_pad,
                handle->config.pclk_pin_func,
                PIN_PULLUP,
                1);
    HAL_PIN_Set(handle->config.hsync_pin_pad,
                handle->config.hsync_pin_func,
                PIN_PULLUP,
                1);

    HAL_RCC_EnableModule(handle->config.timer_rcc_module);

    handle->gptim.Instance = handle->config.timer_instance;
    handle->gptim.Init.Prescaler = 0;
    handle->gptim.Init.CounterMode = GPT_COUNTERMODE_DOWN;
    handle->gptim.Init.Period = 0x0;
    
    if (HAL_GPT_Base_Init(&handle->gptim) != HAL_OK)
    {
        LOG_E("DVP timer init failed!");
        return -RT_ERROR;
    }
    
    GPT_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = GPT_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = GPT_TRIGGERPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = GPT_CLOCKPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0;
    
    if (HAL_GPT_ConfigClockSource(&handle->gptim, &sClockSourceConfig) != HAL_OK)
    {
        LOG_E("DVP timer clock config failed!");
        return -RT_ERROR;
    }
    
    GPT_SlaveConfigTypeDef sSlaveConfig = {0};
    sSlaveConfig.SlaveMode = GPT_SLAVEMODE_GATED;
    sSlaveConfig.InputTrigger = GPT_TS_TI1FP1;
    sSlaveConfig.TriggerPolarity = GPT_INPUTCHANNELPOLARITY_RISING;
    sSlaveConfig.TriggerFilter = 0;
    
    if (HAL_GPT_SlaveConfigSynchronization(&handle->gptim, &sSlaveConfig) != HAL_OK)
    {
        LOG_E("DVP timer slave config failed!");
        return -RT_ERROR;
    }
    
    GPT_IC_InitTypeDef sConfigIC = {0};
    sConfigIC.ICPolarity = GPT_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = GPT_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = GPT_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    
    if (HAL_GPT_IC_ConfigChannel(&handle->gptim, &sConfigIC, GPT_CHANNEL_1) != HAL_OK)
    {
        LOG_E("DVP timer IC config failed!");
        return -RT_ERROR;
    }
    
    __HAL_GPT_ENABLE_DMA(&handle->gptim, GPT_DMA_UPDATE);
    
    return RT_EOK;
}
/*
 *******************************************************************************
 * XCLK (sensor master clock) generation via GPTIM2
 *******************************************************************************
 */

/** @brief Start XCLK PWM on given PA pin index. */
static void dvp_xclk_start(int pin, uint32_t freq)
{
    HAL_StatusTypeDef status;
    uint32_t timer_clk;
    uint32_t period;
    GPT_OC_InitTypeDef sConfigOC = {0};

    if (s_xclk_initialized)
    {
        HAL_GPT_PWM_Stop(&s_xclk_gptim, GPT_CHANNEL_1);
        HAL_GPT_Base_DeInit(&s_xclk_gptim);
        s_xclk_initialized = RT_FALSE;
    }

    HAL_PIN_Set(PAD_PA00 + pin, GPTIM2_CH1, PIN_NOPULL, 1);
    HAL_RCC_EnableModule(RCC_MOD_GPTIM2);

#if defined(SOC_SF32LB52X) && SOC_SF32LB52X == 1
    timer_clk = 24000000;
#else
    timer_clk = HAL_RCC_GetPCLKFreq(s_xclk_gptim.core, 1);
#endif

    period = (timer_clk / freq) - 1;
    if (period < 1 || period > 0xFFFF)
    {
        LOG_E("XCLK: frequency %u Hz out of range (timer_clk=%u Hz)",
              (unsigned)freq, (unsigned)timer_clk);
        return;
    }

    s_xclk_gptim.Instance         = hwp_gptim2;
    s_xclk_gptim.Init.Prescaler   = 0;
    s_xclk_gptim.Init.CounterMode = GPT_COUNTERMODE_UP;
    s_xclk_gptim.Init.Period      = period;

    status = HAL_GPT_Base_Init(&s_xclk_gptim);
    if (status != HAL_OK)
    {
        LOG_E("XCLK: GPTIM2 base init failed (%d)", status);
        return;
    }

    sConfigOC.OCMode     = GPT_OCMODE_PWM1;
    sConfigOC.Pulse      = period / 2 + 1;  /* 50 % duty cycle */
    sConfigOC.OCPolarity = GPT_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = GPT_OCFAST_DISABLE;

    status = HAL_GPT_PWM_ConfigChannel(&s_xclk_gptim, &sConfigOC, GPT_CHANNEL_1);
    if (status != HAL_OK)
    {
        LOG_E("XCLK: GPTIM2 PWM config failed (%d)", status);
        HAL_GPT_Base_DeInit(&s_xclk_gptim);
        return;
    }

    status = HAL_GPT_PWM_Start(&s_xclk_gptim, GPT_CHANNEL_1);
    if (status != HAL_OK)
    {
        LOG_E("XCLK: GPTIM2 PWM start failed (%d)", status);
        HAL_GPT_Base_DeInit(&s_xclk_gptim);
        return;
    }

    s_xclk_initialized = RT_TRUE;
    rt_thread_mdelay(10);  /* wait for clock to stabilise before SCCB access */
    LOG_I("XCLK: %u Hz on PA%d (period=%u)", (unsigned)freq, pin, (unsigned)period);
}

/** @brief Stop XCLK PWM and restore pin to GPIO mode. */
static void dvp_xclk_stop(int pin)
{
    if (!s_xclk_initialized)
        return;

    HAL_GPT_PWM_Stop(&s_xclk_gptim, GPT_CHANNEL_1);
    HAL_GPT_Base_DeInit(&s_xclk_gptim);
    s_xclk_initialized = RT_FALSE;
    HAL_PIN_Set(PAD_PA00 + pin, GPIO_A0 + pin, PIN_NOPULL, 1);
    LOG_I("XCLK: stopped (PA%d)", pin);
}

/** @brief Initialize DVP backend state and hardware resources. */
int dvp_init(bus_adapter_t *self)
{
    if (self == NULL)
    {
        LOG_E("DVP init: invalid parameters!");
        return -RT_EINVAL;
    }
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    if (handle->config.mode > BUS_CAPTURE_MODE_RGB565)
    {
        handle->config.mode = BUS_CAPTURE_MODE_JPEG;
    }
    if (handle->config.pingpong_buffer == RT_NULL)
    {
        handle->config.pingpong_buffer = s_dvp_pingpong_pool;
    }
    if (handle->config.pingpong_pool_size == 0)
    {
        handle->config.pingpong_pool_size = (uint32_t)sizeof(s_dvp_pingpong_pool);
    }
    if (handle->config.pingpong_buffer_size == 0)
    {
        handle->config.pingpong_buffer_size = CAMERA_DVP_PINGPONG_BUFFER_SIZE;
    }
    if (handle->config.timer_instance == RT_NULL ||
        handle->config.dma_instance == RT_NULL ||
        handle->config.data_gpio_instance == RT_NULL ||
        handle->config.data_pin_source_addr == 0)
    {
        dvp_apply_default_resource_config(&handle->config);
    }
    if (handle->config.xclk_pin < 0)
    {
        handle->config.xclk_pin = CAMERA_DVP_XCLK_PIN;
    }
    if (handle->config.xclk_freq == 0)
    {
        handle->config.xclk_freq = CAMERA_DVP_XCLK_FREQ;
    }

    if (dvp_validate_config(&handle->config) != RT_EOK)
    {
        LOG_E("DVP init: configuration not applied or invalid");
        return -RT_EINVAL;
    }
    
    if(handle->config.pingpong_buffer_size % 2 != 0)
    {
        LOG_W("DVP init: pingpong_buffer_size not even, rounding up");
        handle->config.pingpong_buffer_size += 1;
    }
    if (handle->config.pingpong_buffer_size > handle->config.pingpong_pool_size)
    {
        LOG_E("DVP init: pingpong_buffer_size (%d) exceeds pool size (%d)",
              handle->config.pingpong_buffer_size, handle->config.pingpong_pool_size);
        return -RT_EINVAL;
    }
    handle->pingpong_buffer = handle->config.pingpong_buffer;
    
    handle->capture.frame_ready = 0;
    handle->capture.current_size = 0;
    handle->capture.soi_found = 0;
    handle->capture.enable_capture = 1;
    handle->capture.capture_started = 0;
    dvp_reset_jpeg_boundary_state(handle);
    
    if (dvp_config_data_pins(handle) != 0)
        return -RT_ERROR;
    if (dvp_config_control_pins(handle) != 0)
        return -RT_ERROR;
    if (dvp_config_dma(handle) != 0)
        return -RT_ERROR;
    if (dvp_config_timer(handle) != 0)
        return -RT_ERROR;

    if (handle->config.xclk_pin >= 0 && handle->config.xclk_freq > 0)
        dvp_xclk_start(handle->config.xclk_pin, handle->config.xclk_freq);

    LOG_I("DVP initialized successfully");
    const char *mode_str = "RGB565";
    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        mode_str = "JPEG";
    }
    else if (handle->config.mode == BUS_CAPTURE_MODE_RAW)
    {
        mode_str = "RAW";
    }
    else if (handle->config.mode == BUS_CAPTURE_MODE_YUV422)
    {
        mode_str = "YUV422";
    }
    LOG_I("  Mode: %s", mode_str);
    LOG_I("  Buffer size: %d bytes", handle->config.buffer_size);
    LOG_I("  Pingpong buffer: %d bytes", handle->config.pingpong_buffer_size);
    LOG_I("  VSYNC: PA%d (GPIO interrupt)", handle->config.vsync_pin);

    return RT_EOK;
}

/** @brief Copy caller DVP config into singleton handle. */
int dvp_apply_config(bus_adapter_t *self, const dvp_config_t *config)
{
    dvp_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL)
        return -RT_EINVAL;

    if (dvp_validate_config(config) != RT_EOK)
        return -RT_EINVAL;

    handle = (dvp_handle_t *)self->priv;
    memcpy(&handle->config, config, sizeof(*config));
    return RT_EOK;
}

/** @brief Deinitialize DVP backend and hardware resources. */
int dvp_deinit(bus_adapter_t *self)
{
    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    dvp_stop(self);

    if (handle->config.xclk_pin >= 0)
        dvp_xclk_stop(handle->config.xclk_pin);

    HAL_NVIC_DisableIRQ(handle->config.dma_irqn);
    HAL_DMA_DeInit(&handle->dma);

    __HAL_GPT_DISABLE_DMA(&handle->gptim, GPT_DMA_UPDATE);
    __HAL_GPT_DISABLE_IT(&handle->gptim, GPT_IT_UPDATE);
    HAL_GPT_Base_DeInit(&handle->gptim);

    handle->pingpong_buffer = NULL;
    
    rt_pin_irq_enable(handle->config.vsync_pin, PIN_IRQ_DISABLE);
    rt_pin_detach_irq(handle->config.vsync_pin);
    
    handle->capture.frame_ready = 0;
    handle->capture.current_size = 0;
    handle->capture.soi_found = 0;
    handle->capture.enable_capture = 0;
    handle->capture.capture_started = 0;
    dvp_reset_jpeg_boundary_state(handle);
    
    LOG_I("DVP deinitialized");
    return RT_EOK;
}

/** @brief Start DVP capture hardware. */
int dvp_start(bus_adapter_t *self)
{
    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    GPT_TypeDef *timer_instance = dvp_get_timer_instance(handle);

    HAL_DMA_Abort(&handle->dma);
    HAL_GPT_Base_Stop(&handle->gptim);
    __HAL_GPT_SET_COUNTER(&handle->gptim, 0);
    __HAL_GPT_CLEAR_FLAG(&handle->gptim, GPT_FLAG_UPDATE);

    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        handle->capture.soi_found = 0;
    }
    else
    {
        handle->capture.soi_found = 1;
    }
    handle->capture.frame_ready = 0;
    handle->capture.current_size = 0;
    dvp_reset_jpeg_boundary_state(handle);

    if (HAL_GPT_Base_Start(&handle->gptim) != HAL_OK)
    {
        LOG_E("DVP VSYNC: GPTIM restart failed");
        HAL_DMA_Abort(&handle->dma);
        handle->capture.soi_found = 0;
        handle->capture.enable_capture = 0;
        return -RT_ERROR;
    }
    timer_instance->DIER &= ~GPT_DIER_UDE;
    for (int i = 0; i < 100; i++) { __NOP(); }
    timer_instance->DIER |= GPT_DIER_UDE;

    if (HAL_DMA_Start_IT(&handle->dma,
                         handle->config.data_pin_source_addr,
                         (uint32_t)handle->pingpong_buffer,
                         (uint32_t)handle->config.pingpong_buffer_size) != HAL_OK)
    {
        LOG_E("DVP VSYNC: DMA restart failed");
        handle->capture.soi_found = 0;
        handle->capture.enable_capture = 0;
        handle->capture.capture_started = 0;
        return -RT_ERROR;
    }
    handle->capture.capture_started = 1;

    return RT_EOK;
}

/** @brief Stop DVP capture hardware. */
int dvp_stop(bus_adapter_t *self)
{
    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    HAL_GPT_Base_Stop(&handle->gptim);
    HAL_DMA_Abort(&handle->dma);
    
    handle->capture.enable_capture = 0;
    handle->capture.capture_started = 0;
    
    LOG_D("DVP hardware stopped");
    return RT_EOK;
}

/** @brief Arm capture for one frame, optionally replacing user buffer. */
int dvp_start_capture(bus_adapter_t *self, void *new_buffer, uint32_t buffer_size)
{

    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    if (new_buffer != NULL)
    {
        if (buffer_size == 0)
        {
            LOG_E("DVP error: buffer_size must be provided when new_buffer is not NULL");
            return -RT_EINVAL;
        }
        
        handle->config.frame_buffer = (uint8_t *)new_buffer;
        handle->config.buffer_size = buffer_size;
        LOG_D("DVP capture buffer updated to 0x%08X, size: %d bytes", 
                  (uint32_t)new_buffer, buffer_size);
    }
    
    rt_base_t level = rt_hw_interrupt_disable();
    handle->capture.frame_ready = 0;
    handle->capture.current_size = 0;
    handle->capture.soi_found = 0;
    handle->capture.enable_capture = 1;
    handle->capture.capture_started = 0;
    dvp_reset_jpeg_boundary_state(handle);
    rt_hw_interrupt_enable(level);

    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        return dvp_start(self);
    }
    
    LOG_D("DVP capture enabled.");
    return RT_EOK;
}

/** @brief Rearm capture state for next frame without full HW restart. */
int dvp_rearm_capture(bus_adapter_t *self, void *new_buffer, uint32_t buffer_size)
{
    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    if (new_buffer != NULL)
    {
        if (buffer_size == 0)
        {
            LOG_E("DVP error: buffer_size must be provided when new_buffer is not NULL");
            return -RT_EINVAL;
        }

        handle->config.frame_buffer = (uint8_t *)new_buffer;
        handle->config.buffer_size = buffer_size;
    }

    rt_base_t level = rt_hw_interrupt_disable();
    handle->capture.frame_ready = 0;
    handle->capture.current_size = 0;
    handle->capture.soi_found = 0;   /* always 0: JPEG waits for SOI, raw waits for VSYNC */
    handle->capture.enable_capture = 1;
    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        handle->capture.capture_started = 1;
    }
    else
    {
        handle->capture.capture_started = 0;
    }
    dvp_reset_jpeg_boundary_state(handle);
    rt_hw_interrupt_enable(level);

    if (handle->config.mode == BUS_CAPTURE_MODE_JPEG)
    {
        return RT_EOK;
    }

    /*
     * Non-JPEG (RGB565 / YUV422 / RAW) stream mode:
     * DO NOT call dvp_start() here — this function is invoked from within the
     * DMA ISR (user bus callback chain), and dvp_start() calls HAL_DMA_Abort +
     * HAL_DMA_Start_IT which are not safe to call while the DMA ISR is still
     * executing.  Instead, just set enable_capture = 1 and capture_started = 0
     * above.  The dvp_vsync_irq_handler() will see these flags on the next
     * VSYNC rising edge and call dvp_start() from its own (safe) interrupt
     * context, ensuring frame-boundary alignment.
     */
    return RT_EOK;
}

/** @brief Abort current frame capture state. */
int dvp_abort_capture(bus_adapter_t *self)
{
    if (self == NULL)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    rt_base_t level = rt_hw_interrupt_disable();
    handle->capture.enable_capture = 0;
    handle->capture.frame_ready = 0;
    handle->capture.capture_started = 0;
    dvp_reset_jpeg_boundary_state(handle);
    rt_hw_interrupt_enable(level);
    
    LOG_D("DVP capture aborted.");
    return RT_EOK;
}

/** @brief Resize ping-pong DMA buffer. */
int dvp_set_pingpong_size(bus_adapter_t *self, uint32_t new_size)
{
    if (self == NULL || new_size == 0)
        return -RT_EINVAL;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    if (new_size % 2 != 0)
        new_size += 1;

    if (new_size > handle->config.pingpong_pool_size)
    {
        LOG_E("dvp_set_pingpong_size: %d bytes exceeds pool size (%d)",
              new_size, handle->config.pingpong_pool_size);
        return -RT_EINVAL;
    }

    HAL_DMA_Abort(&handle->dma);
    HAL_GPT_Base_Stop(&handle->gptim);

    handle->pingpong_buffer = handle->config.pingpong_buffer;
    handle->config.pingpong_buffer_size = new_size;
    LOG_I("Pingpong buffer resized to %d bytes", new_size);
    return RT_EOK;
}

/** @brief Register/unregister frame-ready callback. */
int dvp_set_frame_notify_callback(bus_adapter_t *self,
                                  bus_frame_notify_callback_t callback,
                                  void *user_data)
{
    if (self == RT_NULL || self->priv == RT_NULL)
    {
        return BUS_ERR_INVALID;
    }

    s_user_frame_callback = callback;
    s_user_frame_callback_data = user_data;
    return BUS_OK;
}

/** @brief Set capture mode in DVP config. */
int dvp_set_mode(bus_adapter_t *self, bus_capture_mode_t mode)
{
    if (self == NULL)
        return BUS_ERR_INVALID;
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;

    if (mode != BUS_CAPTURE_MODE_JPEG   &&
        mode != BUS_CAPTURE_MODE_RAW    &&
        mode != BUS_CAPTURE_MODE_YUV422 &&
        mode != BUS_CAPTURE_MODE_RGB565)
        return BUS_ERR_INVALID;

    handle->config.mode = mode;
    return BUS_OK;
}

#if DEBUG_DVP
/** @brief Dump DVP diagnostics to log (debug build). */
void dvp_dump_state(bus_adapter_t *self)
{
    dvp_handle_t *handle = (dvp_handle_t *)self->priv;
    uint32_t dma_counter = 0;
    uint32_t gpt_cnt = 0;
    uint32_t gpt_cr1 = 0;
    uint32_t gpt_dier = 0;
    GPT_TypeDef *gpt = (GPT_TypeDef *)handle->gptim.Instance;
    int vsync_level = rt_pin_read(handle->config.vsync_pin);

    if (gpt != RT_NULL)
    {
        gpt_cnt = gpt->CNT;
        gpt_cr1 = gpt->CR1;
        gpt_dier = gpt->DIER;
    }
    if (handle->dma.Instance != RT_NULL)
    {
        dma_counter = __HAL_DMA_GET_COUNTER(&handle->dma);
    }

    LOG_W("  bus(dvp): mode=%d cfg_buf=%u",
        (int)handle->config.mode,
        (unsigned int)handle->config.buffer_size);
    LOG_W("  capture: ready=%u enable=%u soi=%u current=%u",
        (unsigned int)handle->capture.frame_ready,
        (unsigned int)handle->capture.enable_capture,
        (unsigned int)handle->capture.soi_found,
        (unsigned int)handle->capture.current_size);
    LOG_W("  mem: frame_buf=0x%08X pingpong=0x%08X pp_size=%u",
        (unsigned int)(rt_ubase_t)handle->config.frame_buffer,
        (unsigned int)(rt_ubase_t)handle->pingpong_buffer,
        (unsigned int)handle->config.pingpong_buffer_size);
    LOG_W("  dma: state=%d err=0x%08X req=%u counter=%u",
        (int)HAL_DMA_GetState(&handle->dma),
        (unsigned int)HAL_DMA_GetError(&handle->dma),
        (unsigned int)handle->dma.Init.Request,
        (unsigned int)dma_counter);
    LOG_W("  gpt: state=%d cnt=%u cr1=0x%08X dier=0x%08X",
        (int)handle->gptim.State,
        (unsigned int)gpt_cnt,
        (unsigned int)gpt_cr1,
        (unsigned int)gpt_dier);
    LOG_W("  vsync: pin=%u level=%d",
        (unsigned int)handle->config.vsync_pin,
        vsync_level);
    {
        uint32_t dmac_isr  = hwp_dmac1->ISR;
        uint32_t dmac_ifcr = hwp_dmac1->IFCR;
        uint32_t dmac_ccr  = hwp_dmac1->CCR1;
        uint32_t dmac_cndtr = hwp_dmac1->CNDTR1;
        LOG_W("  dmac: ISR=0x%08X IFCR=0x%08X CCR=0x%08X CNDTR=%u",
            (unsigned int)dmac_isr,
            (unsigned int)dmac_ifcr,
            (unsigned int)dmac_ccr,
            (unsigned int)dmac_cndtr);
    }
}
#endif

static const bus_adapter_ops_t s_dvp_bus_ops = {
    .init               = dvp_init,
    .deinit             = dvp_deinit,
    .start              = dvp_start,
    .stop               = dvp_stop,
    .set_frame_notify_callback = dvp_set_frame_notify_callback,
    .start_capture      = dvp_start_capture,
    .rearm_capture      = dvp_rearm_capture,
    .abort_capture      = dvp_abort_capture,
    .set_pingpong_size  = dvp_set_pingpong_size,
    .set_mode           = dvp_set_mode,
#if DEBUG_DVP
    .dump_state         = dvp_dump_state,
#else
    .dump_state         = RT_NULL,
#endif
};

static bus_adapter_t s_dvp_bus_adapter = {
    .name = DVP_BUS_ADAPTER_NAME,
    .type = BUS_TYPE_DVP,
    .ops  = &s_dvp_bus_ops,
    .priv = &s_dvp_handle,
};



/** @brief Register DVP adapter at board init. */
static int dvp_bus_adapter_register(void)
{
    int ret = bus_adapter_register(&s_dvp_bus_adapter);
    if (ret != BUS_OK)
    {
        LOG_E("DVP bus adapter register failed: %d", ret);
    }
    return ret;
}
INIT_BOARD_EXPORT(dvp_bus_adapter_register);
