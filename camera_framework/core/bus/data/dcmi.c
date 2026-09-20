/* SPDX-License-Identifier: Apache-2.0 */
#include "data_bus_adapter.h"
#include "camera_jpeg_assembler.h"

#include <bf0_hal.h>
#include <rtdevice.h>
#include <dma_config.h>
#include <ipc/workqueue.h>
#include <rthw.h>
#include <rtthread.h>
#include <string.h>

#define DBG_TAG "camera.dcmi"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#if !defined(SF32LB57X)
#error "The hardware DCMI camera backend requires SF32LB57X"
#endif

#if defined(BSP_USING_DCMI)
#error "The camera DCMI backend owns the peripheral; disable BSP_USING_DCMI"
#endif

#define DCMI_BUFFER_ALIGNMENT 64U
#define DCMI_MAX_DMA_WORDS 0xfffffU
#define DCMI_STREAM_STACK_SIZE 2048U
#define DCMI_STREAM_PRIORITY 19U

typedef struct
{
    DCMI_HandleTypeDef handle;
    DMA_HandleTypeDef hdma;
    uint8_t *frame_buffer;
    uint32_t buffer_size;
    uint32_t error;
} camera_dcmi_hw_t;

typedef struct
{
    camera_dcmi_hw_t hardware;
    struct rt_mutex lock;
    struct rt_work single_work;
    struct rt_thread stream_thread;
    struct rt_semaphore stream_sem;
    rt_bool_t stream_notify_initialized;
    rt_bool_t stream_notify_pending;
    rt_bool_t lock_initialized;
    rt_bool_t work_initialized;
    rt_bool_t initialized;
    rt_bool_t dispatching;
    rt_thread_t dispatch_thread;
    volatile rt_bool_t started;
    volatile rt_bool_t stopping;
    volatile rt_bool_t native_stream;
    volatile rt_bool_t single_pending;
    volatile int error;
    uint32_t sequence;
    bus_capture_mode_t mode;
    bus_frame_notify_callback_t frame_callback;
    void *frame_context;
    bus_stream_config_t stream;
    uint8_t *single_buffer;
    uint32_t single_size;
    uint32_t single_received;
    uint32_t burst_frames;
    uint32_t burst_count;
    rt_tick_t single_end_tick;
    rt_bool_t hardware_started;
    volatile rt_bool_t capture_armed;
    volatile rt_bool_t capture_closing;
    rt_bool_t error_pending;
    uint32_t dma_size;      /* Total DMA region size in bytes. */
    uint32_t expected_size; /* One complete image size in bytes. */
    /* Slots represent two images in FRAME mode, or two halves of one image
     * in HALF_FRAME mode. */
    uint32_t buffer_sequence[2];
    rt_bool_t buffer_valid[2];
    rt_bool_t buffer_pending[2];
    uint32_t buffer_timestamp[2];
} camera_dcmi_t;

static camera_dcmi_t s_dcmi;
static uint8_t s_stream_stack[DCMI_STREAM_STACK_SIZE] __attribute__((aligned(8)));

static int camera_dcmi_stop_locked(camera_dcmi_t *dcmi);
static void camera_dcmi_single_work(struct rt_work *work, void *parameter);
static rt_bool_t camera_dcmi_dispatch_stream(camera_dcmi_t *dcmi);
static void camera_dcmi_stream_thread(void *parameter);

static int camera_dcmi_lock(camera_dcmi_t *dcmi)
{
    rt_base_t level;

    if (rt_interrupt_get_nest() != 0U)
        return -RT_ERROR;
    level = rt_hw_interrupt_disable();
    if (!dcmi->lock_initialized)
    {
        rt_mutex_init(&dcmi->lock, "cam_dcmi", RT_IPC_FLAG_PRIO);
        dcmi->lock_initialized = RT_TRUE;
    }
    rt_hw_interrupt_enable(level);
    return rt_mutex_take(&dcmi->lock, RT_WAITING_FOREVER);
}

static void camera_dcmi_unlock(camera_dcmi_t *dcmi)
{
    rt_mutex_release(&dcmi->lock);
}

/* Keep the token and pending flag paired across stop/deinit/restart. An old
 * wakeup reads current stream state instead of retaining a frame or context. */
static void camera_dcmi_signal_stream(camera_dcmi_t *dcmi)
{
    rt_base_t level = rt_hw_interrupt_disable();

    if (dcmi->stream_notify_initialized && dcmi->started && dcmi->native_stream &&
        dcmi->stream.mode == BUS_STREAM_MODE_FRAME &&
        !dcmi->stopping && !dcmi->stream_notify_pending)
    {
        dcmi->stream_notify_pending = RT_TRUE;
        if (rt_sem_release(&dcmi->stream_sem) != RT_EOK)
            dcmi->stream_notify_pending = RT_FALSE;
    }
    rt_hw_interrupt_enable(level);
}

static int camera_dcmi_init_stream_thread(camera_dcmi_t *dcmi)
{
    int result;

    if (dcmi->stream_notify_initialized)
        return RT_EOK;
    result = rt_sem_init(&dcmi->stream_sem, "cam_dcm", 0U, RT_IPC_FLAG_PRIO);
    if (result != RT_EOK)
        return result;
    result = rt_thread_init(&dcmi->stream_thread, "cam_dcm", camera_dcmi_stream_thread,
                            dcmi, s_stream_stack, sizeof(s_stream_stack),
                            DCMI_STREAM_PRIORITY, 10U);
    if (result != RT_EOK)
    {
        rt_sem_detach(&dcmi->stream_sem);
        return result;
    }
    result = rt_thread_startup(&dcmi->stream_thread);
    if (result != RT_EOK)
    {
        rt_thread_detach(&dcmi->stream_thread);
        rt_sem_detach(&dcmi->stream_sem);
        return result;
    }
    dcmi->stream_notify_initialized = RT_TRUE;
    return RT_EOK;
}

static int camera_dcmi_buffer_valid(const void *buffer, uint32_t size)
{
    return buffer != RT_NULL && size != 0U &&
           ((uintptr_t)buffer & (DCMI_BUFFER_ALIGNMENT - 1U)) == 0U &&
           (size & (DCMI_BUFFER_ALIGNMENT - 1U)) == 0U &&
           size / sizeof(uint32_t) <= DCMI_MAX_DMA_WORDS &&
           (uintptr_t)buffer <= UINTPTR_MAX - size;
}

static int camera_dcmi_mode_valid(bus_capture_mode_t mode)
{
    return mode == BUS_CAPTURE_MODE_RGB565 ||
           mode == BUS_CAPTURE_MODE_YUV422 || mode == BUS_CAPTURE_MODE_RAW ||
           mode == BUS_CAPTURE_MODE_JPEG;
}

static int camera_dcmi_stop_transfer_locked(camera_dcmi_t *dcmi)
{
    camera_dcmi_hw_t *hardware = &dcmi->hardware;
    rt_base_t level = rt_hw_interrupt_disable();

    dcmi->capture_closing = RT_TRUE;
    dcmi->capture_armed = RT_FALSE;
    hardware->handle.Instance->CR &= ~DCMI_CR_CAPTURE;
    rt_hw_interrupt_enable(level);
    if (hardware->hdma.State == HAL_DMA_STATE_BUSY &&
        HAL_DMA_Abort_IT(&hardware->hdma) != HAL_OK)
        return -RT_EBUSY;
    /* A released dynamic channel may already belong to another device. */
    if ((hardware->handle.Instance->CR & DCMI_CR_CAPTURE) != 0U ||
        (hardware->hdma.State != HAL_DMA_STATE_READY &&
         hardware->hdma.State != HAL_DMA_STATE_RESET))
        return -RT_EBUSY;
    hardware->handle.State = HAL_DCMI_STATE_READY;
    if (dcmi->hardware_started)
    {
#ifdef RT_USING_PM
        rt_pm_release(PM_SLEEP_MODE_IDLE);
        rt_pm_hw_device_stop();
#endif
        dcmi->hardware_started = RT_FALSE;
    }
    return RT_EOK;
}

/* Call with interrupts disabled and only while the dynamic DMA channel is
 * owned by this stream. HAL clears each serviced flag before the DMA callback. */
static uint32_t camera_dcmi_dma_pending(camera_dcmi_hw_t *hardware)
{
    uint32_t shift = hardware->hdma.ChannelIndex & 0x1cU;

    return (hardware->hdma.DmaBaseAddress->ISR >> shift) &
           (DMA_FLAG_HT1 | DMA_FLAG_TC1 | DMA_FLAG_TE1);
}

/* A half-transfer interrupt is based on source reads. Check the destination
 * position too before lending that buffer to the CPU. Pending IRQ flags make
 * delayed notifications and a full ring wrap visible during the copy. */
static rt_bool_t camera_dcmi_buffer_idle(camera_dcmi_t *dcmi, uint32_t index)
{
    camera_dcmi_hw_t *hardware = &dcmi->hardware;
    uint32_t total = dcmi->dma_size / sizeof(uint32_t);
    uint32_t half = total / 2U;
    uint32_t reads_left, writes_left;
    uint32_t read_index, write_index;

    if (hardware->hdma.State != HAL_DMA_STATE_BUSY || hardware->hdma.Instance == RT_NULL ||
        hardware->handle.ErrorCode != HAL_DCMI_ERROR_NONE ||
        hardware->hdma.ErrorCode != HAL_DMA_ERROR_NONE ||
        (hardware->handle.Instance->CR & DCMI_CR_CAPTURE) == 0U ||
        (hardware->handle.Instance->RIS & (DCMI_FLAG_ERRRI | DCMI_FLAG_OVRRI)) != 0U)
        return RT_FALSE;
    if (camera_dcmi_dma_pending(hardware) != 0U)
        return RT_FALSE;
    reads_left = __HAL_DMA_GET_COUNTER(&hardware->hdma);
    writes_left = ((GPDMA_Channel_TypeDef *)hardware->hdma.Instance)->CSR &
                  GPDMA_CSR1_DSTNDT_Msk;
    if (reads_left > total || writes_left > total)
        return RT_FALSE;
    read_index = reads_left != 0U && reads_left <= half ? 1U : 0U;
    write_index = writes_left != 0U && writes_left <= half ? 1U : 0U;
    return index != read_index && index != write_index &&
           camera_dcmi_dma_pending(hardware) == 0U;
}

static void camera_dcmi_invalidate(camera_dcmi_t *dcmi)
{
    uint32_t index;

    for (index = 0U; index < 2U; ++index)
    {
        dcmi->buffer_valid[index] = RT_FALSE;
        dcmi->buffer_pending[index] = RT_FALSE;
        dcmi->buffer_sequence[index] = ++dcmi->sequence;
    }
}

/**
 * @brief Handle DMA progress and DCMI frame/error notifications.
 *
 * size identifies the event: half/full DMA region size in bytes for HT/TC,
 * or zero for a DCMI FRAME or hardware error notification. It is not always
 * the captured image length. VSYNC alone does not complete a frame.
 */
static void camera_dcmi_rx(camera_dcmi_t *dcmi, uint32_t size)
{
    camera_dcmi_hw_t *hardware = &dcmi->hardware;
    bus_stream_frame_t frame = {0};
    bus_stream_frame_callback_t callback = RT_NULL;
    void *context = RT_NULL;
    rt_bool_t submit_stream = RT_FALSE;
    rt_bool_t submit_single = RT_FALSE;
    int error = 0;
    rt_base_t level = rt_hw_interrupt_disable();

    if (!dcmi->started || dcmi->stopping || dcmi->error != 0 ||
        dcmi->capture_closing)
        goto done;
    if (hardware->error != 0U ||
        hardware->handle.ErrorCode != HAL_DCMI_ERROR_NONE ||
        hardware->hdma.ErrorCode != HAL_DMA_ERROR_NONE)
        error = -RT_EIO;
    if (dcmi->native_stream)
    {
        if (!dcmi->capture_armed)
            goto done;
        if (error != 0)
        {
            camera_dcmi_invalidate(dcmi);
            dcmi->error = error;
            dcmi->capture_armed = RT_FALSE;
            hardware->handle.Instance->CR &= ~DCMI_CR_CAPTURE;
            if (dcmi->stream.mode == BUS_STREAM_MODE_HALF_FRAME)
            {
                frame.error = error;
                callback = dcmi->stream.frame_callback;
                context = dcmi->stream.callback_context;
            }
            else
            {
                dcmi->error_pending = RT_TRUE;
                submit_stream = RT_TRUE;
            }
        }
        else if (size == dcmi->dma_size / 2U || size == dcmi->dma_size)
        {
            uint32_t index = size == dcmi->dma_size / 2U ? 0U : 1U;
            uint32_t next = index ^ 1U;

            dcmi->buffer_valid[next] = RT_FALSE;
            dcmi->buffer_pending[next] = RT_FALSE;
            if (dcmi->stream.mode == BUS_STREAM_MODE_HALF_FRAME)
            {
                /* HT and TC describe two halves of the same image. The
                 * opposite half becomes unavailable at each DMA boundary. */
                if (index == 0U)
                    ++dcmi->sequence;
                dcmi->buffer_sequence[index] = dcmi->sequence;
                dcmi->buffer_valid[index] = RT_TRUE;
                frame.buffer = dcmi->stream.buffers[0];
                frame.size = dcmi->expected_size;
                frame.sequence = dcmi->buffer_sequence[index];
                frame.timestamp_ticks = rt_tick_get();
                frame.event_type = index == 0U ? BUS_STREAM_EVENT_HALF_FIRST :
                                                BUS_STREAM_EVENT_HALF_SECOND;
                callback = dcmi->stream.frame_callback;
                context = dcmi->stream.callback_context;
            }
            else
            {
                dcmi->buffer_sequence[next] = ++dcmi->sequence;
                dcmi->buffer_valid[index] = RT_FALSE;
                dcmi->buffer_pending[index] = RT_TRUE;
                dcmi->buffer_timestamp[index] = rt_tick_get();
                submit_stream = RT_TRUE;
            }
        }
    }
    else if (!dcmi->single_pending)
    {
        rt_bool_t jpeg = dcmi->mode == BUS_CAPTURE_MODE_JPEG;

        /* A JPEG burst uses one linear DMA region. Only FRAME counts an
         * image; HT is a byte-capacity notification and TC means overflow. */
        if (dcmi->burst_frames != 0U)
        {
            if (size == dcmi->single_size || hardware->hdma.State != HAL_DMA_STATE_BUSY)
                error = -RT_EIO;
            if (error == 0)
            {
                if (size != 0U)
                    goto done;
                ++dcmi->burst_count;
                if (dcmi->burst_count < dcmi->burst_frames)
                    goto done;
            }
            dcmi->error = error;
            dcmi->single_end_tick = rt_tick_get();
            dcmi->single_pending = RT_TRUE;
            hardware->handle.Instance->CR &= ~DCMI_CR_CAPTURE;
            submit_single = RT_TRUE;
        }
        /* Abort callbacks are excluded by capture_closing above. */
        else if (error != 0 || size == dcmi->single_size || (jpeg && size == 0U))
        {
            dcmi->single_received = jpeg ? hardware->handle.Instance->JPEG_BYTE : dcmi->single_size;
            if (jpeg && (size == dcmi->single_size || dcmi->single_received < 4U ||
                         dcmi->single_received >= dcmi->single_size))
                error = -RT_EIO;
            dcmi->error = error;
            dcmi->single_end_tick = rt_tick_get();
            dcmi->single_pending = RT_TRUE;
            /* Snapshot capture also stops at EOF. Stop requests immediately
             * on capacity exhaustion so the circular region cannot run on. */
            hardware->handle.Instance->CR &= ~DCMI_CR_CAPTURE;
            submit_single = RT_TRUE;
        }
    }

done:
    rt_hw_interrupt_enable(level);
    /* Native half-frame callbacks publish metadata to the consumer queue.
     * Pixel processing runs in the consumer's thread. */
    if (callback != RT_NULL)
        callback(context, &frame);
    if (submit_stream)
        camera_dcmi_signal_stream(dcmi);
    if (submit_single)
        rt_work_submit(&dcmi->single_work, 0);
}

static void camera_dcmi_dma_half(DMA_HandleTypeDef *dma)
{
    (void)dma;
    camera_dcmi_rx(&s_dcmi, s_dcmi.hardware.buffer_size / 2U);
}

static void camera_dcmi_dma_full(DMA_HandleTypeDef *dma)
{
    (void)dma;
    camera_dcmi_rx(&s_dcmi, s_dcmi.hardware.buffer_size);
}

static void camera_dcmi_dma_error(DMA_HandleTypeDef *dma)
{
    (void)dma;
    camera_dcmi_rx(&s_dcmi, 0U);
}

void DCMI_IRQHandler(void)
{
    camera_dcmi_hw_t *hardware = &s_dcmi.hardware;
    uint32_t packet_error;

    rt_interrupt_enter();
    packet_error = hardware->handle.Instance->MIS & DCMI_MIS_PACK_SIZE_ERR_MIS;
    if (packet_error != 0U)
    {
        __HAL_DCMI_CLEAR_FLAG(&hardware->handle, DCMI_ISC_PACK_SIZE_ERR_ISC);
        hardware->error |= packet_error;
    }
    HAL_DCMI_IRQHandler(&hardware->handle);
    if (packet_error != 0U)
        camera_dcmi_rx(&s_dcmi, 0U);
    rt_interrupt_leave();
}

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
void DCMI_DMA_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&s_dcmi.hardware.hdma);
    rt_interrupt_leave();
}
#endif

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *handle)
{
    if (handle != &s_dcmi.hardware.handle)
        return;
    camera_dcmi_rx(&s_dcmi, 0U);
    /* HAL masks FRAME on every interrupt, including continuous capture. */
    __HAL_DCMI_ENABLE_IT(handle, DCMI_IE_FRAME_IE | DCMI_IE_OVR_IE |
                        DCMI_IE_FSM_ERR_IE | DCMI_IE_PACK_SIZE_ERR_IE);
}

/* HAL synchronization/overflow errors abort DMA before invoking this hook. */
void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *handle)
{
    if (handle == &s_dcmi.hardware.handle)
        camera_dcmi_rx(&s_dcmi, 0U);
}

static int camera_dcmi_hw_init(camera_dcmi_t *dcmi)
{
    camera_dcmi_hw_t *hardware = &dcmi->hardware;

    hardware->handle.Instance = hwp_dcmi;
    hardware->handle.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
    hardware->handle.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
    hardware->handle.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
    hardware->handle.Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
    hardware->handle.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
    __HAL_LINKDMA(&hardware->handle, DMA_Handle, hardware->hdma);
    if (HAL_DCMI_Init(&hardware->handle) != HAL_OK)
        return -RT_EIO;
    hardware->handle.Instance->IE = 0U;
    hardware->hdma.Init.Request = DCMI_DMA_REQUEST;
    hardware->hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hardware->hdma.Init.MemInc = DMA_MINC_ENABLE;
    hardware->hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hardware->hdma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hardware->hdma.Init.Priority = DMA_PRIORITY_HIGH;
#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    hardware->hdma.Init.IrqPrio = DCMI_DMA_IRQ_PRIO;
#else
    HAL_NVIC_SetPriority(DCMI_DMA_IRQ, DCMI_DMA_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(DCMI_DMA_IRQ);
#endif
    HAL_NVIC_ClearPendingIRQ(DCMI_IRQn);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);
    return RT_EOK;
}

static int camera_dcmi_hw_start(camera_dcmi_t *dcmi)
{
    camera_dcmi_hw_t *hardware = &dcmi->hardware;

#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_start();
#endif
    dcmi->hardware_started = RT_TRUE;
    hardware->handle.Instance->CR |= DCMI_CR_RSTB;
    while ((hardware->handle.Instance->CR & DCMI_CR_RSTB_S) == 0U);
    hardware->hdma.XferHalfCpltCallback = camera_dcmi_dma_half;
    hardware->hdma.XferCpltCallback = camera_dcmi_dma_full;
    hardware->hdma.XferErrorCallback = camera_dcmi_dma_error;
    hardware->hdma.XferAbortCallback = camera_dcmi_dma_error;
    if (HAL_DMA_Start_IT(&hardware->hdma, (uint32_t)&hardware->handle.Instance->DR,
                        (uint32_t)hardware->frame_buffer, hardware->buffer_size / 4U) != HAL_OK)
        return -RT_EIO;
    /* Dynamic allocation programs the destination burst at DMA start.
     * Apply JPEG single-word writes before capture can supply any data. */
    if (dcmi->mode == BUS_CAPTURE_MODE_JPEG)
    {
        __HAL_DMA_DISABLE(&hardware->hdma);
        hardware->hdma.Instance->CCR &= ~GPDMA_CCR1_DBURST_Msk;
        __HAL_DMA_ENABLE(&hardware->hdma);
    }
    hardware->handle.State = HAL_DCMI_STATE_BUSY;
    __HAL_DCMI_CAPTURE_START(&hardware->handle);
    return RT_EOK;
}

static int camera_dcmi_config(bus_adapter_t *self, const bus_adapter_config_t *config)
{
    camera_dcmi_t *dcmi = self->priv;
    int result;

    if (config == RT_NULL || !camera_dcmi_mode_valid(config->mode))
        return BUS_ERR_NOT_SUPPORTED;
    result = camera_dcmi_lock(dcmi);
    if (result != RT_EOK)
        return result;
    if (dcmi->started || dcmi->dispatching)
        result = -RT_EBUSY;
    else
        dcmi->mode = config->mode;
    camera_dcmi_unlock(dcmi);
    return result;
}

static int camera_dcmi_init(bus_adapter_t *self)
{
    camera_dcmi_t *dcmi = self->priv;
    int result = camera_dcmi_lock(dcmi);

    if (result != RT_EOK)
        return result;
    if (dcmi->initialized)
        goto done;
    result = camera_dcmi_init_stream_thread(dcmi);
    if (result != RT_EOK)
        goto done;
    result = camera_dcmi_hw_init(dcmi);
    if (result != RT_EOK)
        goto done;
    if (!dcmi->work_initialized)
    {
        rt_work_init(&dcmi->single_work, camera_dcmi_single_work, dcmi);
        dcmi->work_initialized = RT_TRUE;
    }
    dcmi->initialized = RT_TRUE;

done:
    camera_dcmi_unlock(dcmi);
    return result;
}

static int camera_dcmi_stop_locked(camera_dcmi_t *dcmi)
{
    rt_base_t level;
    int result;

    if (dcmi->dispatching && dcmi->dispatch_thread != rt_thread_self())
        return -RT_EBUSY;
    if (!dcmi->started)
        return RT_EOK;
    level = rt_hw_interrupt_disable();
    dcmi->stopping = RT_TRUE;
    camera_dcmi_invalidate(dcmi);
    dcmi->error_pending = RT_FALSE;
    rt_hw_interrupt_enable(level);
    result = camera_dcmi_stop_transfer_locked(dcmi);
    if (result != RT_EOK)
        return result;
    level = rt_hw_interrupt_disable();
    dcmi->started = RT_FALSE;
    dcmi->single_pending = RT_FALSE;
    dcmi->stopping = RT_FALSE;
    dcmi->native_stream = RT_FALSE;
    dcmi->single_buffer = RT_NULL;
    dcmi->single_size = 0U;
    dcmi->burst_frames = 0U;
    dcmi->burst_count = 0U;
    memset(&dcmi->stream, 0, sizeof(dcmi->stream));
    rt_hw_interrupt_enable(level);
    return RT_EOK;
}

static int camera_dcmi_stop(bus_adapter_t *self)
{
    camera_dcmi_t *dcmi = self->priv;
    int result = camera_dcmi_lock(dcmi);

    if (result != RT_EOK)
        return result;
    result = camera_dcmi_stop_locked(dcmi);
    camera_dcmi_unlock(dcmi);
    return result;
}

static int camera_dcmi_deinit(bus_adapter_t *self)
{
    camera_dcmi_t *dcmi = self->priv;
    int result = camera_dcmi_lock(dcmi);

    if (result != RT_EOK)
        return result;
    if (!dcmi->initialized)
        goto done;
    if (dcmi->dispatching)
    {
        result = -RT_EBUSY;
        goto done;
    }
    result = camera_dcmi_stop_locked(dcmi);
    if (result != RT_EOK)
        goto done;
    HAL_NVIC_DisableIRQ(DCMI_IRQn);
    dcmi->hardware.handle.Instance->IE = 0U;
    __HAL_DCMI_DISABLE(&dcmi->hardware.handle);
    HAL_NVIC_ClearPendingIRQ(DCMI_IRQn);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    HAL_NVIC_DisableIRQ(DCMI_DMA_IRQ);
#endif
    /* HAL_DMA_DeInit writes channel registers; a released dynamic channel
     * can belong to another device. The next prepare initializes it anew. */
    HAL_DCMI_DeInit(&dcmi->hardware.handle);
    dcmi->initialized = RT_FALSE;
    dcmi->frame_callback = RT_NULL;
    dcmi->frame_context = RT_NULL;

done:
    camera_dcmi_unlock(dcmi);
    return result;
}

static int camera_dcmi_prepare_locked(camera_dcmi_t *dcmi, uint8_t *buffer,
                                      uint32_t size, rt_bool_t stream)
{
    camera_dcmi_hw_t *hardware = &dcmi->hardware;
    uint32_t cr;

    if (hardware->hdma.State != HAL_DMA_STATE_READY &&
        hardware->hdma.State != HAL_DMA_STATE_RESET)
        return -RT_EBUSY;
    if ((hardware->handle.Instance->CR & DCMI_CR_CAPTURE) != 0U)
        return -RT_EBUSY;
    hardware->handle.Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
    hardware->handle.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
#ifdef CAMERA_DCMI_PCLK_FALLING
    hardware->handle.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
#else
    hardware->handle.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
#endif
    hardware->handle.Instance->IE = 0U;
    __HAL_DCMI_TRS_IF(&hardware->handle, 0U);
    __HAL_DCMI_CAP_MODE(&hardware->handle, stream ? 0U : 1U);
    __HAL_DCMI_YUV2RGB_DISABLE(&hardware->handle);
    hardware->frame_buffer = buffer;
    hardware->buffer_size = size;
    cr = hardware->handle.Instance->CR & ~(DCMI_CR_HSPOL | DCMI_CR_VSPOL | DCMI_CR_PCKPOL | DCMI_CR_JPEG);
    hardware->handle.Instance->CR = cr | hardware->handle.Init.HSPolarity |
                                   hardware->handle.Init.VSPolarity |
                                   hardware->handle.Init.PCKPolarity;
    if (dcmi->mode == BUS_CAPTURE_MODE_JPEG)
        hardware->handle.Instance->CR |= DCMI_CR_JPEG;
    /* JPEG ends at an arbitrary word; avoid requiring a 16-word FIFO tail. */
    hardware->handle.Instance->DMA_CR = dcmi->mode == BUS_CAPTURE_MODE_JPEG ? 1U : 16U;
    /* GPDMA encodes single-word/16-word bursts as 0/3. Single-word JPEG
     * reads must keep the peripheral address fixed at the DCMI data port. */
    hardware->hdma.Init.BurstSize = dcmi->mode == BUS_CAPTURE_MODE_JPEG ? 0U : 3U;
    hardware->hdma.Init.PeriphInc = dcmi->mode == BUS_CAPTURE_MODE_JPEG ? DMA_PINC_DISABLE : DMA_PINC_ENABLE;

    /* A dynamic channel can move between transfers. Restore the board's
     * preferred channel before HAL init recalculates its allocation group. */
    hardware->hdma.Instance = DCMI_DMA_INSTANCE;
    /* Continuous JPEG capture is bounded by the arena, never a wrapping
     * byte ring. Fixed-size preview frames retain circular DMA. */
    hardware->hdma.Init.Mode = stream && dcmi->mode == BUS_CAPTURE_MODE_JPEG ?
                               DMA_NORMAL : DMA_CIRCULAR;
    if (HAL_DMA_Init(&hardware->hdma) != HAL_OK)
        return -RT_EIO;
    hardware->error = 0U;
    hardware->handle.ErrorCode = HAL_DCMI_ERROR_NONE;
    hardware->hdma.ErrorCode = HAL_DMA_ERROR_NONE;
    __HAL_DCMI_CLEAR_FLAG(&hardware->handle, DCMI_FLAG_FRAMERI | DCMI_FLAG_VSYNCRI |
                         DCMI_FLAG_LINERI | DCMI_FLAG_ERRRI | DCMI_FLAG_OVRRI |
                         DCMI_ISC_PACK_SIZE_ERR_ISC);
    __HAL_DCMI_ENABLE_IT(&hardware->handle, DCMI_IE_FRAME_IE | DCMI_IE_OVR_IE |
                        DCMI_IE_FSM_ERR_IE | DCMI_IE_PACK_SIZE_ERR_IE |
                        DCMI_IE_VSYNC_IE);
    __HAL_DCMI_DISABLE_IT(&hardware->handle, DCMI_IE_LINE_IE);
    __HAL_DCMI_ENABLE(&hardware->handle);

    return RT_EOK;
}

static int camera_dcmi_arm_stream_locked(camera_dcmi_t *dcmi)
{
    camera_dcmi_hw_t *hardware = &dcmi->hardware;
    rt_base_t level = rt_hw_interrupt_disable();
    int result;

    camera_dcmi_invalidate(dcmi);
    rt_hw_interrupt_enable(level);
    result = camera_dcmi_prepare_locked(dcmi, dcmi->stream.buffers[0],
                                        dcmi->dma_size, RT_TRUE);
    if (result != RT_EOK)
        return result;
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)dcmi->stream.buffers[0], (int32_t)dcmi->dma_size);
    level = rt_hw_interrupt_disable();
    dcmi->error_pending = RT_FALSE;
    dcmi->capture_closing = RT_FALSE;
    dcmi->capture_armed = RT_TRUE;
    rt_hw_interrupt_enable(level);
    result = camera_dcmi_hw_start(dcmi);
    if (result == RT_EOK && dcmi->error == 0 &&
        (hardware->hdma.State != HAL_DMA_STATE_BUSY ||
         hardware->hdma.ErrorCode != HAL_DMA_ERROR_NONE))
        result = -RT_EIO;
    return result;
}

static rt_bool_t camera_dcmi_dispatch_stream(camera_dcmi_t *dcmi)
{
    bus_stream_frame_t frame = {0};
    bus_stream_frame_callback_t callback = RT_NULL;
    void *context = RT_NULL;
    rt_bool_t retry = RT_FALSE;
    uint32_t index;
    rt_base_t level;

    if (camera_dcmi_lock(dcmi) != RT_EOK)
        return RT_FALSE;
    if (!dcmi->started || !dcmi->native_stream || dcmi->stopping ||
        dcmi->stream.mode != BUS_STREAM_MODE_FRAME)
        goto done;
    level = rt_hw_interrupt_disable();
    if (dcmi->error_pending)
    {
        dcmi->error_pending = RT_FALSE;
        frame.error = dcmi->error;
        callback = dcmi->stream.frame_callback;
        context = dcmi->stream.callback_context;
    }
    else if (dcmi->error == 0)
    {
        for (index = 0U; index < 2U; ++index)
        {
            if (!dcmi->buffer_pending[index])
                continue;
            if (!camera_dcmi_buffer_idle(dcmi, index))
            {
                retry = RT_TRUE;
                continue;
            }
            dcmi->buffer_pending[index] = RT_FALSE;
            dcmi->buffer_valid[index] = RT_TRUE;
            frame.buffer = dcmi->stream.buffers[index];
            frame.size = dcmi->expected_size;
            frame.sequence = dcmi->buffer_sequence[index];
            frame.timestamp_ticks = dcmi->buffer_timestamp[index];
            callback = dcmi->stream.frame_callback;
            context = dcmi->stream.callback_context;
            break;
        }
    }
    rt_hw_interrupt_enable(level);

done:
    if (callback != RT_NULL)
    {
        dcmi->dispatching = RT_TRUE;
        dcmi->dispatch_thread = rt_thread_self();
    }
    camera_dcmi_unlock(dcmi);
    if (callback != RT_NULL)
    {
        callback(context, &frame);
        if (camera_dcmi_lock(dcmi) == RT_EOK)
        {
            dcmi->dispatching = RT_FALSE;
            dcmi->dispatch_thread = RT_NULL;
            camera_dcmi_unlock(dcmi);
        }
    }
    return retry;
}

/* This task only checks the completed DMA half and hands its descriptor to
 * the stream service. Pixel copies and rendering run in their own tasks. */
static void camera_dcmi_stream_thread(void *parameter)
{
    camera_dcmi_t *dcmi = parameter;

    while (1)
    {
        rt_base_t level;

        if (rt_sem_take(&dcmi->stream_sem, RT_WAITING_FOREVER) != RT_EOK)
            continue;
        level = rt_hw_interrupt_disable();
        dcmi->stream_notify_pending = RT_FALSE;
        rt_hw_interrupt_enable(level);
        if (camera_dcmi_dispatch_stream(dcmi))
        {
            /* HT/TC follows source reads; allow the final destination writes
             * to finish without spinning above the UI and stream threads. */
            rt_thread_delay(1);
            camera_dcmi_signal_stream(dcmi);
        }
    }
}

static int camera_dcmi_start_stream(bus_adapter_t *self, const bus_stream_config_t *config)
{
    camera_dcmi_t *dcmi = self->priv;
    uint32_t expected;
    uintptr_t first, second;
    rt_base_t level;
    int result;

    if (dcmi->mode == BUS_CAPTURE_MODE_JPEG)
        return BUS_ERR_NOT_SUPPORTED;
    if (config == RT_NULL || config->frame_callback == RT_NULL ||
        (config->mode != BUS_STREAM_MODE_FRAME &&
         config->mode != BUS_STREAM_MODE_HALF_FRAME))
        return -RT_EINVAL;
    expected = config->expected_frame_size != 0U ? config->expected_frame_size : config->buffer_size;
    if (!camera_dcmi_buffer_valid(config->buffers[0], config->buffer_size) ||
        expected == 0U || expected != config->buffer_size)
        return -RT_EINVAL;
    if (config->mode == BUS_STREAM_MODE_HALF_FRAME)
    {
        if (config->buffers[1] != RT_NULL ||
            ((expected / 2U) & (DCMI_BUFFER_ALIGNMENT - 1U)) != 0U)
            return -RT_EINVAL;
    }
    else
    {
        if (!camera_dcmi_buffer_valid(config->buffers[1], config->buffer_size) ||
            expected / sizeof(uint32_t) > DCMI_MAX_DMA_WORDS / 2U)
            return -RT_EINVAL;
        first = (uintptr_t)config->buffers[0];
        second = (uintptr_t)config->buffers[1];
        if (second != first + expected || second > UINTPTR_MAX - expected)
            return -RT_EINVAL;
    }
    result = camera_dcmi_lock(dcmi);
    if (result != RT_EOK)
        return result;
    if (!dcmi->initialized)
        result = -RT_ERROR;
    else if (dcmi->started || dcmi->dispatching)
        result = -RT_EBUSY;
    else
    {
        dcmi->stream = *config;
        dcmi->expected_size = expected;
        dcmi->dma_size = config->mode == BUS_STREAM_MODE_HALF_FRAME ?
                         expected : 2U * expected;
        level = rt_hw_interrupt_disable();
        dcmi->error = 0;
        dcmi->buffer_valid[0] = RT_FALSE;
        dcmi->buffer_valid[1] = RT_FALSE;
        dcmi->single_pending = RT_FALSE;
        dcmi->stopping = RT_FALSE;
        dcmi->native_stream = RT_TRUE;
        dcmi->started = RT_TRUE;
        rt_hw_interrupt_enable(level);
        result = camera_dcmi_arm_stream_locked(dcmi);
        if (result != RT_EOK)
        {
            int stop_result = camera_dcmi_stop_locked(dcmi);
            if (stop_result != RT_EOK)
                result = stop_result;
        }
    }
    camera_dcmi_unlock(dcmi);
    return result;
}

static int camera_dcmi_stream_frame_valid(bus_adapter_t *self,
                                          const bus_stream_frame_t *frame)
{
    camera_dcmi_t *dcmi = self->priv;
    rt_base_t level;
    uint32_t index;
    int valid = 0;

    if (frame == RT_NULL || frame->error != 0 || rt_interrupt_get_nest() != 0U)
        return 0;
    __DMB();
    level = rt_hw_interrupt_disable();
    if (dcmi->started && dcmi->native_stream && !dcmi->stopping && dcmi->error == 0 &&
        dcmi->capture_armed && frame->size == dcmi->expected_size)
    {
        if (dcmi->stream.mode == BUS_STREAM_MODE_HALF_FRAME)
        {
            if (frame->buffer == dcmi->stream.buffers[0] &&
                (frame->event_type == BUS_STREAM_EVENT_HALF_FIRST ||
                 frame->event_type == BUS_STREAM_EVENT_HALF_SECOND))
            {
                index = frame->event_type == BUS_STREAM_EVENT_HALF_FIRST ? 0U : 1U;
                valid = dcmi->buffer_valid[index] &&
                        frame->sequence == dcmi->buffer_sequence[index];
            }
        }
        else if (frame->event_type == BUS_STREAM_EVENT_FRAME)
        {
            for (index = 0U; index < 2U; ++index)
            {
                if (frame->buffer == dcmi->stream.buffers[index] && dcmi->buffer_valid[index] &&
                    frame->sequence == dcmi->buffer_sequence[index])
                {
                    valid = camera_dcmi_buffer_idle(dcmi, index);
                    break;
                }
            }
        }
    }
    rt_hw_interrupt_enable(level);
    return valid;
}

static int camera_dcmi_set_frame_callback(bus_adapter_t *self,
                                          bus_frame_notify_callback_t callback,
                                          void *context)
{
    camera_dcmi_t *dcmi = self->priv;
    int result = camera_dcmi_lock(dcmi);

    if (result != RT_EOK)
        return result;
    if (dcmi->started || dcmi->dispatching)
        result = -RT_EBUSY;
    else
    {
        dcmi->frame_callback = callback;
        dcmi->frame_context = context;
    }
    camera_dcmi_unlock(dcmi);
    return result;
}

static int camera_dcmi_start_capture_internal(bus_adapter_t *self, void *buffer,
                                              uint32_t size, uint32_t frames)
{
    camera_dcmi_t *dcmi = self->priv;
    camera_dcmi_hw_t *hardware;
    rt_base_t level;
    int result;

    if (!camera_dcmi_buffer_valid(buffer, size))
        return -RT_EINVAL;
    result = camera_dcmi_lock(dcmi);
    if (result != RT_EOK)
        return result;
    if (!dcmi->initialized || dcmi->frame_callback == RT_NULL)
        result = -RT_ERROR;
    else if (dcmi->started || dcmi->dispatching)
        result = -RT_EBUSY;
    else if (frames != 0U && dcmi->mode != BUS_CAPTURE_MODE_JPEG)
        result = BUS_ERR_NOT_SUPPORTED;
    else
    {
        hardware = &dcmi->hardware;
        result = camera_dcmi_prepare_locked(dcmi, buffer, size, frames != 0U);
        if (result != RT_EOK)
            goto done;
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)buffer, (int32_t)size);
        level = rt_hw_interrupt_disable();
        dcmi->single_buffer = buffer;
        dcmi->single_size = size;
        dcmi->single_received = 0U;
        dcmi->burst_frames = frames;
        dcmi->burst_count = 0U;
        dcmi->error = 0;
        dcmi->single_pending = RT_FALSE;
        dcmi->stopping = RT_FALSE;
        dcmi->native_stream = RT_FALSE;
        dcmi->capture_closing = RT_FALSE;
        dcmi->started = RT_TRUE;
        rt_hw_interrupt_enable(level);
        result = camera_dcmi_hw_start(dcmi);
        if (result == RT_EOK && (hardware->hdma.State != HAL_DMA_STATE_BUSY ||
                                hardware->hdma.ErrorCode != HAL_DMA_ERROR_NONE))
            result = -RT_EIO;
        if (result != RT_EOK)
        {
            int stop_result = camera_dcmi_stop_locked(dcmi);
            if (stop_result != RT_EOK)
                result = stop_result;
        }
    }
done:
    camera_dcmi_unlock(dcmi);
    return result;
}

static int camera_dcmi_start_capture(bus_adapter_t *self, void *buffer, uint32_t size)
{
    return camera_dcmi_start_capture_internal(self, buffer, size, 0U);
}

static int camera_dcmi_start_capture_frames(bus_adapter_t *self, void *buffer,
                                            uint32_t size, uint32_t frames)
{
    if (frames == 0U || frames > size / 4U)
        return -RT_EINVAL;
    return camera_dcmi_start_capture_internal(self, buffer, size, frames);
}

/* Keep DMA owned until its destination counter confirms that the final
 * memory writes have retired, including JPEG transfers ending before TC. */
static rt_bool_t camera_dcmi_single_drained(camera_dcmi_t *dcmi)
{
    camera_dcmi_hw_t *hardware = &dcmi->hardware;
    uint32_t remaining;
    uint32_t words = dcmi->single_size / 4U;
    rt_bool_t ready = RT_FALSE;
    rt_base_t level = rt_hw_interrupt_disable();

    if (hardware->hdma.State == HAL_DMA_STATE_BUSY &&
        (hardware->handle.Instance->FIFO_SR & DCMI_FIFO_SR_FIFO_CNT) == 0U)
    {
        remaining = ((GPDMA_Channel_TypeDef *)hardware->hdma.Instance)->CSR & GPDMA_CSR1_DSTNDT_Msk;
        if (dcmi->burst_frames != 0U)
        {
            uint32_t reads_left = __HAL_DMA_GET_COUNTER(&hardware->hdma);

            __DMB();
            /* JPEG_BYTE has no documented cumulative-frame semantics.
             * Use the non-wrapping DMA destination position instead. */
            ready = remaining != 0U && remaining < words && reads_left == remaining &&
                    (((GPDMA_Channel_TypeDef *)hardware->hdma.Instance)->CSR &
                     (GPDMA_CSR1_DSTNDT_Msk | GPDMA_CSR1_BUSERR_Msk | GPDMA_CSR1_CFGERR_Msk)) == remaining &&
                    __HAL_DMA_GET_COUNTER(&hardware->hdma) == reads_left &&
                    (hardware->handle.Instance->FIFO_SR & DCMI_FIFO_SR_FIFO_CNT) == 0U;
            if (ready)
                dcmi->single_received = (words - remaining) * 4U;
        }
        else if (dcmi->mode == BUS_CAPTURE_MODE_JPEG)
            ready = remaining <= words && words - remaining >= (dcmi->single_received + 3U) / 4U;
        else
            ready = remaining == 0U || remaining == words;
    }
    rt_hw_interrupt_enable(level);
    return ready;
}

static void camera_dcmi_single_work(struct rt_work *work, void *parameter)
{
    camera_dcmi_t *dcmi = parameter;
    bus_frame_notify_callback_t callback = RT_NULL;
    void *context = RT_NULL;
    uint8_t *buffer = RT_NULL;
    uint32_t size = 0U;
    uint32_t burst_frames = 0U;
    uint32_t burst_count = 0U;
    int error;
    rt_bool_t retry_stop = RT_FALSE;

    (void)work;
    if (camera_dcmi_lock(dcmi) != RT_EOK)
        return;
    if (dcmi->started && !dcmi->native_stream && dcmi->single_pending)
    {
        buffer = dcmi->single_buffer;
        camera_dcmi_hw_t *hardware = &dcmi->hardware;

        burst_frames = dcmi->burst_frames;
        burst_count = dcmi->burst_count;
        error = dcmi->error;
        if ((burst_frames != 0U && (burst_count != burst_frames ||
                                   hardware->hdma.State != HAL_DMA_STATE_BUSY)) ||
            hardware->error != 0U ||
            hardware->handle.ErrorCode != HAL_DCMI_ERROR_NONE ||
            hardware->hdma.ErrorCode != HAL_DMA_ERROR_NONE)
            dcmi->error = error = -RT_EIO;
        if (error == 0 && !camera_dcmi_single_drained(dcmi))
        {
            if ((rt_tick_t)(rt_tick_get() - dcmi->single_end_tick) < rt_tick_from_millisecond(100U))
            {
                camera_dcmi_unlock(dcmi);
                rt_work_submit(&dcmi->single_work, 1);
                return;
            }
            dcmi->error = error = -RT_ETIMEOUT;
        }
        size = dcmi->single_received;
        if (error != 0 && dcmi->mode == BUS_CAPTURE_MODE_JPEG)
        {
            uint32_t reads_left = 0U, writes_left = 0U, ccr = 0U;
            rt_base_t level = rt_hw_interrupt_disable();

            /* Normal DMA TC and HAL errors release the dynamic channel
             * before notification. Never inspect its next owner's state. */
            if (hardware->hdma.State == HAL_DMA_STATE_BUSY)
            {
                reads_left = __HAL_DMA_GET_COUNTER(&hardware->hdma);
                writes_left = ((GPDMA_Channel_TypeDef *)hardware->hdma.Instance)->CSR &
                              GPDMA_CSR1_DSTNDT_Msk;
                ccr = hardware->hdma.Instance->CCR;
            }
            rt_hw_interrupt_enable(level);
            LOG_E("jpeg error=%d bytes=%u fifo=%u src_left=%u dst_left=%u",
                  error, (unsigned int)size,
                  (unsigned int)(hardware->handle.Instance->FIFO_SR & DCMI_FIFO_SR_FIFO_CNT),
                  (unsigned int)reads_left, (unsigned int)writes_left);
            LOG_E("jpeg dcmi_err=%x hal_err=%x dma_err=%x ccr=%08x",
                  (unsigned int)hardware->error, (unsigned int)hardware->handle.ErrorCode,
                  (unsigned int)hardware->hdma.ErrorCode, (unsigned int)ccr);
        }
        callback = dcmi->frame_callback;
        context = dcmi->frame_context;
        if (camera_dcmi_stop_locked(dcmi) != RT_EOK)
        {
            callback = RT_NULL;
            retry_stop = RT_TRUE;
        }
        else if (error != 0 || hardware->error != 0U ||
                 hardware->handle.ErrorCode != HAL_DCMI_ERROR_NONE ||
                 hardware->hdma.ErrorCode != HAL_DMA_ERROR_NONE)
        {
            buffer = RT_NULL;
            size = 0U;
        }
        else
        {
            SCB_InvalidateDCache_by_Addr(buffer, (int32_t)((size + 63U) & ~63U));
            if (burst_frames != 0U)
            {
                size_t offset = 0U;
                size_t frame_size = camera_jpeg_find_frame(buffer, size, burst_frames, &offset);

                LOG_D("jpeg continuous frames=%u/%u received=%u offset=%u frame=%u",
                      (unsigned int)burst_count, (unsigned int)burst_frames,
                      (unsigned int)size, (unsigned int)offset, (unsigned int)frame_size);
                if (frame_size == 0U)
                {
                    buffer = RT_NULL;
                    size = 0U;
                }
                else
                {
                    memmove(buffer, buffer + offset, frame_size);
                    size = (uint32_t)frame_size;
                }
            }
            else if (dcmi->mode == BUS_CAPTURE_MODE_JPEG)
            {
                uint32_t end = (uint32_t)camera_jpeg_frame_size(buffer, size);

                /* DVP receive length may include sensor output after EOI. */
                if (end == 0U)
                {
                    LOG_E("jpeg markers invalid bytes=%u soi=%02x%02x tail=%02x%02x",
                          (unsigned int)size, buffer[0], buffer[1], buffer[size - 2U], buffer[size - 1U]);
                    buffer = RT_NULL;
                    size = 0U;
                }
                else
                {
                    if (size - end > 3U)
                        LOG_D("jpeg received=%u frame=%u trailing=%u",
                              (unsigned int)size, (unsigned int)end, (unsigned int)(size - end));
                    size = end;
                }
            }
        }
    }
    if (callback != RT_NULL)
    {
        dcmi->dispatching = RT_TRUE;
        dcmi->dispatch_thread = rt_thread_self();
    }
    camera_dcmi_unlock(dcmi);
    if (retry_stop)
        rt_work_submit(&dcmi->single_work, rt_tick_from_millisecond(100U));
    if (callback != RT_NULL)
    {
        callback(buffer, size, context);
        if (camera_dcmi_lock(dcmi) == RT_EOK)
        {
            dcmi->dispatching = RT_FALSE;
            dcmi->dispatch_thread = RT_NULL;
            camera_dcmi_unlock(dcmi);
        }
    }
}

static int camera_dcmi_start(bus_adapter_t *self)
{
    camera_dcmi_t *dcmi = self->priv;
    int result = camera_dcmi_lock(dcmi);

    if (result != RT_EOK)
        return result;
    if (!dcmi->initialized)
        result = -RT_ERROR;
    else if (dcmi->dispatching)
        result = -RT_EBUSY;
    camera_dcmi_unlock(dcmi);
    return result;
}

static int camera_dcmi_set_mode(bus_adapter_t *self, bus_capture_mode_t mode)
{
    bus_adapter_config_t config = {.mode = mode};
    return camera_dcmi_config(self, &config);
}

static int camera_dcmi_set_pingpong_size(bus_adapter_t *self, uint32_t size)
{
    (void)self;
    return size != 0U && (size & 3U) == 0U ? BUS_OK : BUS_ERR_INVALID;
}

static const bus_adapter_ops_t s_dcmi_ops =
{
    .config = camera_dcmi_config,
    .init = camera_dcmi_init,
    .deinit = camera_dcmi_deinit,
    .start = camera_dcmi_start,
    .stop = camera_dcmi_stop,
    .set_frame_notify_callback = camera_dcmi_set_frame_callback,
    .start_capture = camera_dcmi_start_capture,
    .start_capture_frames = camera_dcmi_start_capture_frames,
    .abort_capture = camera_dcmi_stop,
    .set_pingpong_size = camera_dcmi_set_pingpong_size,
    .set_mode = camera_dcmi_set_mode,
    .start_stream = camera_dcmi_start_stream,
    .stream_frame_valid = camera_dcmi_stream_frame_valid,
};

static bus_adapter_t s_dcmi_adapter =
{
    .name = CAMERA_DATA_BUS_ADAPTER_NAME,
    .type = BUS_TYPE_DVP,
    .ops = &s_dcmi_ops,
    .priv = &s_dcmi,
};

static int camera_dcmi_register(void)
{
    return bus_adapter_register(&s_dcmi_adapter);
}
INIT_BOARD_EXPORT(camera_dcmi_register);
