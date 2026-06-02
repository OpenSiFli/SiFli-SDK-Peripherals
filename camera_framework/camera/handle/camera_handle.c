/******************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file camera_handle.c
 * 
 * @par dependencies
 * - camera_handle.h  : public API and all type definitions
 * - rthw.h           : rt_hw_interrupt_disable/enable for ISR-safe queue ops
 * - rtdbg.h          : LOG_W / LOG_E macros (pulled in via DBG_TAG block)
 * 
 * @author SiFli 思澈科技
 * 
 * @brief Camera handle layer — sits between application code and the
 * underlying camera sensor driver (e.g. ov2640).
 *
 * This module provides two capture modes:
 *
 *  - **Single-shot** (camera_capture_single):
 *    The caller supplies a destination buffer; the function blocks via the
 *    selected driver until one frame is available or the driver's internal
 *    timeout fires, then returns the actual byte count in request->frame_size.
 *
 *  - **Streaming** (camera_start_stream / camera_get_stream_frame /
 *    camera_stop_stream):
 *    Double-buffered DMA streaming with a two-slot FIFO ready queue.
 *    Frames are pushed into the queue by camera_stream_frame_ready_callback,
 *    which runs in ISR context (via the bus-adapter callback chain) each time
 *    a DMA transfer completes.  The application thread dequeues frames one at
 *    a time via camera_get_stream_frame, which blocks on a semaphore until a
 *    frame is ready or the given timeout expires.  When the consumer is too
 *    slow and the two-slot queue is full, the oldest frame is silently
 *    overwritten and dropped_count is incremented.
 *
 * @par Processing flow (typical single-sensor use)
 *
 *  1. camera_handler_instance_init()  — open driver, set JPEG/VGA/
 *                                       quality-10 defaults.
 *  2. camera_change_settings()        — push pixformat / framesize / quality.
 *  3a. camera_capture_single()        — blocking single-shot capture.
 *  3b. camera_capture_single_async()  — non-blocking single-shot (driver callback path).
 *  3c. camera_start_stream()          — arm DMA streaming,
 *      camera_get_stream_frame()      — dequeue frames (blocking),
 *      camera_stop_stream()           — stop DMA + drain queue.
 *  4. camera_deinit()                 — close driver + detach
 *                                       frame semaphore.
 *
 * @version V1.0 2026-4-3
 *
 * @note 1 tab == 4 spaces!
 * 
 *****************************************************************************/
#include "camera_handle_internal.h"
#include "camera_driver_desc.h"
#include "rtthread.h"
#include "rthw.h"

#ifdef CAMERA_HANDLE_TESTING
static const camera_device_ops_t *s_camera_test_ops = RT_NULL;

void camera_handle_set_test_ops(const camera_device_ops_t *ops)
{
    s_camera_test_ops = ops;
}
#endif

static const camera_device_ops_t *camera_get_board_ops(void)
{
#ifdef CAMERA_HANDLE_TESTING
    if (s_camera_test_ops != RT_NULL)
    {
        return s_camera_test_ops;
    }
#endif

    return camera_driver_get_default_ops();
}

#define DBG_TAG "camera.handle"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define CAMERA_JPEG_PARSE_THREAD_STACK_SIZE 2048
#define CAMERA_JPEG_PARSE_THREAD_PRIORITY   12
#define CAMERA_JPEG_PARSE_THREAD_TICK       10
#define CAMERA_STREAM_READY_DEPTH           4

static struct rt_mutex s_camera_api_lock;
static volatile rt_uint8_t s_camera_api_lock_state = 0;

static camera_handle_status_t camera_api_lock(void);
static void camera_api_unlock(void);

typedef enum
{
    CAMERA_API_LOCK_UNINITIALIZED = 0,
    CAMERA_API_LOCK_INITIALIZING  = 1,
    CAMERA_API_LOCK_READY         = 2,
} camera_api_lock_state_t;

camera_handle_status_t camera_get_capabilities(camera_handler_instance_t *instance,
                                               const camera_capabilities_t **caps)
{
    camera_handle_status_t status = camera_api_lock();
    if (status != CAMERA_OK)
    {
        return status;
    }

    if (instance == RT_NULL || caps == RT_NULL)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    if (instance->device_ops == RT_NULL ||
        instance->device_ops->capabilities == RT_NULL)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    *caps = instance->device_ops->capabilities;
    status = CAMERA_OK;

out:
    camera_api_unlock();
    return status;
}

/** @brief Convert RT-Thread error code to camera_handle_status_t. */
static camera_handle_status_t camera_status_from_rt_err(rt_err_t err)
{
    switch (err)
    {
        case RT_EOK:
            return CAMERA_OK;
        case -RT_ETIMEOUT:
            return CAMERA_ERRORTIMEOUT;
        case -RT_ENOMEM:
            return CAMERA_ERRORNOMEMORY;
        case -RT_EINVAL:
            return CAMERA_ERRORPARAMETER;
        case -RT_EBUSY:
            return CAMERA_ERRORRESOURCE;
        default:
            return CAMERA_ERROR;
    }
}

static camera_handle_status_t camera_api_lock(void)
{
    rt_err_t result;
    rt_base_t level;

    if (rt_interrupt_get_nest() != 0)
    {
        return CAMERA_ERRORISR;
    }

    level = rt_hw_interrupt_disable();
    if (s_camera_api_lock_state == CAMERA_API_LOCK_READY)
    {
        rt_hw_interrupt_enable(level);
        result = rt_mutex_take(&s_camera_api_lock, RT_WAITING_FOREVER);
        return camera_status_from_rt_err(result);
    }

    if (s_camera_api_lock_state == CAMERA_API_LOCK_INITIALIZING)
    {
        rt_hw_interrupt_enable(level);
        return CAMERA_ERRORRESOURCE;
    }

    s_camera_api_lock_state = CAMERA_API_LOCK_INITIALIZING;
    rt_hw_interrupt_enable(level);

    result = rt_mutex_init(&s_camera_api_lock, "cam_api", RT_IPC_FLAG_PRIO);
    if (result != RT_EOK)
    {
        level = rt_hw_interrupt_disable();
        s_camera_api_lock_state = CAMERA_API_LOCK_UNINITIALIZED;
        rt_hw_interrupt_enable(level);
        return camera_status_from_rt_err(result);
    }

    level = rt_hw_interrupt_disable();
    s_camera_api_lock_state = CAMERA_API_LOCK_READY;
    rt_hw_interrupt_enable(level);

    result = rt_mutex_take(&s_camera_api_lock, RT_WAITING_FOREVER);
    return camera_status_from_rt_err(result);
}

static void camera_api_unlock(void)
{
    if (s_camera_api_lock_state == CAMERA_API_LOCK_READY)
    {
        rt_mutex_release(&s_camera_api_lock);
    }
}

/** @brief Check whether handle instance is open and has ops table. */
static camera_handle_status_t camera_require_open(camera_handler_instance_t *instance)
{
    if (instance == RT_NULL || !instance->is_open || instance->device_ops == RT_NULL)
    {
        return CAMERA_ERRORRESOURCE;
    }

    return CAMERA_OK;
}

/** @brief Clear stream queue state and drop counter. */
static void camera_stream_reset_queue(camera_handler_instance_t *instance)
{
    camera_jpeg_stream_runtime_t *jpeg = &instance->stream.jpeg;
    rt_base_t level = rt_hw_interrupt_disable();
    instance->stream.head = 0;
    instance->stream.tail = 0;
    instance->stream.count = 0;
    instance->stream.dropped_count = 0;
    jpeg->segment_mode = RT_FALSE;
    jpeg->ring_base = RT_NULL;
    jpeg->ring_size = 0;
    jpeg->normalize_base = RT_NULL;
    jpeg->normalize_size = 0;
    jpeg->soi_found = 0;
    jpeg->prev_valid = 0;
    jpeg->prev_byte = 0;
    jpeg->soi_offset = 0;
    jpeg->seg_head = 0;
    jpeg->seg_tail = 0;
    jpeg->seg_count = 0;
    rt_hw_interrupt_enable(level);
}

static void camera_stream_enqueue_ready_frame(camera_handler_instance_t *instance,
                                              const camera_stream_frame_t *frame)
{
    rt_base_t level = rt_hw_interrupt_disable();
    rt_bool_t dropped = RT_FALSE;
    if (instance->stream.count == CAMERA_STREAM_READY_DEPTH)
    {
        instance->stream.tail = (instance->stream.tail + 1) % CAMERA_STREAM_READY_DEPTH;
        instance->stream.count--;
        instance->stream.dropped_count++;
        dropped = RT_TRUE;
    }
    rt_uint32_t dropped_total = instance->stream.dropped_count;

    instance->stream.ready_frames[instance->stream.head] = *frame;
    instance->stream.head = (instance->stream.head + 1) % CAMERA_STREAM_READY_DEPTH;
    instance->stream.count++;
    rt_hw_interrupt_enable(level);

    if (dropped && ((dropped_total == 1) || ((dropped_total & 0x1FU) == 0U)))
    {
        LOG_W("camera: stream queue full, dropped frames=%u", (unsigned int)dropped_total);
    }

    if (instance->stream.sem_initialized && !dropped)
    {
        rt_sem_release(&instance->stream.frame_sem);
    }
}

static void camera_stream_parse_jpeg_segment(camera_handler_instance_t *instance,
                                             const camera_jpeg_segment_t *segment)
{
    uint8_t *ring_base = instance->stream.jpeg.ring_base;
    rt_size_t ring_size = instance->stream.jpeg.ring_size;
    uint8_t *segment_base = (uint8_t *)segment->buffer;
    rt_size_t segment_size = segment->size;
    rt_size_t segment_off;
    rt_size_t i;

    if (ring_base == RT_NULL || ring_size < 2 || segment_base == RT_NULL || segment_size == 0)
    {
        return;
    }

    segment_off = (rt_size_t)(segment_base - ring_base);
    if (segment_off >= ring_size)
    {
        return;
    }

    for (i = 0; i < segment_size; i++)
    {
        rt_size_t off = segment_off + i;
        uint8_t b;
        if (off >= ring_size)
        {
            break;
        }

        b = ring_base[off];
        if (!instance->stream.jpeg.soi_found)
        {
            if (instance->stream.jpeg.prev_valid &&
                instance->stream.jpeg.prev_byte == 0xFFU &&
                b == 0xD8U)
            {
                instance->stream.jpeg.soi_found = 1;
                instance->stream.jpeg.soi_offset = (off == 0) ? (ring_size - 1) : (off - 1);
            }
        }
        else
        {
            if (instance->stream.jpeg.prev_valid &&
                instance->stream.jpeg.prev_byte == 0xFFU &&
                b == 0xD9U)
            {
                camera_stream_frame_t frame = {0};
                rt_size_t start = instance->stream.jpeg.soi_offset;
                rt_size_t end = off;
                rt_size_t frame_size;

                if (end >= start)
                {
                    frame_size = (end - start) + 1;
                }
                else
                {
                    frame_size = (ring_size - start) + end + 1;
                }

                frame.buffer = ring_base + start;
                frame.buffer_size = ring_size;
                frame.frame_size = frame_size;
                frame.sequence = segment->sequence;
                frame.buffer_index = 0;

                if (end < start)
                {
                    if (instance->stream.jpeg.normalize_base != RT_NULL &&
                        instance->stream.jpeg.normalize_size >= frame_size)
                    {
                        rt_size_t tail_size = ring_size - start;
                        rt_size_t head_size = end + 1;
                        rt_memcpy(instance->stream.jpeg.normalize_base, ring_base + start, tail_size);
                        rt_memcpy(instance->stream.jpeg.normalize_base + tail_size, ring_base, head_size);
                        frame.buffer = instance->stream.jpeg.normalize_base;
                        frame.buffer_size = instance->stream.jpeg.normalize_size;
                        frame.buffer_index = 1;
                    }
                }
                camera_stream_enqueue_ready_frame(instance, &frame);
                instance->stream.jpeg.soi_found = 0;
            }
        }

        instance->stream.jpeg.prev_byte = b;
        instance->stream.jpeg.prev_valid = 1;
    }
}

static rt_bool_t camera_stream_pop_jpeg_segment(camera_handler_instance_t *instance,
                                                camera_jpeg_segment_t *segment)
{
    rt_base_t level = rt_hw_interrupt_disable();
    if (instance->stream.jpeg.seg_count == 0)
    {
        rt_hw_interrupt_enable(level);
        return RT_FALSE;
    }

    *segment = instance->stream.jpeg.segments[instance->stream.jpeg.seg_tail];
    instance->stream.jpeg.seg_tail = (instance->stream.jpeg.seg_tail + 1) % 8;
    instance->stream.jpeg.seg_count--;
    rt_hw_interrupt_enable(level);
    return RT_TRUE;
}

static void camera_jpeg_parser_thread(void *parameter)
{
    camera_handler_instance_t *instance = (camera_handler_instance_t *)parameter;
    camera_jpeg_segment_t segment;

    if (instance == RT_NULL)
    {
        return;
    }

    while (instance->stream.jpeg.parse_thread_running)
    {
        if (rt_sem_take(&instance->stream.jpeg.parse_sem, RT_WAITING_FOREVER) != RT_EOK)
        {
            continue;
        }

        while (camera_stream_pop_jpeg_segment(instance, &segment))
        {
            if (!instance->stream.enabled || !instance->stream.jpeg.segment_mode)
            {
                continue;
            }
            camera_stream_parse_jpeg_segment(instance, &segment);
        }
    }

    instance->stream.jpeg.parse_thread = RT_NULL;
    if (instance->stream.jpeg.exit_sem_initialized)
    {
        rt_sem_release(&instance->stream.jpeg.exit_sem);
    }
}

/** @brief Stream frame callback: enqueue frame and signal semaphore. */
static void camera_stream_frame_ready_callback(void *context,
                                               const camera_stream_frame_t *frame)
{
    camera_handler_instance_t *instance = (camera_handler_instance_t *)context;

    if (instance == RT_NULL || frame == RT_NULL || !instance->stream.enabled)
    {
        return;
    }

    if (instance->stream.jpeg.segment_mode)
    {
        camera_jpeg_segment_t segment;
        rt_base_t level = rt_hw_interrupt_disable();
        if (instance->stream.jpeg.seg_count < 8)
        {
            segment.buffer = frame->buffer;
            segment.size = frame->frame_size;
            segment.sequence = frame->sequence;
            instance->stream.jpeg.segments[instance->stream.jpeg.seg_head] = segment;
            instance->stream.jpeg.seg_head = (instance->stream.jpeg.seg_head + 1) % 8;
            instance->stream.jpeg.seg_count++;
            rt_hw_interrupt_enable(level);
            if (instance->stream.jpeg.parse_sem_initialized)
            {
                rt_sem_release(&instance->stream.jpeg.parse_sem);
            }
        }
        else
        {
            instance->stream.dropped_count++;
            rt_hw_interrupt_enable(level);
        }
        return;
    }
    camera_stream_enqueue_ready_frame(instance, frame);
}

/** @brief Initialize handle instance and open sensor driver. */
camera_handle_status_t camera_handler_instance_init(camera_handler_instance_t **instance)
{
    camera_handle_status_t status;
    rt_err_t result;
    const camera_device_ops_t *ops;
    camera_handler_instance_t *handle;

    status = camera_api_lock();
    if (status != CAMERA_OK)
    {
        return status;
    }

    if (instance == RT_NULL)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    if (*instance == RT_NULL)
    {
        *instance = (camera_handler_instance_t *)rt_calloc(1, sizeof(**instance));
        if (*instance == RT_NULL)
        {
            status = CAMERA_ERRORNOMEMORY;
            goto out;
        }
    }

    handle = *instance;

    if (handle->is_open)
    {
        status = CAMERA_OK;
        goto out;
    }

    ops = camera_get_board_ops();
    if (ops == RT_NULL)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    rt_memset(handle, 0, sizeof(*handle));
    handle->device_ops = ops;

    if (ops->open == RT_NULL)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    result = (rt_err_t)ops->open();
    if (result != RT_EOK)
    {
        rt_free(handle);
        *instance = RT_NULL;
        status = camera_status_from_rt_err(result);
        goto out;
    }

    handle->is_open = RT_TRUE;
    handle->active_config.pixformat = PIXFORMAT_JPEG;
    handle->active_config.framesize = FRAMESIZE_VGA;
    handle->active_config.quality = 10;
    handle->stream.sem_initialized = RT_FALSE;
    handle->stream.jpeg.parse_sem_initialized = RT_FALSE;
    handle->stream.jpeg.exit_sem_initialized = RT_FALSE;
    handle->stream.jpeg.parse_thread = RT_NULL;
    handle->stream.jpeg.parse_thread_running = RT_FALSE;
    handle->stream.enabled = RT_FALSE;
    camera_stream_reset_queue(handle);

    status = CAMERA_OK;

out:
    camera_api_unlock();
    return status;
}

/** @brief Close driver and release handle-owned resources. */
camera_handle_status_t camera_deinit(camera_handler_instance_t **instance)
{
    camera_handle_status_t status = camera_api_lock();
    camera_handler_instance_t *handle;
    if (status != CAMERA_OK)
    {
        return status;
    }

    if (instance == RT_NULL || *instance == RT_NULL)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    handle = *instance;

    if (handle->stream.enabled)
    {
        if (camera_require_open(handle) == CAMERA_OK &&
            handle->device_ops != RT_NULL &&
            handle->device_ops->stop_stream != RT_NULL)
        {
            handle->device_ops->stop_stream();
        }
        handle->stream.enabled = RT_FALSE;
        camera_stream_reset_queue(handle);
    }

    if (handle->stream.jpeg.parse_thread_running)
    {
        rt_err_t wait_res;
        handle->stream.jpeg.parse_thread_running = RT_FALSE;
        if (handle->stream.jpeg.parse_sem_initialized)
        {
            rt_sem_release(&handle->stream.jpeg.parse_sem);
        }
        wait_res = RT_ERROR;
        if (handle->stream.jpeg.exit_sem_initialized)
        {
            wait_res = rt_sem_take(&handle->stream.jpeg.exit_sem, rt_tick_from_millisecond(200));
        }
        if (wait_res != RT_EOK)
        {
            LOG_W("camera: jpeg parse thread exit wait timeout");
        }
    }
    if (handle->stream.jpeg.parse_sem_initialized)
    {
        rt_sem_detach(&handle->stream.jpeg.parse_sem);
        handle->stream.jpeg.parse_sem_initialized = RT_FALSE;
    }
    if (handle->stream.jpeg.exit_sem_initialized)
    {
        rt_sem_detach(&handle->stream.jpeg.exit_sem);
        handle->stream.jpeg.exit_sem_initialized = RT_FALSE;
    }

    if (handle->is_open &&
        handle->device_ops != RT_NULL &&
        handle->device_ops->close != RT_NULL)
    {
        handle->device_ops->close();
    }

    handle->is_open = RT_FALSE;

    if (handle->stream.sem_initialized)
    {
        rt_sem_detach(&handle->stream.frame_sem);
        handle->stream.sem_initialized = RT_FALSE;
    }

    rt_free(handle);
    *instance = RT_NULL;

    status = CAMERA_OK;

out:
    camera_api_unlock();
    return status;
}

/** @brief Apply pixformat/framesize/quality to sensor and cache active config. */
camera_handle_status_t camera_change_settings(camera_handler_instance_t *instance,
                                              const camera_capture_config_t *config)
{
    camera_handle_status_t status;
    rt_err_t result;
    const camera_device_ops_t *device_ops;

    status = camera_api_lock();
    if (status != CAMERA_OK)
    {
        return status;
    }

    if (instance == RT_NULL || config == RT_NULL)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    status = camera_require_open(instance);
    if (status != CAMERA_OK)
    {
        goto out;
    }

    if (instance->stream.enabled)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    device_ops = instance->device_ops;
    if (device_ops->set_pixformat == RT_NULL ||
        device_ops->set_framesize == RT_NULL ||
        device_ops->set_quality == RT_NULL)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    result = (rt_err_t)device_ops->set_pixformat(config->pixformat);
    if (result != RT_EOK)
    {
        status = camera_status_from_rt_err(result);
        goto out;
    }

    result = (rt_err_t)device_ops->set_framesize(config->framesize);
    if (result != RT_EOK)
    {
        status = camera_status_from_rt_err(result);
        goto out;
    }

    result = (rt_err_t)device_ops->set_quality(config->quality);
    if (result != RT_EOK)
    {
        status = camera_status_from_rt_err(result);
        goto out;
    }

    instance->active_config = *config;
    status = CAMERA_OK;

out:
    camera_api_unlock();
    return status;
}

camera_handle_status_t camera_get_stream_dropped_count(
    camera_handler_instance_t *instance,
    rt_uint32_t *dropped_count)
{
    camera_handle_status_t status;

    status = camera_api_lock();
    if (status != CAMERA_OK)
    {
        return status;
    }

    if (instance == RT_NULL || dropped_count == RT_NULL)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    status = camera_require_open(instance);
    if (status != CAMERA_OK)
    {
        goto out;
    }

    *dropped_count = instance->stream.dropped_count;
    status = CAMERA_OK;

out:
    camera_api_unlock();
    return status;
}

/** @brief Start one async single-shot capture. */
camera_handle_status_t camera_capture_single_async(
    camera_handler_instance_t         *instance,
    camera_capture_request_t          *request,
    camera_capture_done_callback_t     callback,
    void                              *context)
{
    camera_handle_status_t status;
    rt_err_t result;

    status = camera_api_lock();
    if (status != CAMERA_OK)
    {
        return status;
    }

    if (instance == RT_NULL || request == RT_NULL || callback == RT_NULL)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    status = camera_require_open(instance);
    if (status != CAMERA_OK ||
        instance->device_ops->capture_async == RT_NULL)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    if (request->buffer == RT_NULL || request->buffer_size == 0)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    if (instance->stream.enabled)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    result = (rt_err_t)instance->device_ops->capture_async(request->buffer,
                                                           request->buffer_size,
                                                           callback,
                                                           context);
    if (result != RT_EOK)
    {
        status = camera_status_from_rt_err(result);
        goto out;
    }

    status = CAMERA_OK;

out:
    camera_api_unlock();
    return status;
}

/** @brief Perform one blocking single-shot capture. */
camera_handle_status_t camera_capture_single(camera_handler_instance_t *instance,
                                             camera_capture_request_t *request)
{
    camera_handle_status_t status;
    rt_size_t read_size;

    status = camera_api_lock();
    if (status != CAMERA_OK)
    {
        return status;
    }

    if (instance == RT_NULL || request == RT_NULL)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    status = camera_require_open(instance);
    if (status != CAMERA_OK ||
        instance->device_ops->capture == RT_NULL)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    if (request->buffer == RT_NULL || request->buffer_size == 0)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    if (instance->stream.enabled)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    read_size = instance->device_ops->capture(request->buffer,
                                              request->buffer_size);
    if (read_size == 0)
    {
        request->frame_size = 0;
        status = CAMERA_ERRORTIMEOUT;
        goto out;
    }

    request->frame_size = read_size;
    status = CAMERA_OK;

out:
    camera_api_unlock();
    return status;
}

/** @brief Start continuous stream using two caller-provided buffers. */
camera_handle_status_t camera_start_stream(camera_handler_instance_t *instance,
                                           const camera_stream_config_t *config)
{
    camera_handle_status_t status;
    camera_stream_start_args_t args;
    rt_err_t result;

    status = camera_api_lock();
    if (status != CAMERA_OK)
    {
        return status;
    }

    if (instance == RT_NULL || config == RT_NULL)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    status = camera_require_open(instance);
    if (status != CAMERA_OK ||
        instance->device_ops->start_stream == RT_NULL)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    if (config->buffers[0] == RT_NULL || config->buffer_size == 0)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    if (instance->active_config.pixformat == PIXFORMAT_JPEG &&
        config->buffers[1] == RT_NULL)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    if (instance->active_config.pixformat != PIXFORMAT_JPEG &&
        config->buffers[1] == RT_NULL)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    if (instance->stream.enabled)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    if (!instance->stream.sem_initialized)
    {
        result = rt_sem_init(&instance->stream.frame_sem,
                             "cam_strm",
                             0,
                             RT_IPC_FLAG_FIFO);
        if (result != RT_EOK)
        {
            status = camera_status_from_rt_err(result);
            goto out;
        }
        instance->stream.sem_initialized = RT_TRUE;
    }

    while (rt_sem_trytake(&instance->stream.frame_sem) == RT_EOK)
    {
    }

    camera_stream_reset_queue(instance);
    instance->stream.enabled = RT_TRUE;
    instance->stream.jpeg.segment_mode = (instance->active_config.pixformat == PIXFORMAT_JPEG) ? RT_TRUE : RT_FALSE;
    instance->stream.jpeg.ring_base = instance->stream.jpeg.segment_mode ? (uint8_t *)config->buffers[0] : RT_NULL;
    instance->stream.jpeg.ring_size = instance->stream.jpeg.segment_mode ? config->buffer_size : 0;
    instance->stream.jpeg.normalize_base = instance->stream.jpeg.segment_mode ? (uint8_t *)config->buffers[1] : RT_NULL;
    instance->stream.jpeg.normalize_size = instance->stream.jpeg.segment_mode ? config->buffer_size : 0;

    if (instance->stream.jpeg.segment_mode)
    {
        if (!instance->stream.jpeg.parse_sem_initialized)
        {
            result = rt_sem_init(&instance->stream.jpeg.parse_sem, "cam_jseg", 0, RT_IPC_FLAG_FIFO);
            if (result != RT_EOK)
            {
                instance->stream.enabled = RT_FALSE;
                camera_stream_reset_queue(instance);
                status = camera_status_from_rt_err(result);
                goto out;
            }
            instance->stream.jpeg.parse_sem_initialized = RT_TRUE;
        }
        if (!instance->stream.jpeg.exit_sem_initialized)
        {
            result = rt_sem_init(&instance->stream.jpeg.exit_sem, "cam_jext", 0, RT_IPC_FLAG_FIFO);
            if (result != RT_EOK)
            {
                instance->stream.enabled = RT_FALSE;
                camera_stream_reset_queue(instance);
                status = camera_status_from_rt_err(result);
                goto out;
            }
            instance->stream.jpeg.exit_sem_initialized = RT_TRUE;
        }

        if (instance->stream.jpeg.parse_thread == RT_NULL)
        {
            instance->stream.jpeg.parse_thread_running = RT_TRUE;
            instance->stream.jpeg.parse_thread = rt_thread_create("cam_jpeg_parse",
                                                                  camera_jpeg_parser_thread,
                                                                  instance,
                                                                  CAMERA_JPEG_PARSE_THREAD_STACK_SIZE,
                                                                  CAMERA_JPEG_PARSE_THREAD_PRIORITY,
                                                                  CAMERA_JPEG_PARSE_THREAD_TICK);
            if (instance->stream.jpeg.parse_thread == RT_NULL)
            {
                instance->stream.jpeg.parse_thread_running = RT_FALSE;
                instance->stream.enabled = RT_FALSE;
                camera_stream_reset_queue(instance);
                status = CAMERA_ERRORNOMEMORY;
                goto out;
            }
            rt_thread_startup(instance->stream.jpeg.parse_thread);
        }
    }

    args.buffers[0] = config->buffers[0];
    args.buffers[1] = (config->buffers[1] != RT_NULL) ? config->buffers[1] : config->buffers[0];
    args.buffer_size = config->buffer_size;
    args.frame_callback = camera_stream_frame_ready_callback;
    args.callback_context = instance;

    result = (rt_err_t)instance->device_ops->start_stream(&args);
    if (result != RT_EOK)
    {
        instance->stream.enabled = RT_FALSE;
        camera_stream_reset_queue(instance);
        status = camera_status_from_rt_err(result);
        goto out;
    }

    status = CAMERA_OK;

out:
    camera_api_unlock();
    return status;
}

/** @brief Pop one frame from stream queue, waiting up to @p timeout ticks. */
camera_handle_status_t camera_get_stream_frame(camera_handler_instance_t *instance,
                                               camera_stream_frame_t *frame,
                                               rt_int32_t timeout)
{
    camera_handle_status_t status;
    rt_base_t level;
    rt_err_t result;

    status = camera_api_lock();
    if (status != CAMERA_OK)
    {
        return status;
    }

    if (instance == RT_NULL || frame == RT_NULL)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    if (!instance->stream.enabled || !instance->stream.sem_initialized)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    result = rt_sem_take(&instance->stream.frame_sem, timeout);
    if (result != RT_EOK)
    {
        status = camera_status_from_rt_err(result);
        goto out;
    }

    level = rt_hw_interrupt_disable();
    if (instance->stream.count == 0)
    {
        rt_hw_interrupt_enable(level);
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    *frame = instance->stream.ready_frames[instance->stream.tail];
    instance->stream.tail = (instance->stream.tail + 1) % CAMERA_STREAM_READY_DEPTH;
    instance->stream.count--;
    rt_hw_interrupt_enable(level);

    status = CAMERA_OK;

out:
    camera_api_unlock();
    return status;
}

/** @brief Stop stream and clear queue state. */
camera_handle_status_t camera_stop_stream(camera_handler_instance_t *instance)
{
    camera_handle_status_t status;
    rt_err_t result;

    status = camera_api_lock();
    if (status != CAMERA_OK)
    {
        return status;
    }

    if (instance == RT_NULL)
    {
        status = CAMERA_ERRORPARAMETER;
        goto out;
    }

    if (!instance->stream.enabled)
    {
        status = CAMERA_OK;
        goto out;
    }

    status = camera_require_open(instance);
    if (status != CAMERA_OK ||
        instance->device_ops->stop_stream == RT_NULL)
    {
        status = CAMERA_ERRORRESOURCE;
        goto out;
    }

    result = (rt_err_t)instance->device_ops->stop_stream();
    instance->stream.enabled = RT_FALSE;
    if (instance->stream.jpeg.parse_sem_initialized)
    {
        rt_sem_release(&instance->stream.jpeg.parse_sem);
    }
    camera_stream_reset_queue(instance);

    if (result != RT_EOK)
    {
        status = camera_status_from_rt_err(result);
        goto out;
    }

    status = CAMERA_OK;

out:
    camera_api_unlock();
    return status;
}


