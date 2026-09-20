#include "camera_sensor_runtime.h"

#include "rthw.h"

#define DBG_TAG "camera.sensor"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

typedef struct
{
    uint16_t width;
    uint16_t height;
} camera_resolution_t;

static const camera_resolution_t s_resolutions[FRAMESIZE_INVALID] =
{
    {   96,   96 },
    {  160,  120 },
    {  128,  128 },
    {  176,  144 },
    {  240,  176 },
    {  240,  240 },
    {  320,  240 },
    {  320,  320 },
    {  400,  296 },
    {  480,  320 },
    {  640,  480 },
    {  800,  600 },
    { 1024,  768 },
    { 1280,  720 },
    { 1280, 1024 },
    { 1600, 1200 },
};

static void camera_sensor_stream_reset(camera_sensor_stream_t *stream)
{
    rt_memset(stream, 0, sizeof(*stream));
}

static rt_int32_t camera_sensor_timeout_ticks(uint32_t timeout_ms)
{
    if (timeout_ms == 0U)
    {
        return 0;
    }
    uint64_t ticks = ((uint64_t)timeout_ms * RT_TICK_PER_SECOND + 999U) / 1000U;
    return ticks > INT32_MAX ? INT32_MAX : (rt_int32_t)ticks;
}

static uint32_t camera_sensor_bytes_per_pixel(pixformat_t pixformat)
{
    switch (pixformat)
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

static int camera_sensor_bus_mode(pixformat_t pixformat,
                                  bus_capture_mode_t *mode)
{
    if (mode == RT_NULL)
    {
        return -RT_EINVAL;
    }

    switch (pixformat)
    {
        case PIXFORMAT_JPEG:   *mode = BUS_CAPTURE_MODE_JPEG;   return RT_EOK;
        case PIXFORMAT_RGB565: *mode = BUS_CAPTURE_MODE_RGB565; return RT_EOK;
        case PIXFORMAT_YUV422: *mode = BUS_CAPTURE_MODE_YUV422; return RT_EOK;
        case PIXFORMAT_RAW8:   *mode = BUS_CAPTURE_MODE_RAW;    return RT_EOK;
        default:                                                   return -RT_EINVAL;
    }
}

static int camera_sensor_stop_capture(camera_sensor_runtime_t *runtime)
{
    int ret;

    if (runtime->data_bus == RT_NULL)
    {
        return RT_EOK;
    }
    ret = bus_adapter_stop(runtime->data_bus);
    if (ret != BUS_OK)
    {
        return ret;
    }
    return bus_adapter_abort_capture(runtime->data_bus);
}

static void camera_sensor_frame_ready(void *buffer, uint32_t length, void *user)
{
    camera_sensor_runtime_t *runtime = (camera_sensor_runtime_t *)user;
    camera_sensor_stream_t *stream = &runtime->stream;
    rt_size_t frame_size = (buffer != RT_NULL) ? length : 0;
    camera_capture_done_callback_t async_callback = RT_NULL;
    void *async_context = RT_NULL;
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    camera_stream_frame_callback_t frame_callback = stream->frame_callback;
    void *callback_context = stream->callback_context;
    rt_hw_interrupt_enable(level);

    if (frame_callback != RT_NULL)
    {
        camera_stream_frame_t frame = {0};

        if (runtime->pixformat == PIXFORMAT_JPEG &&
            runtime->data_bus != RT_NULL &&
            runtime->data_bus->type == BUS_TYPE_DVP)
        {
            frame.buffer = buffer;
            frame.buffer_size = stream->buffer_size;
            frame.frame_size = frame_size;
            frame.sequence = ++stream->sequence;
            frame.buffer_index = 0;
            frame.is_complete = RT_FALSE;
            frame_callback(callback_context, &frame);
            return;
        }

        frame.buffer_index = stream->active_buffer_index;
        frame.buffer = stream->buffers[frame.buffer_index];
        frame.buffer_size = stream->buffer_size;
        frame.frame_size = frame_size;
        frame.sequence = ++stream->sequence;
        frame.is_complete = RT_TRUE;
        frame.timestamp_ticks = rt_tick_get();
        stream->active_buffer_index ^= 1U;

        if (bus_adapter_rearm_capture(runtime->data_bus,
                                      stream->buffers[stream->active_buffer_index],
                                      stream->buffer_size) != RT_EOK)
        {
            level = rt_hw_interrupt_disable();
            camera_sensor_stream_reset(stream);
            rt_hw_interrupt_enable(level);
        }
        frame_callback(callback_context, &frame);
        return;
    }

    if (runtime->pixformat == PIXFORMAT_JPEG)
    {
#if defined(CAMERA_DVP_BACKEND_DCMI)
        /* DCMI delivers one stopped, bounded JPEG frame in the caller's
         * buffer. No segment assembly or second image copy is needed. */
        const uint8_t *jpeg = buffer;
        frame_size = jpeg != RT_NULL && length >= 4U &&
                     jpeg[0] == 0xffU && jpeg[1] == 0xd8U &&
                     jpeg[length - 2U] == 0xffU && jpeg[length - 1U] == 0xd9U ? length : 0U;
#else
        camera_jpeg_result_t result = camera_jpeg_assembler_feed(
            &runtime->jpeg_single, (const uint8_t *)buffer, length);
        if (result == CAMERA_JPEG_INCOMPLETE)
        {
            return;
        }
        frame_size = (result == CAMERA_JPEG_COMPLETE) ?
                     runtime->jpeg_single.frame_size : 0;
#endif
    }

    /* Stop before waking the caller: it may immediately request another frame.
     * The backend may still be leaving its callback after this function returns;
     * camera_stop_capture must also succeed before the caller releases memory. */
    if (camera_sensor_stop_capture(runtime) != RT_EOK)
        frame_size = 0;
    level = rt_hw_interrupt_disable();
    runtime->last_frame_size = frame_size;
    async_callback = runtime->async_capture.callback;
    async_context = runtime->async_capture.context;
    runtime->async_capture.callback = RT_NULL;
    runtime->async_capture.context = RT_NULL;
    runtime->async_capture.in_flight = RT_FALSE;
    rt_hw_interrupt_enable(level);

    if (async_callback != RT_NULL)
    {
        async_callback(async_context,
                       frame_size > 0 ? CAMERA_OK : CAMERA_ERRORTIMEOUT,
                       frame_size);
        return;
    }

    rt_sem_release(&runtime->frame_sem);
}

static void camera_sensor_native_frame_ready(void *context,
                                             const bus_stream_frame_t *event)
{
    camera_sensor_runtime_t *runtime = (camera_sensor_runtime_t *)context;
    camera_stream_frame_t frame = {0};
    camera_stream_frame_callback_t callback;
    void *callback_context;
    rt_base_t level;

    if (runtime == RT_NULL || event == RT_NULL)
    {
        return;
    }

    level = rt_hw_interrupt_disable();
    if (!runtime->is_open || !runtime->stream.native_mode ||
        runtime->stream.frame_callback == RT_NULL)
    {
        rt_hw_interrupt_enable(level);
        return;
    }
    callback = runtime->stream.frame_callback;
    callback_context = runtime->stream.callback_context;
    frame.buffer = event->buffer;
    frame.buffer_size = runtime->stream.buffer_size;
    frame.frame_size = event->size;
    frame.sequence = event->sequence;
    frame.timestamp_ticks = event->timestamp_ticks;
    frame.error = event->error;
    if (event->event_type == BUS_STREAM_EVENT_HALF_FIRST)
        frame.event_type = CAMERA_STREAM_EVENT_HALF_FIRST;
    else if (event->event_type == BUS_STREAM_EVENT_HALF_SECOND)
        frame.event_type = CAMERA_STREAM_EVENT_HALF_SECOND;
    else
        frame.event_type = CAMERA_STREAM_EVENT_FRAME;
    frame.is_complete = event->error == BUS_OK &&
                        frame.event_type != CAMERA_STREAM_EVENT_HALF_FIRST;
    frame.buffer_index = UINT8_MAX;
    if (event->buffer == runtime->stream.buffers[0])
    {
        frame.buffer_index = 0;
    }
    else if (event->buffer == runtime->stream.buffers[1])
    {
        frame.buffer_index = 1;
    }
    rt_hw_interrupt_enable(level);

    if (frame.error == RT_EOK &&
        (frame.buffer_index == UINT8_MAX || frame.frame_size == 0 ||
         frame.frame_size > frame.buffer_size))
    {
        return;
    }
    callback(callback_context, &frame);
}

int camera_sensor_runtime_open(camera_sensor_runtime_t *runtime,
                               const camera_sensor_runtime_config_t *config)
{
    bus_adapter_config_t bus_config;
    int ret;

    if (runtime == RT_NULL || config == RT_NULL || config->adapter_name == RT_NULL ||
        config->default_config == RT_NULL)
    {
        return -RT_EINVAL;
    }
    if (runtime->is_open)
    {
        return RT_EOK;
    }
    if (camera_sensor_bus_mode(config->default_config->pixformat,
                               &bus_config.mode) != RT_EOK)
    {
        return -RT_EINVAL;
    }

    rt_memset(runtime, 0, sizeof(*runtime));
    runtime->pixformat = config->default_config->pixformat;
    runtime->framesize = config->default_config->framesize;
    runtime->quality = config->default_config->quality;
    runtime->frame_timeout_ms = config->frame_timeout_ms;

    ret = rt_sem_init(&runtime->frame_sem, "cam_frm", 0, RT_IPC_FLAG_FIFO);
    if (ret != RT_EOK)
    {
        return ret;
    }

    runtime->data_bus = bus_adapter_find(config->adapter_name);
    if (runtime->data_bus == RT_NULL || runtime->data_bus->type != config->adapter_type)
    {
        rt_sem_detach(&runtime->frame_sem);
        runtime->data_bus = RT_NULL;
        return -RT_ERROR;
    }

    ret = bus_adapter_config(runtime->data_bus, &bus_config);
    if (ret == BUS_OK)
    {
        ret = bus_adapter_set_frame_notify_callback(runtime->data_bus,
                                                    camera_sensor_frame_ready,
                                                    runtime);
    }
    if (ret == BUS_OK || ret == RT_EOK)
    {
        ret = bus_adapter_init(runtime->data_bus);
    }
    if (ret != RT_EOK && ret != BUS_OK)
    {
        rt_sem_detach(&runtime->frame_sem);
        runtime->data_bus = RT_NULL;
        return -RT_ERROR;
    }

    runtime->is_open = RT_TRUE;
    return RT_EOK;
}

int camera_sensor_runtime_close(camera_sensor_runtime_t *runtime)
{
    int ret;

    if (runtime == RT_NULL)
    {
        return -RT_EINVAL;
    }
    if (!runtime->is_open)
    {
        return RT_EOK;
    }

    ret = camera_sensor_stop_capture(runtime);
    if (ret != BUS_OK)
    {
        return ret;
    }
    ret = bus_adapter_deinit(runtime->data_bus);
    if (ret != BUS_OK)
    {
        return ret;
    }
    camera_sensor_stream_reset(&runtime->stream);
    runtime->data_bus = RT_NULL;
    rt_sem_detach(&runtime->frame_sem);
    runtime->is_open = RT_FALSE;
    return RT_EOK;
}

rt_bool_t camera_sensor_runtime_busy(const camera_sensor_runtime_t *runtime)
{
    return runtime != RT_NULL &&
           (runtime->async_capture.in_flight ||
            runtime->stream.frame_callback != RT_NULL);
}

int camera_sensor_runtime_set_pixformat(camera_sensor_runtime_t *runtime,
                                        pixformat_t pixformat)
{
    bus_capture_mode_t mode;
    int ret;

    if (runtime == RT_NULL || !runtime->is_open || camera_sensor_runtime_busy(runtime))
    {
        return -RT_EBUSY;
    }
    if (camera_sensor_bus_mode(pixformat, &mode) != RT_EOK)
    {
        return -RT_EINVAL;
    }

    ret = bus_adapter_stop(runtime->data_bus);
    if (ret != BUS_OK)
    {
        return ret;
    }
    ret = bus_adapter_set_mode(runtime->data_bus, mode);
    if (ret != BUS_OK)
    {
        return -RT_ERROR;
    }
    ret = bus_adapter_start(runtime->data_bus);
    if (ret != RT_EOK && ret != BUS_OK)
    {
        return -RT_ERROR;
    }
    runtime->pixformat = pixformat;
    return RT_EOK;
}

int camera_sensor_runtime_set_framesize(camera_sensor_runtime_t *runtime,
                                        framesize_t framesize)
{
    uint16_t width;
    uint16_t height;
    uint32_t bytes_per_pixel;
    int ret;

    if (runtime == RT_NULL || !runtime->is_open || camera_sensor_runtime_busy(runtime))
    {
        return -RT_EBUSY;
    }
    if (camera_sensor_get_resolution(framesize, &width, &height) != RT_EOK)
    {
        return -RT_EINVAL;
    }

    if (runtime->pixformat != PIXFORMAT_JPEG)
    {
        bytes_per_pixel = camera_sensor_bytes_per_pixel(runtime->pixformat);
        if (bytes_per_pixel == 0U)
        {
            return -RT_EINVAL;
        }
        ret = bus_adapter_set_pingpong_size(runtime->data_bus,
                                            (uint32_t)width * bytes_per_pixel * 2U);
        if (ret != RT_EOK && ret != BUS_OK)
        {
            return ret;
        }
    }

    runtime->framesize = framesize;
    return RT_EOK;
}

static rt_size_t camera_sensor_runtime_capture_blocking(camera_sensor_runtime_t *runtime,
                                                        void *buffer,
                                                        rt_size_t size,
                                                        uint32_t timeout_ms,
                                                        uint32_t frame_count)
{
    rt_err_t ret;
    rt_size_t frame_size;

    if (runtime == RT_NULL || !runtime->is_open || buffer == RT_NULL || size == 0 ||
        camera_sensor_runtime_busy(runtime))
    {
        return 0;
    }

    while (rt_sem_trytake(&runtime->frame_sem) == RT_EOK) {}
    runtime->last_frame_size = 0;
    if (runtime->pixformat == PIXFORMAT_JPEG)
    {
        camera_jpeg_assembler_reset(&runtime->jpeg_single, buffer, size);
    }

    ret = frame_count != 0U ?
          bus_adapter_start_capture_frames(runtime->data_bus, buffer, size, frame_count) :
          bus_adapter_start_capture(runtime->data_bus, buffer, size);
    if (ret != RT_EOK && ret != BUS_OK)
    {
        return 0;
    }

    ret = rt_sem_take(&runtime->frame_sem,
                      camera_sensor_timeout_ticks(timeout_ms));
    if (ret != RT_EOK)
    {
        LOG_W("capture timeout: pix=%d frame=%d req=%u",
              (int)runtime->pixformat,
              (int)runtime->framesize,
              (unsigned int)size);
        bus_adapter_dump_state(runtime->data_bus);
        camera_sensor_stop_capture(runtime);
        return 0;
    }

    rt_base_t level = rt_hw_interrupt_disable();
    frame_size = runtime->last_frame_size;
    rt_hw_interrupt_enable(level);
    return frame_size;
}

rt_size_t camera_sensor_runtime_capture_timeout(camera_sensor_runtime_t *runtime,
                                                void *buffer,
                                                rt_size_t size,
                                                uint32_t timeout_ms)
{
    return camera_sensor_runtime_capture_blocking(runtime, buffer, size, timeout_ms, 0U);
}

rt_size_t camera_sensor_runtime_capture_frames_timeout(camera_sensor_runtime_t *runtime,
                                                       void *buffer,
                                                       rt_size_t size,
                                                       uint32_t frame_count,
                                                       uint32_t timeout_ms)
{
    if (frame_count == 0U || runtime == RT_NULL || runtime->pixformat != PIXFORMAT_JPEG)
        return 0U;
    return camera_sensor_runtime_capture_blocking(runtime, buffer, size, timeout_ms, frame_count);
}

rt_size_t camera_sensor_runtime_capture(camera_sensor_runtime_t *runtime,
                                        void *buffer,
                                        rt_size_t size)
{
    return camera_sensor_runtime_capture_timeout(runtime, buffer, size,
                                                 runtime != RT_NULL ? runtime->frame_timeout_ms : 0U);
}

int camera_sensor_runtime_capture_async(camera_sensor_runtime_t *runtime,
                                        void *buffer,
                                        rt_size_t size,
                                        camera_capture_done_callback_t callback,
                                        void *context)
{
    rt_base_t level;
    int ret;

    if (runtime == RT_NULL || !runtime->is_open)
    {
        return -RT_ERROR;
    }
    if (buffer == RT_NULL || size == 0 || callback == RT_NULL)
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();
    if (runtime->stream.frame_callback != RT_NULL || runtime->async_capture.in_flight)
    {
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }
    runtime->async_capture.callback = callback;
    runtime->async_capture.context = context;
    runtime->async_capture.in_flight = RT_TRUE;
    rt_hw_interrupt_enable(level);

    if (runtime->pixformat == PIXFORMAT_JPEG)
    {
        camera_jpeg_assembler_reset(&runtime->jpeg_single, buffer, size);
    }

    ret = bus_adapter_start_capture(runtime->data_bus, buffer, size);
    if (ret != RT_EOK && ret != BUS_OK)
    {
        level = rt_hw_interrupt_disable();
        runtime->async_capture.callback = RT_NULL;
        runtime->async_capture.context = RT_NULL;
        runtime->async_capture.in_flight = RT_FALSE;
        rt_hw_interrupt_enable(level);
        return ret;
    }
    return RT_EOK;
}

int camera_sensor_runtime_start_stream(camera_sensor_runtime_t *runtime,
                                       const camera_stream_start_args_t *args)
{
    rt_base_t level;
    rt_bool_t native_mode;
    uint32_t expected_size = 0U;
    int ret;

    if (runtime == RT_NULL || !runtime->is_open || args == RT_NULL ||
        args->buffers[0] == RT_NULL ||
        args->buffer_size == 0 || args->buffer_size > UINT32_MAX ||
        args->frame_callback == RT_NULL ||
        (args->mode != CAMERA_STREAM_MODE_FRAME &&
         args->mode != CAMERA_STREAM_MODE_HALF_FRAME) ||
        (args->mode == CAMERA_STREAM_MODE_FRAME && args->buffers[1] == RT_NULL) ||
        (args->mode == CAMERA_STREAM_MODE_HALF_FRAME && args->buffers[1] != RT_NULL))
    {
        return -RT_EINVAL;
    }
    if (camera_sensor_runtime_busy(runtime))
    {
        return -RT_EBUSY;
    }

    native_mode = runtime->data_bus != RT_NULL &&
                  runtime->data_bus->ops != RT_NULL &&
                  runtime->data_bus->ops->start_stream != RT_NULL;
    if (args->mode == CAMERA_STREAM_MODE_HALF_FRAME && !native_mode)
        return -RT_ENOSYS;
    if (native_mode)
    {
        uint16_t width, height;
        uint32_t bytes_per_pixel = camera_sensor_bytes_per_pixel(runtime->pixformat);
        uint64_t bytes;

        if (bytes_per_pixel == 0U ||
            camera_sensor_get_resolution(runtime->framesize, &width, &height) != RT_EOK)
        {
            return -RT_EINVAL;
        }
        bytes = (uint64_t)width * height * bytes_per_pixel;
        if (bytes == 0U || bytes > UINT32_MAX || bytes > args->buffer_size ||
            (args->mode == CAMERA_STREAM_MODE_HALF_FRAME &&
             (bytes != args->buffer_size || (height & 1U) != 0U)))
        {
            return -RT_EINVAL;
        }
        expected_size = (uint32_t)bytes;
    }
    runtime->stream.native_mode = native_mode;
    runtime->stream.mode = args->mode;
    runtime->stream.buffers[0] = args->buffers[0];
    runtime->stream.buffers[1] = args->buffers[1];
    runtime->stream.buffer_size = args->buffer_size;
    runtime->stream.active_buffer_index = 0;
    runtime->stream.sequence = 0;
    level = rt_hw_interrupt_disable();
    runtime->stream.callback_context = args->callback_context;
    runtime->stream.frame_callback = args->frame_callback;
    rt_hw_interrupt_enable(level);

    if (runtime->stream.native_mode)
    {
        bus_stream_config_t config = {0};
        config.buffers[0] = runtime->stream.buffers[0];
        config.buffers[1] = runtime->stream.buffers[1];
        config.buffer_size = (uint32_t)runtime->stream.buffer_size;
        config.expected_frame_size = expected_size;
        config.mode = args->mode == CAMERA_STREAM_MODE_HALF_FRAME ?
                      BUS_STREAM_MODE_HALF_FRAME : BUS_STREAM_MODE_FRAME;
        config.frame_callback = camera_sensor_native_frame_ready;
        config.callback_context = runtime;
        ret = bus_adapter_start_stream(runtime->data_bus, &config);
    }
    else
    {
        ret = bus_adapter_start_capture(runtime->data_bus,
                                        runtime->stream.buffers[0],
                                        runtime->stream.buffer_size);
    }
    if (ret != RT_EOK && ret != BUS_OK)
    {
        if (camera_sensor_stop_capture(runtime) == BUS_OK)
        {
            level = rt_hw_interrupt_disable();
            camera_sensor_stream_reset(&runtime->stream);
            rt_hw_interrupt_enable(level);
        }
        return ret;
    }
    return RT_EOK;
}

int camera_sensor_runtime_stop_stream(camera_sensor_runtime_t *runtime)
{
    rt_base_t level;
    int ret;

    if (runtime == RT_NULL || !runtime->is_open)
    {
        return -RT_ERROR;
    }

    ret = camera_sensor_stop_capture(runtime);
    if (ret != BUS_OK)
    {
        return ret;
    }
    level = rt_hw_interrupt_disable();
    camera_sensor_stream_reset(&runtime->stream);
    rt_hw_interrupt_enable(level);
    return RT_EOK;
}

rt_bool_t camera_sensor_runtime_stream_frame_is_valid(
    camera_sensor_runtime_t *runtime, const camera_stream_frame_t *frame)
{
    bus_stream_frame_t bus_frame = {0};
    rt_bool_t valid = RT_FALSE;
    rt_base_t level;

    if (runtime == RT_NULL || frame == RT_NULL)
    {
        return RT_FALSE;
    }
    level = rt_hw_interrupt_disable();
    if (runtime->is_open && runtime->stream.native_mode &&
        runtime->stream.frame_callback != RT_NULL && frame->error == RT_EOK &&
        ((runtime->stream.mode == CAMERA_STREAM_MODE_FRAME &&
          frame->event_type == CAMERA_STREAM_EVENT_FRAME && frame->is_complete) ||
         (runtime->stream.mode == CAMERA_STREAM_MODE_HALF_FRAME &&
          (frame->event_type == CAMERA_STREAM_EVENT_HALF_FIRST ||
           frame->event_type == CAMERA_STREAM_EVENT_HALF_SECOND))) &&
        frame->buffer != RT_NULL &&
        frame->buffer_index < 2 &&
        frame->buffer == runtime->stream.buffers[frame->buffer_index] &&
        frame->buffer_size == runtime->stream.buffer_size &&
        frame->frame_size > 0 && frame->frame_size <= frame->buffer_size)
    {
        bus_frame.buffer = frame->buffer;
        bus_frame.size = (uint32_t)frame->frame_size;
        bus_frame.sequence = frame->sequence;
        bus_frame.error = frame->error;
        bus_frame.timestamp_ticks = frame->timestamp_ticks;
        if (frame->event_type == CAMERA_STREAM_EVENT_HALF_FIRST)
            bus_frame.event_type = BUS_STREAM_EVENT_HALF_FIRST;
        else if (frame->event_type == CAMERA_STREAM_EVENT_HALF_SECOND)
            bus_frame.event_type = BUS_STREAM_EVENT_HALF_SECOND;
        valid = bus_adapter_stream_frame_valid(runtime->data_bus, &bus_frame);
    }
    rt_hw_interrupt_enable(level);
    return valid;
}

int camera_sensor_get_resolution(framesize_t framesize,
                                 uint16_t *width,
                                 uint16_t *height)
{
    if (framesize >= FRAMESIZE_INVALID || width == RT_NULL || height == RT_NULL)
    {
        return -RT_EINVAL;
    }
    *width = s_resolutions[framesize].width;
    *height = s_resolutions[framesize].height;
    return RT_EOK;
}
