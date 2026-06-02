#include <string.h>
#include <finsh.h>

#include "camera_test.h"
#include "camera_handle_internal.h"

typedef struct
{
    int open_calls;
    int close_calls;
    int capture_calls;
    int start_stream_calls;
    int stop_stream_calls;
    camera_stream_start_args_t last_stream_args;
} fake_camera_state_t;

static fake_camera_state_t s_fake_state;

static const pixformat_t s_fake_pixformats[] = { PIXFORMAT_JPEG, PIXFORMAT_RGB565 };
static const framesize_t s_fake_framesizes[] = { FRAMESIZE_QVGA, FRAMESIZE_VGA };
static const camera_capabilities_t s_fake_caps =
{
    .pixformats = s_fake_pixformats,
    .num_pixformats = sizeof(s_fake_pixformats) / sizeof(s_fake_pixformats[0]),
    .framesizes = s_fake_framesizes,
    .num_framesizes = sizeof(s_fake_framesizes) / sizeof(s_fake_framesizes[0]),
    .max_buffer_size = 4096,
};

static int fake_open(void)
{
    s_fake_state.open_calls++;
    return RT_EOK;
}

static int fake_close(void)
{
    s_fake_state.close_calls++;
    return RT_EOK;
}

static int fake_set_pixformat(pixformat_t pixformat)
{
    return pixformat == PIXFORMAT_INVALID ? -RT_EINVAL : RT_EOK;
}

static int fake_set_framesize(framesize_t framesize)
{
    return framesize == FRAMESIZE_INVALID ? -RT_EINVAL : RT_EOK;
}

static int fake_set_quality(uint8_t quality)
{
    return quality > 63 ? -RT_EINVAL : RT_EOK;
}

static rt_size_t fake_capture(void *buffer, rt_size_t buffer_size)
{
    s_fake_state.capture_calls++;
    if (buffer == RT_NULL || buffer_size == 0)
    {
        return 0;
    }

    ((rt_uint8_t *)buffer)[0] = 0x5A;
    return 1;
}

static int fake_capture_async(void *buffer,
                              rt_size_t buffer_size,
                              camera_capture_done_callback_t callback,
                              void *context)
{
    if (buffer == RT_NULL || buffer_size == 0 || callback == RT_NULL)
    {
        return -RT_EINVAL;
    }

    ((rt_uint8_t *)buffer)[0] = 0xA5;
    callback(context, CAMERA_OK, 1);
    return RT_EOK;
}

static int fake_start_stream(const camera_stream_start_args_t *args)
{
    s_fake_state.start_stream_calls++;
    s_fake_state.last_stream_args = *args;
    return RT_EOK;
}

static int fake_stop_stream(void)
{
    s_fake_state.stop_stream_calls++;
    memset(&s_fake_state.last_stream_args, 0, sizeof(s_fake_state.last_stream_args));
    return RT_EOK;
}

static const camera_device_ops_t s_fake_ops =
{
    .capabilities = &s_fake_caps,
    .open = fake_open,
    .close = fake_close,
    .set_pixformat = fake_set_pixformat,
    .set_framesize = fake_set_framesize,
    .set_quality = fake_set_quality,
    .capture = fake_capture,
    .capture_async = fake_capture_async,
    .start_stream = fake_start_stream,
    .stop_stream = fake_stop_stream,
};

static void fake_reset(void)
{
    memset(&s_fake_state, 0, sizeof(s_fake_state));
    camera_handle_set_test_ops(&s_fake_ops);
}

static void fake_emit_stream_frame(rt_uint32_t sequence, rt_uint8_t buffer_index)
{
    camera_stream_frame_t frame =
    {
        .buffer = s_fake_state.last_stream_args.buffers[buffer_index],
        .buffer_size = s_fake_state.last_stream_args.buffer_size,
        .frame_size = 64 + sequence,
        .sequence = sequence,
        .buffer_index = buffer_index,
    };

    CAMERA_TEST_ASSERT_NOT_NULL(s_fake_state.last_stream_args.frame_callback);
    s_fake_state.last_stream_args.frame_callback(s_fake_state.last_stream_args.callback_context,
                                                 &frame);
}

static void test_handle_init_and_deinit(void)
{
    camera_handler_instance_t *instance = RT_NULL;

    fake_reset();
    CAMERA_TEST_ASSERT_EQ(camera_handler_instance_init(&instance), CAMERA_OK);
    CAMERA_TEST_ASSERT_NOT_NULL(instance);
    CAMERA_TEST_ASSERT_EQ(s_fake_state.open_calls, 1);

    CAMERA_TEST_ASSERT_EQ(camera_deinit(&instance), CAMERA_OK);
    CAMERA_TEST_ASSERT_TRUE(instance == RT_NULL);
    CAMERA_TEST_ASSERT_EQ(s_fake_state.close_calls, 1);
}

static void test_capture_rejected_while_streaming(void)
{
    camera_handler_instance_t *instance = RT_NULL;
    rt_uint8_t stream_buffers[2][128];
    rt_uint8_t capture_buffer[32];
    camera_stream_config_t stream_config =
    {
        .buffers = { stream_buffers[0], stream_buffers[1] },
        .buffer_size = sizeof(stream_buffers[0]),
    };
    camera_capture_request_t capture_request =
    {
        .buffer = capture_buffer,
        .buffer_size = sizeof(capture_buffer),
        .frame_size = 0,
    };

    fake_reset();
    CAMERA_TEST_ASSERT_EQ(camera_handler_instance_init(&instance), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(camera_start_stream(instance, &stream_config), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(camera_capture_single(instance, &capture_request), CAMERA_ERRORRESOURCE);
    CAMERA_TEST_ASSERT_EQ(s_fake_state.capture_calls, 0);
    CAMERA_TEST_ASSERT_EQ(camera_stop_stream(instance), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(camera_deinit(&instance), CAMERA_OK);
}

static void test_stream_frame_queue_and_drop_count(void)
{
    camera_handler_instance_t *instance = RT_NULL;
    rt_uint8_t stream_buffers[2][128];
    camera_stream_config_t stream_config =
    {
        .buffers = { stream_buffers[0], stream_buffers[1] },
        .buffer_size = sizeof(stream_buffers[0]),
    };
    camera_stream_frame_t frame;
    rt_uint32_t dropped_count = 0;

    fake_reset();
    CAMERA_TEST_ASSERT_EQ(camera_handler_instance_init(&instance), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(camera_start_stream(instance, &stream_config), CAMERA_OK);

    fake_emit_stream_frame(1, 0);
    CAMERA_TEST_ASSERT_EQ(camera_get_stream_frame(instance, &frame, 0), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(frame.sequence, 1);
    CAMERA_TEST_ASSERT_EQ(frame.buffer_index, 0);
    CAMERA_TEST_ASSERT_EQ(camera_get_stream_dropped_count(instance, &dropped_count), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(dropped_count, 0);

    fake_emit_stream_frame(2, 0);
    fake_emit_stream_frame(3, 1);
    fake_emit_stream_frame(4, 0);
    CAMERA_TEST_ASSERT_EQ(camera_get_stream_dropped_count(instance, &dropped_count), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(dropped_count, 1);

    CAMERA_TEST_ASSERT_EQ(camera_stop_stream(instance), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(camera_get_stream_dropped_count(instance, &dropped_count), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(dropped_count, 0);
    CAMERA_TEST_ASSERT_EQ(camera_deinit(&instance), CAMERA_OK);
}

static void test_stop_stream_is_idempotent(void)
{
    camera_handler_instance_t *instance = RT_NULL;
    rt_uint8_t stream_buffers[2][128];
    camera_stream_config_t stream_config =
    {
        .buffers = { stream_buffers[0], stream_buffers[1] },
        .buffer_size = sizeof(stream_buffers[0]),
    };

    fake_reset();
    CAMERA_TEST_ASSERT_EQ(camera_handler_instance_init(&instance), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(camera_start_stream(instance, &stream_config), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(camera_stop_stream(instance), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(camera_stop_stream(instance), CAMERA_OK);
    CAMERA_TEST_ASSERT_EQ(s_fake_state.stop_stream_calls, 1);
    CAMERA_TEST_ASSERT_EQ(camera_deinit(&instance), CAMERA_OK);
}

static int run_camera_handle_tests(void)
{
    static const camera_test_case_t cases[] =
    {
        { "test_handle_init_and_deinit", test_handle_init_and_deinit },
        { "test_capture_rejected_while_streaming", test_capture_rejected_while_streaming },
        { "test_stream_frame_queue_and_drop_count", test_stream_frame_queue_and_drop_count },
        { "test_stop_stream_is_idempotent", test_stop_stream_is_idempotent },
    };

    return camera_test_run_suite("camera_handle", cases, sizeof(cases) / sizeof(cases[0]));
}
MSH_CMD_EXPORT(run_camera_handle_tests, run camera handle tests);

int main(void)
{
    rt_kprintf("camera handle test app\n");
    run_camera_handle_tests();

    while (1)
    {
        rt_thread_mdelay(1000);
    }
}
