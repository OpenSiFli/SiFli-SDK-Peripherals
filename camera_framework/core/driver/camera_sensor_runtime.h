#ifndef CAMERA_SENSOR_RUNTIME_H
#define CAMERA_SENSOR_RUNTIME_H

#include "camera_handle_internal.h"
#include "camera_jpeg_assembler.h"
#include "data_bus_adapter.h"

typedef struct
{
    uint8_t *buffers[2];
    rt_size_t buffer_size;
    rt_uint8_t active_buffer_index;
    rt_uint32_t sequence;
    rt_bool_t native_mode;
    camera_stream_mode_t mode;
    camera_stream_frame_callback_t frame_callback;
    void *callback_context;
} camera_sensor_stream_t;

typedef struct
{
    camera_capture_done_callback_t callback;
    void *context;
    rt_bool_t in_flight;
} camera_sensor_async_capture_t;

typedef struct
{
    const char *adapter_name;
    bus_type_t adapter_type;
    uint32_t frame_timeout_ms;
    const camera_capture_config_t *default_config;
} camera_sensor_runtime_config_t;

typedef struct
{
    framesize_t framesize;
    uint8_t quality;
    pixformat_t pixformat;
    volatile rt_size_t last_frame_size;
    struct rt_semaphore frame_sem;
    camera_sensor_stream_t stream;
    camera_sensor_async_capture_t async_capture;
    camera_jpeg_assembler_t jpeg_single;
    bus_adapter_t *data_bus;
    uint32_t frame_timeout_ms;
    rt_bool_t is_open;
} camera_sensor_runtime_t;

int camera_sensor_runtime_open(camera_sensor_runtime_t *runtime,
                               const camera_sensor_runtime_config_t *config);
int camera_sensor_runtime_close(camera_sensor_runtime_t *runtime);
rt_bool_t camera_sensor_runtime_busy(const camera_sensor_runtime_t *runtime);
int camera_sensor_runtime_set_pixformat(camera_sensor_runtime_t *runtime,
                                        pixformat_t pixformat);
int camera_sensor_runtime_set_framesize(camera_sensor_runtime_t *runtime,
                                        framesize_t framesize);
rt_size_t camera_sensor_runtime_capture(camera_sensor_runtime_t *runtime,
                                        void *buffer,
                                        rt_size_t size);
rt_size_t camera_sensor_runtime_capture_timeout(camera_sensor_runtime_t *runtime,
                                                void *buffer,
                                                rt_size_t size,
                                                uint32_t timeout_ms);
rt_size_t camera_sensor_runtime_capture_frames_timeout(camera_sensor_runtime_t *runtime,
                                                       void *buffer,
                                                       rt_size_t size,
                                                       uint32_t frame_count,
                                                       uint32_t timeout_ms);
int camera_sensor_runtime_capture_async(camera_sensor_runtime_t *runtime,
                                        void *buffer,
                                        rt_size_t size,
                                        camera_capture_done_callback_t callback,
                                        void *context);
int camera_sensor_runtime_start_stream(camera_sensor_runtime_t *runtime,
                                       const camera_stream_start_args_t *args);
int camera_sensor_runtime_stop_stream(camera_sensor_runtime_t *runtime);
rt_bool_t camera_sensor_runtime_stream_frame_is_valid(
    camera_sensor_runtime_t *runtime, const camera_stream_frame_t *frame);
int camera_sensor_get_resolution(framesize_t framesize,
                                 uint16_t *width,
                                 uint16_t *height);

#endif /* CAMERA_SENSOR_RUNTIME_H */
