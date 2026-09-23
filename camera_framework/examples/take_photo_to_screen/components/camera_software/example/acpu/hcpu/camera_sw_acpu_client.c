/* SPDX-License-Identifier: MIT */
#include <acpu_ctrl.h>
#include <rtthread.h>
#include <string.h>

#include "camera_sw_acpu_client.h"

static uint32_t camera_sw_acpu_elapsed_ms(rt_tick_t started)
{
    return (uint32_t)((rt_tick_get() - started) * 1000U / RT_TICK_PER_SECOND);
}

static int camera_sw_acpu_valid_isp_flags(uint32_t flags)
{
    return ((flags & ~CAMERA_SW_ACPU_ISP_FLAG_ALL) == 0U) &&
           (((flags & CAMERA_SW_ACPU_ISP_FLAG_AWB_APPLY) == 0U) ||
            ((flags & CAMERA_SW_ACPU_ISP_FLAG_AWB_STATS) != 0U));
}

int camera_sw_acpu_query_cpu_usage(camera_sw_acpu_usage_t *usage)
{
    camera_sw_acpu_cpu_usage_task_t task;
    uint8_t error_code = 0U;

    memset(&task, 0, sizeof(task));
    task.usage_x100 = -1;
    task.profiler_x100 = -1;
    task.status = -1;
    (void)acpu_run_task(CAMERA_SW_ACPU_TASK_CPU_USAGE, &task, sizeof(task),
                        &error_code);
    if ((error_code != 0U) || (task.status != 0) || (task.usage_x100 < 0) ||
        (task.window_ms == 0U) || (task.busy_ms > task.window_ms))
        return -1;
    if (usage != RT_NULL)
    {
        usage->usage_x100 = task.usage_x100;
        usage->profiler_x100 = task.profiler_x100;
        usage->profiler_fresh = task.profiler_fresh;
        usage->busy_ms = task.busy_ms;
        usage->window_ms = task.window_ms;
        usage->profiler_timebase_us = task.profiler_timebase_us;
    }
    return 0;
}

int camera_sw_acpu_encode_rgb565_be(const uint8_t *pixels, uint16_t width,
                                    uint16_t height, uint32_t pitch,
                                    uint8_t *jpeg, uint32_t jpeg_capacity,
                                    uint32_t *jpeg_size,
                                    uint32_t *elapsed_ms)
{
    camera_sw_acpu_jpeg_task_t task;
    uint8_t error_code = 0U;
    rt_tick_t started;

    if ((pixels == RT_NULL) || (width == 0U) || (height == 0U) ||
        (pitch < (uint32_t)width * 2U) || (jpeg == RT_NULL) ||
        (jpeg_size == RT_NULL) || (elapsed_ms == RT_NULL))
        return -1;

    task.input = pixels;
    task.output = jpeg;
    task.input_pitch = pitch;
    task.output_capacity = jpeg_capacity;
    task.output_size = 0U;
    task.width = width;
    task.height = height;
    task.quality = 1U;
    task.input_big_endian = 1U;
    task.status = -1;

    started = rt_tick_get();
    (void)acpu_run_task(CAMERA_SW_ACPU_TASK_JPEG, &task, sizeof(task),
                        &error_code);
    *elapsed_ms = camera_sw_acpu_elapsed_ms(started);
    if ((error_code != 0U) || (task.status != 0) ||
        (task.output_size < 4U) ||
        (task.output_size > jpeg_capacity) || (jpeg[0] != 0xFFU) ||
        (jpeg[1] != 0xD8U) || (jpeg[task.output_size - 2U] != 0xFFU) ||
        (jpeg[task.output_size - 1U] != 0xD9U))
        return -1;

    *jpeg_size = task.output_size;
    return 0;
}

int camera_sw_acpu_decode_jpeg(const uint8_t *jpeg, uint32_t jpeg_size,
                               uint8_t *destination,
                               uint32_t destination_size,
                               uint16_t *width, uint16_t *height,
                               uint32_t isp_flags,
                               camera_sw_acpu_metrics_t *metrics)
{
    camera_sw_acpu_jpeg_decode_task_t task;
    uint8_t error_code = 0U;
    rt_tick_t started;

    if ((jpeg == RT_NULL) || (jpeg_size < 4U) ||
        (destination == RT_NULL) || (destination_size == 0U) ||
        (width == RT_NULL) || (height == RT_NULL) || (metrics == RT_NULL) ||
        !camera_sw_acpu_valid_isp_flags(isp_flags))
        return -1;

    memset(&task, 0, sizeof(task));
    memset(metrics, 0, sizeof(*metrics));
    task.input = jpeg;
    task.output = destination;
    task.input_size = jpeg_size;
    task.output_capacity = destination_size;
    task.protocol_version = CAMERA_SW_ACPU_PROTOCOL_VERSION;
    task.isp_flags = isp_flags;
    task.status = -1;

    started = rt_tick_get();
    (void)acpu_run_task(CAMERA_SW_ACPU_TASK_JPEG_DECODE, &task, sizeof(task),
                        &error_code);
    metrics->total_ms = camera_sw_acpu_elapsed_ms(started);
    if ((error_code != 0U) || (task.status != 0) ||
        (task.width == 0U) || (task.height == 0U) ||
        ((uint32_t)task.width * task.height * 2U > destination_size) ||
        (task.protocol_version != CAMERA_SW_ACPU_PROTOCOL_VERSION) ||
        (task.isp_result.applied_flags != isp_flags))
        return -1;

    *width = task.width;
    *height = task.height;
    metrics->isp = task.isp_result;
    return 0;
}

int camera_sw_acpu_process_rgb565_be(uint8_t *pixels, uint16_t width,
                                     uint16_t height, uint32_t stride,
                                     uint32_t isp_flags,
                                     camera_sw_acpu_metrics_t *metrics)
{
    camera_sw_acpu_isp_task_t task;
    uint8_t error_code = 0U;
    rt_tick_t started;

    if ((pixels == RT_NULL) || (width == 0U) || (height == 0U) ||
        (stride < (uint32_t)width * 2U) || (stride > UINT32_MAX / height) ||
        (metrics == RT_NULL) || !camera_sw_acpu_valid_isp_flags(isp_flags))
        return -1;

    memset(&task, 0, sizeof(task));
    memset(metrics, 0, sizeof(*metrics));
    task.pixels = pixels;
    task.capacity = stride * height;
    task.stride = stride;
    task.width = width;
    task.height = height;
    task.protocol_version = CAMERA_SW_ACPU_PROTOCOL_VERSION;
    task.isp_flags = isp_flags;
    task.status = -1;

    started = rt_tick_get();
    (void)acpu_run_task(CAMERA_SW_ACPU_TASK_ISP, &task, sizeof(task),
                        &error_code);
    metrics->total_ms = camera_sw_acpu_elapsed_ms(started);
    if ((error_code != 0U) || (task.status != 0) ||
        (task.protocol_version != CAMERA_SW_ACPU_PROTOCOL_VERSION) ||
        (task.isp_result.applied_flags != isp_flags))
        return -1;

    metrics->isp = task.isp_result;
    return 0;
}
