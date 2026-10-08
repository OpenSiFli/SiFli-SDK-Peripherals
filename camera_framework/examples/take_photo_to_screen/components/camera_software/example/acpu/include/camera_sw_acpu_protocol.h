/* SPDX-License-Identifier: MIT */
#ifndef CAMERA_SW_ACPU_PROTOCOL_H
#define CAMERA_SW_ACPU_PROTOCOL_H

#include <stdint.h>

#include "camera_sw_isp_pipeline.h"

#define CAMERA_SW_ACPU_TASK_JPEG 2U
#define CAMERA_SW_ACPU_TASK_JPEG_DECODE 3U
#define CAMERA_SW_ACPU_TASK_CPU_USAGE 4U
#define CAMERA_SW_ACPU_TASK_ISP 5U

#define CAMERA_SW_ACPU_PROTOCOL_VERSION 6U

#define CAMERA_SW_ACPU_ISP_FLAG_AWB_STATS CAMERA_SW_ISP_FLAG_AWB_STATS
#define CAMERA_SW_ACPU_ISP_FLAG_AWB_APPLY CAMERA_SW_ISP_FLAG_AWB_APPLY
#define CAMERA_SW_ACPU_ISP_FLAG_CCM CAMERA_SW_ISP_FLAG_CCM
#define CAMERA_SW_ACPU_ISP_FLAG_GAMMA CAMERA_SW_ISP_FLAG_GAMMA
#define CAMERA_SW_ACPU_ISP_FLAG_FILTER CAMERA_SW_ISP_FLAG_FILTER
#define CAMERA_SW_ACPU_ISP_FLAG_TONE CAMERA_SW_ISP_FLAG_TONE
#define CAMERA_SW_ACPU_ISP_FLAG_VIVID CAMERA_SW_ISP_FLAG_VIVID
#define CAMERA_SW_ACPU_ISP_FLAG_DITHER CAMERA_SW_ISP_FLAG_DITHER
#define CAMERA_SW_ACPU_ISP_FLAG_ALL CAMERA_SW_ISP_FLAG_ALL

typedef camera_sw_isp_stats_t camera_sw_acpu_isp_stats_t;
typedef camera_sw_isp_result_t camera_sw_acpu_isp_result_t;

typedef struct
{
    const uint8_t *input;
    uint8_t *output;
    uint32_t input_pitch;
    uint32_t output_capacity;
    uint32_t output_size;
    uint16_t width;
    uint16_t height;
    uint8_t quality;
    uint8_t input_big_endian;
    int32_t status;
} camera_sw_acpu_jpeg_task_t;

typedef struct
{
    const uint8_t *input;
    uint8_t *output;
    uint32_t input_size;
    uint32_t output_capacity;
    uint16_t width;
    uint16_t height;
    uint32_t protocol_version;
    uint32_t isp_flags;
    camera_sw_acpu_isp_result_t isp_result;
    int32_t status;
} camera_sw_acpu_jpeg_decode_task_t;

typedef struct
{
    uint8_t *pixels;
    uint32_t capacity;
    uint32_t stride;
    uint16_t width;
    uint16_t height;
    uint32_t protocol_version;
    uint32_t isp_flags;
    camera_sw_acpu_isp_result_t isp_result;
    int32_t status;
} camera_sw_acpu_isp_task_t;

typedef struct
{
    int32_t usage_x100;
    int32_t status;
    uint32_t busy_ms;
    uint32_t window_ms;
    int32_t profiler_x100;
    /* 1 when profiler_x100 comes from the settle this query just issued, 0 when
     * it is the value of the profiler's own previous settle. */
    int32_t profiler_fresh;
    uint32_t profiler_timebase_us;
} camera_sw_acpu_cpu_usage_task_t;

#endif
