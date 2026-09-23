/* SPDX-License-Identifier: MIT */
#ifndef CAMERA_SW_ACPU_CLIENT_H
#define CAMERA_SW_ACPU_CLIENT_H

#include <stdint.h>

#include "camera_sw_acpu_protocol.h"

#define CAMERA_SW_ACPU_ISP_FLAGS_COLOR \
    (CAMERA_SW_ACPU_ISP_FLAG_AWB_STATS | CAMERA_SW_ACPU_ISP_FLAG_CCM | \
     CAMERA_SW_ACPU_ISP_FLAG_GAMMA)
#define CAMERA_SW_ACPU_ISP_FLAGS_FULL \
    (CAMERA_SW_ACPU_ISP_FLAGS_COLOR | CAMERA_SW_ACPU_ISP_FLAG_FILTER)
#define CAMERA_SW_ACPU_ISP_FLAGS_AWB \
    (CAMERA_SW_ACPU_ISP_FLAG_AWB_STATS | CAMERA_SW_ACPU_ISP_FLAG_AWB_APPLY)
#define CAMERA_SW_ACPU_ISP_FLAGS_TONE \
    (CAMERA_SW_ACPU_ISP_FLAG_AWB_STATS | CAMERA_SW_ACPU_ISP_FLAG_TONE)
#define CAMERA_SW_ACPU_ISP_FLAGS_VIVID CAMERA_SW_ACPU_ISP_FLAG_VIVID

typedef struct
{
    uint32_t total_ms;
    uint32_t epic_lcd_ms;
    uint32_t end_to_end_ms;
    camera_sw_acpu_isp_result_t isp;
} camera_sw_acpu_metrics_t;

typedef struct
{
    int32_t usage_x100;
    int32_t profiler_x100;
    int32_t profiler_fresh;
    uint32_t busy_ms;
    uint32_t window_ms;
    uint32_t profiler_timebase_us;
} camera_sw_acpu_usage_t;

int camera_sw_acpu_encode_rgb565_be(const uint8_t *pixels, uint16_t width,
                                    uint16_t height, uint32_t pitch,
                                    uint8_t *jpeg, uint32_t jpeg_capacity,
                                    uint32_t *jpeg_size,
                                    uint32_t *elapsed_ms);
int camera_sw_acpu_decode_jpeg(const uint8_t *jpeg, uint32_t jpeg_size,
                               uint8_t *destination,
                               uint32_t destination_size,
                               uint16_t *width, uint16_t *height,
                               uint32_t isp_flags,
                               camera_sw_acpu_metrics_t *metrics);
int camera_sw_acpu_process_rgb565_be(uint8_t *pixels, uint16_t width,
                                     uint16_t height, uint32_t stride,
                                     uint32_t isp_flags,
                                     camera_sw_acpu_metrics_t *metrics);
int camera_sw_acpu_query_cpu_usage(camera_sw_acpu_usage_t *usage);

#endif
