#ifndef CAMERA_IMAGE_PROCESSOR_H
#define CAMERA_IMAGE_PROCESSOR_H

#include <stdint.h>

#include "camera_sw_acpu_client.h"

int camera_image_process_rgb565_be(uint8_t *pixels, uint16_t width,
                                   uint16_t height, uint32_t stride,
                                   uint32_t isp_flags,
                                   camera_sw_acpu_metrics_t *metrics);
int camera_image_encode_rgb565_be(uint8_t *pixels, uint16_t width,
                                  uint16_t height, uint32_t pitch,
                                  const uint8_t **jpeg, uint32_t *jpeg_size,
                                  uint32_t *elapsed_ms);
int camera_image_decode_jpeg(const uint8_t *jpeg, uint32_t jpeg_size,
                             uint8_t *destination,
                             uint32_t destination_size,
                             uint16_t *width, uint16_t *height,
                             uint32_t isp_flags,
                             camera_sw_acpu_metrics_t *metrics);
/* Query and reset the ACPU counters. @p usage may be RT_NULL to only reset. */
int camera_image_query_acpu_usage(camera_sw_acpu_usage_t *usage);

#endif
