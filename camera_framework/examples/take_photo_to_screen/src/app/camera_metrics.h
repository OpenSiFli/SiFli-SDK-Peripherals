#ifndef CAMERA_METRICS_H
#define CAMERA_METRICS_H

#include <stdint.h>

uint32_t camera_metrics_average_ms_x10(uint32_t ticks,
                                       uint32_t samples,
                                       uint32_t tick_rate);
uint32_t camera_metrics_percent_x100(uint64_t busy_ms, uint64_t total_ms);
uint32_t camera_metrics_awb_minimum_candidates(uint32_t pixel_count);
int camera_metrics_awb_should_bypass(uint32_t valid_pixels,
                                     uint32_t pixel_count);

#endif
