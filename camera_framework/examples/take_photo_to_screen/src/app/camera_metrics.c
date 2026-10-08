#include "camera_metrics.h"

uint32_t camera_metrics_average_ms_x10(uint32_t ticks,
                                       uint32_t samples,
                                       uint32_t tick_rate)
{
    if ((samples == 0U) || (tick_rate == 0U))
    {
        return 0U;
    }

    return (uint32_t)(((uint64_t)ticks * 10000U) /
                      ((uint64_t)samples * tick_rate));
}

uint32_t camera_metrics_percent_x100(uint64_t busy_ms, uint64_t total_ms)
{
    uint64_t value;

    if (total_ms == 0U)
        return 0U;
    if (busy_ms > UINT64_MAX / 10000U)
        return UINT32_MAX;
    value = (busy_ms * 10000U) / total_ms;
    return value > UINT32_MAX ? UINT32_MAX : (uint32_t)value;
}

uint32_t camera_metrics_awb_minimum_candidates(uint32_t pixel_count)
{
    uint32_t minimum = (pixel_count + 99U) / 100U;

    return minimum == 0U ? 1U : minimum;
}

int camera_metrics_awb_should_bypass(uint32_t valid_pixels,
                                     uint32_t pixel_count)
{
    return valid_pixels < camera_metrics_awb_minimum_candidates(pixel_count);
}
