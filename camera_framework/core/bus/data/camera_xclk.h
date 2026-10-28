#ifndef CAMERA_XCLK_H_
#define CAMERA_XCLK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_XCLK_OK       0
#define CAMERA_XCLK_INVALID -1
#define CAMERA_XCLK_RANGE   -2
#define CAMERA_XCLK_HW      -3

int camera_xclk_calculate_period(uint32_t timer_hz,
                                 uint32_t output_hz,
                                 uint32_t *period);
int camera_xclk_start(uint32_t frequency_hz);
int camera_xclk_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_XCLK_H_ */
