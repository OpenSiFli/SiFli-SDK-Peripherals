#ifndef CAMERA_ISP_FLAGS_H
#define CAMERA_ISP_FLAGS_H

#include <stdint.h>

/*
 * Software ISP flag set resolved from menuconfig.
 *
 * The choice "ISP pipeline" is persisted to project/proj.conf, which is
 * re-applied to the build directory on every build, so the generated rtconfig.h
 * decides here whether the software ISP runs at all. When it is selected, every
 * path that can apply the pipeline uses this same flag set, on the single
 * execution core chosen in menuconfig.
 */
uint32_t camera_isp_flags(void);

const char *camera_isp_mode_name(void);

#endif
