/******************************************************************************
 *
 * @file camera_driver_desc.h
 *
 * @brief Static camera driver descriptor export helpers.
 *
 * Sensor drivers export one descriptor into a dedicated linker section so the
 * handle layer can discover compiled-in camera drivers without hard-coded
 * references to specific sensors.
 *
 *****************************************************************************/

#ifndef __CAMERA_DRIVER_DESC_H__
#define __CAMERA_DRIVER_DESC_H__

#include "../handle/camera_handle_internal.h"

typedef struct
{
    const char *name;
    const camera_device_ops_t *ops;
} camera_driver_desc_t;

const camera_device_ops_t *camera_driver_get_default_ops(void);

#define CAMERA_DRIVER_EXPORT(name, ops_ptr)                              \
    RT_USED static const camera_driver_desc_t __cameradriver_##name      \
    SECTION("CameraDriverDescTab") =                                     \
    {                                                                    \
        #name,                                                           \
        ops_ptr                                                          \
    }

#endif /* __CAMERA_DRIVER_DESC_H__ */
