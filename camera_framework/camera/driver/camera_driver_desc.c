/******************************************************************************
 *
 * @file camera_driver_desc.c
 *
 * @brief Camera driver descriptor discovery.
 *
 *****************************************************************************/

#include "camera_driver_desc.h"
#include "rtthread.h"

const camera_device_ops_t *camera_driver_get_default_ops(void)
{
    const camera_driver_desc_t *table_begin = RT_NULL;
    const camera_driver_desc_t *table_end = RT_NULL;
    const camera_driver_desc_t *desc;

#if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
    extern const int CameraDriverDescTab$$Base;
    extern const int CameraDriverDescTab$$Limit;
    table_begin = (const camera_driver_desc_t *)&CameraDriverDescTab$$Base;
    table_end = (const camera_driver_desc_t *)&CameraDriverDescTab$$Limit;
#elif defined(__ICCARM__) || defined(__ICCRX__)
#error "Camera driver descriptor section scan is not implemented for IAR"
#elif defined(__GNUC__)
    extern const camera_driver_desc_t __start_CameraDriverDescTab[];
    extern const camera_driver_desc_t __stop_CameraDriverDescTab[];
    table_begin = __start_CameraDriverDescTab;
    table_end = __stop_CameraDriverDescTab;
#endif

    if (table_begin == RT_NULL || table_end == RT_NULL || table_begin == table_end)
    {
        return RT_NULL;
    }

    for (desc = table_begin; desc < table_end; desc++)
    {
        if (desc->ops != RT_NULL)
        {
            return desc->ops;
        }
    }

    return RT_NULL;
}
