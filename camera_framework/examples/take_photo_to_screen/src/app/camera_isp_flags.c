#include "camera_isp_flags.h"

#include <rtconfig.h>

#ifdef CAMERA_ISP_SOFTWARE
#include "camera_sw_isp_pipeline.h"
#endif

uint32_t camera_isp_flags(void)
{
#if !defined(CAMERA_ISP_SOFTWARE)
    return 0U;
#else
    uint32_t flags = 0U;

#if defined(CAMERA_SW_ISP_VIVID)
    flags = CAMERA_SW_ISP_FLAG_AWB_STATS | CAMERA_SW_ISP_FLAG_AWB_APPLY |
            CAMERA_SW_ISP_FLAG_GAMMA | CAMERA_SW_ISP_FLAG_TONE |
            CAMERA_SW_ISP_FLAG_VIVID;
#if defined(CAMERA_SW_ISP_DITHER)
    flags |= CAMERA_SW_ISP_FLAG_DITHER;
#endif

#else
#if defined(CAMERA_SW_ISP_AWB)
    flags |= CAMERA_SW_ISP_FLAG_AWB_STATS | CAMERA_SW_ISP_FLAG_AWB_APPLY;
#endif
#if defined(CAMERA_SW_ISP_GAMMA)
    flags |= CAMERA_SW_ISP_FLAG_GAMMA;
#endif
#if defined(CAMERA_SW_ISP_TONE)
    flags |= CAMERA_SW_ISP_FLAG_AWB_STATS | CAMERA_SW_ISP_FLAG_TONE;
#endif
#if defined(CAMERA_SW_ISP_DITHER)
    if ((flags & CAMERA_SW_ISP_FLAG_GAMMA) != 0U)
        flags |= CAMERA_SW_ISP_FLAG_DITHER;
#endif
#endif

    return flags;
#endif
}

const char *camera_isp_mode_name(void)
{
#if defined(CAMERA_ISP_SOFTWARE) && defined(CAMERA_SW_ISP_RUN_ON_ACPU)
    return "software-acpu";
#elif defined(CAMERA_ISP_SOFTWARE)
    return "software-hcpu";
#else
    return "sensor-hardware";
#endif
}
