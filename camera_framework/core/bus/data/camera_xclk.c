#include "camera_xclk.h"

#include "rtthread.h"

#define DBG_TAG "camera.xclk"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/*
 * The sensor master clock is a board fact, not a framework one:
 *
 *  - bsp_pinmux.c muxes the MCLK pad onto a timer channel;
 *  - the board's Kconfig.board hardcodes CAMERA_XCLK_ENABLE and the PWM
 *    device/channel that drive that pad (CAMERA_XCLK_PWM_DEVICE_NAME /
 *    CAMERA_XCLK_PWM_CHANNEL), and may define CAMERA_XCLK_EXTERNAL_OSC when
 *    the module carries its own oscillator.
 *
 * The framework therefore never looks for a pad, never guesses a channel and
 * never carries a device name of its own: it programs the PWM the board named
 * and nothing else.
 *
 * CAMERA_XCLK_ENABLE is owned by the board, CAMERA_XCLK_REQUIRED is derived
 * by the framework (camera_framework/Kconfig) from the selected sensor, so the
 * dependency only ever runs board -> framework:
 *
 *  - CAMERA_XCLK_ENABLE: the board says it feeds MCLK;
 *  - CAMERA_XCLK_REQUIRED:  the compiled-in sensor cannot work without a clock
 *    (an OV2640 behind an ArduCam FIFO module is the exception).
 *
 * "Needs a clock" must not be expressed as "the board declared one": a board
 * that simply forgot the three symbols would otherwise compile a firmware whose
 * sensor never answers SCCB. A sensor that needs MCLK without a board that
 * supplies it is a build error, not a runtime surprise.
 */
#if defined(CAMERA_XCLK_REQUIRED) && !defined(CAMERA_XCLK_EXTERNAL_OSC) && \
    !defined(CAMERA_XCLK_ENABLE)
#error "this sensor needs an MCU camera MCLK: define CAMERA_XCLK_ENABLE, CAMERA_XCLK_PWM_DEVICE_NAME and CAMERA_XCLK_PWM_CHANNEL in the board's Kconfig.board, or define CAMERA_XCLK_EXTERNAL_OSC for a module with its own oscillator"
#endif

#if defined(CAMERA_XCLK_ENABLE)

/* FIXED_GPTBTIM_SRC_CLK (via register.h) bounds the fastest usable MCLK. */
#include "bf0_hal.h"
#include "rtdevice.h"

#if !defined(CAMERA_XCLK_PWM_DEVICE_NAME) || !defined(CAMERA_XCLK_PWM_CHANNEL)
#error "the board's Kconfig.board must define CAMERA_XCLK_PWM_DEVICE_NAME and CAMERA_XCLK_PWM_CHANNEL for the camera MCLK"
#endif

static struct rt_device_pwm *camera_xclk_device(void)
{
    struct rt_device_pwm *device =
        (struct rt_device_pwm *)rt_device_find(CAMERA_XCLK_PWM_DEVICE_NAME);

    if (device == RT_NULL)
        LOG_E("find %s device failed", CAMERA_XCLK_PWM_DEVICE_NAME);
    return device;
}

/* rt_pwm_set() takes nanoseconds; round up so the driven clock is never faster
 * than the sensor asked for. */
static uint32_t camera_xclk_period_ns(uint32_t frequency_hz)
{
    return (uint32_t)(2ULL * ((500000000ULL + frequency_hz - 1U) / frequency_hz));
}

int camera_xclk_start(uint32_t frequency_hz)
{
    struct rt_device_pwm *device;
    uint32_t period_ns;
    rt_err_t ret;

    if (frequency_hz == 0U)
        return CAMERA_XCLK_INVALID;
#if defined(FIXED_GPTBTIM_SRC_CLK)
    /* The pad can only be muxed to a channel of the fixed timer clock. */
    if (frequency_hz > FIXED_GPTBTIM_SRC_CLK / 2U)
        return CAMERA_XCLK_INVALID;
#endif

    device = camera_xclk_device();
    if (device == RT_NULL)
        return CAMERA_XCLK_HW;

    period_ns = camera_xclk_period_ns(frequency_hz);
    ret = rt_pwm_set(device, CAMERA_XCLK_PWM_CHANNEL, period_ns, period_ns / 2U);
    if (ret != RT_EOK)
    {
        LOG_E("XCLK PWM set failed: %d", ret);
        return CAMERA_XCLK_HW;
    }

    ret = rt_pwm_enable(device, CAMERA_XCLK_PWM_CHANNEL);
    if (ret != RT_EOK)
    {
        LOG_E("XCLK PWM enable failed: %d", ret);
        return CAMERA_XCLK_HW;
    }

    LOG_I("requested=%u Hz period=%u ns on %s ch%d",
          (unsigned int)frequency_hz, (unsigned int)period_ns,
          CAMERA_XCLK_PWM_DEVICE_NAME, (int)CAMERA_XCLK_PWM_CHANNEL);
    return CAMERA_XCLK_OK;
}

int camera_xclk_stop(void)
{
    struct rt_device_pwm *device;
    rt_err_t ret;

    device = camera_xclk_device();
    if (device == RT_NULL)
        return CAMERA_XCLK_HW;

    ret = rt_pwm_disable(device, CAMERA_XCLK_PWM_CHANNEL);
    if (ret != RT_EOK)
    {
        LOG_E("XCLK PWM disable failed: %d", ret);
        return CAMERA_XCLK_HW;
    }

    LOG_I("stopped (%s ch%d)", CAMERA_XCLK_PWM_DEVICE_NAME,
          (int)CAMERA_XCLK_PWM_CHANNEL);
    return CAMERA_XCLK_OK;
}

#else /* CAMERA_XCLK_ENABLE */

/* No MCU clock is driven. That is legitimate only when the sensor does not need
 * one (checked above: CAMERA_XCLK_REQUIRED with CAMERA_XCLK_EXTERNAL_OSC). */
int camera_xclk_start(uint32_t frequency_hz)
{
    LOG_W("board declares no MCU camera MCLK; requested %u Hz is not driven "
          "(module uses its own oscillator)", (unsigned int)frequency_hz);
    return CAMERA_XCLK_OK;
}

int camera_xclk_stop(void)
{
    return CAMERA_XCLK_OK;
}

#endif /* CAMERA_XCLK_ENABLE */
