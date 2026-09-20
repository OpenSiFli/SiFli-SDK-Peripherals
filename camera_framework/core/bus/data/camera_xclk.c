#include "camera_xclk.h"

#include "bf0_hal.h"
#include "drv_io.h"
#include "rtthread.h"

#define DBG_TAG "camera.xclk"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#if defined(SOC_SF32LB57X) && SOC_SF32LB57X == 1 && \
    defined(BSP_USING_PWMT2) && defined(BOARD_DVP_PINMAP_0)

#include "rtdevice.h"

int camera_xclk_start(int pin, uint32_t frequency_hz)
{
    struct rt_device_pwm *device;
    const int pwm_channel = 1;
    const uint32_t drive_mask = HPSYS_PINMUX_PAD_PA02_DS0 | HPSYS_PINMUX_PAD_PA02_DS1;
    uint32_t pad_before;
    uint32_t pad_after;
    uint32_t period_ns;
    uint32_t calculated_hz;
    rt_err_t ret;

    if (pin != 2 || frequency_hz == 0U ||
        frequency_hz > FIXED_GPTBTIM_SRC_CLK / 2U)
    {
        return CAMERA_XCLK_INVALID;
    }

    device = (struct rt_device_pwm *)rt_device_find("pwmt2");
    if (device == RT_NULL)
    {
        LOG_E("find pwmt2 device failed");
        return CAMERA_XCLK_HW;
    }

    period_ns = (uint32_t)(2ULL * ((500000000ULL + frequency_hz - 1U) /
                                  frequency_hz));
    ret = rt_pwm_set(device, pwm_channel, period_ns, period_ns / 2U);
    if (ret != RT_EOK)
    {
        LOG_E("XCLK PWM set failed: %d", ret);
        return CAMERA_XCLK_HW;
    }

    HAL_PIN_Set(PAD_PA02, GPTIM2_CH1, PIN_PULLUP, 1);

    /* DS0=DS1=1 selects the strongest PA02 output drive for MCLK. */
    pad_before = hwp_pinmux1->PAD_PA02;
    ret = HAL_PIN_Update(PAD_PA02, drive_mask, drive_mask, 1);
    pad_after = hwp_pinmux1->PAD_PA02;
    if (ret != 0 || (pad_after & drive_mask) != drive_mask)
    {
        LOG_E("PA02 drive configuration failed: rc=%d pad=%08x",
              ret, (unsigned int)pad_after);
        return CAMERA_XCLK_HW;
    }
    LOG_I("PA02 drive ds0=%u ds1=%u: pad=%08x -> %08x",
          (unsigned int)((pad_after & HPSYS_PINMUX_PAD_PA02_DS0) != 0U),
          (unsigned int)((pad_after & HPSYS_PINMUX_PAD_PA02_DS1) != 0U),
          (unsigned int)pad_before, (unsigned int)pad_after);

    ret = rt_pwm_enable(device, pwm_channel);
    if (ret != RT_EOK)
    {
        LOG_E("XCLK PWM enable failed: %d", ret);
        return CAMERA_XCLK_HW;
    }

    calculated_hz = (uint32_t)((uint64_t)FIXED_GPTBTIM_SRC_CLK /
                              ((uint64_t)hwp_gptim2->PSC + 1U) /
                              ((uint64_t)hwp_gptim2->ARR + 1U));
    LOG_I("requested=%u Hz calc=%u Hz on PA%d (pwmt2 ch1)",
          (unsigned int)frequency_hz, (unsigned int)calculated_hz, pin);
    LOG_I("period=%u ns pulse=%u ns timer=%u Hz psc=%u arr=%u ccr=%u",
          (unsigned int)period_ns, (unsigned int)(period_ns / 2U),
          (unsigned int)FIXED_GPTBTIM_SRC_CLK,
          (unsigned int)hwp_gptim2->PSC,
          (unsigned int)hwp_gptim2->ARR,
          (unsigned int)hwp_gptim2->CCR1);
    return CAMERA_XCLK_OK;
}

int camera_xclk_stop(int pin)
{
    struct rt_device_pwm *device;
    const int pwm_channel = 1;
    rt_err_t ret;

    if (pin != 2)
    {
        return CAMERA_XCLK_INVALID;
    }

    device = (struct rt_device_pwm *)rt_device_find("pwmt2");
    if (device == RT_NULL)
    {
        LOG_E("find pwmt2 device failed");
        return CAMERA_XCLK_HW;
    }

    ret = rt_pwm_disable(device, pwm_channel);
    if (ret != RT_EOK)
    {
        LOG_E("XCLK PWM disable failed: %d", ret);
        return CAMERA_XCLK_HW;
    }

    LOG_I("stopped (PA%d, pwmt2 ch1)", pin);
    return CAMERA_XCLK_OK;
}

#else

static GPT_HandleTypeDef s_xclk_gptim;
static rt_bool_t s_xclk_initialized = RT_FALSE;
static int s_xclk_pin = -1;
static uint32_t s_xclk_channel = GPT_CHANNEL_1;

int camera_xclk_start(int pin, uint32_t frequency_hz)
{
    HAL_StatusTypeDef status;
    GPT_OC_InitTypeDef output_config = {0};
    uint32_t timer_hz;
    uint32_t period;
    uint32_t calculated_hz;
    int ret;

    if (pin < 0 || pin > 44)
    {
        return CAMERA_XCLK_INVALID;
    }

    if (s_xclk_initialized)
    {
        ret = camera_xclk_stop(s_xclk_pin);
        if (ret != CAMERA_XCLK_OK)
        {
            return ret;
        }
    }

#if !defined(SOC_SF32LB57X) || SOC_SF32LB57X != 1
    if (frequency_hz == 24000000U)
    {
        timer_hz = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);
        s_xclk_gptim.Instance = hwp_gptim1;
        s_xclk_channel = GPT_CHANNEL_2;
        HAL_PIN_Set(PAD_PA00 + pin, GPTIM1_CH2, PIN_NOPULL, 1);
        HAL_RCC_EnableModule(RCC_MOD_GPTIM1);
    }
    else
#endif
    {
#if defined(FIXED_GPTBTIM_SRC_CLK)
        timer_hz = FIXED_GPTBTIM_SRC_CLK;
#elif defined(SOC_SF32LB52X) && SOC_SF32LB52X == 1
        timer_hz = 24000000U;
#else
        timer_hz = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);
#endif
        s_xclk_gptim.Instance = hwp_gptim2;
        s_xclk_channel = GPT_CHANNEL_1;
        HAL_PIN_Set(PAD_PA00 + pin, GPTIM2_CH1, PIN_PULLUP, 1);
        HAL_RCC_EnableModule(RCC_MOD_GPTIM2);
    }

    ret = camera_xclk_calculate_period(timer_hz, frequency_hz, &period);
    if (ret != CAMERA_XCLK_OK)
    {
        LOG_E("invalid frequency %u Hz (timer=%u Hz)",
              (unsigned int)frequency_hz,
              (unsigned int)timer_hz);
        return ret;
    }

    s_xclk_gptim.Init.Prescaler = 0;
    s_xclk_gptim.Init.CounterMode = GPT_COUNTERMODE_UP;
    s_xclk_gptim.Init.Period = period;

    status = HAL_GPT_Base_Init(&s_xclk_gptim);
    if (status != HAL_OK)
    {
        LOG_E("XCLK timer base init failed: %d", status);
        HAL_PIN_Set(PAD_PA00 + pin, GPIO_A0 + pin, PIN_NOPULL, 1);
        return CAMERA_XCLK_HW;
    }

    output_config.OCMode = GPT_OCMODE_PWM1;
    output_config.Pulse = period / 2U + 1U;
    output_config.OCPolarity = GPT_OCPOLARITY_HIGH;
    output_config.OCFastMode = GPT_OCFAST_DISABLE;
    status = HAL_GPT_PWM_ConfigChannel(&s_xclk_gptim,
                                       &output_config,
                                       s_xclk_channel);
    if (status != HAL_OK)
    {
        LOG_E("XCLK timer PWM config failed: %d", status);
        HAL_GPT_Base_DeInit(&s_xclk_gptim);
        HAL_PIN_Set(PAD_PA00 + pin, GPIO_A0 + pin, PIN_NOPULL, 1);
        return CAMERA_XCLK_HW;
    }

    status = HAL_GPT_PWM_Start(&s_xclk_gptim, s_xclk_channel);
    if (status != HAL_OK)
    {
        LOG_E("XCLK timer PWM start failed: %d", status);
        HAL_GPT_Base_DeInit(&s_xclk_gptim);
        HAL_PIN_Set(PAD_PA00 + pin, GPIO_A0 + pin, PIN_NOPULL, 1);
        return CAMERA_XCLK_HW;
    }

    s_xclk_initialized = RT_TRUE;
    s_xclk_pin = pin;
    rt_thread_mdelay(10);
    calculated_hz = (uint32_t)((uint64_t)timer_hz /
                              ((uint64_t)s_xclk_gptim.Instance->PSC + 1U) /
                              ((uint64_t)s_xclk_gptim.Instance->ARR + 1U));
    LOG_I("requested=%u Hz calc=%u Hz on PA%d",
          (unsigned int)frequency_hz, (unsigned int)calculated_hz, pin);
    LOG_I("timer=%u Hz psc=%u arr=%u ccr=%u",
          (unsigned int)timer_hz,
          (unsigned int)s_xclk_gptim.Instance->PSC,
          (unsigned int)s_xclk_gptim.Instance->ARR,
          (unsigned int)(s_xclk_channel == GPT_CHANNEL_1 ?
                         s_xclk_gptim.Instance->CCR1 : s_xclk_gptim.Instance->CCR2));
    return CAMERA_XCLK_OK;
}

int camera_xclk_stop(int pin)
{
    HAL_StatusTypeDef pwm_status;
    HAL_StatusTypeDef base_status;
    int active_pin;

    if (!s_xclk_initialized)
    {
        return CAMERA_XCLK_OK;
    }

    active_pin = s_xclk_pin;
    pwm_status = HAL_GPT_PWM_Stop(&s_xclk_gptim, s_xclk_channel);
    base_status = HAL_GPT_Base_DeInit(&s_xclk_gptim);
    s_xclk_initialized = RT_FALSE;
    s_xclk_pin = -1;

    if (pin >= 0 && pin <= 44)
    {
        active_pin = pin;
    }
    HAL_PIN_Set(PAD_PA00 + active_pin,
                GPIO_A0 + active_pin,
                PIN_NOPULL,
                1);
    if (pwm_status != HAL_OK || base_status != HAL_OK)
    {
        LOG_E("stop failed: pwm=%d base=%d", pwm_status, base_status);
        return CAMERA_XCLK_HW;
    }

    LOG_I("stopped (PA%d)", active_pin);
    return CAMERA_XCLK_OK;
}

#endif
