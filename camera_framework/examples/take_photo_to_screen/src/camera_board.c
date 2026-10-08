/* SPDX-License-Identifier: Apache-2.0 */
#include <bf0_hal.h>
#include <drv_io.h>
#include <rtconfig.h>
#include <rtthread.h>

#if defined(CAMERA_DVP_BACKEND_DCMI) || defined(CAMERA_SERIAL_SPI_2BIT)

typedef struct
{
    int pin;                  /* PAx index, -1 = do not mux */
    pin_function function;
    const char *name;
} camera_dcmi_pad_t;

#if defined(CAMERA_SERIAL_SPI_2BIT)

/*
 * 2-bit SPI wiring: the sensor drives one serial clock and two data lines, and
 * the DCMI recovers the line and frame boundaries from the packet markers
 * embedded in the stream, so no VSYNC/HSYNC pad is involved.
 */
static const camera_dcmi_pad_t camera_serial_pads[] =
{
    { CAMERA_SERIAL_CLK_PIN, DCMI_CLK, "clk" },
    { CAMERA_SERIAL_D0_PIN, DCMI_DI0, "d0" },
    { CAMERA_SERIAL_D1_PIN, DCMI_DI1, "d1" },
};

#define CAMERA_DCMI_PADS camera_serial_pads
#define CAMERA_DCMI_PAD_COUNT \
    (sizeof(camera_serial_pads) / sizeof(camera_serial_pads[0]))
#define CAMERA_DCMI_IF_NAME "2-bit SPI"

#else

static const camera_dcmi_pad_t camera_dvp_pads[] =
{
    { CAMERA_DVP_VSYNC_PIN, DCMI_VSYNC, "vsync" },
    { CAMERA_DVP_HSYNC_PIN, DCMI_HSYNC, "hsync" },
    { CAMERA_DVP_PCLK_PIN, DCMI_CLK, "pclk" },
    { CAMERA_DVP_D0_PIN, DCMI_DI0, "d0" },
    { CAMERA_DVP_D1_PIN, DCMI_DI1, "d1" },
    { CAMERA_DVP_D2_PIN, DCMI_DI2, "d2" },
    { CAMERA_DVP_D3_PIN, DCMI_DI3, "d3" },
    { CAMERA_DVP_D4_PIN, DCMI_DI4, "d4" },
    { CAMERA_DVP_D5_PIN, DCMI_DI5, "d5" },
    { CAMERA_DVP_D6_PIN, DCMI_DI6, "d6" },
    { CAMERA_DVP_D7_PIN, DCMI_DI7, "d7" },
};

#define CAMERA_DCMI_PADS camera_dvp_pads
#define CAMERA_DCMI_PAD_COUNT \
    (sizeof(camera_dvp_pads) / sizeof(camera_dvp_pads[0]))
#define CAMERA_DCMI_IF_NAME "dvp"

#endif /* CAMERA_SERIAL_SPI_2BIT */

void HAL_DCMI_MspInit(DCMI_HandleTypeDef *hdcmi)
{
    const camera_dcmi_pad_t *pads = CAMERA_DCMI_PADS;
    const rt_size_t count = CAMERA_DCMI_PAD_COUNT;
    rt_size_t index;

    (void)hdcmi;

    /* Only the DPI carrier needs this: it shares PA35/PA36 with the USB data
     * lines. Everywhere else BSP_CAMERA_PowerUp() is a weak no-op. */
    BSP_CAMERA_PowerUp();

    for (index = 0U; index < count; index++)
    {
        if (pads[index].pin >= 0)
        {
            HAL_PIN_Set(PAD_PA00 + pads[index].pin,
                        pads[index].function, PIN_NOPULL, 1);
        }
    }

    rt_kprintf("camera: %s", CAMERA_DCMI_IF_NAME);
    for (index = 0U; index < count; index++)
    {
        rt_kprintf(" %s=PA%d", pads[index].name, pads[index].pin);
    }
    rt_kprintf(" (MCLK=CONFIG_CAMERA_XCLK_PIN)\n");
}

#endif /* CAMERA_DVP_BACKEND_DCMI || CAMERA_SERIAL_SPI_2BIT */
