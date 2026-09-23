#ifndef CAMERA_PROFILE_H
#define CAMERA_PROFILE_H

#include <rtconfig.h>

/*
 * Per camera profile: the fastest image path that is also correct for the
 * sensor, backend and ISP that menuconfig selected. Switching the camera in
 * menuconfig switches the whole path with it, so a profile never leaves a wrong
 * colour order or a slow transform behind.
 *
 * | sensor / backend                    | capture byte order | ISP in swap | display swap |
 * |-------------------------------------|--------------------|-------------|--------------|
 * | OV2640 8-bit DVP                    | panel native       | 0           | 0            |
 * | GC032A 8-bit DVP                    | big endian         | 0           | 1            |
 * | GC032A 2-bit SPI / BF30A2           | big endian         | 0           | 1            |
 * | any of the above + software ISP     | as above           | see below   | 1            |
 * | JPEG only sensors (Arducam FIFO)    | not RGB565         | -           | -            |
 *
 * Rotation and the background layer stay off for every camera: the cover crop
 * already fills the panel, while EPIC charges ~245 ms per frame for the 90
 * degree rotation plus a two-layer blend instead of ~5 ms (measured with
 * OV2640 + hardware ISP, which then runs at 29.9 fps). Use
 * `camera_preview 1 1` to try them at runtime.
 */

/*
 * Byte order of a captured RGB565 frame. The DVP hardware path keeps whatever
 * order the sensor puts on the wire, and the sensors differ: OV2640 sends the
 * low byte first, GC032A the high byte first. The 2-bit GPIO sampling mode
 * packs pixels in software and always produces big endian RGB565, which is
 * also the convention of the software ISP and JPEG components; the DCMI SPI
 * 2-bit mode reconstructs the same wire order as the DVP path.
 * A new camera has to be checked on hardware - `camera_preview 0 0 1` flips the
 * display swap at runtime - and its order then recorded here.
 */
#if !defined(CAMERA_USING_DVP)
#define CAMERA_PROFILE_SOURCE_BE 1
#elif defined(SENSOR_USING_GC032A)
#define CAMERA_PROFILE_SOURCE_BE 1
#else
#define CAMERA_PROFILE_SOURCE_BE 0
#endif

/*
 * The software ISP only handles big endian RGB565 (camera_sw_isp's
 * rgb565_be_load() reads pixel[0] << 8 | pixel[1]), so a native order frame is
 * swapped once on its way in, and EPIC swaps its big endian output back on the
 * way to the panel. With the sensor's own ISP there is nothing to convert.
 */
#if defined(CAMERA_ISP_SOFTWARE)
#define CAMERA_PROFILE_ISP_SWAP_INPUT (!CAMERA_PROFILE_SOURCE_BE)
#define CAMERA_PROFILE_DISPLAY_SWAP 1
#else
#define CAMERA_PROFILE_ISP_SWAP_INPUT 0
#define CAMERA_PROFILE_DISPLAY_SWAP CAMERA_PROFILE_SOURCE_BE
#endif

/*
 * EPIC preview transforms, see the table above. Both remain switchable at
 * runtime through the camera_preview command.
 */
#define CAMERA_PROFILE_ROTATE 0
#define CAMERA_PROFILE_BACKGROUND 0

/*
 * Optional MCLK override, applied once after the sensor has been configured
 * through the framework's public camera_xclk_start(). 0 keeps whatever the
 * sensor driver asked for (OV2640 24 MHz, GC032A 8-bit DVP 24 MHz, GC032A
 * 2-bit SPI 24 MHz in the DCMI SPI packet mode and 6 MHz in the GPIO
 * sampling mode, which is the rate that sampler can still keep up with).
 *
 * A sensor's frame rate follows its pixel clock: the GC032A 8-bit DVP driver
 * used to request 12 MHz and measured ~15 fps at VGA, which is why it now asks
 * for the vendor's 24 MHz (~30 fps). Keep this at 0 unless a specific camera
 * needs a different rate; a non-zero value re-drives MCLK *after* the sensor
 * was configured, so the exposure/gain reference moves with it (a brighter or
 * darker image is expected when AEC is off, and it must be checked on
 * hardware).
 */
#define CAMERA_PROFILE_XCLK_HZ 0

#endif /* CAMERA_PROFILE_H */
