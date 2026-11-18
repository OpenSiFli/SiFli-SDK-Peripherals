#ifndef CAMERA_APP_CONFIG_H
#define CAMERA_APP_CONFIG_H

#include <rtconfig.h>

/*
 * Camera application feature switches.
 *
 * Preview and photo capture used to be one path: the display thread owned the
 * LCD, the frame pool and the shutter.  On carriers where the LCD and DCMI
 * share pins the two cannot run together, so the preview side is compiled out
 * by default and only the capture/storage path stays in the image.
 *
 * Enable preview again with:
 *   CONFIG_CAMERA_APP_PREVIEW=y
 * in project/proj.conf (or through menuconfig).  That also pulls display/*.c
 * back into the build, because src/SConscript uses the same symbol.
 */
#if defined(CAMERA_APP_PREVIEW) && (CAMERA_APP_PREVIEW != 0)
#define CAMERA_APP_USE_PREVIEW 1
#else
#define CAMERA_APP_USE_PREVIEW 0
#endif

#endif /* CAMERA_APP_CONFIG_H */
