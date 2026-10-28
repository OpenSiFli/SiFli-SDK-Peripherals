#ifndef PHOTO_OVERLAY_H
#define PHOTO_OVERLAY_H

#include <stdint.h>

int photo_overlay_draw(uint8_t *pixels, uint16_t width, uint16_t height,
                       const char *filename, const char *message);

#endif
