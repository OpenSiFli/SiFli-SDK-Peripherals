#ifndef PREVIEW_GEOMETRY_H
#define PREVIEW_GEOMETRY_H

#include <stdint.h>

typedef struct
{
    uint16_t rotated_width;
    uint16_t rotated_height;
    uint16_t scaled_width;
    uint16_t scaled_height;
    uint16_t crop_x;
    uint16_t crop_y;
    uint16_t epic_scale;
    int16_t angle_tenths;
} preview_geometry_t;

int preview_geometry_calculate(uint16_t source_width, uint16_t source_height,
                               uint16_t display_width, uint16_t display_height,
                               uint16_t angle_tenths,
                               preview_geometry_t *geometry);

#endif
