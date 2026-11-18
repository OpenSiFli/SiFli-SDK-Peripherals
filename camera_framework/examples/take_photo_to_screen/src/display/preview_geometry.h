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

/*
 * Where the preview is drawn on the panel.
 *
 * A RAMLESS panel must always get the whole panel as its window, because the
 * LCDC scans the complete frame continuously (full_panel != 0). A panel with
 * GRAM only needs the smaller of the panel and the (rotated) source frame,
 * anchored at the top left: a panel smaller than the frame gets a window the
 * size of the panel, and a panel at least as large as the frame gets a 1:1
 * window.
 */
typedef struct
{
    uint16_t width;
    uint16_t height;
} preview_viewport_t;

int preview_geometry_calculate(uint16_t source_width, uint16_t source_height,
                               uint16_t display_width, uint16_t display_height,
                               uint16_t angle_tenths,
                               preview_geometry_t *geometry);

int preview_viewport_calculate(uint16_t panel_width, uint16_t panel_height,
                               uint16_t source_width, uint16_t source_height,
                               uint16_t angle_tenths, int full_panel,
                               preview_viewport_t *viewport);

#endif
