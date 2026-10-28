#include "photo_overlay.h"

#include <stddef.h>
#include <string.h>

#define PHOTO_FONT_WIDTH 5U
#define PHOTO_FONT_HEIGHT 7U
#define PHOTO_FONT_ADVANCE 6U

typedef struct
{
    char character;
    uint8_t columns[PHOTO_FONT_WIDTH];
} photo_glyph_t;

static const photo_glyph_t photo_glyphs[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {'_', {0x40, 0x40, 0x40, 0x40, 0x40}},
    {'0', {0x3e, 0x51, 0x49, 0x45, 0x3e}},
    {'1', {0x00, 0x42, 0x7f, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4b, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7f, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3c, 0x4a, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1e}},
    {'A', {0x7e, 0x11, 0x11, 0x11, 0x7e}},
    {'C', {0x3e, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7f, 0x41, 0x41, 0x22, 0x1c}},
    {'E', {0x7f, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7f, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3e, 0x41, 0x49, 0x49, 0x7a}},
    {'H', {0x7f, 0x08, 0x08, 0x08, 0x7f}},
    {'I', {0x00, 0x41, 0x7f, 0x41, 0x00}},
    {'J', {0x20, 0x40, 0x41, 0x3f, 0x01}},
    {'L', {0x7f, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7f, 0x02, 0x0c, 0x02, 0x7f}},
    {'N', {0x7f, 0x04, 0x08, 0x10, 0x7f}},
    {'O', {0x3e, 0x41, 0x41, 0x41, 0x3e}},
    {'P', {0x7f, 0x09, 0x09, 0x09, 0x06}},
    {'R', {0x7f, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7f, 0x01, 0x01}},
    {'U', {0x3f, 0x40, 0x40, 0x40, 0x3f}},
    {'Y', {0x03, 0x04, 0x78, 0x04, 0x03}},
};

static const uint8_t *photo_overlay_glyph(char character)
{
    size_t index;

    for (index = 0U; index < sizeof(photo_glyphs) / sizeof(photo_glyphs[0]);
         index++)
    {
        if (photo_glyphs[index].character == character)
            return photo_glyphs[index].columns;
    }
    return photo_glyphs[0].columns;
}

static void photo_overlay_text(uint8_t *pixels, uint16_t width,
                               uint16_t height, uint16_t x, uint16_t y,
                               const char *text)
{
    while ((*text != '\0') && (x + PHOTO_FONT_WIDTH <= width))
    {
        const uint8_t *glyph = photo_overlay_glyph(*text++);
        uint16_t column;

        for (column = 0U; column < PHOTO_FONT_WIDTH; column++)
        {
            uint16_t row;

            for (row = 0U; row < PHOTO_FONT_HEIGHT; row++)
            {
                if (((glyph[column] >> row) & 1U) != 0U && y + row < height)
                {
                    uint32_t offset = ((uint32_t)(y + row) * width +
                                       x + column) * 2U;
                    pixels[offset] = 0xffU;
                    pixels[offset + 1U] = 0xffU;
                }
            }
        }
        x = (uint16_t)(x + PHOTO_FONT_ADVANCE);
    }
}

static uint16_t photo_overlay_center_x(uint16_t width, const char *text)
{
    size_t length = strlen(text);
    size_t text_width;

    if (length == 0U)
        return width / 2U;
    text_width = length * PHOTO_FONT_ADVANCE -
                 (PHOTO_FONT_ADVANCE - PHOTO_FONT_WIDTH);
    return text_width < width ? (uint16_t)((width - text_width) / 2U) : 0U;
}

int photo_overlay_draw(uint8_t *pixels, uint16_t width, uint16_t height,
                       const char *filename, const char *message)
{
    uint16_t lines = message == NULL ? 1U : 2U;
    uint16_t bar_height = (uint16_t)(lines * 8U + 2U);
    uint16_t top;

    if ((pixels == NULL) || (filename == NULL) || (width == 0U) ||
        (height < bar_height))
        return -1;
    top = (uint16_t)(height - bar_height);
    memset(pixels + (uint32_t)top * width * 2U, 0,
           (uint32_t)bar_height * width * 2U);
    if (message != NULL)
        photo_overlay_text(pixels, width, height,
                           photo_overlay_center_x(width, message),
                           (uint16_t)(top + 1U), message);
    photo_overlay_text(pixels, width, height,
                       photo_overlay_center_x(width, filename),
                       (uint16_t)(top + (lines - 1U) * 8U + 1U), filename);
    return 0;
}
