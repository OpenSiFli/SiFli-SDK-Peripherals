#include "bmp_encoder.h"

#include <string.h>

static void write_le16(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8);
}

static void write_le32(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8);
    output[2] = (uint8_t)(value >> 16);
    output[3] = (uint8_t)(value >> 24);
}

static void rgb565_be_pixel_to_bgr888(const uint8_t *source, uint8_t *output)
{
    uint16_t pixel = ((uint16_t)source[0] << 8) | source[1];
    uint8_t red = (uint8_t)((pixel >> 11) & 0x1fU);
    uint8_t green = (uint8_t)((pixel >> 5) & 0x3fU);
    uint8_t blue = (uint8_t)(pixel & 0x1fU);

    output[0] = (uint8_t)((blue << 3) | (blue >> 2));
    output[1] = (uint8_t)((green << 2) | (green >> 4));
    output[2] = (uint8_t)((red << 3) | (red >> 2));
}

uint32_t bmp_encoder_row_stride(uint16_t width)
{
    return ((uint32_t)width * 3U + 3U) & ~3U;
}

int bmp_encoder_build_header(uint8_t header[BMP_ENCODER_HEADER_SIZE],
                             uint16_t source_width, uint16_t source_height)
{
    uint32_t row_stride;
    uint32_t image_size;

    if ((header == NULL) || (source_width == 0U) || (source_height == 0U))
        return -1;

    row_stride = bmp_encoder_row_stride(source_height);
    image_size = row_stride * source_width;
    memset(header, 0, BMP_ENCODER_HEADER_SIZE);
    header[0] = 'B';
    header[1] = 'M';
    write_le32(&header[2], BMP_ENCODER_HEADER_SIZE + image_size);
    write_le32(&header[10], BMP_ENCODER_HEADER_SIZE);
    write_le32(&header[14], 40U);
    write_le32(&header[18], source_height);
    write_le32(&header[22], source_width);
    write_le16(&header[26], 1U);
    write_le16(&header[28], 24U);
    write_le32(&header[34], image_size);
    return 0;
}

int bmp_encoder_rgb565_be_to_bgr888(const uint8_t *source, uint8_t *output,
                                    uint32_t pixel_count)
{
    uint32_t index;

    if ((source == NULL) || (output == NULL))
        return -1;

    for (index = 0; index < pixel_count; index++)
        rgb565_be_pixel_to_bgr888(&source[index * 2U], &output[index * 3U]);
    return 0;
}

int bmp_encoder_rotated_row(const uint8_t *source, uint16_t source_width,
                            uint16_t source_height, uint16_t bmp_row,
                            uint8_t *output, size_t output_size)
{
    uint32_t row_stride = bmp_encoder_row_stride(source_height);
    uint16_t output_x;
    uint16_t source_x;

    if ((source == NULL) || (output == NULL) || (source_width == 0U) ||
        (source_height == 0U) || (bmp_row >= source_width) ||
        (output_size < row_stride))
        return -1;

    source_x = (uint16_t)(source_width - 1U - bmp_row);
    memset(output, 0, row_stride);
    for (output_x = 0; output_x < source_height; output_x++)
    {
        uint16_t source_y = (uint16_t)(source_height - 1U - output_x);
        const uint8_t *pixel = &source[((uint32_t)source_y * source_width +
                                        source_x) * 2U];
        rgb565_be_pixel_to_bgr888(pixel, &output[(uint32_t)output_x * 3U]);
    }
    return 0;
}
