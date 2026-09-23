#ifndef BMP_ENCODER_H
#define BMP_ENCODER_H

#include <stddef.h>
#include <stdint.h>

#define BMP_ENCODER_HEADER_SIZE 54U

uint32_t bmp_encoder_row_stride(uint16_t width);
int bmp_encoder_build_header(uint8_t header[BMP_ENCODER_HEADER_SIZE],
                             uint16_t source_width, uint16_t source_height);
int bmp_encoder_rgb565_be_to_bgr888(const uint8_t *source, uint8_t *output,
                                    uint32_t pixel_count);
int bmp_encoder_rotated_row(const uint8_t *source, uint16_t source_width,
                            uint16_t source_height, uint16_t bmp_row,
                            uint8_t *output, size_t output_size);

#endif
