#ifndef JPEG_INFO_H
#define JPEG_INFO_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint16_t width;
    uint16_t height;
    uint8_t sof_marker;
    uint8_t components;
    uint8_t sampling[3];
    uint8_t has_dht;
    uint8_t has_dqt;
} jpeg_info_t;

int jpeg_info_parse(const uint8_t *data, size_t size, jpeg_info_t *info);

#endif
