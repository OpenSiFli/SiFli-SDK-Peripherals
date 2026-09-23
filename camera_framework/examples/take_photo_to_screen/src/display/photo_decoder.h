#ifndef PHOTO_DECODER_H
#define PHOTO_DECODER_H

#include <stdint.h>

enum
{
    PHOTO_DECODER_OK = 0,
    PHOTO_DECODER_UNSUPPORTED = -1,
};

typedef struct
{
    const char *name;
    int (*decode)(const uint8_t *jpeg, uint32_t jpeg_size,
                  uint8_t *destination, uint32_t destination_size,
                  uint16_t *width, uint16_t *height, uint32_t *decode_ms);
} photo_decoder_backend_t;

const photo_decoder_backend_t *photo_decoder_backend(void);

#endif
