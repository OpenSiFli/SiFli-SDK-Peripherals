#ifndef CAMERA_JPEG_ASSEMBLER_H
#define CAMERA_JPEG_ASSEMBLER_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    CAMERA_JPEG_ERROR = -1,
    CAMERA_JPEG_INCOMPLETE = 0,
    CAMERA_JPEG_COMPLETE = 1,
} camera_jpeg_result_t;

typedef struct
{
    uint8_t *buffer;
    size_t capacity;
    size_t frame_size;
    uint8_t previous_byte;
    uint8_t previous_valid;
    uint8_t soi_found;
    uint8_t complete;
} camera_jpeg_assembler_t;

void camera_jpeg_assembler_reset(camera_jpeg_assembler_t *assembler,
                                 uint8_t *buffer,
                                 size_t capacity);

camera_jpeg_result_t camera_jpeg_assembler_feed(
    camera_jpeg_assembler_t *assembler,
    const uint8_t *segment,
    size_t segment_size);

/* Return the SOI..EOI length within received bytes, or zero for an incomplete
 * or malformed marker stream. This does not decode the compressed image. */
size_t camera_jpeg_frame_size(const uint8_t *buffer, size_t size);

/* Find a one-based frame after optional inter-frame padding. The first SOI
 * must be at offset zero. Failure returns zero and clears frame_offset. */
size_t camera_jpeg_find_frame(const uint8_t *buffer, size_t size,
                             uint32_t frame_number, size_t *frame_offset);

#endif /* CAMERA_JPEG_ASSEMBLER_H */
