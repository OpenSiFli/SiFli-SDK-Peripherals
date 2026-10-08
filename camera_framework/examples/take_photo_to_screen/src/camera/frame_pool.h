#ifndef FRAME_POOL_H
#define FRAME_POOL_H

#include <stdint.h>

#include "camera_source.h"

typedef enum
{
    FRAME_SLOT_FREE,
    FRAME_SLOT_CAPTURING,
    FRAME_SLOT_READY,
    FRAME_SLOT_DISPLAYING,
} frame_slot_state_t;

typedef struct
{
    uint8_t *data;
    uint32_t capacity;
    uint32_t size;
    uint32_t sequence;
    frame_slot_state_t state;
} frame_slot_t;

typedef struct
{
    frame_slot_t slots[2];
    uint16_t width;
    uint16_t height;
    camera_pixel_format_t format;
} frame_pool_t;

void frame_pool_init(frame_pool_t *pool, uint8_t *first, uint8_t *second,
                     uint32_t capacity, uint16_t width, uint16_t height,
                     camera_pixel_format_t format);
int frame_pool_begin_capture(frame_pool_t *pool, uint8_t **dst, uint8_t *slot);
int frame_pool_complete_capture(frame_pool_t *pool, uint8_t slot,
                                uint32_t size, uint32_t sequence);
int frame_pool_cancel_capture(frame_pool_t *pool, uint8_t slot);
int frame_pool_acquire_latest(frame_pool_t *pool, camera_frame_t *frame);
int frame_pool_release(frame_pool_t *pool, camera_frame_t *frame);

#endif
