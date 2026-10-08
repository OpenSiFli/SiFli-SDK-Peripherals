#include "frame_pool.h"

#include <stddef.h>

void frame_pool_init(frame_pool_t *pool, uint8_t *first, uint8_t *second,
                     uint32_t capacity, uint16_t width, uint16_t height,
                     camera_pixel_format_t format)
{
    if (pool == NULL)
        return;

    pool->slots[0] = (frame_slot_t){first, capacity, 0, 0, FRAME_SLOT_FREE};
    pool->slots[1] = (frame_slot_t){second, capacity, 0, 0, FRAME_SLOT_FREE};
    pool->width = width;
    pool->height = height;
    pool->format = format;
}

int frame_pool_begin_capture(frame_pool_t *pool, uint8_t **dst, uint8_t *slot)
{
    uint8_t index;
    int oldest_ready = -1;

    if ((pool == NULL) || (dst == NULL) || (slot == NULL))
        return -1;

    for (index = 0; index < 2; index++)
    {
        if (pool->slots[index].state == FRAME_SLOT_FREE)
        {
            pool->slots[index].state = FRAME_SLOT_CAPTURING;
            *dst = pool->slots[index].data;
            *slot = index;
            return 0;
        }
        if ((pool->slots[index].state == FRAME_SLOT_READY) &&
            ((oldest_ready < 0) ||
             (pool->slots[index].sequence <
              pool->slots[(uint8_t)oldest_ready].sequence)))
            oldest_ready = index;
    }

    if (oldest_ready >= 0)
    {
        index = (uint8_t)oldest_ready;
        pool->slots[index].state = FRAME_SLOT_CAPTURING;
        *dst = pool->slots[index].data;
        *slot = index;
        return 0;
    }

    return -1;
}

int frame_pool_complete_capture(frame_pool_t *pool, uint8_t slot,
                                uint32_t size, uint32_t sequence)
{
    frame_slot_t *frame_slot;

    if ((pool == NULL) || (slot >= 2))
        return -1;

    frame_slot = &pool->slots[slot];
    if ((frame_slot->state != FRAME_SLOT_CAPTURING) || (size > frame_slot->capacity))
        return -1;

    frame_slot->size = size;
    frame_slot->sequence = sequence;
    frame_slot->state = FRAME_SLOT_READY;
    return 0;
}

int frame_pool_cancel_capture(frame_pool_t *pool, uint8_t slot)
{
    if ((pool == NULL) || (slot >= 2) ||
        (pool->slots[slot].state != FRAME_SLOT_CAPTURING))
        return -1;

    pool->slots[slot].state = FRAME_SLOT_FREE;
    return 0;
}

int frame_pool_acquire_latest(frame_pool_t *pool, camera_frame_t *frame)
{
    uint8_t index;
    int latest = -1;

    if ((pool == NULL) || (frame == NULL))
        return -1;

    for (index = 0; index < 2; index++)
    {
        if (pool->slots[index].state == FRAME_SLOT_DISPLAYING)
            return -1;
        if ((pool->slots[index].state == FRAME_SLOT_READY) &&
            ((latest < 0) ||
             (pool->slots[index].sequence > pool->slots[(uint8_t)latest].sequence)))
            latest = index;
    }

    if (latest < 0)
        return -1;

    for (index = 0; index < 2; index++)
    {
        if ((index != (uint8_t)latest) &&
            (pool->slots[index].state == FRAME_SLOT_READY))
            pool->slots[index].state = FRAME_SLOT_FREE;
    }

    pool->slots[(uint8_t)latest].state = FRAME_SLOT_DISPLAYING;
    *frame = (camera_frame_t){
        .data = pool->slots[(uint8_t)latest].data,
        .size = pool->slots[(uint8_t)latest].size,
        .width = pool->width,
        .height = pool->height,
        .format = pool->format,
        .sequence = pool->slots[(uint8_t)latest].sequence,
        .owner = &pool->slots[(uint8_t)latest],
    };
    return 0;
}

int frame_pool_release(frame_pool_t *pool, camera_frame_t *frame)
{
    frame_slot_t *slot;

    if ((pool == NULL) || (frame == NULL) || (frame->owner == NULL))
        return -1;

    slot = (frame_slot_t *)frame->owner;
    if ((slot != &pool->slots[0]) && (slot != &pool->slots[1]))
        return -1;
    if (slot->state != FRAME_SLOT_DISPLAYING)
        return -1;

    slot->state = FRAME_SLOT_FREE;
    frame->owner = NULL;
    return 0;
}
