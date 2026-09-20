#include "camera_jpeg_assembler.h"

void camera_jpeg_assembler_reset(camera_jpeg_assembler_t *assembler,
                                 uint8_t *buffer,
                                 size_t capacity)
{
    if (assembler == NULL)
    {
        return;
    }

    assembler->buffer = buffer;
    assembler->capacity = capacity;
    assembler->frame_size = 0;
    assembler->previous_byte = 0;
    assembler->previous_valid = 0;
    assembler->soi_found = 0;
    assembler->complete = 0;
}

camera_jpeg_result_t camera_jpeg_assembler_feed(
    camera_jpeg_assembler_t *assembler,
    const uint8_t *segment,
    size_t segment_size)
{
    size_t i;

    if (assembler == NULL || assembler->buffer == NULL ||
        assembler->capacity < 2 || segment == NULL || segment_size == 0)
    {
        return CAMERA_JPEG_ERROR;
    }
    if (assembler->complete)
    {
        return CAMERA_JPEG_COMPLETE;
    }

    if (assembler->soi_found)
    {
        uintptr_t buffer_addr = (uintptr_t)assembler->buffer;
        uintptr_t segment_addr = (uintptr_t)segment;
        if (segment_addr >= buffer_addr &&
            segment_addr < buffer_addr + assembler->capacity &&
            (size_t)(segment_addr - buffer_addr) < assembler->frame_size)
        {
            return CAMERA_JPEG_ERROR;
        }
    }

    for (i = 0; i < segment_size; i++)
    {
        uint8_t byte = segment[i];

        if (!assembler->soi_found)
        {
            if (assembler->previous_valid &&
                assembler->previous_byte == 0xFFU && byte == 0xD8U)
            {
                assembler->buffer[0] = 0xFFU;
                assembler->buffer[1] = 0xD8U;
                assembler->frame_size = 2;
                assembler->soi_found = 1;
            }
            assembler->previous_byte = byte;
            assembler->previous_valid = 1;
            continue;
        }

        if (assembler->frame_size >= assembler->capacity)
        {
            return CAMERA_JPEG_ERROR;
        }
        assembler->buffer[assembler->frame_size++] = byte;
        if (assembler->previous_byte == 0xFFU && byte == 0xD9U)
        {
            assembler->complete = 1;
            return CAMERA_JPEG_COMPLETE;
        }
        assembler->previous_byte = byte;
        assembler->previous_valid = 1;
    }

    return CAMERA_JPEG_INCOMPLETE;
}

size_t camera_jpeg_frame_size(const uint8_t *buffer, size_t size)
{
    size_t offset = 2U;
    int in_scan = 0;
    int saw_scan = 0;

    if (buffer == NULL || size < 4U || buffer[0] != 0xFFU || buffer[1] != 0xD8U)
    {
        return 0U;
    }
    while (offset < size)
    {
        uint8_t marker;
        size_t segment_size;

        if (in_scan)
        {
            while (offset < size && buffer[offset] != 0xFFU)
            {
                ++offset;
            }
        }
        else if (buffer[offset] != 0xFFU)
        {
            return 0U;
        }
        while (offset < size && buffer[offset] == 0xFFU)
        {
            ++offset;
        }
        if (offset == size)
        {
            return 0U;
        }
        marker = buffer[offset++];
        if (in_scan && (marker == 0x00U || (marker >= 0xD0U && marker <= 0xD7U)))
        {
            continue;
        }
        if (marker == 0xD9U)
        {
            return saw_scan ? offset : 0U;
        }
        if (marker == 0x00U || marker == 0xD8U || (marker >= 0xD0U && marker <= 0xD7U))
        {
            return 0U;
        }
        if (marker == 0x01U)
        {
            continue;
        }
        if (size - offset < 2U)
        {
            return 0U;
        }
        segment_size = ((size_t)buffer[offset] << 8) | buffer[offset + 1U];
        if (segment_size < 2U || segment_size > size - offset)
        {
            return 0U;
        }
        offset += segment_size;
        /* DNL may occur inside entropy data; other segments end that scan. */
        in_scan = marker == 0xDAU || (in_scan && marker == 0xDCU);
        if (marker == 0xDAU)
        {
            saw_scan = 1;
        }
    }
    return 0U;
}

size_t camera_jpeg_find_frame(const uint8_t *buffer, size_t size,
                             uint32_t frame_number, size_t *frame_offset)
{
    size_t offset = 0U;
    uint32_t number = 1U;

    if (frame_offset == NULL)
    {
        return 0U;
    }
    *frame_offset = 0U;
    if (buffer == NULL || frame_number == 0U)
    {
        return 0U;
    }
    while (offset < size)
    {
        size_t frame_size = camera_jpeg_frame_size(buffer + offset, size - offset);

        if (frame_size == 0U)
        {
            return 0U;
        }
        if (number == frame_number)
        {
            *frame_offset = offset;
            return frame_size;
        }
        ++number;
        offset += frame_size;
        /* Only bytes after a complete EOI may be skipped between frames. */
        while (size - offset >= 2U &&
               (buffer[offset] != 0xFFU || buffer[offset + 1U] != 0xD8U))
        {
            ++offset;
        }
        if (size - offset < 2U)
        {
            return 0U;
        }
    }
    return 0U;
}
