/*
 * SPDX-License-Identifier: LicenseRef-ChaN-TJpgDec
 *
 * Copyright (C) 2021, ChaN
 *
 * Portable wrapper around the vendored Tiny JPEG Decompressor. The original
 * copyright and license header must remain attached to the vendored sources;
 * this file adds only reader/scale/output integration.
 */

#include "camera_sw_jpeg_decoder.h"

#include <string.h>

/*
 * Relative include: this wrapper must always compile against the vendored
 * tjpgdcnf.h (RGB565 / 2048 / FASTDECODE 1). A bare "tjpgd.h" would resolve to
 * LVGL v8's sjpg copy whenever that include directory comes first, giving this
 * translation unit a different JDEC layout than the implementation it links.
 */
#include "../../vendor/tjpgd/tjpgd.h"

/* The public header promises RGB565_BE output; enforce the matching core
 * configuration at build time instead of failing at runtime. */
#if JD_FORMAT != 1
#error "camera_sw_jpeg_decoder requires JD_FORMAT 1 (RGB565)"
#endif

typedef struct
{
    camera_sw_jpeg_reader_fn reader;
    void *reader_context;
    uint8_t *destination;
    uint16_t width;
    uint16_t height;
} camera_sw_jpeg_context_t;

static size_t camera_sw_jpeg_input(JDEC *decoder, uint8_t *buffer,
                                   size_t size)
{
    camera_sw_jpeg_context_t *context =
        (camera_sw_jpeg_context_t *)decoder->device;

    return context->reader(context->reader_context, buffer, size);
}

static int camera_sw_jpeg_swap_write(uint8_t *destination, uint16_t width,
                                     uint16_t height, const uint8_t *rgb,
                                     const JRECT *rect)
{
    uint32_t rect_width;
    uint32_t rect_height;
    uint32_t y;

    if ((destination == NULL) || (rgb == NULL) || (rect == NULL) ||
        (rect->left > rect->right) || (rect->top > rect->bottom) ||
        (rect->right >= width) || (rect->bottom >= height))
        return -1;

    rect_width = (uint32_t)rect->right - rect->left + 1U;
    rect_height = (uint32_t)rect->bottom - rect->top + 1U;
    for (y = 0U; y < rect_height; y++)
    {
        const uint8_t *in = rgb + y * rect_width * 2U;
        uint8_t *out = destination +
                       (((uint32_t)rect->top + y) * width + rect->left) * 2U;
        uint32_t x;

        for (x = 0U; x < rect_width; x++)
        {
            uint8_t low = *in++;
            uint8_t high = *in++;

            *out++ = high;
            *out++ = low;
        }
    }
    return 0;
}

static int camera_sw_jpeg_output(JDEC *decoder, void *bitmap, JRECT *rect)
{
    camera_sw_jpeg_context_t *context =
        (camera_sw_jpeg_context_t *)decoder->device;

    return camera_sw_jpeg_swap_write(context->destination, context->width,
                                     context->height, bitmap, rect) == 0;
}

int camera_sw_jpeg_decode(camera_sw_jpeg_reader_fn reader,
                          void *reader_context, uint8_t *work,
                          uint32_t work_size, camera_sw_jpeg_frame_t *output)
{
    camera_sw_jpeg_context_t context;
    JDEC decoder;
    JRESULT result;
    int scale;
    uint16_t scaled_width;
    uint16_t scaled_height;

    if ((reader == NULL) || (reader_context == NULL) || (work == NULL) ||
        (work_size == 0U) ||
        (output == NULL) || (output->data == NULL) ||
        (output->stride == 0U) || (output->height == 0U))
        return -1;

    context.reader = reader;
    context.reader_context = reader_context;
    context.destination = output->data;
    result = camera_fw_jd_prepare(&decoder, camera_sw_jpeg_input, work, work_size,
                                  &context);
    if (result != JDR_OK)
        return -(int)result;

    for (scale = 0; scale <= 3; scale++)
    {
        uint32_t candidate_width = decoder.width >> scale;
        uint32_t candidate_height = decoder.height >> scale;

        if ((candidate_width != 0U) && (candidate_height != 0U) &&
            (candidate_width * 2U <= output->stride) &&
            (candidate_height <= output->height))
            break;
    }
    if (scale > 3)
        return -1;

    scaled_width = (uint16_t)(decoder.width >> scale);
    scaled_height = (uint16_t)(decoder.height >> scale);
    context.width = scaled_width;
    context.height = scaled_height;
    result = camera_fw_jd_decomp(&decoder, camera_sw_jpeg_output, (uint8_t)scale);
    if (result != JDR_OK)
        return -(int)result;

    output->width = scaled_width;
    output->height = scaled_height;
    output->stride = (uint32_t)scaled_width * 2U;
    output->pixel_format = CAMERA_SW_JPEG_PIXEL_RGB565_BE;
    return 0;
}
