#ifndef CAMERA_SOURCE_H
#define CAMERA_SOURCE_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    CAMERA_PIXEL_RGB565_BE,
    CAMERA_PIXEL_JPEG,
} camera_pixel_format_t;

typedef struct
{
    uint8_t *data;
    uint32_t size;
    uint16_t width;
    uint16_t height;
    uint32_t sequence;
    uint32_t timestamp_ms;
    camera_pixel_format_t format;
    void *owner;
} camera_frame_t;

typedef void (*camera_source_frame_cb_t)(void *ctx, int result);
typedef int (*camera_source_writer_t)(void *context, const uint8_t *data,
                                      size_t size);

typedef struct
{
    uint8_t *data;
    uint32_t size;
    uint32_t sequence;
} camera_source_stream_frame_t;

typedef struct
{
    int (*open)(void);
    int (*close)(void);
    int (*capture)(uint8_t *dst, uint32_t size, camera_source_frame_cb_t cb,
                   void *ctx);
    int (*stream_start)(uint8_t *buffer, uint32_t size);
    int (*stream_wait)(camera_source_stream_frame_t *frame,
                       uint32_t timeout_ms);
    int (*stream_stop)(void);
    int (*capture_still)(camera_source_writer_t writer, void *context,
                         uint32_t *captured_size);
} camera_source_ops_t;

#endif
