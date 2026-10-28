#ifndef CAMERA_DISPLAY_H
#define CAMERA_DISPLAY_H

#include <stdint.h>

#include "../camera/camera_source.h"

#define CAMERA_DISPLAY_WIDTH 390U
#define CAMERA_DISPLAY_HEIGHT 450U
#define CAMERA_DISPLAY_BUFFER_SIZE \
    (CAMERA_DISPLAY_WIDTH * CAMERA_DISPLAY_HEIGHT * 2U)

int camera_display_open(uint8_t *buffer, uint32_t buffer_size);
int camera_display_show(const camera_frame_t *frame, uint32_t *decode_ms,
                        uint32_t *write_ms);
int camera_display_show_photo(const uint8_t *jpeg, uint32_t jpeg_size,
                              const char *filename, uint32_t *decode_ms);
int camera_display_show_message(const char *filename, const char *message);
void camera_display_close(void);

#endif
