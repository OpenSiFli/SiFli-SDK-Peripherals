#ifndef CAMERA_DISPLAY_H
#define CAMERA_DISPLAY_H

#include <stdint.h>

#include "../camera/camera_source.h"

int camera_display_open(void);
int camera_display_show(const camera_frame_t *frame, uint32_t *decode_ms,
                        uint32_t *write_ms);
int camera_display_show_photo(const uint8_t *jpeg, uint32_t jpeg_size,
                              const char *filename, uint32_t *decode_ms);
int camera_display_show_message(const char *filename, const char *message);
void camera_display_close(void);

#endif
