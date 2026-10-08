#ifndef CAMERA_KEYS_H
#define CAMERA_KEYS_H

typedef void (*camera_key_handler_t)(void);

int camera_keys_init(camera_key_handler_t shutter_handler,
                     camera_key_handler_t album_handler,
                     camera_key_handler_t next_handler);

#endif
