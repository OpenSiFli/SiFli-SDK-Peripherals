#ifndef PHOTO_STORAGE_H
#define PHOTO_STORAGE_H

#include <stddef.h>
#include <stdint.h>

#include "../camera/camera_source.h"

#define PHOTO_STORAGE_PATH_SIZE 14U

enum
{
    PHOTO_STORAGE_OK = 0,
    PHOTO_STORAGE_ERROR_ARGUMENT = -1,
    PHOTO_STORAGE_ERROR_DEVICE = -2,
    PHOTO_STORAGE_ERROR_MOUNT = -3,
    PHOTO_STORAGE_ERROR_NAME = -4,
    PHOTO_STORAGE_ERROR_OPEN = -5,
    PHOTO_STORAGE_ERROR_WRITE = -6,
    PHOTO_STORAGE_ERROR_SYNC = -7,
    PHOTO_STORAGE_ERROR_CLOSE = -8,
    PHOTO_STORAGE_ERROR_TOO_LARGE = -9,
    PHOTO_STORAGE_ERROR_READ = -10,
    PHOTO_STORAGE_ERROR_EMPTY = -11,
};

typedef int (*photo_storage_jpeg_producer_t)(camera_source_writer_t writer,
                                             void *context,
                                             uint32_t *captured_size);

int photo_storage_format_path(uint16_t index, char *path, size_t path_size);
int photo_storage_format_jpeg_path(uint16_t index, char *path,
                                   size_t path_size);
int photo_storage_save_bmp(const camera_frame_t *frame, char *saved_path,
                           size_t saved_path_size);
int photo_storage_save_jpeg(photo_storage_jpeg_producer_t producer,
                            char *saved_path, size_t saved_path_size,
                            uint32_t *saved_size);
int photo_storage_save_jpeg_data(const uint8_t *jpeg, uint32_t jpeg_size,
                                 char *saved_path, size_t saved_path_size);

/*
 * Mount the card unless it is already mounted and report whether it is ready
 * for the save/load calls above (PHOTO_STORAGE_OK) or not.
 */
int photo_storage_ready(void);

int photo_storage_find_latest_jpeg(char *path, size_t path_size,
                                   uint16_t *index);
int photo_storage_find_next_jpeg(uint16_t current, char *path,
                                 size_t path_size, uint16_t *index);
int photo_storage_load_jpeg(const char *path, uint8_t *buffer,
                            uint32_t capacity, uint32_t *loaded_size);

#endif
