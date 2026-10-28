#include "photo_storage.h"

#include <string.h>

int photo_storage_format_path(uint16_t index, char *path, size_t path_size)
{
    static const char pattern[PHOTO_STORAGE_PATH_SIZE] = "/IMG_0000.BMP";

    if ((path == NULL) || (path_size < PHOTO_STORAGE_PATH_SIZE) ||
        (index == 0U) || (index > 9999U))
        return -1;

    memcpy(path, pattern, sizeof(pattern));
    path[5] = (char)('0' + ((index / 1000U) % 10U));
    path[6] = (char)('0' + ((index / 100U) % 10U));
    path[7] = (char)('0' + ((index / 10U) % 10U));
    path[8] = (char)('0' + (index % 10U));
    return 0;
}

int photo_storage_format_jpeg_path(uint16_t index, char *path,
                                   size_t path_size)
{
    if (photo_storage_format_path(index, path, path_size) != 0)
        return -1;
    path[10] = 'J';
    path[11] = 'P';
    path[12] = 'G';
    return 0;
}

#ifndef CAMERA_PHOTO_STORAGE_HOST_TEST

#include <dfs_fs.h>
#include <dfs_posix.h>
#include <rtdevice.h>
#include <rtthread.h>

#include "bmp_encoder.h"
#include "photo_catalog.h"

/*
 * Which SDMMC instance the carrier wires decides the block device name:
 * SDMMC1 enumerates as "sd0" and SDMMC2 as "sd1" (see the RT-Thread mmcsd
 * block layer), so both are probed and the first one that shows up is used.
 */
static const char *const photo_storage_devices[] =
{
    "sd1",
    "sd0",
};
#define PHOTO_STORAGE_MOUNT_POINT "/"
#define PHOTO_STORAGE_MAX_ROW_SIZE 1440U
#define PHOTO_STORAGE_DEVICE_RETRIES 30

static int photo_storage_mount(void)
{
    const char *device_name = RT_NULL;
    rt_device_t device;
    const char *mounted_path;
    uint32_t index;
    int retry;

    for (retry = 0; (retry < PHOTO_STORAGE_DEVICE_RETRIES) &&
                    (device_name == RT_NULL); retry++)
    {
        for (index = 0U;
             index < sizeof(photo_storage_devices) /
                     sizeof(photo_storage_devices[0]);
             index++)
        {
            if (rt_device_find(photo_storage_devices[index]) != RT_NULL)
            {
                device_name = photo_storage_devices[index];
                break;
            }
        }
        if (device_name == RT_NULL)
            rt_thread_mdelay(100);
    }
    if (device_name == RT_NULL)
        return PHOTO_STORAGE_ERROR_DEVICE;

    device = rt_device_find(device_name);
    mounted_path = dfs_filesystem_get_mounted_path(device);
    if (mounted_path != RT_NULL)
        return strcmp(mounted_path, PHOTO_STORAGE_MOUNT_POINT) == 0 ?
               PHOTO_STORAGE_OK : PHOTO_STORAGE_ERROR_MOUNT;

    if (dfs_mount(device_name, PHOTO_STORAGE_MOUNT_POINT,
                  "elm", 0, RT_NULL) != 0)
        return PHOTO_STORAGE_ERROR_MOUNT;
    return PHOTO_STORAGE_OK;
}

int photo_storage_ready(void)
{
    return photo_storage_mount();
}

static int photo_storage_next_path(char *path, size_t path_size,
                                   int jpeg)
{
    struct stat info;
    uint16_t index;

    for (index = 1; index <= 9999U; index++)
    {
        if ((jpeg ? photo_storage_format_jpeg_path(index, path, path_size) :
                    photo_storage_format_path(index, path, path_size)) != 0)
            return PHOTO_STORAGE_ERROR_NAME;
        if (stat(path, &info) != 0)
            return PHOTO_STORAGE_OK;
    }
    return PHOTO_STORAGE_ERROR_NAME;
}

static int photo_storage_write_all(int fd, const uint8_t *data, size_t size)
{
    size_t written = 0;

    while (written < size)
    {
        int result = write(fd, data + written, size - written);
        if (result <= 0)
            return PHOTO_STORAGE_ERROR_WRITE;
        written += (size_t)result;
    }
    return PHOTO_STORAGE_OK;
}

static int photo_storage_find_jpeg(uint16_t current, int find_latest,
                                   char *path, size_t path_size,
                                   uint16_t *selected_index)
{
    DIR *directory;
    struct dirent *entry;
    uint16_t lowest = 0U;
    uint16_t selected = 0U;
    int result;

    if ((path == RT_NULL) || (path_size < PHOTO_STORAGE_PATH_SIZE) ||
        (selected_index == RT_NULL))
        return PHOTO_STORAGE_ERROR_ARGUMENT;
    result = photo_storage_mount();
    if (result != PHOTO_STORAGE_OK)
        return result;
    directory = opendir(PHOTO_STORAGE_MOUNT_POINT);
    if (directory == RT_NULL)
        return PHOTO_STORAGE_ERROR_OPEN;
    while ((entry = readdir(directory)) != RT_NULL)
    {
        uint16_t index;

        if (photo_catalog_parse_index(entry->d_name, &index) != 0)
            continue;
        if ((lowest == 0U) || (index < lowest))
            lowest = index;
        if (find_latest)
        {
            if (index > selected)
                selected = index;
        }
        else if ((index > current) &&
                 ((selected == 0U) || (index < selected)))
        {
            selected = index;
        }
    }
    closedir(directory);
    if (!find_latest && (selected == 0U))
        selected = lowest;
    if (selected == 0U)
        return PHOTO_STORAGE_ERROR_EMPTY;
    if (photo_storage_format_jpeg_path(selected, path, path_size) != 0)
        return PHOTO_STORAGE_ERROR_NAME;
    *selected_index = selected;
    return PHOTO_STORAGE_OK;
}

int photo_storage_find_latest_jpeg(char *path, size_t path_size,
                                   uint16_t *index)
{
    return photo_storage_find_jpeg(0U, 1, path, path_size, index);
}

int photo_storage_find_next_jpeg(uint16_t current, char *path,
                                 size_t path_size, uint16_t *index)
{
    return photo_storage_find_jpeg(current, 0, path, path_size, index);
}

int photo_storage_load_jpeg(const char *path, uint8_t *buffer,
                            uint32_t capacity, uint32_t *loaded_size)
{
    struct stat info;
    uint32_t loaded = 0U;
    int fd;

    if ((path == RT_NULL) || (buffer == RT_NULL) || (capacity == 0U) ||
        (loaded_size == RT_NULL))
        return PHOTO_STORAGE_ERROR_ARGUMENT;
    *loaded_size = 0U;
    if (stat(path, &info) != 0)
        return PHOTO_STORAGE_ERROR_OPEN;
    if ((info.st_size <= 0) || ((uint32_t)info.st_size > capacity))
        return PHOTO_STORAGE_ERROR_TOO_LARGE;
    fd = open(path, O_RDONLY, 0);
    if (fd < 0)
        return PHOTO_STORAGE_ERROR_OPEN;
    while (loaded < (uint32_t)info.st_size)
    {
        int count = read(fd, buffer + loaded,
                         (uint32_t)info.st_size - loaded);

        if (count <= 0)
        {
            close(fd);
            return PHOTO_STORAGE_ERROR_READ;
        }
        loaded += (uint32_t)count;
    }
    if (close(fd) != 0)
        return PHOTO_STORAGE_ERROR_CLOSE;
    *loaded_size = loaded;
    return PHOTO_STORAGE_OK;
}

int photo_storage_save_bmp(const camera_frame_t *frame, char *saved_path,
                           size_t saved_path_size)
{
    uint8_t header[BMP_ENCODER_HEADER_SIZE];
    uint8_t row[PHOTO_STORAGE_MAX_ROW_SIZE];
    uint32_t row_stride;
    uint16_t bmp_row;
    int fd = -1;
    int result;

    if ((frame == RT_NULL) || (frame->data == RT_NULL) ||
        (frame->format != CAMERA_PIXEL_RGB565_BE) ||
        (frame->width != 640U) || (frame->height != 480U) ||
        (frame->size != 640U * 480U * 2U) ||
        (saved_path == RT_NULL) ||
        (saved_path_size < PHOTO_STORAGE_PATH_SIZE))
        return PHOTO_STORAGE_ERROR_ARGUMENT;
    saved_path[0] = '\0';

    result = photo_storage_mount();
    if (result != PHOTO_STORAGE_OK)
        return result;
    result = photo_storage_next_path(saved_path, saved_path_size, 0);
    if (result != PHOTO_STORAGE_OK)
        return result;
    if (bmp_encoder_build_header(header, frame->width, frame->height) != 0)
        return PHOTO_STORAGE_ERROR_ARGUMENT;

    row_stride = bmp_encoder_row_stride(frame->height);
    if (row_stride > sizeof(row))
        return PHOTO_STORAGE_ERROR_ARGUMENT;

    fd = open(saved_path, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0)
        return PHOTO_STORAGE_ERROR_OPEN;
    result = photo_storage_write_all(fd, header, sizeof(header));
    for (bmp_row = 0; (result == PHOTO_STORAGE_OK) &&
         (bmp_row < frame->width); bmp_row++)
    {
        if (bmp_encoder_rotated_row(frame->data, frame->width, frame->height,
                                    bmp_row, row, sizeof(row)) != 0)
            result = PHOTO_STORAGE_ERROR_ARGUMENT;
        else
            result = photo_storage_write_all(fd, row, row_stride);
    }
    if ((result == PHOTO_STORAGE_OK) && (fsync(fd) != 0))
        result = PHOTO_STORAGE_ERROR_SYNC;
    if (close(fd) != 0)
    {
        if (result == PHOTO_STORAGE_OK)
            result = PHOTO_STORAGE_ERROR_CLOSE;
    }
    fd = -1;

    if (result != PHOTO_STORAGE_OK)
    {
        unlink(saved_path);
        saved_path[0] = '\0';
    }
    return result;
}

typedef struct
{
    int fd;
    uint32_t size;
} photo_storage_jpeg_writer_t;

static int photo_storage_write_jpeg_chunk(void *context, const uint8_t *data,
                                          size_t size)
{
    photo_storage_jpeg_writer_t *writer = context;
    int result = photo_storage_write_all(writer->fd, data, size);

    if (result == PHOTO_STORAGE_OK)
        writer->size += (uint32_t)size;
    return result;
}

int photo_storage_save_jpeg(photo_storage_jpeg_producer_t producer,
                            char *saved_path, size_t saved_path_size,
                            uint32_t *saved_size)
{
    photo_storage_jpeg_writer_t writer = {.fd = -1};
    uint32_t captured_size = 0U;
    int result;

    if ((producer == RT_NULL) || (saved_path == RT_NULL) ||
        (saved_path_size < PHOTO_STORAGE_PATH_SIZE) ||
        (saved_size == RT_NULL))
        return PHOTO_STORAGE_ERROR_ARGUMENT;
    saved_path[0] = '\0';
    *saved_size = 0U;

    result = photo_storage_mount();
    if (result == PHOTO_STORAGE_OK)
        result = photo_storage_next_path(saved_path, saved_path_size, 1);
    if (result != PHOTO_STORAGE_OK)
        return result;

    writer.fd = open(saved_path, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (writer.fd < 0)
        return PHOTO_STORAGE_ERROR_OPEN;
    if (producer(photo_storage_write_jpeg_chunk, &writer,
                 &captured_size) != 0)
        result = PHOTO_STORAGE_ERROR_WRITE;
    else if ((captured_size == 0U) || (captured_size != writer.size))
        result = PHOTO_STORAGE_ERROR_WRITE;
    else if (fsync(writer.fd) != 0)
        result = PHOTO_STORAGE_ERROR_SYNC;
    else
        result = PHOTO_STORAGE_OK;

    if ((close(writer.fd) != 0) && (result == PHOTO_STORAGE_OK))
        result = PHOTO_STORAGE_ERROR_CLOSE;
    if (result != PHOTO_STORAGE_OK)
    {
        unlink(saved_path);
        saved_path[0] = '\0';
    }
    else
    {
        *saved_size = writer.size;
    }
    return result;
}

int photo_storage_save_jpeg_data(const uint8_t *jpeg, uint32_t jpeg_size,
                                 char *saved_path, size_t saved_path_size)
{
    int fd = -1;
    int result;

    if ((jpeg == RT_NULL) || (jpeg_size < 4U) ||
        (jpeg[0] != 0xFFU) || (jpeg[1] != 0xD8U) ||
        (jpeg[jpeg_size - 2U] != 0xFFU) ||
        (jpeg[jpeg_size - 1U] != 0xD9U) ||
        (saved_path == RT_NULL) ||
        (saved_path_size < PHOTO_STORAGE_PATH_SIZE))
        return PHOTO_STORAGE_ERROR_ARGUMENT;
    saved_path[0] = '\0';

    result = photo_storage_mount();
    if (result == PHOTO_STORAGE_OK)
        result = photo_storage_next_path(saved_path, saved_path_size, 1);
    if (result != PHOTO_STORAGE_OK)
        return result;

    fd = open(saved_path, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0)
        return PHOTO_STORAGE_ERROR_OPEN;
    result = photo_storage_write_all(fd, jpeg, jpeg_size);
    if ((result == PHOTO_STORAGE_OK) && (fsync(fd) != 0))
        result = PHOTO_STORAGE_ERROR_SYNC;
    if ((close(fd) != 0) && (result == PHOTO_STORAGE_OK))
        result = PHOTO_STORAGE_ERROR_CLOSE;
    if (result != PHOTO_STORAGE_OK)
    {
        unlink(saved_path);
        saved_path[0] = '\0';
    }
    return result;
}

#endif
