/******************************************************************************
 * @file    main.c
 * @brief   OV2640 take_photo_to_sdcard example - capture JPEG frames via the
 *          camera handle high-level API and save them to the SD card.
 *
 * The example uses the camera_handle.h API to grab one or more JPEG frames
 * into a PSRAM buffer and then writes each frame as a numbered .jpg file
 * under /photo on the mounted SD card filesystem.
 *
 * Call sequence (per `take_photo` invocation):
 *   camera_handler_instance_init()  - prepare handle state, register and open RT-Thread device
 *   camera_change_settings()        - configure JPEG + framesize + quality
 *   camera_capture_single() (loop)  - blocking single-frame grab
 *   write to /photo/photo_NNN.jpg   - save each captured frame
 *   camera_deinit()                 - close the device
 *
 * Note: low-level pin muxing for SCCB / DVP / XCLK is performed by the
 * OV2640 driver itself, so this example does not call HAL_PIN_Set().
 *****************************************************************************/

#include "rtthread.h"
#include "bf0_hal.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <rtdevice.h>
#include "mem_section.h"
#include "dfs_file.h"
#include "dfs_posix.h"
#include "spi_msd.h"
#include "camera_handle.h"

/* ------------------------------------------------------------------ *
 * PSRAM heap - holds the JPEG frame buffer (internal SRAM is too
 * small for VGA-and-above JPEG output).
 * ------------------------------------------------------------------ */
static uint8_t psram_heap_pool[4096 * 1024] L2_RET_BSS_SECT(psram_heap_pool);
static struct rt_memheap psram_memheap;

#define JPEG_MIN_BUFFER_SIZE    (64 * 1024)
#define JPEG_MAX_BUFFER_SIZE    (2 * 1024 * 1024)
#define PHOTO_DIR               "/photo"

/**
 * @brief Initialize the PSRAM heap pool.
 *
 * @return Return 0 on success (fixed value).
 */
int psram_heap_init(void)
{
    rt_memheap_init(&psram_memheap, "psram_heap", (void *)psram_heap_pool,
                    sizeof(psram_heap_pool));
    return 0;
}

/**
 * @brief Allocate memory from the PSRAM heap.
 *
 * @param size is the number of bytes to allocate.
 *
 * @return Return a pointer on success, RT_NULL on failure.
 */
void *psram_heap_malloc(uint32_t size)
{
    return rt_memheap_alloc(&psram_memheap, size);
}

/**
 * @brief Release memory previously returned by psram_heap_malloc().
 *
 * @param p is the pointer to free.
 */
void psram_heap_free(void *p)
{
    rt_memheap_free(p);
}

/* ------------------------------------------------------------------ *
 * SD card mount
 * ------------------------------------------------------------------ */

/**
 * @brief Locate the SD card device and mount it as the root FAT volume.
 */
void sdcard_init(void)
{
    rt_device_t msd = rt_device_find("sd0");
    if (msd == RT_NULL)
    {
        rt_kprintf("sd card not found\n");
        return;
    }

    if (dfs_mount("sd0", "/", "elm", 0, 0) != 0)
    {
        rt_kprintf("mount fs on tf card to / fail\n");
        rt_kprintf("sd card might not be formatted or is corrupted.\n");
        return;
    }

    rt_kprintf("mount fs on tf card to / success\n");
}

/* ------------------------------------------------------------------ *
 * Frame-size parsing and buffer-size computation
 * ------------------------------------------------------------------ */

/**
 * @brief Convert a frame-size name (e.g. "VGA") into the framesize_t enum.
 *
 * @param str is the frame-size string to look up.
 *
 * @return Return the matching framesize_t, or FRAMESIZE_INVALID if unknown.
 */
static framesize_t format_string_to_framesize(const char *str)
{
    if (strcmp(str, "QQVGA") == 0)      return FRAMESIZE_QQVGA;
    else if (strcmp(str, "QCIF") == 0)  return FRAMESIZE_QCIF;
    else if (strcmp(str, "QVGA") == 0)  return FRAMESIZE_QVGA;
    else if (strcmp(str, "CIF") == 0)   return FRAMESIZE_CIF;
    else if (strcmp(str, "VGA") == 0)   return FRAMESIZE_VGA;
    else if (strcmp(str, "SVGA") == 0)  return FRAMESIZE_SVGA;
    else if (strcmp(str, "XGA") == 0)   return FRAMESIZE_XGA;
    else if (strcmp(str, "HD") == 0)    return FRAMESIZE_HD;
    else if (strcmp(str, "SXGA") == 0)  return FRAMESIZE_SXGA;
    else if (strcmp(str, "UXGA") == 0)  return FRAMESIZE_UXGA;
    else                                return FRAMESIZE_INVALID;
}

/**
 * @brief Resolve a framesize_t into pixel width and height.
 *
 * @param size   is the input framesize_t.
 * @param width  is the output pointer that receives the width in pixels.
 * @param height is the output pointer that receives the height in pixels.
 *
 * @return Return RT_EOK on success, or -RT_EINVAL when @p size is unknown.
 */
static int framesize_to_resolution(framesize_t size, uint16_t *width, uint16_t *height)
{
    if (width == RT_NULL || height == RT_NULL)
    {
        return -RT_EINVAL;
    }

    switch (size)
    {
        case FRAMESIZE_QQVGA: *width = 160;  *height = 120;  break;
        case FRAMESIZE_QCIF:  *width = 176;  *height = 144;  break;
        case FRAMESIZE_QVGA:  *width = 320;  *height = 240;  break;
        case FRAMESIZE_CIF:   *width = 400;  *height = 296;  break;
        case FRAMESIZE_VGA:   *width = 640;  *height = 480;  break;
        case FRAMESIZE_SVGA:  *width = 800;  *height = 600;  break;
        case FRAMESIZE_XGA:   *width = 1024; *height = 768;  break;
        case FRAMESIZE_HD:    *width = 1280; *height = 720;  break;
        case FRAMESIZE_SXGA:  *width = 1280; *height = 1024; break;
        case FRAMESIZE_UXGA:  *width = 1600; *height = 1200; break;
        default:
            return -RT_EINVAL;
    }

    return RT_EOK;
}

/**
 * @brief Estimate a reasonable JPEG output buffer size for a given resolution.
 *
 * JPEG output length is variable; we use width * height as a rough upper
 * bound (about 1 byte per pixel for typical photographic content) and clamp
 * it to the [JPEG_MIN_BUFFER_SIZE, JPEG_MAX_BUFFER_SIZE] range.
 *
 * @param size is the target framesize_t.
 *
 * @return Return the buffer size in bytes (never zero).
 */
static rt_size_t calc_jpeg_buffer_size(framesize_t size)
{
    uint16_t width  = 0;
    uint16_t height = 0;
    if (framesize_to_resolution(size, &width, &height) != RT_EOK)
    {
        return JPEG_MAX_BUFFER_SIZE;
    }

    rt_size_t buf_size = (rt_size_t)width * (rt_size_t)height;
    if (buf_size < JPEG_MIN_BUFFER_SIZE)
    {
        buf_size = JPEG_MIN_BUFFER_SIZE;
    }
    if (buf_size > JPEG_MAX_BUFFER_SIZE)
    {
        buf_size = JPEG_MAX_BUFFER_SIZE;
    }
    return buf_size;
}

static rt_bool_t caps_has_pixformat(const camera_capabilities_t *caps, pixformat_t fmt)
{
    rt_uint8_t i;
    if (caps == RT_NULL || caps->pixformats == RT_NULL)
    {
        return RT_FALSE;
    }
    for (i = 0; i < caps->num_pixformats; i++)
    {
        if (caps->pixformats[i] == fmt)
        {
            return RT_TRUE;
        }
    }
    return RT_FALSE;
}

static rt_bool_t caps_has_framesize(const camera_capabilities_t *caps, framesize_t size)
{
    rt_uint8_t i;
    if (caps == RT_NULL || caps->framesizes == RT_NULL)
    {
        return RT_FALSE;
    }
    for (i = 0; i < caps->num_framesizes; i++)
    {
        if (caps->framesizes[i] == size)
        {
            return RT_TRUE;
        }
    }
    return RT_FALSE;
}

/* ------------------------------------------------------------------ *
 * SD-card output path helpers
 * ------------------------------------------------------------------ */

/**
 * @brief Create the photo directory if it does not exist.
 *
 * @return Return RT_EOK on success or when the directory already exists.
 */
static int ensure_photo_dir(void)
{
    int ret = mkdir(PHOTO_DIR, 0);
    if (ret == 0)
    {
        return RT_EOK;
    }

    int err = rt_get_errno();
    if (ret < 0 && (err == EEXIST || err == -EEXIST))
    {
        return RT_EOK;
    }

    return -RT_ERROR;
}

/* ------------------------------------------------------------------ *
 * MSH command: take_photo
 * ------------------------------------------------------------------ */

/**
 * @brief MSH command: capture one or more JPEG frames and save each as
 *        /photo/photo_NNN.jpg on the SD card.
 *
 * Usage:
 *   take_photo <framesize> <quality> <count>
 *
 *   framesize : QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA
 *   quality   : JPEG quality (0 = best, 63 = most compressed)
 *   count     : number of frames to capture (>= 1)
 *
 * Example:
 *   take_photo VGA 10 3
 *
 * @param argc is the argument count (4 including the command name).
 * @param argv is the argument vector.
 */
void take_photo(int argc, char **argv)
{
    camera_handler_instance_t     *camera_instance = RT_NULL;
    const camera_capabilities_t   *caps = RT_NULL;
    camera_capture_config_t        cfg;
    camera_capture_request_t       req;
    camera_handle_status_t         status;
    uint8_t                       *buffer = RT_NULL;
    rt_size_t                      buffer_size;
    int                            quality;
    int                            count;

    if (argc != 4)
    {
        rt_kprintf("Usage: take_photo <framesize> <quality> <count>\n");
        rt_kprintf("Framesize options: QQVGA, QCIF, QVGA, CIF, VGA, SVGA, XGA, HD, SXGA, UXGA\n");
        rt_kprintf("quality: 0 (highest) to 63 (lowest)\n");
        rt_kprintf("count: number of photos to capture (>=1)\n");
        rt_kprintf("Example: take_photo VGA 10 3\n");
        return;
    }

    framesize_t framesize = format_string_to_framesize(argv[1]);
    if (framesize == FRAMESIZE_INVALID)
    {
        rt_kprintf("Unsupported framesize: %s\n", argv[1]);
        return;
    }

    quality = atoi(argv[2]);
    if (quality < 0 || quality > 63)
    {
        rt_kprintf("Quality must be between 0 and 63\n");
        return;
    }

    count = atoi(argv[3]);
    if (count <= 0)
    {
        rt_kprintf("Count must be >= 1\n");
        return;
    }

    if (ensure_photo_dir() != RT_EOK)
    {
        rt_kprintf("Failed to create or access %s\n", PHOTO_DIR);
        return;
    }

    /* 1) Initialise the camera handler instance.  The driver is selected at
     *    compile time via Kconfig (SENSOR_USING_OV2640) and the RT-Thread
     *    device is registered internally. */
    status = camera_handler_instance_init(&camera_instance);
    if (status != CAMERA_OK)
    {
        rt_kprintf("Failed to initialize camera handler (%d)\n", status);
        return;
    }

    status = camera_get_capabilities(camera_instance, &caps);
    if (status != CAMERA_OK || caps == RT_NULL)
    {
        rt_kprintf("Failed to query camera capabilities (%d)\n", status);
        goto close_camera;
    }
    if (!caps_has_pixformat(caps, PIXFORMAT_JPEG))
    {
        rt_kprintf("Camera does not support PIXFORMAT_JPEG\n");
        goto close_camera;
    }
    if (!caps_has_framesize(caps, framesize))
    {
        rt_kprintf("Camera does not support requested framesize: %s\n", argv[1]);
        goto close_camera;
    }

    /* 2) Push JPEG + framesize + quality configuration. The handle layer
     *    inserts a 500 ms AEC/AWB settle delay internally. */
    cfg.pixformat = PIXFORMAT_JPEG;
    cfg.framesize = framesize;
    cfg.quality   = quality;
    status = camera_change_settings(camera_instance, &cfg);
    if (status != CAMERA_OK)
    {
        rt_kprintf("Failed to configure camera (%d)\n", status);
        goto close_camera;
    }

    /* 4) Allocate the JPEG frame buffer in PSRAM. */
    buffer_size = (caps->max_buffer_size != 0) ?
                  caps->max_buffer_size :
                  calc_jpeg_buffer_size(framesize);
    buffer = psram_heap_malloc(buffer_size);
    if (buffer == RT_NULL)
    {
        rt_kprintf("Failed to allocate %u bytes for JPEG capture!\n",
                   (unsigned int)buffer_size);
        goto close_camera;
    }

    rt_kprintf("JPEG capture: framesize=%s, quality=%d, buffer=%u bytes @ %p\n",
               argv[1], quality, (unsigned int)buffer_size, buffer);

    /* 5) Grab `count` frames and write each one as a numbered .jpg file. */
    for (int photo_idx = 0; photo_idx < count; photo_idx++)
    {
        req.buffer      = buffer;
        req.buffer_size = buffer_size;
        req.frame_size  = 0;

        status = camera_capture_single(camera_instance, &req);
        if (status != CAMERA_OK || req.frame_size == 0)
        {
            rt_kprintf("Capture failed or timed out (index=%d, status=%d)\n",
                       photo_idx, status);
            continue;
        }

        char file_path[64];
        rt_snprintf(file_path, sizeof(file_path),
                    "%s/photo_%03d.jpg", PHOTO_DIR, photo_idx + 1);
        int fd = open(file_path, O_WRONLY | O_CREAT | O_TRUNC, 0);
        if (fd < 0)
        {
            rt_kprintf("Failed to open %s for writing\n", file_path);
            continue;
        }

        int written = write(fd, buffer, req.frame_size);
        close(fd);
        if (written != (int)req.frame_size)
        {
            rt_kprintf("Write failed for %s (%d/%u bytes)\n",
                       file_path, written, (unsigned int)req.frame_size);
            continue;
        }

        rt_kprintf("Saved %s (%u bytes)\n", file_path,
                   (unsigned int)req.frame_size);
    }

    psram_heap_free(buffer);
    buffer = RT_NULL;

close_camera:
    camera_deinit(&camera_instance);
    if (buffer != RT_NULL)
    {
        psram_heap_free(buffer);
    }
}
MSH_CMD_EXPORT(take_photo, Capture JPEG photo(s) using ov2640 and save to SD card);

/**
 * @brief Program entry point: initialize the PSRAM heap, mount the SD card
 *        and idle, waiting for MSH commands.
 *
 * @return Return 0 (never actually returns; the main loop spins forever).
 */
int main(void)
{
    rt_kprintf("OV2640 Camera Take Photo to SD Card Example\n");
    psram_heap_init();
    sdcard_init();

    while (1)
    {
        rt_thread_mdelay(1000);
    }
}
