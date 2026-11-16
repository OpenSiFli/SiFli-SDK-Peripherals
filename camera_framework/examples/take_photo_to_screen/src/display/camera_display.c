#include "camera_display.h"

#include <bf0_hal.h>
#include <finsh.h>
#include <mem_section.h>
#include <rtconfig.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <stdlib.h>
#include <string.h>

#include "photo_decoder.h"
#include "photo_overlay.h"
#include "../camera/camera_profile.h"

/*
 * The panel size comes from the generated rtconfig.h, not from the preview
 * frame, so switching the panel in proj.conf resizes these buffers. A RAMLESS
 * panel (every DPI panel on LB57) is scanned as a complete frame, so the buffer
 * is always one full panel.
 */
#define CAMERA_DISPLAY_WIDTH LCD_HOR_RES_MAX
#define CAMERA_DISPLAY_HEIGHT LCD_VER_RES_MAX
#define CAMERA_DISPLAY_BUFFER_SIZE \
    (CAMERA_DISPLAY_WIDTH * CAMERA_DISPLAY_HEIGHT * 2U)
#define CAMERA_DISPLAY_BUFFER_COUNT 2U
#define CAMERA_DISPLAY_PUSH_TIMEOUT_MS 5000U

/*
 * Full-screen canvases live in the dedicated PSRAM2 framebuffer section, like
 * ai_camera's LVGL buffers. Nothing here competes with the camera frames in
 * PSRAM1, and two canvases let the CPU render the next frame while the LCDC
 * scans the previous one.
 */
L2_NON_RET_BSS_SECT_BEGIN(frambuf)
L2_NON_RET_BSS_SECT(frambuf,
                    ALIGN(64) static uint8_t preview_buffers[
                        CAMERA_DISPLAY_BUFFER_COUNT][CAMERA_DISPLAY_BUFFER_SIZE]);
L2_NON_RET_BSS_SECT_END

static rt_device_t preview_lcd;
static struct rt_device_graphic_ops *preview_ops;
static uint8_t *preview_buffer;
static uint16_t panel_width;
static uint16_t panel_height;
static uint16_t preview_width;
static uint16_t preview_height;
static uint16_t placed_width[CAMERA_DISPLAY_BUFFER_COUNT];
static uint16_t placed_height[CAMERA_DISPLAY_BUFFER_COUNT];
static uint8_t back_index;

static struct rt_semaphore buffers_free;
static rt_bool_t buffers_free_initialized;

static rt_bool_t preview_swap = (CAMERA_PROFILE_DISPLAY_SWAP != 0);

#define CAMERA_DISPLAY_TIMING_INTERVAL_MS 5000U

typedef struct
{
    rt_tick_t window_start;
    uint32_t samples;
    uint32_t blend_ms;
    uint32_t wait_ms;
    uint32_t dst_clean_ms;
    uint32_t push_ms;
} camera_display_timing_t;

static camera_display_timing_t display_timing;
static camera_display_timing_t display_stage;

static uint32_t camera_display_ms(rt_tick_t ticks)
{
    return (uint32_t)((uint64_t)ticks * 1000U / RT_TICK_PER_SECOND);
}

static uint32_t camera_display_preview_bytes(void)
{
    return (uint32_t)preview_width * preview_height * 2U;
}

static const char *camera_display_format_name(uint8_t format)
{
    switch (format)
    {
        case RTGRAPHIC_PIXEL_FORMAT_RGB332: return "rgb332";
        case RTGRAPHIC_PIXEL_FORMAT_RGB565: return "rgb565";
        case RTGRAPHIC_PIXEL_FORMAT_RGB666: return "rgb666";
        case RTGRAPHIC_PIXEL_FORMAT_RGB888: return "rgb888";
        default: return "unknown";
    }
}

static void camera_display_report_timing(void)
{
    rt_tick_t now = rt_tick_get();

    display_timing.blend_ms += display_stage.blend_ms;
    display_timing.wait_ms += display_stage.wait_ms;
    display_timing.dst_clean_ms += display_stage.dst_clean_ms;
    display_timing.push_ms += display_stage.push_ms;
    display_timing.samples++;
    if ((display_timing.samples < 2U) ||
        ((rt_tick_t)(now - display_timing.window_start) <
         rt_tick_from_millisecond(CAMERA_DISPLAY_TIMING_INTERVAL_MS)))
        return;

    rt_kprintf("camera: stage n=%u dclean=%u blend=%u wait=%u push=%u ms\n",
               (unsigned)display_timing.samples,
               (unsigned)(display_timing.dst_clean_ms / display_timing.samples),
               (unsigned)(display_timing.blend_ms / display_timing.samples),
               (unsigned)(display_timing.wait_ms / display_timing.samples),
               (unsigned)(display_timing.push_ms / display_timing.samples));
    display_timing = (camera_display_timing_t){.window_start = now};
}

static rt_err_t camera_display_push_complete(rt_device_t dev, void *buffer)
{
    (void)dev;
    (void)buffer;
    (void)rt_sem_release(&buffers_free);
    return RT_EOK;
}

/*
 * Copy the frame into the centre of the panel canvas at its native size.
 *
 * There is no scaling: a 640x480 frame keeps every pixel and the rest of the
 * canvas stays black. A frame larger than the panel is centre-cropped, which
 * still avoids the per-frame EPIC scaling cost.
 */
static int camera_display_blit(const camera_frame_t *frame)
{
    uint16_t copy_width = (frame->width < preview_width) ? frame->width
                                                         : preview_width;
    uint16_t copy_height = (frame->height < preview_height) ? frame->height
                                                            : preview_height;
    uint16_t source_x = (uint16_t)((frame->width - copy_width) / 2U);
    uint16_t source_y = (uint16_t)((frame->height - copy_height) / 2U);
    uint16_t dest_x = (uint16_t)((preview_width - copy_width) / 2U);
    uint16_t dest_y = (uint16_t)((preview_height - copy_height) / 2U);
    uint16_t row;

    if ((copy_width == 0U) || (copy_height == 0U))
        return -1;

    /* A size change leaves old pixels outside the new centre rectangle. */
    if ((copy_width != placed_width[back_index]) ||
        (copy_height != placed_height[back_index]))
    {
        memset(preview_buffer, 0, camera_display_preview_bytes());
        placed_width[back_index] = copy_width;
        placed_height[back_index] = copy_height;
    }

    for (row = 0U; row < copy_height; row++)
    {
        const uint8_t *source = frame->data +
                                ((uint32_t)(source_y + row) * frame->width +
                                 source_x) * 2U;
        uint8_t *destination = preview_buffer +
                               ((uint32_t)(dest_y + row) * preview_width +
                                dest_x) * 2U;

        if (!preview_swap)
        {
            memcpy(destination, source, (uint32_t)copy_width * 2U);
        }
        else
        {
            uint16_t column;

            for (column = 0U; column < copy_width; column++)
            {
                destination[column * 2U] = source[column * 2U + 1U];
                destination[column * 2U + 1U] = source[column * 2U];
            }
        }
    }
    return 0;
}

/*
 * Hand one complete canvas to the panel. The draw is asynchronous: the
 * completion callback releases the canvas, so the CPU can render into the
 * other one while this one is being scanned.
 */
static int camera_display_submit(rt_bool_t cpu_written)
{
    rt_tick_t started = rt_tick_get();
    rt_tick_t after_clean = started;

    if ((preview_buffer == RT_NULL) || (preview_ops == RT_NULL) ||
        (preview_width == 0U) || (preview_height == 0U))
        return -1;

    if (cpu_written)
    {
        mpu_dcache_clean(preview_buffer, camera_display_preview_bytes());
        after_clean = rt_tick_get();
    }
    if (preview_ops->set_window != RT_NULL)
        preview_ops->set_window(0, 0, preview_width - 1, preview_height - 1);

    if (preview_ops->draw_rect_async != RT_NULL)
    {
        preview_ops->draw_rect_async((const char *)preview_buffer, 0, 0,
                                     preview_width - 1, preview_height - 1);
    }
    else
    {
        preview_ops->draw_rect((const char *)preview_buffer, 0, 0,
                               preview_width - 1, preview_height - 1);
        (void)rt_sem_release(&buffers_free);
    }
    display_stage.dst_clean_ms = camera_display_ms(after_clean - started);
    display_stage.push_ms = camera_display_ms(rt_tick_get() - after_clean);
    return 0;
}

int camera_display_open(void)
{
    struct rt_device_graphic_info info;
    rt_uint16_t format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
    rt_uint8_t brightness = 100;
    uint32_t index;

    preview_lcd = rt_device_find("lcd");
    if ((preview_lcd == RT_NULL) ||
        (rt_device_open(preview_lcd, RT_DEVICE_OFLAG_RDWR) != RT_EOK))
        goto error;
    if (rt_device_control(preview_lcd, RTGRAPHIC_CTRL_GET_INFO, &info) != RT_EOK)
        goto error;
    if ((info.width == 0U) || (info.height == 0U) ||
        (info.width > CAMERA_DISPLAY_WIDTH) ||
        (info.height > CAMERA_DISPLAY_HEIGHT))
        goto error;

    if ((rt_device_control(preview_lcd, RTGRAPHIC_CTRL_SET_BUF_FORMAT,
                           &format) != RT_EOK) ||
        (rt_device_control(preview_lcd, RTGRAPHIC_CTRL_SET_BRIGHTNESS,
                           &brightness) != RT_EOK))
        goto error;

    preview_ops = rt_graphix_ops(preview_lcd);
    if ((preview_ops == RT_NULL) || (preview_ops->set_window == RT_NULL) ||
        (preview_ops->draw_rect == RT_NULL))
        goto error;

    if (rt_sem_init(&buffers_free, "cam_lcd", CAMERA_DISPLAY_BUFFER_COUNT,
                    RT_IPC_FLAG_FIFO) != RT_EOK)
        goto error;
    buffers_free_initialized = RT_TRUE;
    if (rt_device_set_tx_complete(preview_lcd,
                                  camera_display_push_complete) != RT_EOK)
        goto error;

    panel_width = info.width;
    panel_height = info.height;
    preview_width = panel_width;
    preview_height = panel_height;
    back_index = 0U;
    memset(placed_width, 0, sizeof(placed_width));
    memset(placed_height, 0, sizeof(placed_height));

    rt_kprintf("camera: panel %ux%u fmt=%s align=%u, buffer %ux%u (%u bytes x %u)\n",
               (unsigned)panel_width, (unsigned)panel_height,
               camera_display_format_name(info.pixel_format),
               (unsigned)info.draw_align,
               (unsigned)preview_width, (unsigned)preview_height,
               (unsigned)CAMERA_DISPLAY_BUFFER_SIZE,
               (unsigned)CAMERA_DISPLAY_BUFFER_COUNT);

    /*
     * Black is the border colour. Both canvases are cleared and written back
     * once; the per-frame copy only touches the centred rectangle.
     */
    for (index = 0U; index < CAMERA_DISPLAY_BUFFER_COUNT; index++)
    {
        memset(preview_buffers[index], 0, CAMERA_DISPLAY_BUFFER_SIZE);
        mpu_dcache_clean(preview_buffers[index], CAMERA_DISPLAY_BUFFER_SIZE);
    }
    return 0;

error:
    camera_display_close();
    return -1;
}

int camera_display_show(const camera_frame_t *frame, uint32_t *decode_ms,
                        uint32_t *write_ms)
{
    rt_tick_t wait_started;
    rt_tick_t blit_started;

    if (frame == RT_NULL)
        return -1;
    if (decode_ms != RT_NULL)
        *decode_ms = 0U;
    if (write_ms != RT_NULL)
        *write_ms = 0U;
    if ((frame->format != CAMERA_PIXEL_RGB565_BE) ||
        (frame->data == RT_NULL) || (frame->width == 0U) ||
        (frame->height == 0U) ||
        (frame->size != (uint32_t)frame->width * frame->height * 2U))
        return -1;

    /*
     * Wait for a canvas the LCDC is no longer scanning. At most two pushes are
     * outstanding, so a frame is normally free immediately.
     */
    wait_started = rt_tick_get();
    if (rt_sem_take(&buffers_free,
                    rt_tick_from_millisecond(
                        CAMERA_DISPLAY_PUSH_TIMEOUT_MS)) != RT_EOK)
    {
        rt_kprintf("camera: no free display buffer\n");
        return -1;
    }
    display_stage.wait_ms = camera_display_ms(rt_tick_get() - wait_started);

    preview_buffer = preview_buffers[back_index];
    blit_started = rt_tick_get();
    if (camera_display_blit(frame) != 0)
    {
        (void)rt_sem_release(&buffers_free);
        return -1;
    }
    display_stage.blend_ms = camera_display_ms(rt_tick_get() - blit_started);

    if (camera_display_submit(RT_TRUE) != 0)
    {
        (void)rt_sem_release(&buffers_free);
        return -1;
    }
    back_index ^= 1U;
    camera_display_report_timing();
    if (write_ms != RT_NULL)
        *write_ms = (uint32_t)((rt_tick_get() - wait_started) * 1000U /
                               RT_TICK_PER_SECOND);
    return 0;
}

int camera_display_show_photo(const uint8_t *jpeg, uint32_t jpeg_size,
                              const char *filename, uint32_t *decode_ms)
{
    (void)jpeg;
    (void)jpeg_size;
    (void)filename;
    if (decode_ms != RT_NULL)
        *decode_ms = 0U;
    return PHOTO_DECODER_UNSUPPORTED;
}

int camera_display_show_message(const char *filename, const char *message)
{
    if ((filename == RT_NULL) || (message == RT_NULL) ||
        (preview_width == 0U) || (preview_height == 0U))
        return -1;
    if (rt_sem_take(&buffers_free,
                    rt_tick_from_millisecond(
                        CAMERA_DISPLAY_PUSH_TIMEOUT_MS)) != RT_EOK)
        return -1;
    preview_buffer = preview_buffers[back_index];
    memset(preview_buffer, 0, camera_display_preview_bytes());
    if (photo_overlay_draw(preview_buffer, preview_width,
                           preview_height, filename, message) != 0)
    {
        (void)rt_sem_release(&buffers_free);
        return -1;
    }
    if (camera_display_submit(RT_TRUE) != 0)
    {
        (void)rt_sem_release(&buffers_free);
        return -1;
    }
    /* The overlay covered the whole canvas, so force a black clear next use. */
    placed_width[back_index] = 0U;
    placed_height[back_index] = 0U;
    back_index ^= 1U;
    return 0;
}

void camera_display_close(void)
{
    if (preview_lcd != RT_NULL)
    {
        (void)rt_device_set_tx_complete(preview_lcd, RT_NULL);
        if (buffers_free_initialized)
        {
            (void)rt_sem_take(&buffers_free, rt_tick_from_millisecond(100));
            (void)rt_sem_take(&buffers_free, rt_tick_from_millisecond(100));
        }
        rt_device_close(preview_lcd);
    }
    preview_lcd = RT_NULL;
    preview_ops = RT_NULL;
    preview_buffer = RT_NULL;
    panel_width = 0U;
    panel_height = 0U;
    preview_width = 0U;
    preview_height = 0U;
    memset(placed_width, 0, sizeof(placed_width));
    memset(placed_height, 0, sizeof(placed_height));
    back_index = 0U;
    buffers_free_initialized = RT_FALSE;
}

static int camera_display_options_cmd(int argc, char **argv)
{
    if (argc >= 4)
        preview_swap = (atoi(argv[3]) != 0) ? RT_TRUE : RT_FALSE;
    rt_kprintf("camera: preview swap=%d (cpu copy, no rotation/background)\n",
               (int)preview_swap);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(camera_display_options_cmd, camera_preview,
                     "set preview byte swap");
