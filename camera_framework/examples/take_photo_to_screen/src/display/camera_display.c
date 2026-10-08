#include "camera_display.h"

#include <bf0_hal.h>
#include <drv_epic.h>
#include <finsh.h>
#include <mem_section.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <stdlib.h>
#include <string.h>

#include "photo_decoder.h"
#include "photo_overlay.h"
#include "preview_geometry.h"
#include "../camera/camera_profile.h"

#define CAMERA_DECODE_WIDTH 640U
#define CAMERA_DECODE_HEIGHT 480U
#define CAMERA_DECODE_BUFFER_SIZE \
    (CAMERA_DECODE_WIDTH * CAMERA_DECODE_HEIGHT * 2U)

L2_NON_RET_BSS_SECT_BEGIN(camera_display_buffers)
L2_NON_RET_BSS_SECT(camera_display_buffers,
                    ALIGN(64) static uint8_t jpeg_decode_buffer[
                        CAMERA_DECODE_BUFFER_SIZE]);
L2_NON_RET_BSS_SECT_END

static rt_device_t preview_lcd;
static struct rt_device_graphic_ops *preview_ops;
static uint8_t *preview_buffer;
static rt_bool_t jpeg_info_printed;

#define CAMERA_DISPLAY_TIMING_INTERVAL_MS 5000U

typedef struct
{
    rt_tick_t window_start;
    uint32_t samples;
    uint32_t src_clean_ms;
    uint32_t blend_ms;
    uint32_t wait_ms;
    uint32_t invalidate_ms;
    uint32_t dst_clean_ms;
    uint32_t push_ms;
} camera_display_timing_t;

static camera_display_timing_t display_timing;
static camera_display_timing_t display_stage;

/*
 * Runtime state of the profile options, exposed as
 *   camera_preview [rotate 0|1] [background 0|1] [swap 0|1]
 * so a new camera/backend can be checked on hardware without rebuilding.
 */
static rt_bool_t preview_rotate = (CAMERA_PROFILE_ROTATE != 0);
static rt_bool_t preview_background =
    (CAMERA_PROFILE_BACKGROUND != 0);
static rt_bool_t preview_swap = (CAMERA_PROFILE_DISPLAY_SWAP != 0);

static uint32_t camera_display_ms(rt_tick_t ticks)
{
    return (uint32_t)((uint64_t)ticks * 1000U / RT_TICK_PER_SECOND);
}

static void camera_display_report_timing(void)
{
    rt_tick_t now = rt_tick_get();

    display_timing.src_clean_ms += display_stage.src_clean_ms;
    display_timing.blend_ms += display_stage.blend_ms;
    display_timing.wait_ms += display_stage.wait_ms;
    display_timing.invalidate_ms += display_stage.invalidate_ms;
    display_timing.dst_clean_ms += display_stage.dst_clean_ms;
    display_timing.push_ms += display_stage.push_ms;
    display_timing.samples++;
    if ((display_timing.samples < 2U) ||
        ((rt_tick_t)(now - display_timing.window_start) <
         rt_tick_from_millisecond(CAMERA_DISPLAY_TIMING_INTERVAL_MS)))
        return;

    rt_kprintf("camera: stage n=%u clean=%u blend=%u wait=%u inv=%u "
               "dclean=%u push=%u ms\n",
               (unsigned)display_timing.samples,
               (unsigned)(display_timing.src_clean_ms / display_timing.samples),
               (unsigned)(display_timing.blend_ms / display_timing.samples),
               (unsigned)(display_timing.wait_ms / display_timing.samples),
               (unsigned)(display_timing.invalidate_ms /
                          display_timing.samples),
               (unsigned)(display_timing.dst_clean_ms / display_timing.samples),
               (unsigned)(display_timing.push_ms / display_timing.samples));
    display_timing = (camera_display_timing_t){.window_start = now};
}

static int camera_display_submit(void)
{
    rt_tick_t started = rt_tick_get();
    rt_tick_t after_clean;

    mpu_dcache_clean(preview_buffer, CAMERA_DISPLAY_BUFFER_SIZE);
    after_clean = rt_tick_get();
    preview_ops->set_window(0, 0, CAMERA_DISPLAY_WIDTH - 1,
                            CAMERA_DISPLAY_HEIGHT - 1);
    preview_ops->draw_rect((const char *)preview_buffer, 0, 0,
                           CAMERA_DISPLAY_WIDTH - 1,
                           CAMERA_DISPLAY_HEIGHT - 1);
    display_stage.dst_clean_ms = camera_display_ms(after_clean - started);
    display_stage.push_ms = camera_display_ms(rt_tick_get() - after_clean);
    return 0;
}

int camera_display_open(uint8_t *buffer, uint32_t buffer_size)
{
    struct rt_device_graphic_info info;
    rt_uint16_t format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
    rt_uint8_t brightness = 100;

    if ((buffer == RT_NULL) || (buffer_size < CAMERA_DISPLAY_BUFFER_SIZE))
        return -1;

    preview_lcd = rt_device_find("lcd");
    if ((preview_lcd == RT_NULL) ||
        (rt_device_open(preview_lcd, RT_DEVICE_OFLAG_RDWR) != RT_EOK))
        goto error;
    if ((rt_device_control(preview_lcd, RTGRAPHIC_CTRL_GET_INFO, &info) != RT_EOK) ||
        (info.width != CAMERA_DISPLAY_WIDTH) ||
        (info.height != CAMERA_DISPLAY_HEIGHT) ||
        (rt_device_control(preview_lcd, RTGRAPHIC_CTRL_SET_BUF_FORMAT,
                           &format) != RT_EOK) ||
        (rt_device_control(preview_lcd, RTGRAPHIC_CTRL_SET_BRIGHTNESS,
                           &brightness) != RT_EOK))
        goto error;

    preview_ops = rt_graphix_ops(preview_lcd);
    if ((preview_ops == RT_NULL) || (preview_ops->set_window == RT_NULL) ||
        (preview_ops->draw_rect == RT_NULL))
        goto error;

    preview_buffer = buffer;
    memset(preview_buffer, 0, CAMERA_DISPLAY_BUFFER_SIZE);
    return camera_display_submit();

error:
    camera_display_close();
    return -1;
}

static int camera_display_render(const camera_frame_t *display_frame)
{
    EPIC_LayerConfigTypeDef input[2];
    EPIC_LayerConfigTypeDef output;
    EPIC_AreaTypeDef transformed_area;
    EPIC_PointTypeDef pivot;
    preview_geometry_t geometry;
    rt_err_t result;
    rt_tick_t started;
    rt_tick_t after_clean;
    rt_tick_t after_blend;
    rt_tick_t after_wait;

    if ((preview_lcd == RT_NULL) || (preview_buffer == RT_NULL) ||
        (display_frame == RT_NULL) || (display_frame->data == RT_NULL) ||
        (display_frame->width == 0U) || (display_frame->height == 0U) ||
        (display_frame->size != (uint32_t)display_frame->width *
                                display_frame->height * 2U) ||
        (display_frame->format != CAMERA_PIXEL_RGB565_BE))
        return -1;
    if (preview_geometry_calculate(display_frame->width,
                                   display_frame->height,
                                   CAMERA_DISPLAY_WIDTH,
                                   CAMERA_DISPLAY_HEIGHT,
                                   preview_rotate ? 900U : 0U,
                                   &geometry) != 0)
        return -1;

    HAL_EPIC_LayerConfigInit(&input[0]);
    input[0].data = preview_buffer;
    input[0].color_mode = EPIC_COLOR_RGB565;
    input[0].width = CAMERA_DISPLAY_WIDTH;
    input[0].total_width = CAMERA_DISPLAY_WIDTH;
    input[0].height = CAMERA_DISPLAY_HEIGHT;
    input[0].data_size = CAMERA_DISPLAY_BUFFER_SIZE;
    input[0].alpha = 255;

    HAL_EPIC_LayerConfigInit(&input[1]);
    input[1].data = display_frame->data;
    input[1].color_mode = preview_swap ? EPIC_INPUT_RGB565_SWAP
                                       : EPIC_INPUT_RGB565;
    input[1].width = display_frame->width;
    input[1].total_width = display_frame->width;
    input[1].height = display_frame->height;
    input[1].data_size = display_frame->size;
    input[1].alpha = 255;
    input[1].transform_cfg.angle = geometry.angle_tenths;
    input[1].transform_cfg.scale_x = geometry.epic_scale;
    input[1].transform_cfg.scale_y = geometry.epic_scale;
    input[1].transform_cfg.pivot_x = display_frame->width / 2U;
    input[1].transform_cfg.pivot_y = display_frame->height / 2U;

    pivot.x = input[1].transform_cfg.pivot_x;
    pivot.y = input[1].transform_cfg.pivot_y;
    EPIC_GetTransformedArea(&transformed_area, input[1].width, input[1].height,
                            input[1].transform_cfg.angle,
                            input[1].transform_cfg.scale_x,
                            input[1].transform_cfg.scale_y, &pivot);
    input[1].x_offset = (int16_t)(-transformed_area.x0 - geometry.crop_x);
    input[1].y_offset = (int16_t)(-transformed_area.y0 - geometry.crop_y);

    HAL_EPIC_LayerConfigInit(&output);
    output.data = preview_buffer;
    output.color_mode = EPIC_COLOR_RGB565;
    output.width = CAMERA_DISPLAY_WIDTH;
    output.total_width = CAMERA_DISPLAY_WIDTH;
    output.height = CAMERA_DISPLAY_HEIGHT;
    output.data_size = CAMERA_DISPLAY_BUFFER_SIZE;
    output.alpha = 255;

    started = rt_tick_get();
    mpu_dcache_clean(display_frame->data, display_frame->size);
    after_clean = rt_tick_get();
    if (preview_background)
        result = drv_epic_blend(input, 2, &output, RT_NULL);
    else
        result = drv_epic_blend(&input[1], 1, &output, RT_NULL);
    after_blend = rt_tick_get();
    if (result != RT_EOK)
        return -1;
    if (drv_epic_wait_done() != RT_EOK)
        return -1;
    after_wait = rt_tick_get();
    mpu_dcache_invalidate(preview_buffer, CAMERA_DISPLAY_BUFFER_SIZE);
    display_stage.src_clean_ms = camera_display_ms(after_clean - started);
    display_stage.blend_ms = camera_display_ms(after_blend - after_clean);
    display_stage.wait_ms = camera_display_ms(after_wait - after_blend);
    display_stage.invalidate_ms =
        camera_display_ms(rt_tick_get() - after_wait);
    return 0;
}

static int camera_display_decode(const uint8_t *jpeg, uint32_t jpeg_size,
                                 camera_frame_t *decoded_frame,
                                 uint32_t *decode_ms)
{
    const photo_decoder_backend_t *decoder = photo_decoder_backend();
    uint16_t width = 0U;
    uint16_t height = 0U;
    uint32_t elapsed_ms = 0U;
    int result;

    if ((jpeg == RT_NULL) || (jpeg_size == 0U) || (decoded_frame == RT_NULL))
        return PHOTO_DECODER_UNSUPPORTED;
    result = decoder->decode(jpeg, jpeg_size, jpeg_decode_buffer,
                             sizeof(jpeg_decode_buffer), &width, &height,
                             &elapsed_ms);
    if (!jpeg_info_printed)
    {
        rt_kprintf("camera: jpeg decoder=%s result=%d dim=%ux%u\n",
                   decoder->name, result, width, height);
        jpeg_info_printed = RT_TRUE;
    }
    if (result != PHOTO_DECODER_OK)
        return result;
    if (decode_ms != RT_NULL)
        *decode_ms = elapsed_ms;
    *decoded_frame = (camera_frame_t){
        .data = jpeg_decode_buffer,
        .size = (uint32_t)width * height * 2U,
        .width = width,
        .height = height,
        .format = CAMERA_PIXEL_RGB565_BE,
    };
    return PHOTO_DECODER_OK;
}

int camera_display_show(const camera_frame_t *frame, uint32_t *decode_ms,
                        uint32_t *write_ms)
{
    camera_frame_t decoded_frame;
    rt_tick_t started;

    if (frame == RT_NULL)
        return -1;
    if (decode_ms != RT_NULL)
        *decode_ms = 0U;
    if (write_ms != RT_NULL)
        *write_ms = 0U;
    if (frame->format == CAMERA_PIXEL_JPEG)
    {
        if (camera_display_decode(frame->data, frame->size, &decoded_frame,
                                  decode_ms) != PHOTO_DECODER_OK)
            return -1;
        frame = &decoded_frame;
    }

    started = rt_tick_get();
    if (camera_display_render(frame) != 0)
        return -1;
    if (camera_display_submit() != 0)
        return -1;
    camera_display_report_timing();
    if (write_ms != RT_NULL)
        *write_ms = (uint32_t)((rt_tick_get() - started) * 1000U /
                               RT_TICK_PER_SECOND);
    return 0;
}

int camera_display_show_photo(const uint8_t *jpeg, uint32_t jpeg_size,
                              const char *filename, uint32_t *decode_ms)
{
    camera_frame_t decoded_frame;

    if (decode_ms != RT_NULL)
        *decode_ms = 0U;
    if ((filename == RT_NULL) ||
        (camera_display_decode(jpeg, jpeg_size, &decoded_frame,
                               decode_ms) !=
         PHOTO_DECODER_OK))
        return PHOTO_DECODER_UNSUPPORTED;
    if (camera_display_render(&decoded_frame) != 0)
        return -1;
    photo_overlay_draw(preview_buffer, CAMERA_DISPLAY_WIDTH,
                       CAMERA_DISPLAY_HEIGHT, filename, RT_NULL);
    return camera_display_submit();
}

int camera_display_show_message(const char *filename, const char *message)
{
    if ((preview_buffer == RT_NULL) || (filename == RT_NULL) ||
        (message == RT_NULL))
        return -1;
    memset(preview_buffer, 0, CAMERA_DISPLAY_BUFFER_SIZE);
    if (photo_overlay_draw(preview_buffer, CAMERA_DISPLAY_WIDTH,
                           CAMERA_DISPLAY_HEIGHT, filename, message) != 0)
        return -1;
    return camera_display_submit();
}

void camera_display_close(void)
{
    if (preview_lcd != RT_NULL)
        rt_device_close(preview_lcd);
    preview_lcd = RT_NULL;
    preview_ops = RT_NULL;
    preview_buffer = RT_NULL;
}

static int camera_display_options_cmd(int argc, char **argv)
{
    if (argc >= 2)
        preview_rotate = (atoi(argv[1]) != 0) ? RT_TRUE : RT_FALSE;
    if (argc >= 3)
        preview_background = (atoi(argv[2]) != 0) ? RT_TRUE : RT_FALSE;
    if (argc >= 4)
        preview_swap = (atoi(argv[3]) != 0) ? RT_TRUE : RT_FALSE;
    rt_kprintf("camera: preview rotate=%d background=%d swap=%d\n",
               (int)preview_rotate, (int)preview_background,
               (int)preview_swap);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(camera_display_options_cmd, camera_preview,
                     "set preview rotate/background/swap flags");
