#include "camera_app.h"

#include <bf0_hal.h>
#include <drv_i2c.h>
#include <rtconfig.h>
#include <cpu_usage_profiler.h>
#include <finsh.h>
#if defined(FINSH_USING_MSH)
#include <msh.h>
#endif
#include <mem_section.h>
#include <rthw.h>
#include <rtthread.h>
#include <stdlib.h>
#include <string.h>

#include "../camera/camera_profile.h"
#include "../camera/frame_pool.h"
#include "../display/camera_display.h"
#include "../image_processing/camera_image_processor.h"
#include "../storage/photo_storage.h"
#include "camera_handle.h"
#include "camera_xclk.h"
#include "camera_isp_flags.h"
#include "camera_metrics.h"

/*
 * Camera acquisition uses the shared camera framework only.
 *
 * The active sensor, its interface and its data bus are selected in menuconfig
 * (SENSOR_USING_*, CAMERA_*_INTERFACE_*, CAMERA_DVP_BACKEND_*), and the runtime
 * pixel format and geometry are read back from camera_get_capabilities(). No
 * project-local sensor driver is referenced any more.
 *
 * Besides the LCD preview the example saves stills to the SD card: the shutter
 * key (or the "camera_photo" console command) queues one capture and the display
 * thread writes it as /IMG_nnnn.JPG (src/storage/photo_storage.c).
 *
 * A sensor with its own JPEG encoder (OV2640) is asked for the largest JPEG
 * geometry it supports - UXGA 1600x1200, the full 2 MP - and the file is that
 * sensor JPEG, taken straight from one continuous DCMI receive. The preview is
 * parked for it: the sensor is reconfigured, the still arena *is* the preview
 * memory, and the preview configuration and stream are restored before the
 * display continues. A sensor without a JPEG encoder (GC032A, BF30A2) keeps the
 * software encoder and saves the preview frame at the preview resolution.
 *
 * That needs a carrier whose SDMMC pads do not collide with the camera or the
 * panel: they do on the DPI HDK, they do not on the SPI HDK.
 */

/* One preview frame slot. VGA RGB565 is the largest geometry this example
 * previews; a sensor configuration that does not fit is refused with a log. */
#define CAMERA_APP_FRAME_MAX_BYTES (640U * 480U * 2U)

/* JPEG quality requested when the selected sensor only streams JPEG, and for
 * the full resolution still capture. */
#define CAMERA_APP_JPEG_QUALITY 10U

/*
 * Still capture from the sensor's own JPEG encoder.
 *
 * The first frames after a mode switch still carry the previous exposure, so
 * CAMERA_APP_STILL_WARMUP_FRAMES complete frames are received and discarded and
 * the last one is kept; the backend leaves that last JPEG at the start of the
 * arena. The receive is one hardware run, which is why the arena has to hold
 * all of them at once.
 */
#define CAMERA_APP_STILL_WARMUP_FRAMES 9U
#define CAMERA_APP_STILL_FRAMES (CAMERA_APP_STILL_WARMUP_FRAMES + 1U)
/* Bounds the whole receive sequence, settle frames included. */
#define CAMERA_APP_STILL_TIMEOUT_MS 5000U
/* Preview frames dropped after a still: the sensor's auto exposure starts over
 * when the geometry changes back, and showing those frames would flash. */
#define CAMERA_APP_STILL_RESUME_SETTLE_FRAMES 2U
/* The stream must be idle before the sensor may be reconfigured. */
#define CAMERA_APP_STILL_PAUSE_TIMEOUT_MS 3000U
/* Waiting for hardware and callbacks to retire before the camera is reused;
 * both the still capture channel and the preview stream use it. */
#define CAMERA_APP_STOP_TIMEOUT_MS 200U

#define CAMERA_APP_EVENT_FRAME (1U << 0)
#define CAMERA_APP_STATS_INTERVAL_MS 5000U
#define CAMERA_APP_STATS_WINDOW_MS 5000U

L2_NON_RET_BSS_SECT_BEGIN(camera_app_buffers)
L2_NON_RET_BSS_SECT(camera_app_buffers,
                    ALIGN(64) static struct
                    {
                        uint8_t dma[2][CAMERA_APP_FRAME_MAX_BYTES];
                        uint8_t stable[2][CAMERA_APP_FRAME_MAX_BYTES];
                    } camera_frames);
L2_NON_RET_BSS_SECT(camera_app_buffers,
                    ALIGN(64) static uint8_t camera_preview_buffer[CAMERA_DISPLAY_BUFFER_SIZE]);
L2_NON_RET_BSS_SECT_END

#define CAMERA_APP_STILL_ARENA (camera_frames.dma[0])
#define CAMERA_APP_STILL_CAPACITY (sizeof(camera_frames))

/* The reference implementation fits its ten UXGA frames, which measure below
 * 192 KB each, inside 1.92 MB. Keep at least that much room for ours. */
typedef char camera_app_still_arena_size_check[
    (CAMERA_APP_STILL_CAPACITY >= (1920U * 1024U)) ? 1 : -1];

/* Active camera framework session, resolved by camera_app_camera_open(). */
static camera_handler_instance_t *camera_instance;
static uint32_t camera_frame_bytes;
static uint16_t camera_frame_width;
static uint16_t camera_frame_height;
static camera_pixel_format_t camera_frame_format;
/* Preview configuration, re-applied every time a still capture gave the sensor
 * a different geometry. */
static camera_capture_config_t camera_preview_config;
/* Largest JPEG geometry the sensor can encode itself, or FRAMESIZE_INVALID when
 * it has no JPEG encoder and the preview frame has to be encoded instead. */
static framesize_t camera_still_framesize = FRAMESIZE_INVALID;
/* Still capture handshake with the capture thread. */
static volatile rt_bool_t camera_still_pending;
static struct rt_semaphore camera_still_parked;
static struct rt_semaphore camera_still_released;

static frame_pool_t camera_pool;
static struct rt_mutex camera_pool_lock;
static struct rt_event camera_events;
static rt_thread_t capture_thread;
static rt_thread_t display_thread;
static volatile rt_bool_t camera_running;
static uint32_t captured_frames;
static uint32_t displayed_frames;
static uint32_t dropped_frames;
static uint32_t failed_frames;
static uint32_t capture_period_ticks;
static uint32_t capture_period_samples;
static uint32_t copy_ticks;
static uint32_t copied_frames;
static uint32_t display_ticks;
static uint32_t display_isp_ms;
static uint32_t display_write_ms;
static int last_capture_error;

/* One queued still capture; the display thread turns it into a file. */
static volatile rt_bool_t photo_pending;
static uint32_t photos_saved;
static int last_photo_error;

typedef struct
{
    uint32_t captured;
    uint32_t displayed;
    uint32_t dropped;
    uint32_t failed;
    uint32_t decode_samples;
    uint32_t decode_total_ms;
    uint32_t decode_max_ms;
    uint32_t isp_samples;
    uint32_t isp_total_ms;
    uint32_t isp_max_ms;
    uint32_t isp_awb_stats_ms;
    uint32_t isp_awb_apply_ms;
    uint32_t isp_ccm_ms;
    uint32_t isp_gamma_ms;
    uint32_t isp_tone_ms;
    uint32_t isp_filter_ms;
    uint32_t isp_applied_flags;
    uint32_t photo_samples;
    uint32_t photo_failed;
    uint32_t photo_encode_total_ms;
    uint32_t photo_encode_max_ms;
    uint32_t photo_write_total_ms;
    uint32_t photo_write_max_ms;
    uint32_t photo_hcpu_samples;
    uint32_t photo_hcpu_x100_total;
    uint32_t photo_acpu_profiler_samples;
    uint32_t photo_acpu_profiler_x100_total;
} camera_window_stats_t;

static camera_window_stats_t camera_window;

/* ------------------------------------------------------------------ *
 * Camera framework session
 * ------------------------------------------------------------------ */

/**
 * @brief Map a camera framework framesize to its pixel dimensions.
 *
 * @return Return 0 on success, or -1 when @p size is not a known framesize.
 */
static int camera_app_framesize_resolution(framesize_t size, uint16_t *width,
                                           uint16_t *height)
{
    switch (size)
    {
        case FRAMESIZE_96X96:   *width = 96;   *height = 96;   break;
        case FRAMESIZE_QQVGA:   *width = 160;  *height = 120;  break;
        case FRAMESIZE_128X128: *width = 128;  *height = 128;  break;
        case FRAMESIZE_QCIF:    *width = 176;  *height = 144;  break;
        case FRAMESIZE_HQVGA:   *width = 240;  *height = 160;  break;
        case FRAMESIZE_240X240: *width = 240;  *height = 240;  break;
        case FRAMESIZE_QVGA:    *width = 320;  *height = 240;  break;
        case FRAMESIZE_320X320: *width = 320;  *height = 320;  break;
        case FRAMESIZE_CIF:     *width = 400;  *height = 296;  break;
        case FRAMESIZE_HVGA:    *width = 480;  *height = 320;  break;
        case FRAMESIZE_VGA:     *width = 640;  *height = 480;  break;
        case FRAMESIZE_SVGA:    *width = 800;  *height = 600;  break;
        case FRAMESIZE_XGA:     *width = 1024; *height = 768;  break;
        case FRAMESIZE_HD:      *width = 1280; *height = 720;  break;
        case FRAMESIZE_SXGA:    *width = 1280; *height = 1024; break;
        case FRAMESIZE_UXGA:    *width = 1600; *height = 1200; break;
        case FRAMESIZE_240X320: *width = 240;  *height = 320;  break;
        default:
            return -1;
    }

    return 0;
}

static rt_bool_t camera_app_caps_has_pixformat(const camera_capabilities_t *caps,
                                               pixformat_t format)
{
    rt_uint8_t index;

    if ((caps == RT_NULL) || (caps->pixformats == RT_NULL))
        return RT_FALSE;
    for (index = 0U; index < caps->num_pixformats; index++)
    {
        if (caps->pixformats[index] == format)
            return RT_TRUE;
    }
    return RT_FALSE;
}

/**
 * @brief Pick the geometry of a still taken by the sensor itself.
 *
 * Only a sensor with an on-sensor JPEG encoder qualifies: the still is then the
 * sensor's own JPEG at its largest supported geometry instead of an encode of
 * the preview frame, so the file resolution no longer follows the preview
 * resolution. The OV2640 answers with UXGA 1600x1200, its full 2 MP.
 *
 * @param caps is the capability descriptor of the active sensor.
 *
 * @return Return the framesize to capture stills at, or FRAMESIZE_INVALID when
 *         the sensor cannot encode JPEG and the preview frame has to be used.
 */
static framesize_t camera_app_pick_still_framesize(const camera_capabilities_t *caps)
{
#if !defined(CAMERA_DVP_BACKEND_DCMI)
    /* Receiving a bounded sequence of JPEG frames in one hardware run is a DCMI
     * feature; the other backends cannot hand the sensor over for a still. */
    (void)caps;
    return FRAMESIZE_INVALID;
#else
    framesize_t chosen = FRAMESIZE_INVALID;
    uint32_t largest_pixels = 0U;
    rt_uint8_t index;

    if (!camera_app_caps_has_pixformat(caps, PIXFORMAT_JPEG))
        return FRAMESIZE_INVALID;

    for (index = 0U; index < caps->num_framesizes; index++)
    {
        uint16_t width;
        uint16_t height;
        uint32_t pixels;

        if (camera_app_framesize_resolution(caps->framesizes[index], &width,
                                            &height) != 0)
            continue;
        pixels = (uint32_t)width * (uint32_t)height;
        if (pixels <= largest_pixels)
            continue;
        largest_pixels = pixels;
        chosen = caps->framesizes[index];
    }
    return chosen;
#endif
}

/**
 * @brief Choose the preview configuration advertised by the active sensor.
 *
 * RGB565 is preferred because the ISP and the display path consume it directly;
 * a JPEG-only sensor is accepted and decoded for the screen. Among the frame
 * sizes the sensor supports, the largest one that still fits one frame slot is
 * used, so a larger sensor cannot overflow the preview buffers.
 *
 * @param caps   is the capability descriptor of the active sensor.
 * @param config is the configuration to fill in.
 *
 * @return Return 0 on success, or -1 when no usable configuration exists.
 */
static int camera_app_pick_config(const camera_capabilities_t *caps,
                                  camera_capture_config_t *config)
{
    uint16_t chosen_width = 0U;
    uint16_t chosen_height = 0U;
    uint32_t best_bytes = 0U;
    pixformat_t format;
    rt_uint8_t index;

    if ((caps == RT_NULL) || (caps->framesizes == RT_NULL) ||
        (caps->num_framesizes == 0U))
        return -1;

    if (camera_app_caps_has_pixformat(caps, PIXFORMAT_RGB565))
        format = PIXFORMAT_RGB565;
    else if (camera_app_caps_has_pixformat(caps, PIXFORMAT_JPEG))
        format = PIXFORMAT_JPEG;
    else
    {
        rt_kprintf("camera: sensor has neither RGB565 nor JPEG output\n");
        return -1;
    }

    *config = (camera_capture_config_t){
        .pixformat = format,
        .framesize = FRAMESIZE_INVALID,
        .quality = (format == PIXFORMAT_JPEG) ? (uint8_t)CAMERA_APP_JPEG_QUALITY
                                              : (uint8_t)0,
    };

    for (index = 0U; index < caps->num_framesizes; index++)
    {
        uint16_t width;
        uint16_t height;
        uint32_t bytes;

        if (camera_app_framesize_resolution(caps->framesizes[index], &width,
                                            &height) != 0)
            continue;
        /* A JPEG length is variable; keep its pixel count inside the slot. */
        bytes = (uint32_t)width * (uint32_t)height *
                ((format == PIXFORMAT_RGB565) ? 2U : 1U);
        if ((bytes > CAMERA_APP_FRAME_MAX_BYTES) || (bytes <= best_bytes))
            continue;
        best_bytes = bytes;
        chosen_width = width;
        chosen_height = height;
        config->framesize = caps->framesizes[index];
    }

    if (config->framesize == FRAMESIZE_INVALID)
    {
        rt_kprintf("camera: no supported framesize fits %u bytes\n",
                   (unsigned)CAMERA_APP_FRAME_MAX_BYTES);
        return -1;
    }

    camera_frame_width = chosen_width;
    camera_frame_height = chosen_height;
    camera_frame_format = (format == PIXFORMAT_RGB565) ? CAMERA_PIXEL_RGB565_BE
                                                       : CAMERA_PIXEL_JPEG;
    /* A JPEG frame is copied as one opaque slot, its length is not known here. */
    camera_frame_bytes = (format == PIXFORMAT_RGB565) ? best_bytes
                                                      : CAMERA_APP_FRAME_MAX_BYTES;
    /* The preview configuration is kept so a still capture can hand the sensor
     * back exactly the geometry the preview was picked with. */
    camera_preview_config = *config;
    camera_still_framesize = camera_app_pick_still_framesize(caps);
    return 0;
}

static void camera_app_camera_close(void)
{
    if (camera_instance != RT_NULL)
        (void)camera_deinit(&camera_instance);
}

/**
 * @brief Open the camera through the framework handle layer.
 *
 * @return Return 0 on success, or -1 (with the session closed again).
 */
static int camera_app_camera_open(void)
{
    const camera_capabilities_t *caps = RT_NULL;
    camera_capture_config_t config;
    camera_handle_status_t status;

    status = camera_handler_instance_init(&camera_instance);
    if (status != CAMERA_OK)
    {
        rt_kprintf("camera: handle init failed status=%d\n", (int)status);
        return -1;
    }

    status = camera_get_capabilities(camera_instance, &caps);
    if ((status != CAMERA_OK) || (caps == RT_NULL))
    {
        rt_kprintf("camera: capability query failed status=%d\n", (int)status);
        camera_app_camera_close();
        return -1;
    }
    if (camera_app_pick_config(caps, &config) != 0)
    {
        camera_app_camera_close();
        return -1;
    }

    status = camera_change_settings(camera_instance, &config);
    if (status != CAMERA_OK)
    {
        rt_kprintf("camera: apply settings failed status=%d\n", (int)status);
        camera_app_camera_close();
        return -1;
    }

#if (CAMERA_PROFILE_XCLK_HZ != 0)
    /* The framework's sensor driver started MCLK when it opened the sensor;
     * re-drive it here to tune the frame rate for this camera profile. */
    if (camera_xclk_start(CAMERA_XCLK_PIN, CAMERA_PROFILE_XCLK_HZ) != CAMERA_XCLK_OK)
        rt_kprintf("camera: xclk override to %u Hz failed\n",
                   (unsigned)CAMERA_PROFILE_XCLK_HZ);
#endif

    return 0;
}

static int camera_app_stream_start(void)
{
    camera_stream_config_t config;
    camera_handle_status_t status;

    config.buffers[0] = camera_frames.dma[0];
    config.buffers[1] = camera_frames.dma[1];
    config.buffer_size = camera_frame_bytes;

    status = camera_start_stream(camera_instance, &config);
    if (status != CAMERA_OK)
    {
        rt_kprintf("camera: stream start failed status=%d\n", (int)status);
        return -1;
    }
    return 0;
}

/**
 * @brief Stop the stream, retrying while the backend still owns the camera.
 *
 * A completion callback that has not returned yet makes the backend refuse the
 * stop for a moment. Retrying keeps that from failing a still capture, which
 * cannot reconfigure the sensor before the stream is really down.
 *
 * @return Return 0 when the stream is stopped, or -1 when it kept refusing.
 */
static int camera_app_stream_stop(void)
{
    rt_tick_t started = rt_tick_get();

    while (camera_stop_stream(camera_instance) != CAMERA_OK)
    {
        if ((rt_tick_t)(rt_tick_get() - started) >=
            rt_tick_from_millisecond(CAMERA_APP_STOP_TIMEOUT_MS))
        {
            rt_kprintf("camera: stream stop failed\n");
            return -1;
        }
        rt_thread_delay(1);
    }
    return 0;
}

/**
 * @brief Wait for the next streamed frame, with a bounded timeout.
 *
 * @param frame is the framework frame descriptor to fill in.
 *
 * @return Return 0 on success, or a negative value on timeout/error.
 */
static int camera_app_stream_wait(camera_stream_frame_t *frame)
{
    camera_handle_status_t status = camera_get_stream_frame(
        camera_instance, frame, rt_tick_from_millisecond(2000));

    if (status != CAMERA_OK)
        return -(int)status;
    if ((frame->buffer == RT_NULL) || (frame->frame_size == 0U))
        return -RT_ERROR;
    return 0;
}

static void camera_capture_thread(void *parameter)
{
    rt_bool_t stream_running = RT_TRUE;
    rt_tick_t last_capture_tick = 0;
    uint32_t settle_frames = 0U;

    (void)parameter;
    while (camera_running)
    {
        camera_stream_frame_t dma_frame;
        int result;

        if (camera_still_pending)
        {
            /* Hand the camera over to the still capture. Receiving stops here,
             * the display thread gets the confirmation it waits for, and the
             * stream is only started again after the preview was restored. The
             * arena the still is written into is this stream's own memory, so
             * nothing may be receiving while it is in use. */
            if (stream_running)
            {
                (void)camera_app_stream_stop();
                stream_running = RT_FALSE;
            }
            last_capture_tick = 0;
            (void)rt_sem_release(&camera_still_parked);
            (void)rt_sem_take(&camera_still_released, RT_WAITING_FOREVER);
            /* The geometry switch left the sensor's exposure hunting; the first
             * frames of the resumed preview would flash. */
            settle_frames = CAMERA_APP_STILL_RESUME_SETTLE_FRAMES;
            continue;
        }

        if (!stream_running)
        {
            if (camera_app_stream_start() != 0)
            {
                rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
                failed_frames++;
                rt_mutex_release(&camera_pool_lock);
                rt_thread_mdelay(100);
                continue;
            }
            stream_running = RT_TRUE;
            last_capture_tick = 0;
        }

        result = camera_app_stream_wait(&dma_frame);
        if ((result == 0) && (settle_frames > 0U))
        {
            /* Received and dropped on purpose: the frame is not handed to the
             * pool, so the LCD keeps the last still preview image. */
            settle_frames--;
            continue;
        }
        if (result == 0)
        {
            rt_tick_t now = rt_tick_get();

            rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
            captured_frames++;
            camera_window.captured++;
            last_capture_error = 0;
            if (last_capture_tick != 0)
            {
                capture_period_ticks += now - last_capture_tick;
                capture_period_samples++;
            }
            last_capture_tick = now;
            rt_mutex_release(&camera_pool_lock);
        }
        else
        {
            (void)camera_app_stream_stop();
            stream_running = RT_FALSE;
            last_capture_tick = 0;
            rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
            failed_frames++;
            last_capture_error = result;
            rt_mutex_release(&camera_pool_lock);
        }

        if (result == 0)
        {
            uint8_t *destination;
            uint8_t slot;
            int store_result;

            rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
            store_result = frame_pool_begin_capture(&camera_pool, &destination,
                                                    &slot);
            rt_mutex_release(&camera_pool_lock);
            if (store_result == 0)
            {
                rt_tick_t copy_start = rt_tick_get();
                rt_tick_t elapsed;

                memcpy(destination, dma_frame.buffer, dma_frame.frame_size);
                elapsed = rt_tick_get() - copy_start;
                rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
                store_result = frame_pool_complete_capture(
                    &camera_pool, slot, dma_frame.frame_size,
                    dma_frame.sequence);
                if (store_result == 0)
                {
                    copy_ticks += elapsed;
                    copied_frames++;
                }
                else
                {
                    frame_pool_cancel_capture(&camera_pool, slot);
                    failed_frames++;
                }
                rt_mutex_release(&camera_pool_lock);
                if (store_result == 0)
                    rt_event_send(&camera_events, CAMERA_APP_EVENT_FRAME);
            }
            else
            {
                rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
                dropped_frames++;
                camera_window.dropped++;
                rt_mutex_release(&camera_pool_lock);
            }
        }

        if (result != 0)
        {
            rt_thread_mdelay(100);
        }
    }

    if (stream_running)
        (void)camera_app_stream_stop();
}

static int camera_app_acquire_frame(camera_frame_t *frame)
{
    int result;

    rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
    result = frame_pool_acquire_latest(&camera_pool, frame);
    rt_mutex_release(&camera_pool_lock);
    return result;
}

static void camera_app_release_frame(camera_frame_t *frame)
{
    rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
    frame_pool_release(&camera_pool, frame);
    rt_mutex_release(&camera_pool_lock);
}

static void camera_app_account_isp(const camera_sw_acpu_metrics_t *metrics)
{
    rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
    camera_window.isp_samples++;
    camera_window.isp_total_ms += metrics->total_ms;
    if (metrics->total_ms > camera_window.isp_max_ms)
        camera_window.isp_max_ms = metrics->total_ms;
    camera_window.isp_awb_stats_ms += metrics->isp.awb_stats_ms;
    camera_window.isp_awb_apply_ms += metrics->isp.awb_apply_ms;
    camera_window.isp_ccm_ms += metrics->isp.ccm_ms;
    camera_window.isp_gamma_ms += metrics->isp.gamma_ms;
    camera_window.isp_tone_ms += metrics->isp.tone_ms;
    camera_window.isp_filter_ms += metrics->isp.filter_ms;
    camera_window.isp_applied_flags = metrics->isp.applied_flags;
    rt_mutex_release(&camera_pool_lock);
}

/*
 * The software ISP only accepts big endian RGB565 while the DVP backend delivers
 * the panel native order, so such frames are converted once on their way in and
 * the display side swaps the ISP output back through EPIC. See
 * camera_profile.h for the per camera table; the conversion is compiled out for
 * the backends that already produce big endian frames.
 */
static void camera_app_swap_rgb565(uint8_t *pixels, uint32_t count)
{
    uint32_t index;

    for (index = 0U; index < count; index++)
    {
        uint8_t low = pixels[0];

        pixels[0] = pixels[1];
        pixels[1] = low;
        pixels += 2U;
    }
}

/*
 * Per-photo measurement of the JPEG encode.
 *
 * Both counters are sampled right after the encoder returns, so the window is
 * the encode itself: the card write stays outside it, and it is not a window
 * that an operator happens to fill with key presses. HCPU and ACPU report the
 * same cpu_get_usage() definition (see camera_sw_acpu_usage_t); the ACPU side is
 * the interesting one because that is where the encoder runs.
 */
typedef struct
{
    uint32_t encode_ms;
    uint32_t write_ms;
    int32_t hcpu_usage_x100;
    int32_t hcpu_fresh;
    camera_sw_acpu_usage_t acpu;
    int saved;
} camera_photo_measurement_t;

static void camera_app_acpu_window_reset(void);
static const char *camera_app_jpeg_core_name(void);

/*
 * Who produces the still's JPEG: the sensor itself when it has an encoder, or
 * the software codec on one of the cores. The two paths differ in where their
 * encode time comes from, so every photo log names the producer next to it.
 */
static const char *camera_app_photo_codec_name(void)
{
    return (camera_still_framesize != FRAMESIZE_INVALID) ? "sensor"
                                                         : camera_app_jpeg_core_name();
}

static void camera_app_hcpu_window_reset(void)
{
#if defined(USING_CPU_USAGE_PROFILER) && defined(FINSH_USING_MSH)
    char command[] = "cpu_prof_reset";

    (void)msh_exec(command, sizeof(command) - 1U);
#endif
}

static int32_t camera_app_hcpu_usage_x100(void)
{
    float usage = cpu_get_usage();

    return (usage >= 0.0f) ? (int32_t)(usage * 100.0f) : -1;
}

/*
 * Settle the HCPU profiler and return the usage of the interval that just
 * ended.
 *
 * The profiler credits a thread's run time only when that thread is switched
 * away, so a burst that ended in this very thread - the software JPEG encode is
 * one, it runs inline in the display thread - is not accounted for yet when
 * this settle runs right after it. Yield one tick first, which bills the burst
 * to the interval the settle publishes; without it the interval holds nothing
 * but idle slices and reads as 0.00%.
 *
 * cpu_prof_reset() reports -1 when there was nothing to account for, which is
 * what a window with fewer than two thread switches produces: the settle zeroes
 * the counters and the first switch after it is not accumulated either. Fall
 * back to the value of the previous settle (the profiler keeps it around) and
 * tell the caller it is not fresh.
 */
static int32_t camera_app_hcpu_usage_settle(int32_t *fresh)
{
    int32_t previous = camera_app_hcpu_usage_x100();
    int32_t settled;

    rt_thread_mdelay(1);
    camera_app_hcpu_window_reset();
    settled = camera_app_hcpu_usage_x100();
    if (fresh != RT_NULL)
        *fresh = (settled >= 0) ? 1 : 0;
    return (settled >= 0) ? settled : previous;
}

static const char *camera_app_usage_text(int32_t usage_x100, char *buffer,
                                         size_t size)
{
    if (usage_x100 < 0)
        return "n/a";
    rt_snprintf(buffer, size, "%d.%02d%%", usage_x100 / 100,
                usage_x100 % 100);
    return buffer;
}

/*
 * Same as camera_app_usage_text(), but a value this sample did not settle gets a
 * trailing '*' so a carried over reading is not mistaken for the window it is
 * printed next to.
 */
static const char *camera_app_usage_text_settled(int32_t usage_x100,
                                                 int32_t fresh, char *buffer,
                                                 size_t size)
{
    size_t length;

    camera_app_usage_text(usage_x100, buffer, size);
    if ((usage_x100 >= 0) && (fresh == 0))
    {
        length = strlen(buffer);
        if ((length + 1U) < size)
        {
            buffer[length] = '*';
            buffer[length + 1U] = '\0';
        }
    }
    return buffer;
}

/**
 * @brief Fold one photo measurement into the window camera_stats reports.
 */
static void camera_app_account_photo(
    const camera_photo_measurement_t *measurement)
{
    rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
    if (measurement->saved != 0)
    {
        camera_window.photo_samples++;
        camera_window.photo_encode_total_ms += measurement->encode_ms;
        if (measurement->encode_ms > camera_window.photo_encode_max_ms)
            camera_window.photo_encode_max_ms = measurement->encode_ms;
        camera_window.photo_write_total_ms += measurement->write_ms;
        if (measurement->write_ms > camera_window.photo_write_max_ms)
            camera_window.photo_write_max_ms = measurement->write_ms;
        if (measurement->hcpu_usage_x100 >= 0)
        {
            camera_window.photo_hcpu_x100_total +=
                (uint32_t)measurement->hcpu_usage_x100;
            camera_window.photo_hcpu_samples++;
        }
        if (measurement->acpu.profiler_x100 >= 0)
        {
            camera_window.photo_acpu_profiler_x100_total +=
                (uint32_t)measurement->acpu.profiler_x100;
            camera_window.photo_acpu_profiler_samples++;
        }
    }
    else
    {
        camera_window.photo_failed++;
    }
    rt_mutex_release(&camera_pool_lock);
}

/* Open the CPU measurement window of one photo. */
static void camera_app_photo_measurement_start(void)
{
    camera_app_hcpu_window_reset();
    camera_app_acpu_window_reset();
}
/*
 * Sample both CPU counters right at the end of the JPEG production, whether
 * that was the software encoder or the sensor's own capture. The card write
 * that follows would otherwise sit inside the window and dilute the figures
 * with idle time, and it would also push the HCPU interval past the profiler's
 * own 2 s settle. The HCPU settle yields one tick before it publishes, so the
 * write clock of the caller has to start after this call, not before it.
 */
static void camera_app_photo_measurement_sample(
    camera_photo_measurement_t *measurement)
{
    measurement->hcpu_usage_x100 = camera_app_hcpu_usage_settle(
        &measurement->hcpu_fresh);
    measurement->acpu = (camera_sw_acpu_usage_t){.usage_x100 = -1,
                                                 .profiler_x100 = -1};
    (void)camera_image_query_acpu_usage(&measurement->acpu);
}

/*
 * Fold the sample into the camera_stats window and log the CPU share of the
 * JPEG production itself; both cores report the same cpu_get_usage()
 * definition, and the codec name says which path produced the file.
 */
static void camera_app_photo_measurement_finish(
    camera_photo_measurement_t *measurement)
{
    char hcpu_text[12];
    char acpu_text[12];

    camera_app_account_photo(measurement);
    if (measurement->saved == 0)
        return;
    rt_kprintf("camera: photo cpu hcpu=%s acpu=%s codec=%s\n",
               camera_app_usage_text_settled(measurement->hcpu_usage_x100,
                                             measurement->hcpu_fresh,
                                             hcpu_text, sizeof(hcpu_text)),
               camera_app_usage_text_settled(measurement->acpu.profiler_x100,
                                             measurement->acpu.profiler_fresh,
                                             acpu_text, sizeof(acpu_text)),
               camera_app_photo_codec_name());
}

/**
 * @brief Encode one preview frame and write it to the card as /IMG_nnnn.JPG.
 *
 * This is the fallback for a sensor without an on-sensor JPEG encoder: there is
 * no larger geometry to switch to, so the still is the frame the display thread
 * already owns, at the preview resolution. Runs in the display thread, which
 * pauses for the encode and the card write. A sensor that streams JPEG is stored
 * as it is, an RGB565 frame goes through the software encoder.
 */
static int camera_app_save_photo(const camera_frame_t *frame)
{
    camera_photo_measurement_t measurement = {0};
    const uint8_t *jpeg = RT_NULL;
    uint32_t jpeg_size = 0U;
    uint32_t encoder_ms = 0U;
    char path[PHOTO_STORAGE_PATH_SIZE];
    rt_tick_t encode_started;
    rt_tick_t write_started;
    int result;

    if ((frame == RT_NULL) || (frame->data == RT_NULL) ||
        (frame->size == 0U))
        return -1;

    camera_app_photo_measurement_start();
    encode_started = rt_tick_get();
    if (frame->format == CAMERA_PIXEL_JPEG)
    {
        jpeg = frame->data;
        jpeg_size = frame->size;
    }
    else if (frame->format == CAMERA_PIXEL_RGB565_BE)
    {
#if !CAMERA_PROFILE_SOURCE_BE
        /* The encoder wants big endian pixels while this sensor writes the
         * other order. The frame was rendered already and is released right
         * after this call, so converting it in place is safe. */
        camera_app_swap_rgb565(frame->data,
                               (uint32_t)frame->width * frame->height);
#endif
        if (camera_image_encode_rgb565_be(frame->data, frame->width,
                                          frame->height,
                                          (uint32_t)frame->width * 2U,
                                          &jpeg, &jpeg_size,
                                          &encoder_ms) != 0)
        {
            rt_kprintf("camera: photo encode failed\n");
            camera_app_photo_measurement_finish(&measurement);
            return -1;
        }
    }
    else
    {
        rt_kprintf("camera: photo unsupported format %d\n",
                   (int)frame->format);
        camera_app_photo_measurement_finish(&measurement);
        return -1;
    }

    write_started = rt_tick_get();
    measurement.encode_ms = (uint32_t)((write_started - encode_started) *
                                       1000U / RT_TICK_PER_SECOND);
    if (encoder_ms != 0U)
        measurement.encode_ms = encoder_ms;
    camera_app_photo_measurement_sample(&measurement);

    write_started = rt_tick_get();
    result = photo_storage_save_jpeg_data(jpeg, jpeg_size, path, sizeof(path));
    measurement.write_ms = (uint32_t)((rt_tick_get() - write_started) *
                                      1000U / RT_TICK_PER_SECOND);
    if (result != PHOTO_STORAGE_OK)
    {
        rt_kprintf("camera: photo save failed error=%d\n", result);
        camera_app_photo_measurement_finish(&measurement);
        return -1;
    }

    photos_saved++;
    measurement.saved = 1;
    rt_kprintf("camera: photo %s %u B %ux%u encode=%u write=%u ms\n",
               path, (unsigned)jpeg_size, frame->width, frame->height,
               (unsigned)measurement.encode_ms,
               (unsigned)measurement.write_ms);
    camera_app_photo_measurement_finish(&measurement);
    return 0;
}

int camera_app_take_photo(void)
{
    if (!camera_running)
        return -1;
    photo_pending = RT_TRUE;
    return 0;
}

/* Milliseconds between two ticks, for the still capture log. */
static uint32_t camera_app_ms_between(rt_tick_t start, rt_tick_t end)
{
    return (uint32_t)((uint64_t)(end - start) * 1000U / RT_TICK_PER_SECOND);
}

/* The backend leaves one JPEG at the arena start; both markers have to be there
 * before the file is worth writing. */
static rt_bool_t camera_app_is_jpeg(const uint8_t *data, uint32_t size)
{
    return (data != RT_NULL) && (size >= 4U) && (data[0] == 0xffU) &&
           (data[1] == 0xd8U) && (data[size - 2U] == 0xffU) &&
           (data[size - 1U] == 0xd9U);
}

/*
 * Release the still capture channel. The framework keeps the camera until it
 * confirms that hardware and completion callbacks are idle, which a callback
 * still returning can refuse for a moment, so retry briefly.
 */
static void camera_app_stop_still_capture(void)
{
    rt_tick_t started = rt_tick_get();

    while (camera_stop_capture(camera_instance) != CAMERA_OK)
    {
        if ((rt_tick_t)(rt_tick_get() - started) >=
            rt_tick_from_millisecond(CAMERA_APP_STOP_TIMEOUT_MS))
        {
            rt_kprintf("camera: still capture did not stop\n");
            return;
        }
        rt_thread_delay(1);
    }
}

/**
 * @brief Save one still from the sensor's own JPEG encoder.
 *
 * Runs on the display thread with no frame acquired: the still arena is the
 * preview memory itself. The stream is parked through camera_still_pending, the
 * sensor is moved to camera_still_framesize, one continuous receive yields the
 * warmup frames plus the frame that is kept, the file is written, and both the
 * preview configuration and the stream are restored on every path - a failed
 * still only costs one preview pause.
 *
 * It logs the same two lines as the software path: "capture=" is this path's
 * receive window over "frames=" sensor frames (an elapsed window, not a
 * compression cost), and "setup=" covers the reconfiguration and settle that
 * only this path has to pay. The CPU share that follows shows how little of the
 * two cores the sensor-driven still costs.
 *
 * @return Return 0 when the file was written, or -1 on any failure.
 */
static int camera_app_save_still_photo(void)
{
    camera_capture_config_t still_config =
    {
        .pixformat = PIXFORMAT_JPEG,
        .framesize = camera_still_framesize,
        .quality = (uint8_t)CAMERA_APP_JPEG_QUALITY,
    };
    camera_capture_request_t request =
    {
        .buffer = CAMERA_APP_STILL_ARENA,
        .buffer_size = CAMERA_APP_STILL_CAPACITY,
        .frame_size = 0U,
    };
    camera_photo_measurement_t measurement = {0};
    camera_handle_status_t status;
    uint32_t jpeg_size = 0U;
    uint32_t setup_ms;
    uint32_t capture_ms = 0U;
    rt_tick_t started;
    char path[PHOTO_STORAGE_PATH_SIZE] = {0};
    int result;

    /* A token left behind by an aborted handshake must not let the capture
     * thread resume early, so the semaphore starts empty for every capture. */
    (void)rt_sem_take(&camera_still_released, 0);
    /* A reconfiguration is refused while the stream runs, so the capture thread
     * stops it first and acknowledges here. */
    camera_still_pending = RT_TRUE;
    if (rt_sem_take(&camera_still_parked,
                    rt_tick_from_millisecond(CAMERA_APP_STILL_PAUSE_TIMEOUT_MS)) != RT_EOK)
    {
        rt_kprintf("camera: still capture could not park the stream\n");
        camera_still_pending = RT_FALSE;
        /* The capture thread may already be on its way to the handshake, and it
         * would wait forever for a release this path never reaches. */
        (void)rt_sem_release(&camera_still_released);
        return -1;
    }

    /* Both CPU windows open before the sensor is touched, so they cover the whole
     * still: the reconfiguration, the settle frames and the frame that is kept.
     * This path leaves the cores mostly idle, which is the point of measuring it. */
    camera_app_photo_measurement_start();
    started = rt_tick_get();
    status = camera_change_settings(camera_instance, &still_config);
    /* Moving the sensor to another geometry and letting it settle is a cost this
     * path has and the software encoder path does not. */
    setup_ms = camera_app_ms_between(started, rt_tick_get());
    if (status == CAMERA_OK)
    {
        rt_tick_t capture_started = rt_tick_get();

        status = camera_capture_frames_timeout(camera_instance, &request,
                                               CAMERA_APP_STILL_FRAMES,
                                               CAMERA_APP_STILL_TIMEOUT_MS);
        /* One continuous run receives the warmup frames and the frame that is
         * kept. The sensor paces those frames itself (a UXGA JPEG takes about
         * 146 ms on this clock profile), so this is an elapsed window and not a
         * compression cost: the compression is pipelined inside the sensor and
         * is not observable from here. */
        capture_ms = camera_app_ms_between(capture_started, rt_tick_get());
        if (status == CAMERA_OK)
            jpeg_size = (uint32_t)request.frame_size;
    }
    /* Stop the channel on every path: an aborted or overflowing receive leaves
     * the DCMI armed just like a completed one. */
    camera_app_stop_still_capture();
    /* The JPEG exists now, which is where the CPU windows are read - same place
     * as on the software path, before the card write starts. */
    camera_app_photo_measurement_sample(&measurement);

    if ((status != CAMERA_OK) ||
        !camera_app_is_jpeg(CAMERA_APP_STILL_ARENA, jpeg_size))
    {
        rt_kprintf("camera: still capture failed status=%d bytes=%u\n",
                   (int)status, (unsigned)jpeg_size);
        result = -1;
    }
    else
    {
        rt_tick_t write_started = rt_tick_get();
        uint16_t width = 0U;
        uint16_t height = 0U;

        result = photo_storage_save_jpeg_data(CAMERA_APP_STILL_ARENA,
                                              jpeg_size, path, sizeof(path));
        measurement.write_ms = camera_app_ms_between(write_started,
                                                    rt_tick_get());
        (void)camera_app_framesize_resolution(camera_still_framesize, &width,
                                              &height);
        if (result != PHOTO_STORAGE_OK)
            rt_kprintf("camera: still save failed error=%d\n", result);
        else
        {
            photos_saved++;
            /* camera_stats reports this as "photo encode", which on this path is
             * the receive window above, not CPU encoder time. */
            measurement.encode_ms = capture_ms;
            measurement.saved = 1;
            rt_kprintf("camera: photo %s %u B %ux%u setup=%u capture=%u frames=%u write=%u ms\n",
                       path, (unsigned)jpeg_size, width, height,
                       (unsigned)setup_ms, (unsigned)capture_ms,
                       (unsigned)CAMERA_APP_STILL_FRAMES,
                       (unsigned)measurement.write_ms);
        }
    }
    /* Accounts the photo and logs the CPU share of this still window; the sensor
     * time printed above is what "encode" is for this path. */
    camera_app_photo_measurement_finish(&measurement);

    /* Back to the preview: the sensor first, then the pool, and the stream last
     * through the capture thread. Without the reset the pool would hand out
     * slots that now hold JPEG data. */
    status = camera_change_settings(camera_instance, &camera_preview_config);
    if (status != CAMERA_OK)
        rt_kprintf("camera: preview restore failed status=%d\n", (int)status);
    rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
    frame_pool_init(&camera_pool, camera_frames.stable[0], camera_frames.stable[1],
                    camera_frame_bytes, camera_frame_width, camera_frame_height,
                    camera_frame_format);
    rt_mutex_release(&camera_pool_lock);
    camera_still_pending = RT_FALSE;
    (void)rt_sem_release(&camera_still_released);

    return (result == PHOTO_STORAGE_OK) ? 0 : -1;
}

static int camera_app_apply_isp(camera_frame_t *frame, uint32_t *isp_ms)
{
    camera_sw_acpu_metrics_t metrics;
    uint32_t flags = camera_isp_flags();
    uint32_t elapsed;

    if (isp_ms != RT_NULL)
        *isp_ms = 0U;
    if ((flags == 0U) || (frame == RT_NULL) || (frame->data == RT_NULL) ||
        (frame->format != CAMERA_PIXEL_RGB565_BE) ||
        (frame->size != (uint32_t)frame->width * frame->height * 2U))
        return 0;

    {
        rt_tick_t started = rt_tick_get();

#if CAMERA_PROFILE_ISP_SWAP_INPUT
        camera_app_swap_rgb565(frame->data,
                               (uint32_t)frame->width * frame->height);
#endif
        if (camera_image_process_rgb565_be(frame->data, frame->width,
                                           frame->height,
                                           (uint32_t)frame->width * 2U, flags,
                                           &metrics) != 0)
            return -1;
        elapsed = (uint32_t)((rt_tick_get() - started) * 1000U /
                             RT_TICK_PER_SECOND);
    }

    if (isp_ms != RT_NULL)
        *isp_ms = elapsed;
    camera_app_account_isp(&metrics);
    return 0;
}

static void camera_app_acpu_window_reset(void)
{
    (void)camera_image_query_acpu_usage(RT_NULL);
}

typedef struct
{
    rt_tick_t tick;
    uint32_t displayed;
    uint32_t period_ticks;
    uint32_t period_samples;
    uint32_t copy_ticks;
    uint32_t copied;
    uint32_t display_ticks;
    uint32_t isp_ms;              /* HCPU wall time of the whole ISP call */
    uint32_t isp_acpu_total_ms;   /* ACPU side total, from the IPC metrics */
    uint32_t isp_samples;
    uint32_t isp_awb_stats_ms;
    uint32_t isp_awb_apply_ms;
    uint32_t isp_ccm_ms;
    uint32_t isp_gamma_ms;
    uint32_t isp_tone_ms;
    uint32_t isp_filter_ms;
    uint32_t write_ms;
} camera_stats_checkpoint_t;

static void camera_app_print_stats(camera_stats_checkpoint_t *last)
{
    rt_tick_t now = rt_tick_get();
    uint32_t elapsed_ms = (uint32_t)((now - last->tick) * 1000U /
                                     RT_TICK_PER_SECOND);

    if (elapsed_ms >= CAMERA_APP_STATS_INTERVAL_MS)
    {
        uint32_t captured;
        uint32_t displayed;
        uint32_t dropped;
        uint32_t failed;
        uint32_t period_total;
        uint32_t period_count;
        uint32_t copy_total;
        uint32_t copy_count;
        uint32_t display_total;
        uint32_t isp_total;
        uint32_t isp_acpu_total;
        uint32_t isp_samples;
        uint32_t isp_awb_stats_total;
        uint32_t isp_awb_apply_total;
        uint32_t isp_ccm_total;
        uint32_t isp_gamma_total;
        uint32_t isp_tone_total;
        uint32_t isp_filter_total;
        uint32_t write_total;
        uint32_t frames;
        uint32_t fps_x10;
        uint32_t period_ms_x10;
        uint32_t copy_ms_x10;
        uint32_t display_ms_x10;
        uint32_t isp_ms_x10;
        uint32_t write_ms_x10;
        int capture_error;

        rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
        captured = captured_frames;
        displayed = displayed_frames;
        dropped = dropped_frames;
        failed = failed_frames;
        period_total = capture_period_ticks;
        period_count = capture_period_samples;
        copy_total = copy_ticks;
        copy_count = copied_frames;
        display_total = display_ticks;
        isp_total = display_isp_ms;
        isp_acpu_total = camera_window.isp_total_ms;
        isp_samples = camera_window.isp_samples;
        isp_awb_stats_total = camera_window.isp_awb_stats_ms;
        isp_awb_apply_total = camera_window.isp_awb_apply_ms;
        isp_ccm_total = camera_window.isp_ccm_ms;
        isp_gamma_total = camera_window.isp_gamma_ms;
        isp_tone_total = camera_window.isp_tone_ms;
        isp_filter_total = camera_window.isp_filter_ms;
        write_total = display_write_ms;
        capture_error = last_capture_error;
        rt_mutex_release(&camera_pool_lock);

        frames = displayed - last->displayed;
        fps_x10 = elapsed_ms == 0U ? 0U : frames * 10000U / elapsed_ms;
        period_ms_x10 = camera_metrics_average_ms_x10(
            period_total - last->period_ticks,
            period_count - last->period_samples, RT_TICK_PER_SECOND);
        copy_ms_x10 = camera_metrics_average_ms_x10(
            copy_total - last->copy_ticks,
            copy_count - last->copied, RT_TICK_PER_SECOND);
        display_ms_x10 = camera_metrics_average_ms_x10(
            display_total - last->display_ticks, frames, RT_TICK_PER_SECOND);
        isp_ms_x10 = camera_metrics_average_ms_x10(
            isp_total - last->isp_ms, frames, 1000U);
        write_ms_x10 = camera_metrics_average_ms_x10(
            write_total - last->write_ms, frames, 1000U);

        rt_kprintf("camera: captured=%u displayed=%u dropped=%u failed=%u "
                   "fps=%u.%u period=%u.%u copy=%u.%u display=%u.%u ms "
                   "isp=%u.%u write=%u.%u ms\n ",
                   captured, displayed, dropped, failed,
                   fps_x10 / 10U, fps_x10 % 10U,
                   period_ms_x10 / 10U, period_ms_x10 % 10U,
                   copy_ms_x10 / 10U, copy_ms_x10 % 10U,
                   display_ms_x10 / 10U, display_ms_x10 % 10U,
                   isp_ms_x10 / 10U, isp_ms_x10 % 10U,
                   write_ms_x10 / 10U, write_ms_x10 % 10U
                );
        if (isp_samples != last->isp_samples)
        {
            /* The ACPU stage times come from the IPC metrics and the HCPU wall
             * time from this side, so the difference between them is the IPC
             * overhead. A fused color path (VIVID on) reports its whole
             * per-pixel loop in the gamma bucket. */
            static const char *const stage_name[6] =
            {
                "awb_stats", "awb_apply", "ccm", "gamma", "tone", "filter",
            };
            const uint32_t stage_total[6] =
            {
                isp_awb_stats_total, isp_awb_apply_total, isp_ccm_total,
                isp_gamma_total, isp_tone_total, isp_filter_total,
            };
            const uint32_t stage_last[6] =
            {
                last->isp_awb_stats_ms, last->isp_awb_apply_ms,
                last->isp_ccm_ms, last->isp_gamma_ms, last->isp_tone_ms,
                last->isp_filter_ms,
            };
            uint32_t isp_frames = isp_samples - last->isp_samples;
            uint32_t acpu_ms_x10 = camera_metrics_average_ms_x10(
                isp_acpu_total - last->isp_acpu_total_ms, isp_frames, 1000U);
            uint32_t ipc_ms_x10 = isp_ms_x10 > acpu_ms_x10 ?
                                  isp_ms_x10 - acpu_ms_x10 : 0U;
            uint32_t index;

            rt_kprintf("camera: isp n=%u acpu=%u.%u hcpu=%u.%u ipc=%u.%u ms\n",
                       isp_frames, acpu_ms_x10 / 10U, acpu_ms_x10 % 10U,
                       isp_ms_x10 / 10U, isp_ms_x10 % 10U,
                       ipc_ms_x10 / 10U, ipc_ms_x10 % 10U);
            rt_kprintf("camera: isp stage");
            for (index = 0U; index < 6U; index++)
            {
                uint32_t value_x10 = camera_metrics_average_ms_x10(
                    stage_total[index] - stage_last[index], isp_frames, 1000U);

                rt_kprintf(" %s=%u.%u", stage_name[index], value_x10 / 10U,
                           value_x10 % 10U);
            }
            rt_kprintf(" ms\n");
        }
        *last = (camera_stats_checkpoint_t){
            .tick = now,
            .displayed = displayed,
            .period_ticks = period_total,
            .period_samples = period_count,
            .copy_ticks = copy_total,
            .copied = copy_count,
            .display_ticks = display_total,
            .isp_ms = isp_total,
            .isp_acpu_total_ms = isp_acpu_total,
            .isp_samples = isp_samples,
            .isp_awb_stats_ms = isp_awb_stats_total,
            .isp_awb_apply_ms = isp_awb_apply_total,
            .isp_ccm_ms = isp_ccm_total,
            .isp_gamma_ms = isp_gamma_total,
            .isp_tone_ms = isp_tone_total,
            .isp_filter_ms = isp_filter_total,
            .write_ms = write_total,
        };
    }
}

static void camera_display_thread(void *parameter)
{
    camera_stats_checkpoint_t last = {.tick = rt_tick_get()};

    (void)parameter;
    while (camera_running)
    {
        rt_uint32_t events = 0;

        rt_event_recv(&camera_events, CAMERA_APP_EVENT_FRAME,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      rt_tick_from_millisecond(1000), &events);
        if ((events & CAMERA_APP_EVENT_FRAME) != 0U)
        {
            camera_frame_t frame;

            if (camera_app_acquire_frame(&frame) == 0)
            {
                /* The shutter is sampled once per frame, before anything is done
                 * with it: a capture requested while the frame was being
                 * displayed waits for the next one. Reading the flag again after
                 * the display would hand that request to the software encoder at
                 * the preview resolution instead of to the sensor still below,
                 * which is what a single press must never do. */
                rt_bool_t photo_request = (photo_pending != 0);

                if (photo_request)
                    photo_pending = RT_FALSE;

                if (photo_request && (camera_still_framesize != FRAMESIZE_INVALID))
                {
                    /* The still arena is the preview memory, so the frame has to
                     * go back before the sensor is reconfigured. */
                    camera_app_release_frame(&frame);
                    last_photo_error = camera_app_save_still_photo();
                }
                else
                {
                    rt_tick_t display_start;
                    uint32_t decode_ms = 0U;
                    uint32_t write_ms = 0U;
                    uint32_t isp_ms = 0U;
                    int result;

                    display_start = rt_tick_get();
                    (void)camera_app_apply_isp(&frame, &isp_ms);
                    result = camera_display_show(&frame, &decode_ms, &write_ms);
                    rt_tick_t elapsed = rt_tick_get() - display_start;

                    if (photo_request)
                        last_photo_error = camera_app_save_photo(&frame);

                    rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
                    display_ticks += elapsed;
                    if (result == 0)
                    {
                        displayed_frames++;
                        camera_window.displayed++;
                        display_isp_ms += isp_ms;
                        display_write_ms += write_ms;
                        if (frame.format == CAMERA_PIXEL_JPEG)
                        {
                            camera_window.decode_samples++;
                            camera_window.decode_total_ms += decode_ms;
                            if (decode_ms > camera_window.decode_max_ms)
                                camera_window.decode_max_ms = decode_ms;
                        }
                    }
                    else
                    {
                        failed_frames++;
                        camera_window.failed++;
                    }
                    rt_mutex_release(&camera_pool_lock);
                    camera_app_release_frame(&frame);
                }
            }
        }
        camera_app_print_stats(&last);
    }
}

static const char *camera_app_jpeg_core_name(void)
{
#if defined(CAMERA_SW_JPEG_RUN_ON_ACPU)
    return "acpu";
#else
    return "hcpu";
#endif
}

static void camera_app_log_config(void)
{
    rt_kprintf("camera: cfg %ux%u %s isp=%s jpeg=%s stages=0x%08x\n",
               camera_frame_width, camera_frame_height,
               (camera_frame_format == CAMERA_PIXEL_JPEG) ? "jpeg" : "rgb565",
               camera_isp_mode_name(), camera_app_jpeg_core_name(),
               (unsigned)camera_isp_flags());
}

int camera_app_start(void)
{
    if (camera_running)
        return 0;

    if (camera_app_camera_open() != 0)
    {
        rt_kprintf("camera: init failed stage=sensor\n");
        return -1;
    }

    frame_pool_init(&camera_pool, camera_frames.stable[0],
                    camera_frames.stable[1], camera_frame_bytes,
                    camera_frame_width, camera_frame_height,
                    camera_frame_format);
    rt_mutex_init(&camera_pool_lock, "cam_pool", RT_IPC_FLAG_PRIO);
    rt_event_init(&camera_events, "cam_evt", RT_IPC_FLAG_PRIO);
    rt_sem_init(&camera_still_parked, "cam_still", 0, RT_IPC_FLAG_PRIO);
    rt_sem_init(&camera_still_released, "cam_photo", 0, RT_IPC_FLAG_PRIO);

    if (camera_display_open(camera_preview_buffer,
                            sizeof(camera_preview_buffer)) != 0)
    {
        rt_kprintf("camera: init failed stage=display\n");
        camera_app_camera_close();
        return -1;
    }

    capture_thread = rt_thread_create("cam_cap", camera_capture_thread,
                                      RT_NULL, 4096, 12, 10);
    display_thread = rt_thread_create("cam_disp", camera_display_thread,
                                      RT_NULL, 6144, 13, 10);
    if ((capture_thread == RT_NULL) || (display_thread == RT_NULL))
    {
        rt_kprintf("camera: init failed stage=threads\n");
        if (capture_thread != RT_NULL) rt_thread_delete(capture_thread);
        if (display_thread != RT_NULL) rt_thread_delete(display_thread);
        camera_display_close();
        camera_app_camera_close();
        return -1;
    }

    if (camera_app_stream_start() != 0)
    {
        rt_kprintf("camera: init failed stage=stream\n");
        rt_thread_delete(capture_thread);
        rt_thread_delete(display_thread);
        camera_display_close();
        camera_app_camera_close();
        return -1;
    }

    camera_running = RT_TRUE;
    rt_thread_startup(display_thread);
    rt_thread_startup(capture_thread);
    camera_app_log_config();
    if (camera_still_framesize != FRAMESIZE_INVALID)
    {
        uint16_t width = 0U;
        uint16_t height = 0U;

        (void)camera_app_framesize_resolution(camera_still_framesize, &width,
                                              &height);
        rt_kprintf("camera: stills are sensor JPEG %ux%u, %u warmup frames\n",
                   width, height, (unsigned)CAMERA_APP_STILL_WARMUP_FRAMES);
    }
    else
    {
        rt_kprintf("camera: stills are encoded preview frames, no sensor JPEG\n");
    }
    rt_kprintf("camera: preview started %ux%u -> 390x450\n",
               camera_frame_width, camera_frame_height);
    if (photo_storage_ready() == PHOTO_STORAGE_OK)
        rt_kprintf("camera: sd card ready, photos go to /IMG_nnnn.JPG\n");
    else
        rt_kprintf("camera: no sd card, photo capture unavailable\n");
    return 0;
}

int camera_app_is_running(void)
{
    return camera_running ? 1 : 0;
}

static int camera_app_stats_cmd(int argc, char **argv)
{
    camera_window_stats_t win;
    uint32_t window_ms = CAMERA_APP_STATS_WINDOW_MS;
    int32_t hcpu_usage_x100 = -1;
    int32_t hcpu_fresh = 0;
    camera_sw_acpu_usage_t acpu = {.usage_x100 = -1, .profiler_x100 = -1};

    if (argc >= 2)
    {
        int value = atoi(argv[1]);

        if (value > 0)
            window_ms = (uint32_t)value;
    }

    rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
    camera_window = (camera_window_stats_t){0};
    rt_mutex_release(&camera_pool_lock);

    /* Commit and zero both CPU counters here and read them again after the
     * window, so the usage describes this window only. */
    camera_app_hcpu_window_reset();
    camera_app_acpu_window_reset();

    rt_thread_mdelay(window_ms);

    rt_mutex_take(&camera_pool_lock, RT_WAITING_FOREVER);
    win = camera_window;
    camera_window = (camera_window_stats_t){0};
    rt_mutex_release(&camera_pool_lock);

    hcpu_usage_x100 = camera_app_hcpu_usage_settle(&hcpu_fresh);
    if (camera_image_query_acpu_usage(&acpu) != 0)
        rt_kprintf("camera stats: acpu query failed\n");

    rt_kprintf("camera stats: window=%u ms\n", window_ms);
    rt_kprintf("camera stats: frames captured=%u displayed=%u "
               "dropped=%u failed=%u\n",
               win.captured, win.displayed, win.dropped, win.failed);
    if (win.decode_samples != 0U)
        rt_kprintf("camera stats: decode n=%u avg=%u ms max=%u ms\n",
                   win.decode_samples,
                   win.decode_total_ms / win.decode_samples,
                   win.decode_max_ms);
    else
        rt_kprintf("camera stats: decode n=0\n");
    if (win.isp_samples != 0U)
    {
        rt_kprintf("camera stats: isp n=%u avg=%u ms max=%u ms flags=0x%08x "
                   "applied=0x%08x\n",
                   win.isp_samples, win.isp_total_ms / win.isp_samples,
                   win.isp_max_ms, (unsigned)camera_isp_flags(),
                   (unsigned)win.isp_applied_flags);
        rt_kprintf("camera stats: isp stage avg awb_stats=%u awb_apply=%u "
                   "ccm=%u gamma=%u tone=%u filter=%u ms\n",
                   win.isp_awb_stats_ms / win.isp_samples,
                   win.isp_awb_apply_ms / win.isp_samples,
                   win.isp_ccm_ms / win.isp_samples,
                   win.isp_gamma_ms / win.isp_samples,
                   win.isp_tone_ms / win.isp_samples,
                   win.isp_filter_ms / win.isp_samples);
    }
    else
        rt_kprintf("camera stats: isp n=0 flags=0x%08x\n",
                   (unsigned)camera_isp_flags());
    if (win.photo_samples == 0U)
    {
        char hcpu_text[12];
        char usage[12];

        if (acpu.window_ms != 0U)
            rt_kprintf("camera stats: acpu usage=%s tb=%u us\n",
                       camera_app_usage_text_settled(acpu.profiler_x100,
                                                     acpu.profiler_fresh,
                                                     usage, sizeof(usage)),
                       acpu.profiler_timebase_us);
        rt_kprintf("camera stats: cpu hcpu=%s acpu=%s\n",
                   camera_app_usage_text_settled(hcpu_usage_x100, hcpu_fresh,
                                                 hcpu_text,
                                                 sizeof(hcpu_text)),
                   camera_app_usage_text_settled(acpu.profiler_x100,
                                                 acpu.profiler_fresh,
                                                 usage, sizeof(usage)));
    }
    else
        rt_kprintf("camera stats: cpu preview skipped, photos reset the "
                   "counters\n");
    rt_kprintf("camera stats: photos=%u last_photo_error=%d\n",
               (unsigned)photos_saved, last_photo_error);
    rt_kprintf("camera stats: photo n=%u failed=%u codec=%s\n",
               win.photo_samples, win.photo_failed,
               camera_app_photo_codec_name());
    if (win.photo_samples != 0U)
    {
        char hcpu_text[12];
        char acpu_text[12];

        rt_kprintf("camera stats: photo encode avg=%u max=%u ms write avg=%u "
                   "max=%u ms\n",
                   win.photo_encode_total_ms / win.photo_samples,
                   win.photo_encode_max_ms,
                   win.photo_write_total_ms / win.photo_samples,
                   win.photo_write_max_ms);
        rt_kprintf("camera stats: photo cpu hcpu=%s acpu=%s\n",
                   camera_app_usage_text(
                       (win.photo_hcpu_samples != 0U) ?
                       (int32_t)(win.photo_hcpu_x100_total /
                                 win.photo_hcpu_samples) : -1,
                       hcpu_text, sizeof(hcpu_text)),
                   camera_app_usage_text(
                       (win.photo_acpu_profiler_samples != 0U) ?
                       (int32_t)(win.photo_acpu_profiler_x100_total /
                                 win.photo_acpu_profiler_samples) : -1,
                       acpu_text, sizeof(acpu_text)));
    }
    return 0;
}
MSH_CMD_EXPORT_ALIAS(camera_app_stats_cmd, camera_stats,
                     "collect camera statistics for a time window");

static int camera_sccb_scan_cmd(int argc, char **argv)
{
    struct rt_i2c_configuration configuration =
    {
        .mode = 0,
        .addr = 0,
        .timeout = 100,
        .max_hz = 100000,
    };
    struct rt_i2c_bus_device *bus;
    bf0_i2c_t *device;
    uint8_t dummy = 0U;
    uint16_t address;
    int found = 0;

    if (argc >= 3)
    {
        int sda = atoi(argv[1]);
        int scl = atoi(argv[2]);

        if ((sda < 0) || (sda > 95) || (scl < 0) || (scl > 95))
        {
            rt_kprintf("camera: invalid pin index\n");
            return -1;
        }
        HAL_PIN_Set(PAD_PA00 + sda, I2C1_SDA, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA00 + scl, I2C1_SCL, PIN_PULLUP, 1);
        rt_kprintf("camera: SCCB pads SDA=PA%d SCL=PA%d\n", sda, scl);
    }

    bus = rt_i2c_bus_device_find(CAMERA_SCCB_I2C_BUS_NAME);
    if (bus == RT_NULL)
    {
        rt_kprintf("camera: %s not found\n", CAMERA_SCCB_I2C_BUS_NAME);
        return -1;
    }
    (void)rt_i2c_open(bus, RT_DEVICE_FLAG_RDWR);
    (void)rt_i2c_configure(bus, &configuration);
    /* The SDK I2C device starts with its rt_i2c_bus_device member. */
    device = (bf0_i2c_t *)bus;

    for (address = 0x08U; address < 0x78U; address++)
    {
        if (HAL_I2C_Master_Transmit(&device->handle, (uint16_t)(address << 1),
                                    &dummy, 1U, 20U) == HAL_OK)
        {
            rt_kprintf("camera: SCCB device at 0x%02x\n", (unsigned)address);
            found++;
        }
    }

    rt_kprintf("camera: SCCB scan done, %d device(s)\n", found);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(camera_sccb_scan_cmd, camera_sccb_scan,
                     "scan the camera SCCB bus [sda_pin scl_pin]");

static int camera_photo_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (camera_app_take_photo() != 0)
    {
        rt_kprintf("camera: preview is not running\n");
        return -1;
    }
    rt_kprintf("camera: photo queued\n");
    return 0;
}
MSH_CMD_EXPORT_ALIAS(camera_photo_cmd, camera_photo,
                     "save one full resolution still to the SD card");
