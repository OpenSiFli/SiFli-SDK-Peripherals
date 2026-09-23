#include "camera_image_processor.h"

#include <mem_section.h>
#include <rtthread.h>
#include <string.h>

#include "camera_sw_isp_pipeline.h"
#if defined(CAMERA_SW_JPEG_RUN_ON_HCPU) && defined(CAMERA_SW_JPEG_DECODER)
#include "camera_sw_jpeg_decoder.h"
#endif
#if defined(CAMERA_SW_JPEG_RUN_ON_HCPU) && defined(CAMERA_SW_JPEG_ENCODER)
#include "camera_sw_jpeg_encoder.h"
#endif

#define CAMERA_IMAGE_JPEG_WORK_SIZE 16384U
#define CAMERA_IMAGE_JPEG_OUTPUT_SIZE (384U * 1024U)
#define CAMERA_IMAGE_JPEG_DECODE_MAX_WIDTH 640U

#if defined(CAMERA_SW_JPEG_RUN_ON_HCPU) && defined(CAMERA_SW_JPEG_DECODER)
typedef struct
{
    const uint8_t *data;
    uint32_t size;
    uint32_t offset;
} camera_image_jpeg_reader_t;

static uint8_t camera_image_jpeg_work[CAMERA_IMAGE_JPEG_WORK_SIZE];

static size_t camera_image_jpeg_read(void *context, uint8_t *buffer,
                                     size_t size)
{
    camera_image_jpeg_reader_t *reader = context;
    uint32_t remaining;

    if ((reader == RT_NULL) || (reader->offset > reader->size))
        return 0U;
    remaining = reader->size - reader->offset;
    if (size > remaining)
        size = remaining;
    if ((buffer != RT_NULL) && (size != 0U))
        memcpy(buffer, reader->data + reader->offset, size);
    reader->offset += (uint32_t)size;
    return size;
}
#endif

#if defined(CAMERA_SW_JPEG_ENCODER)
L2_NON_RET_BSS_SECT_BEGIN(camera_image_buffers)
L2_NON_RET_BSS_SECT(camera_image_buffers,
                    ALIGN(64) static uint8_t camera_image_jpeg_output[
                        CAMERA_IMAGE_JPEG_OUTPUT_SIZE]);
L2_NON_RET_BSS_SECT_END
#endif

#if defined(CAMERA_SW_JPEG_RUN_ON_HCPU) && defined(CAMERA_SW_JPEG_ENCODER)
static uint8_t camera_image_jpeg_workspace[
    CAMERA_SW_JPEG_ENCODER_STATIC_WORKSPACE_SIZE] __attribute__((aligned(8)));
static camera_sw_jpeg_encoder_t camera_image_encoder;
static int camera_image_encoder_ready;
#endif

static uint32_t camera_image_now_ms(void)
{
    return (uint32_t)((uint64_t)rt_tick_get() * 1000U / RT_TICK_PER_SECOND);
}

int camera_image_process_rgb565_be(uint8_t *pixels, uint16_t width,
                                   uint16_t height, uint32_t stride,
                                   uint32_t isp_flags,
                                   camera_sw_acpu_metrics_t *metrics)
{
#if defined(CAMERA_SW_ISP_RUN_ON_HCPU)
    uint32_t started = camera_image_now_ms();
    int result;

    if (metrics == RT_NULL)
        return -1;
    memset(metrics, 0, sizeof(*metrics));
    result = camera_sw_isp_process_rgb565_be(pixels, width, height, stride,
                                             isp_flags, &metrics->isp,
                                             camera_image_now_ms);
    metrics->total_ms = camera_image_now_ms() - started;
    return result;
#elif defined(CAMERA_SW_ISP_RUN_ON_ACPU)
    return camera_sw_acpu_process_rgb565_be(pixels, width, height, stride,
                                            isp_flags, metrics);
#else
    (void)pixels;
    (void)width;
    (void)height;
    (void)stride;
    (void)isp_flags;
    if (metrics == RT_NULL)
        return -1;
    memset(metrics, 0, sizeof(*metrics));
    return 0;
#endif
}

int camera_image_encode_rgb565_be(uint8_t *pixels, uint16_t width,
                                  uint16_t height, uint32_t pitch,
                                  const uint8_t **jpeg, uint32_t *jpeg_size,
                                  uint32_t *elapsed_ms)
{
#if defined(CAMERA_SW_JPEG_RUN_ON_HCPU) && defined(CAMERA_SW_JPEG_ENCODER)
    camera_sw_jpeg_frame_t frame;
    uint32_t started = camera_image_now_ms();

    if ((pixels == RT_NULL) || (jpeg == RT_NULL) || (jpeg_size == RT_NULL) ||
        (elapsed_ms == RT_NULL))
        return -1;
    if (!camera_image_encoder_ready) {
        if (camera_sw_jpeg_encoder_init(&camera_image_encoder,
                                        camera_image_jpeg_workspace,
                                        sizeof(camera_image_jpeg_workspace)) != 0)
            return -1;
        camera_image_encoder_ready = 1;
    }
    frame.data = pixels;
    frame.width = width;
    frame.height = height;
    frame.stride = pitch;
    frame.pixel_format = CAMERA_SW_JPEG_PIXEL_RGB565_BE;
    if (camera_sw_jpeg_encode_to_buffer(
            &camera_image_encoder, &frame, CAMERA_SW_JPEG_QUALITY_HIGH,
            camera_image_jpeg_output, sizeof(camera_image_jpeg_output),
            jpeg_size) != 0)
        return -1;
    *elapsed_ms = camera_image_now_ms() - started;
    *jpeg = camera_image_jpeg_output;
    return 0;
#elif defined(CAMERA_SW_JPEG_RUN_ON_ACPU) && defined(CAMERA_SW_JPEG_ENCODER)
    if (camera_sw_acpu_encode_rgb565_be(
            pixels, width, height, pitch, camera_image_jpeg_output,
            sizeof(camera_image_jpeg_output), jpeg_size, elapsed_ms) != 0)
        return -1;
    *jpeg = camera_image_jpeg_output;
    return 0;
#else
    (void)pixels;
    (void)width;
    (void)height;
    (void)pitch;
    (void)jpeg;
    (void)jpeg_size;
    (void)elapsed_ms;
    return -1;
#endif
}

int camera_image_decode_jpeg(const uint8_t *jpeg, uint32_t jpeg_size,
                             uint8_t *destination,
                             uint32_t destination_size,
                             uint16_t *width, uint16_t *height,
                             uint32_t isp_flags,
                             camera_sw_acpu_metrics_t *metrics)
{
#if defined(CAMERA_SW_JPEG_RUN_ON_HCPU) && defined(CAMERA_SW_JPEG_DECODER)
    camera_image_jpeg_reader_t reader;
    camera_sw_jpeg_frame_t output;
    int result;
    uint32_t started = camera_image_now_ms();
    uint32_t decode_ms;

    if ((jpeg == RT_NULL) || (jpeg_size < 4U) || (destination == RT_NULL) ||
        (destination_size == 0U) || (width == RT_NULL) || (height == RT_NULL) ||
        (metrics == RT_NULL))
        return -1;
    reader = (camera_image_jpeg_reader_t){
        .data = jpeg,
        .size = jpeg_size,
    };
    output = (camera_sw_jpeg_frame_t){
        .data = destination,
        .height = (uint16_t)(destination_size /
                             (CAMERA_IMAGE_JPEG_DECODE_MAX_WIDTH * 2U)),
        .stride = CAMERA_IMAGE_JPEG_DECODE_MAX_WIDTH * 2U,
        .pixel_format = CAMERA_SW_JPEG_PIXEL_RGB565_BE,
    };
    if ((output.height == 0U) || camera_sw_jpeg_decode(
            camera_image_jpeg_read, &reader, camera_image_jpeg_work,
            sizeof(camera_image_jpeg_work), &output) != 0)
        return -1;
    *width = output.width;
    *height = output.height;
    decode_ms = camera_image_now_ms() - started;
    result = camera_image_process_rgb565_be(destination, *width, *height,
                                            (uint32_t)*width * 2U,
                                            isp_flags, metrics);
    if (result == 0)
        metrics->isp.decode_ms = decode_ms;
    return result;
#elif defined(CAMERA_SW_JPEG_DECODER) && defined(CAMERA_SW_ISP_RUN_ON_HCPU)
    camera_sw_acpu_metrics_t decode_metrics;
    int result = camera_sw_acpu_decode_jpeg(jpeg, jpeg_size, destination,
                                            destination_size, width, height,
                                            0U, &decode_metrics);
    if (result != 0)
        return result;
    result = camera_image_process_rgb565_be(destination, *width, *height,
                                            (uint32_t)*width * 2U,
                                            isp_flags, metrics);
    if (result == 0) {
        metrics->isp.decode_ms = decode_metrics.isp.decode_ms;
        metrics->total_ms += decode_metrics.total_ms;
    }
    return result;
#elif defined(CAMERA_SW_JPEG_DECODER)
    return camera_sw_acpu_decode_jpeg(jpeg, jpeg_size, destination,
                                      destination_size, width, height,
                                      isp_flags, metrics);
#else
    (void)jpeg;
    (void)jpeg_size;
    (void)destination;
    (void)destination_size;
    (void)width;
    (void)height;
    (void)isp_flags;
    (void)metrics;
    return -1;
#endif
}

int camera_image_query_acpu_usage(camera_sw_acpu_usage_t *usage)
{
#if defined(CAMERA_SW_ISP_RUN_ON_ACPU) || defined(CAMERA_SW_JPEG_RUN_ON_ACPU)
    return camera_sw_acpu_query_cpu_usage(usage);
#else
    (void)usage;
    return -1;
#endif
}
