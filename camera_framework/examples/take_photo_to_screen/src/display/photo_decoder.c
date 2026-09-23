#include "photo_decoder.h"

#include "../app/camera_isp_flags.h"
#include "../image_processing/camera_image_processor.h"
#include "../camera/jpeg_info.h"

static uint32_t photo_decoder_isp_flags(void)
{
    return camera_isp_flags();
}

static int photo_decoder_software_decode(const uint8_t *jpeg,
                                         uint32_t jpeg_size,
                                         uint8_t *destination,
                                         uint32_t destination_size,
                                         uint16_t *width, uint16_t *height,
                                         uint32_t *decode_ms)
{
    jpeg_info_t info;
    camera_sw_acpu_metrics_t metrics;

    if ((jpeg_info_parse(jpeg, jpeg_size, &info) != 0) ||
        (info.sof_marker != 0xc0U) || (info.components != 3U) ||
        ((info.sampling[0] != 0x11U) && (info.sampling[0] != 0x21U) &&
         (info.sampling[0] != 0x22U)) ||
        (info.sampling[1] != 0x11U) || (info.sampling[2] != 0x11U))
        return PHOTO_DECODER_UNSUPPORTED;
    if (camera_image_decode_jpeg(jpeg, jpeg_size, destination,
                                 destination_size, width, height,
                                 photo_decoder_isp_flags(), &metrics) != 0)
        return PHOTO_DECODER_UNSUPPORTED;
    if (decode_ms != 0)
        *decode_ms = metrics.isp.decode_ms;
    return PHOTO_DECODER_OK;
}

const photo_decoder_backend_t *photo_decoder_backend(void)
{
    static const photo_decoder_backend_t backend = {
        .name = "acpu-software",
        .decode = photo_decoder_software_decode,
    };

    return &backend;
}
