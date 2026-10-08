#include "jpeg_info.h"

#include <string.h>

static uint16_t jpeg_read_be16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int jpeg_is_sof(uint8_t marker)
{
    return ((marker >= 0xC0U) && (marker <= 0xC3U)) ||
           ((marker >= 0xC5U) && (marker <= 0xC7U)) ||
           ((marker >= 0xC9U) && (marker <= 0xCBU)) ||
           ((marker >= 0xCDU) && (marker <= 0xCFU));
}

int jpeg_info_parse(const uint8_t *data, size_t size, jpeg_info_t *info)
{
    size_t offset = 2U;
    int found_sof = 0;

    if ((data == NULL) || (info == NULL) || (size < 4U) ||
        (data[0] != 0xFFU) || (data[1] != 0xD8U))
        return -1;
    memset(info, 0, sizeof(*info));

    while (offset + 1U < size)
    {
        uint8_t marker;
        uint16_t segment_size;
        const uint8_t *payload;
        size_t payload_size;
        uint8_t component;

        if (data[offset] != 0xFFU)
            return -1;
        while ((offset < size) && (data[offset] == 0xFFU))
            offset++;
        if (offset >= size)
            return -1;
        marker = data[offset++];
        if ((marker == 0xD9U) || (marker == 0xDAU))
            break;
        if ((marker == 0x01U) ||
            ((marker >= 0xD0U) && (marker <= 0xD7U)))
            continue;
        if (offset + 2U > size)
            return -1;
        segment_size = jpeg_read_be16(data + offset);
        if ((segment_size < 2U) || (offset + segment_size > size))
            return -1;
        payload = data + offset + 2U;
        payload_size = segment_size - 2U;

        if (marker == 0xC4U)
            info->has_dht = 1U;
        else if (marker == 0xDBU)
            info->has_dqt = 1U;
        else if (jpeg_is_sof(marker))
        {
            if (payload_size < 6U)
                return -1;
            info->sof_marker = marker;
            info->height = jpeg_read_be16(payload + 1U);
            info->width = jpeg_read_be16(payload + 3U);
            info->components = payload[5];
            if (payload_size < 6U + 3U * info->components)
                return -1;
            for (component = 0U;
                 (component < info->components) && (component < 3U);
                 component++)
                info->sampling[component] = payload[7U + 3U * component];
            found_sof = 1;
        }
        offset += segment_size;
    }
    return found_sof ? 0 : -1;
}
