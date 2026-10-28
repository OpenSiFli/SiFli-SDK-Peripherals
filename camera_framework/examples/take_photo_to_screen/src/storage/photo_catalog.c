#include "photo_catalog.h"

#include <string.h>

int photo_catalog_parse_index(const char *name, uint16_t *index)
{
    uint16_t value = 0U;
    size_t position;

    if ((name == NULL) || (index == NULL) ||
        (strlen(name) != 12U) || (memcmp(name, "IMG_", 4U) != 0) ||
        (memcmp(name + 8U, ".JPG", 4U) != 0))
        return -1;
    for (position = 4U; position < 8U; position++)
    {
        if ((name[position] < '0') || (name[position] > '9'))
            return -1;
        value = (uint16_t)(value * 10U + (uint16_t)(name[position] - '0'));
    }
    if (value == 0U)
        return -1;
    *index = value;
    return 0;
}

int photo_catalog_latest(const uint16_t *indices, size_t count,
                         uint16_t *index)
{
    size_t position;
    uint16_t latest;

    if ((indices == NULL) || (count == 0U) || (index == NULL))
        return -1;
    latest = indices[0];
    for (position = 1U; position < count; position++)
    {
        if (indices[position] > latest)
            latest = indices[position];
    }
    *index = latest;
    return 0;
}

int photo_catalog_next(const uint16_t *indices, size_t count,
                       uint16_t current, uint16_t *index)
{
    size_t position;
    uint16_t lowest;
    uint16_t next = 0U;

    if ((indices == NULL) || (count == 0U) || (index == NULL))
        return -1;
    lowest = indices[0];
    for (position = 0U; position < count; position++)
    {
        uint16_t candidate = indices[position];

        if (candidate < lowest)
            lowest = candidate;
        if ((candidate > current) && ((next == 0U) || (candidate < next)))
            next = candidate;
    }
    *index = next == 0U ? lowest : next;
    return 0;
}
