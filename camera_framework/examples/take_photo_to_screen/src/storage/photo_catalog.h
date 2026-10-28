#ifndef PHOTO_CATALOG_H
#define PHOTO_CATALOG_H

#include <stddef.h>
#include <stdint.h>

int photo_catalog_parse_index(const char *name, uint16_t *index);
int photo_catalog_latest(const uint16_t *indices, size_t count,
                         uint16_t *index);
int photo_catalog_next(const uint16_t *indices, size_t count,
                       uint16_t current, uint16_t *index);

#endif
