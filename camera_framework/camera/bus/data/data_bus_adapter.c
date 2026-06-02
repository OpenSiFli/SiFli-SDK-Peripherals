/********************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file data_bus_adapter.c
 *
 * @par dependencies
 * - data_bus_adapter.h
 * - string.h
 *
 * @author SiFli 思澈科技
 *
 * @brief Data bus adapter registry and dispatch implementation.
 *
 * This module provides a lightweight static registry for `bus_adapter_t`
 * instances and implements thin wrapper APIs that validate pointers then
 * dispatch lifecycle/capture operations through `bus_adapter_ops_t`.
 *
 * Processing flow:
 * - Backend registers itself once via `bus_adapter_register()`.
 * - Upper layers locate backend by name with `bus_adapter_find()`.
 * - Wrapper APIs (`bus_adapter_*`) check `self` and op pointers.
 * - On valid dispatch, wrappers call into backend implementation directly.
 *
 * Notes:
 * - Registry is static and has a fixed capacity (`BUS_ADAPTER_MAX`).
 * - Duplicate registration by adapter name is treated as success.
 * - Wrappers return unified error codes for invalid/unsupported operations.
 *
 * @version V1.0 2026-5-11
 *
 * @note 1 tab == 4 spaces!
 *
 ******************************************************************************/
#include "data_bus_adapter.h"
#include <string.h>

#define BUS_ADAPTER_MAX 4
static bus_adapter_t *g_adapters[BUS_ADAPTER_MAX];
static int g_adapter_count = 0;

/** @brief Register adapter in static registry (idempotent by name). */
int bus_adapter_register(bus_adapter_t *adapter)
{
    if (!adapter || !adapter->name || !adapter->ops)
        return BUS_ERR_INVALID;

    if (g_adapter_count >= BUS_ADAPTER_MAX)
        return BUS_ERR_NO_SLOT;

    for (int i = 0; i < g_adapter_count; i++) {
        if (strcmp(g_adapters[i]->name, adapter->name) == 0)
            return BUS_OK;
    }

    g_adapters[g_adapter_count++] = adapter;
    return BUS_OK;
}

/** @brief Find adapter by name. */
bus_adapter_t *bus_adapter_find(const char *name)
{
    if (!name)
        return NULL;
    for (int i = 0; i < g_adapter_count; i++) {
        if (strcmp(g_adapters[i]->name, name) == 0)
            return g_adapters[i];
    }
    return NULL;
}

/** @brief Check adapter and ops table pointer. */
static int bus_adapter_check(bus_adapter_t *self)
{
    if (!self || !self->ops)
    {
        return BUS_ERR_INVALID;
    }

    return BUS_OK;
}

/** @brief Wrapper for init op. */
int bus_adapter_init(bus_adapter_t *self)
{
    if (bus_adapter_check(self) != BUS_OK)
        return BUS_ERR_INVALID;
    if (!self->ops->init)
        return BUS_ERR_NOT_SUPPORTED;
    return self->ops->init(self);
}

/** @brief Wrapper for deinit op. */
int bus_adapter_deinit(bus_adapter_t *self)
{
    if (bus_adapter_check(self) != BUS_OK)
        return BUS_ERR_INVALID;
    if (!self->ops->deinit)
        return BUS_ERR_NOT_SUPPORTED;
    return self->ops->deinit(self);
}

/** @brief Wrapper for start op. */
int bus_adapter_start(bus_adapter_t *self)
{
    if (bus_adapter_check(self) != BUS_OK)
        return BUS_ERR_INVALID;
    if (!self->ops->start)
        return BUS_ERR_NOT_SUPPORTED;
    return self->ops->start(self);
}

/** @brief Wrapper for stop op. */
int bus_adapter_stop(bus_adapter_t *self)
{
    if (bus_adapter_check(self) != BUS_OK)
        return BUS_ERR_INVALID;
    if (!self->ops->stop)
        return BUS_ERR_NOT_SUPPORTED;
    return self->ops->stop(self);
}

/** @brief Wrapper for frame callback registration op. */
int bus_adapter_set_frame_notify_callback(bus_adapter_t *self,
                                          bus_frame_notify_callback_t callback,
                                          void *user_data)
{
    if (bus_adapter_check(self) != BUS_OK)
        return BUS_ERR_INVALID;
    if (!self->ops->set_frame_notify_callback)
        return BUS_ERR_NOT_SUPPORTED;
    return self->ops->set_frame_notify_callback(self, callback, user_data);
}

/** @brief Wrapper for start_capture op. */
int bus_adapter_start_capture(bus_adapter_t *self, void *buffer, uint32_t size)
{
    if (bus_adapter_check(self) != BUS_OK)
        return BUS_ERR_INVALID;
    if (!self->ops->start_capture)
        return BUS_ERR_NOT_SUPPORTED;
    return self->ops->start_capture(self, buffer, size);
}

/** @brief Wrapper for rearm_capture op. */
int bus_adapter_rearm_capture(bus_adapter_t *self, void *buffer, uint32_t size)
{
    if (bus_adapter_check(self) != BUS_OK)
        return BUS_ERR_INVALID;
    if (!self->ops->rearm_capture)
        return BUS_ERR_NOT_SUPPORTED;
    return self->ops->rearm_capture(self, buffer, size);
}

/** @brief Wrapper for abort_capture op. */
int bus_adapter_abort_capture(bus_adapter_t *self)
{
    if (bus_adapter_check(self) != BUS_OK)
        return BUS_ERR_INVALID;
    if (!self->ops->abort_capture)
        return BUS_ERR_NOT_SUPPORTED;
    return self->ops->abort_capture(self);
}

/** @brief Wrapper for set_pingpong_size op. */
int bus_adapter_set_pingpong_size(bus_adapter_t *self, uint32_t size)
{
    if (bus_adapter_check(self) != BUS_OK)
        return BUS_ERR_INVALID;
    if (!self->ops->set_pingpong_size)
        return BUS_ERR_NOT_SUPPORTED;
    return self->ops->set_pingpong_size(self, size);
}

/** @brief Wrapper for set_mode op. */
int bus_adapter_set_mode(bus_adapter_t *self, bus_capture_mode_t mode)
{
    if (bus_adapter_check(self) != BUS_OK)
        return BUS_ERR_INVALID;
    if (!self->ops->set_mode)
        return BUS_ERR_NOT_SUPPORTED;
    return self->ops->set_mode(self, mode);
}

/** @brief Call optional adapter dump_state hook. */
void bus_adapter_dump_state(bus_adapter_t *self)
{
    if (!self || !self->ops || !self->ops->dump_state)
        return;
    self->ops->dump_state(self);
}
