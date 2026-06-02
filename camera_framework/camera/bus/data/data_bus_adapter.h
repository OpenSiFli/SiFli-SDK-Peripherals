/********************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file data_bus_adapter.h
 *
 * @par dependencies
 * - stdint.h
 * - stddef.h
 *
 * @author SiFli 思澈科技
 *
 * @brief Data bus adapter interface for camera drivers.
 *
 * This header defines a generic data-bus abstraction used by camera sensor
 * drivers (for example OV2640) to decouple upper-layer capture logic from
 * concrete transport backends such as DVP, CSI, or SPI.
 *
 * Main design points:
 * - A concrete backend registers one `bus_adapter_t` instance by name.
 * - Backend private state is carried through `bus_adapter_t::priv`.
 * - All operations are grouped in `bus_adapter_ops_t` and uniformly receive
 *   `bus_adapter_t *self` as the first argument.
 * - Thin wrapper APIs validate pointers and dispatch calls to the backend ops.
 *
 * Notes:
 * - `bus_adapter_set_frame_notify_callback()` callback is typically invoked in bus ISR
 *   context; callback work should be short and non-blocking.
 * - Wrapper APIs return unified `bus_status_t` style error codes when dispatch
 *   is invalid or unsupported.
 *
 * @version V1.0 2026-5-11
 *
 * @note 1 tab == 4 spaces!
 *
 ******************************************************************************/
#ifndef __DATA_BUS_ADAPTER_H__
#define __DATA_BUS_ADAPTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

typedef enum {
    BUS_TYPE_INVALID = 0,
    BUS_TYPE_DVP,
    BUS_TYPE_SPI,
    BUS_TYPE_DCMI,
    BUS_TYPE_MAX,
} bus_type_t;

/*
 * Generic capture pixel/stream mode advertised to bus adapters.
 *
 * Concrete adapters (e.g. DVP) may translate this to their native enum.
 * Values are stable and shared between upper-layer drivers and bus adapters,
 * so any new format must be appended at the tail to preserve ABI.
 */
typedef enum {
    BUS_CAPTURE_MODE_JPEG   = 0,  /* JPEG stream (variable length, SOI/EOI framed) */
    BUS_CAPTURE_MODE_RAW    = 1,  /* RAW8 / fixed-size capture                      */
    BUS_CAPTURE_MODE_YUV422 = 2,  /* YUV422 fixed-size capture                      */
    BUS_CAPTURE_MODE_RGB565 = 3,  /* RGB565 fixed-size capture                      */
} bus_capture_mode_t;

/* Error codes returned by bus_adapter_* APIs.
 * Negative values represent errors; BUS_OK (0) means success. */
typedef enum {
    BUS_OK              =  0,
    BUS_ERR_INVALID     = -1,  /* invalid argument (NULL self / bad cfg) */
    BUS_ERR_NOT_SUPPORTED = -2,/* op not implemented by this adapter */
    BUS_ERR_NO_SLOT     = -3,  /* registry full */
    BUS_ERR_HW          = -4,  /* underlying hardware reported failure */
} bus_status_t;

struct bus_adapter;
typedef struct bus_adapter bus_adapter_t;

/* Frame ready callback. Called from bus IRQ context; keep work minimal. */
typedef void (*bus_frame_notify_callback_t)(void *buffer,
                                            uint32_t length,
                                            void *user_data);

/*
 * Bus adapter operations.
 *
 * - init / deinit / start / stop are lifecycle.
 * - set_frame_notify_callback registers a callback invoked when a complete frame is ready.
 * - start_capture / abort_capture / rearm_capture drive a single-frame capture
 *   sequence that the bus implementation uses to build its data path.
 * - start_capture / rearm_capture / abort_capture represent the minimal
 *   capture-control surface shared across bus backends.
 *
 * Every op takes the owning adapter as `self`; implementations cast `self->priv`
 * to access their private state.
 *
 * Any op may be NULL if the bus does not support that operation; callers must
 * handle NULL gracefully (see bus_adapter_* helper wrappers below).
 */
typedef struct bus_adapter_ops {
    int (*init)(bus_adapter_t *self);
    int (*deinit)(bus_adapter_t *self);
    int (*start)(bus_adapter_t *self);
    int (*stop)(bus_adapter_t *self);
    int (*set_frame_notify_callback)(bus_adapter_t *self,
                                     bus_frame_notify_callback_t callback,
                                     void *user_data);

    int (*start_capture)(bus_adapter_t *self, void *buffer, uint32_t size);
    int (*rearm_capture)(bus_adapter_t *self, void *buffer, uint32_t size);
    int (*abort_capture)(bus_adapter_t *self);
    int (*set_pingpong_size)(bus_adapter_t *self, uint32_t size);
    /*
     * Change the capture mode at runtime.
     *
     * Implementations are expected to (1) abort any in-flight transfer and
     * stop the data path before mutating internal state, and (2) leave the
     * adapter in the same lifecycle state as on entry (i.e. caller is still
     * responsible for re-issuing bus_adapter_start when needed). Returns
     * BUS_OK or a negative bus_status_t value.
     */
    int (*set_mode)(bus_adapter_t *self, bus_capture_mode_t mode);
    /*
     * Optional diagnostics hook. Invoked when the upper layer needs to dump
     * bus-specific hardware state (DMA counters, timer registers, etc.).
     * NULL means this adapter provides no diagnostic output.
     */
    void (*dump_state)(bus_adapter_t *self);
} bus_adapter_ops_t;

struct bus_adapter {
    const char *name;
    bus_type_t type;
    const bus_adapter_ops_t *ops;
    void *priv;
};

/** @brief Register adapter in global registry (idempotent by name). */
int bus_adapter_register(bus_adapter_t *adapter);

/** @brief Find adapter by name. */
bus_adapter_t *bus_adapter_find(const char *name);

/*
 * Thin wrappers that validate self/ops and dispatch to ops table.
 *
 * Return convention:
 * - BUS_OK on success
 * - BUS_ERR_INVALID when self/ops is invalid
 * - BUS_ERR_NOT_SUPPORTED when the specific op is not implemented
 * - or underlying adapter return code
 */

/** @brief Wrapper for init op. */
int bus_adapter_init(bus_adapter_t *self);

/** @brief Wrapper for deinit op. */
int bus_adapter_deinit(bus_adapter_t *self);

/** @brief Wrapper for start op. */
int bus_adapter_start(bus_adapter_t *self);

/** @brief Wrapper for stop op. */
int bus_adapter_stop(bus_adapter_t *self);

/** @brief Wrapper for frame callback registration op. */
int bus_adapter_set_frame_notify_callback(bus_adapter_t *self,
                                          bus_frame_notify_callback_t callback,
                                          void *user_data);

/** @brief Wrapper for start_capture op. */
int bus_adapter_start_capture(bus_adapter_t *self, void *buffer, uint32_t size);

/** @brief Wrapper for rearm_capture op. */
int bus_adapter_rearm_capture(bus_adapter_t *self, void *buffer, uint32_t size);

/** @brief Wrapper for abort_capture op. */
int bus_adapter_abort_capture(bus_adapter_t *self);

/** @brief Wrapper for set_pingpong_size op. */
int bus_adapter_set_pingpong_size(bus_adapter_t *self, uint32_t size);

/** @brief Wrapper for set_mode op. */
int bus_adapter_set_mode(bus_adapter_t *self, bus_capture_mode_t mode);

/** @brief Call optional adapter dump_state hook. */
void bus_adapter_dump_state(bus_adapter_t *self);

#ifdef __cplusplus
}
#endif

#endif /* __DATA_BUS_ADAPTER_H__ */
