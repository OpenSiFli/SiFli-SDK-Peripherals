/**
 * @file    ov2640.h
 * @brief   OV2640 camera sensor driver types and internal control IDs
 *
 * This header provides the OV2640 sensor data structures, runtime state used
 * by the driver implementation, and the internal command IDs consumed by
 * ov2640.c helper dispatch.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#ifndef OV2640_H_
#define OV2640_H_

#include <stdint.h>
#include <stdbool.h>
#include "sccb.h"
#include "rtthread.h"
#include "rtconfig.h"
#include "../../handle/camera_handle_internal.h"
#include "data_bus_adapter.h"

#define OV2640_ADDR 0x30  /* 7-bit I2C/SCCB slave address */

/*
 *******************************************************************************
 * Data types
 *******************************************************************************
 */

/** @brief Runtime state for continuous stream capture. */
typedef struct
{
    uint8_t    *buffers[2];           /* ping-pong buffer pair */
    rt_size_t   buffer_size;          /* size of each buffer in bytes */
    rt_uint8_t  active_buffer_index;  /* index of the buffer currently being filled */
    rt_uint32_t sequence;             /* monotonically increasing frame counter */
    camera_stream_frame_callback_t frame_callback;  /* user callback per frame; NULL = single-shot mode */
    void       *callback_context;     /* opaque pointer forwarded to callback */
} sensor_stream_state_t;

typedef struct
{
    camera_capture_done_callback_t callback;
    void *context;
    rt_bool_t in_flight;
} sensor_async_capture_state_t;

/** @brief OV2640 driver runtime instance. */
typedef struct {
    framesize_t                     framesize;      /* active frame size */
    uint8_t                         quality;        /* active JPEG quality */
    pixformat_t                     pixformat;      /* active pixel format */
    volatile rt_size_t              last_frame_size;/* last completed frame size */
    struct rt_semaphore             frame_sem;      /* posted by frame callback, consumed by capture path */
    sensor_stream_state_t           stream;         /* streaming state (inactive for single-shot capture) */
    sensor_async_capture_state_t    async_capture;  /* non-blocking single-shot state */
    rt_uint8_t                      current_bank;   /* cached BANK_SEL value; ov2640_bank_t cast at use site */
} sensor_device_t;

/* Internal command IDs used by ov2640.c helper dispatch. Not part of the
 * application-facing API. */

/* --- Image format & resolution ------------------------------------------ */
#define CMD_SET_PIXFORMAT       0x01  /* pixformat_t  */
#define CMD_SET_FRAMESIZE       0x02  /* framesize_t  */

/* --- Image quality ------------------------------------------------------- */
#define CMD_SET_QUALITY         0x06  /* int  0…63   */

/* --- Capture control ----------------------------------------------------- */
#define CMD_START_STREAM        0x22  /* camera_stream_start_args_t* */
#define CMD_STOP_STREAM         0x23  /* no argument */

/*
 *******************************************************************************
 * Public API
 *******************************************************************************
 */

/** @brief OV2640 camera_device_ops table. */
extern const camera_device_ops_t ov2640_ops;

#endif /* OV2640_H_ */
