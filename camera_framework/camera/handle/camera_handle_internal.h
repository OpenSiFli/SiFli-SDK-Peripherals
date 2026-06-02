/******************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file camera_handle_internal.h
 *
 * @brief Internal camera handle contracts shared by handle and sensor drivers.
 *
 * This header is private to the camera framework implementation. Application
 * code should include only camera_handle.h.
 *****************************************************************************/

#ifndef __CAMERA_HANDLE_INTERNAL_H
#define __CAMERA_HANDLE_INTERNAL_H

#include "camera_handle.h"

typedef struct camera_device_ops camera_device_ops_t;

typedef struct
{
  void *buffers[2];                          /* ping-pong destination buffers */
  rt_size_t buffer_size;                     /* size of each buffer in bytes  */
  camera_stream_frame_callback_t frame_callback; /* per-frame notify callback */
  void *callback_context;                    /* opaque callback user pointer   */
} camera_stream_start_args_t;

struct camera_device_ops
{
  const camera_capabilities_t *capabilities; /* static capability descriptor */
  int (*open)(void);                         /* bring up sensor + bus */
  int (*close)(void);                        /* tear down sensor + bus */
  int (*set_pixformat)(pixformat_t pixformat);
  int (*set_framesize)(framesize_t framesize);
  int (*set_quality)(uint8_t quality);
  rt_size_t (*capture)(void *buffer, rt_size_t buffer_size); /* blocking single-shot */
  int (*capture_async)(void *buffer,
                       rt_size_t buffer_size,
                       camera_capture_done_callback_t callback,
                       void *context);       /* non-blocking single-shot */
  int (*start_stream)(const camera_stream_start_args_t *args);
  int (*stop_stream)(void);
};

typedef struct
{
  void *buffer;
  rt_size_t size;
  rt_uint32_t sequence;
} camera_jpeg_segment_t;

typedef struct
{
  rt_bool_t segment_mode;
  uint8_t *ring_base;
  rt_size_t ring_size;
  uint8_t *normalize_base;
  rt_size_t normalize_size;
  rt_size_t soi_offset;
  rt_uint8_t soi_found;
  rt_uint8_t prev_byte;
  rt_uint8_t prev_valid;
  camera_jpeg_segment_t segments[8];
  rt_uint8_t seg_head;
  rt_uint8_t seg_tail;
  rt_uint8_t seg_count;
  struct rt_semaphore parse_sem;
  rt_bool_t parse_sem_initialized;
  struct rt_semaphore exit_sem;
  rt_bool_t exit_sem_initialized;
  rt_thread_t parse_thread;
  rt_bool_t parse_thread_running;
} camera_jpeg_stream_runtime_t;

typedef struct
{
  rt_bool_t enabled;
  struct rt_semaphore frame_sem;
  rt_bool_t sem_initialized;
  camera_stream_frame_t ready_frames[4];
  rt_uint8_t head;
  rt_uint8_t tail;
  rt_uint8_t count;
  rt_uint32_t dropped_count;
  camera_jpeg_stream_runtime_t jpeg;
} camera_stream_runtime_t;

struct camera_handler_instance
{
  camera_capture_config_t active_config;
  camera_stream_runtime_t stream;
  const camera_device_ops_t *device_ops;
  rt_bool_t is_open;
};

#ifdef CAMERA_HANDLE_TESTING
void camera_handle_set_test_ops(const camera_device_ops_t *ops);
#endif

#endif /* __CAMERA_HANDLE_INTERNAL_H */
