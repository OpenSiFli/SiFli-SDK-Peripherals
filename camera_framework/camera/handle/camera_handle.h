/******************************************************************************
 * Copyright (C) 2026 SiFli, Inc.(Gmbh) or its affiliates.
 * 
 * All Rights Reserved.
 * 
 * @file camera_handle.h
 * 
 * @par dependencies 
 * - <Driver_Layer>.h
 * - stdbool.h
 * - stdint.h
 * 
 * @author SiFli 思澈科技
 * 
 * @brief Provide the HAL APIs of camera handler 
 * and corresponding operations.
 * 
 * Processing flow:
 * 
 * Call directly.
 * 
 * @version V1.0 2026-4-3
 *
 * @note 1 tab == 4 spaces!
 * 
 *****************************************************************************/
 
#ifndef __CAMERA_HANDLE_H
#define __CAMERA_HANDLE_H

//******************************** Includes *********************************//
#include <stdint.h>
#include <rtdevice.h>
//******************************** Includes *********************************//

//******************************** Defines **********************************//
#define CAMERA_DEVICE_NAME "camera"

#ifdef __cplusplus
extern "C" {
#endif
//******************************** Defines **********************************//

//******************************** Typedefs *********************************//

/* --------------------------------------------------------------------------
 * Application-facing types
 *
 * These types are intended for callers (applications/examples). Keep their
 * layout stable because they form the public API between the application
 * and the camera handle layer.
 * -------------------------------------------------------------------------*/
typedef enum
{
  PIXFORMAT_RGB565,
  PIXFORMAT_YUV422,
  PIXFORMAT_JPEG,
  PIXFORMAT_RAW8,
  PIXFORMAT_INVALID
} pixformat_t;

typedef enum
{
  FRAMESIZE_96X96,
  FRAMESIZE_QQVGA,
  FRAMESIZE_128X128,
  FRAMESIZE_QCIF,
  FRAMESIZE_HQVGA,
  FRAMESIZE_240X240,
  FRAMESIZE_QVGA,
  FRAMESIZE_320X320,
  FRAMESIZE_CIF,
  FRAMESIZE_HVGA,
  FRAMESIZE_VGA,
  FRAMESIZE_SVGA,
  FRAMESIZE_XGA,
  FRAMESIZE_HD,
  FRAMESIZE_SXGA,
  FRAMESIZE_UXGA,
  FRAMESIZE_INVALID
} framesize_t;

/** @brief Static capability descriptor exposed by sensor driver. */
typedef struct
{
  const pixformat_t *pixformats;     /* array of supported pixel formats   */
  rt_uint8_t         num_pixformats; /* length of pixformats[]             */
  const framesize_t *framesizes;     /* array of supported frame sizes     */
  rt_uint8_t         num_framesizes; /* length of framesizes[]             */
  rt_size_t          max_buffer_size;/* worst-case bytes for one raw frame */
} camera_capabilities_t;

/* Frame descriptor passed to consumers via callbacks or dequeue APIs. This
 * structure is part of the public contract: applications receive a shallow
 * copy and must not free the underlying buffer (driver owns DMA buffers). */
typedef struct
{
  void *buffer;
  rt_size_t buffer_size;
  rt_size_t frame_size;
  rt_uint32_t sequence;
  rt_uint8_t buffer_index;
} camera_stream_frame_t;

typedef void (*camera_stream_frame_callback_t)(void *context,
                         const camera_stream_frame_t *frame);

/* Return/status codes used by camera handle APIs. Placed here so the
 * application-facing callback types can reference them. */
typedef enum
{
  CAMERA_OK                = 0,     /* Operation completed successfully.  */
  CAMERA_ERROR             = 1,     /* Run-time error without case matched*/
  CAMERA_ERRORTIMEOUT      = 2,     /* Operation failed with timeout      */
  CAMERA_ERRORRESOURCE     = 3,     /* Resource not available.            */
  CAMERA_ERRORPARAMETER    = 4,     /* Parameter error.                   */
  CAMERA_ERRORNOMEMORY     = 5,     /* Out of memory.                     */
  CAMERA_ERRORISR          = 6,     /* Not allowed in ISR context         */
  CAMERA_RESERVED  = 0x7FFFFFFF     /* Reserved  May check the caller     */
} camera_handle_status_t;

/** @brief Completion callback for async single-shot capture. */
typedef void (*camera_capture_done_callback_t)(void *context,
                                               camera_handle_status_t status,
                                               rt_size_t frame_size);

typedef struct
{
  pixformat_t pixformat;
  framesize_t framesize;
  uint8_t quality;
} camera_capture_config_t;

typedef struct
{
  void *buffer;
  rt_size_t buffer_size;
  rt_size_t frame_size;
} camera_capture_request_t;

typedef struct
{
  void *buffers[2];
  rt_size_t buffer_size;
} camera_stream_config_t;

typedef struct camera_handler_instance camera_handler_instance_t;


//******************************** APIs *************************************//

/** @brief Query driver capability descriptor. */
camera_handle_status_t camera_get_capabilities(camera_handler_instance_t *instance,
                                               const camera_capabilities_t **caps);

/** @brief Allocate, initialize and open one camera handle instance. */
camera_handle_status_t camera_handler_instance_init(
                         camera_handler_instance_t **instance);


/** @brief Blocking single-shot capture. */
camera_handle_status_t camera_capture_single(camera_handler_instance_t *instance,
                                             camera_capture_request_t *request);

/** @brief Non-blocking single-shot capture. */
camera_handle_status_t camera_capture_single_async(
    camera_handler_instance_t         *instance,
    camera_capture_request_t          *request,
    camera_capture_done_callback_t     callback,
    void                              *context);

/** @brief Start continuous stream with two DMA buffers. */
camera_handle_status_t camera_start_stream(camera_handler_instance_t *instance,
                                           const camera_stream_config_t *config);

/** @brief Dequeue next stream frame, waiting up to @p timeout ticks. */
camera_handle_status_t camera_get_stream_frame(camera_handler_instance_t *instance,
                                               camera_stream_frame_t *frame,
                                               rt_int32_t timeout);

/** @brief Stop stream and clear queue state. */
camera_handle_status_t camera_stop_stream(camera_handler_instance_t *instance);

/** @brief Apply pixformat/framesize/quality and cache active config. */
camera_handle_status_t camera_change_settings(camera_handler_instance_t *instance,
                                              const camera_capture_config_t *config);

/** @brief Read the total number of dropped stream frames since last start. */
camera_handle_status_t camera_get_stream_dropped_count(
                        camera_handler_instance_t *instance,
                        rt_uint32_t *dropped_count);

/** @brief Close driver session, release handle resources and clear pointer. */
camera_handle_status_t camera_deinit(camera_handler_instance_t **instance);

//******************************** APIs *************************************//

#ifdef __cplusplus
}
#endif


#endif // __CAMERA_HANDLE_H
