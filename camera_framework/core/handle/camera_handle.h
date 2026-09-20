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
  FRAMESIZE_240X320,
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

typedef enum
{
  CAMERA_STREAM_MODE_FRAME = 0,
  CAMERA_STREAM_MODE_HALF_FRAME,
} camera_stream_mode_t;

typedef enum
{
  CAMERA_STREAM_EVENT_FRAME = 0,
  CAMERA_STREAM_EVENT_HALF_FIRST,
  CAMERA_STREAM_EVENT_HALF_SECOND,
} camera_stream_event_type_t;

/* Stream events borrow caller-owned DMA memory, which must stay allocated
 * until a successful stop. FRAME describes one completed image. HALF_FIRST
 * and HALF_SECOND both describe the same full image buffer; event_type selects
 * its upper or lower half, and is_complete is true only for HALF_SECOND.
 * DMA keeps running while events are consumed. Cache maintenance remains the
 * caller's responsibility before CPU access. */
typedef struct
{
  void *buffer;
  rt_size_t buffer_size;
  rt_size_t frame_size;
  rt_uint32_t sequence;
  rt_uint8_t buffer_index;
  rt_bool_t is_complete;
  rt_err_t error; /* zero for data; negative capture error may have no buffer */
  rt_uint32_t timestamp_ticks; /* capture event tick; zero if unavailable */
  camera_stream_event_type_t event_type;
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

/**
 * @brief Blocking single-shot capture with a per-request wait timeout.
 *
 * The timeout bounds the frame wait. On every return, call camera_stop_capture
 * successfully before changing settings or reusing/freeing the destination.
 */
camera_handle_status_t camera_capture_single_timeout(camera_handler_instance_t *instance,
                                                     camera_capture_request_t *request,
                                                     uint32_t timeout_ms);

/**
 * @brief Receive a bounded continuous JPEG sequence and retain its final frame.
 *
 * frame_count must be at least one. The backend starts hardware once, receives
 * the requested consecutive frames and places the final JPEG at request->buffer.
 * buffer_size is the total capture storage capacity. Unsupported backends fail;
 * this operation never falls back to repeated single-shot captures.
 * timeout_ms bounds the whole frame wait. On every return, camera_stop_capture
 * must succeed before changing settings or reusing/freeing the destination.
 */
camera_handle_status_t camera_capture_frames_timeout(camera_handler_instance_t *instance,
                                                     camera_capture_request_t *request,
                                                     uint32_t frame_count,
                                                     uint32_t timeout_ms);

/**
 * @brief Stop a synchronous capture and check that callbacks are idle.
 *
 * This call may wait for the framework API mutex.
 * CAMERA_OK means the backend stop succeeded. Reusing the destination also
 * requires a backend that confirms hardware and callback retirement, as DCMI
 * does. CAMERA_ERRORRESOURCE can mean a completion callback is still returning;
 * retry from another task after yielding. Any failure retains ownership.
 * Async capture and active streams must use their own lifecycle APIs.
 */
camera_handle_status_t camera_stop_capture(camera_handler_instance_t *instance);

/**
 * @brief Start a non-blocking single-shot capture.
 *
 * Conflicting capture, configuration, stream-start and deinit operations
 * return CAMERA_ERRORRESOURCE until the completion callback runs.
 */
camera_handle_status_t camera_capture_single_async(
    camera_handler_instance_t         *instance,
    camera_capture_request_t          *request,
    camera_capture_done_callback_t     callback,
    void                              *context);

/** @brief Start continuous full-frame streaming with two DMA buffers. */
camera_handle_status_t camera_start_stream(camera_handler_instance_t *instance,
                                           const camera_stream_config_t *config);

/**
 * @brief Start a stream with complete-frame or half-frame notifications.
 *
 * HALF_FRAME uses buffers[0] as one complete raw-image buffer, buffers[1] must
 * be NULL, and buffer_size must equal the image size. The image height must be
 * even. Each HT/TC notification retains the full image address and size.
 * Unsupported backends or formats reject HALF_FRAME.
 */
camera_handle_status_t camera_start_stream_mode(camera_handler_instance_t *instance,
                                                const camera_stream_config_t *config,
                                                camera_stream_mode_t mode);

/** @brief Dequeue next stream frame, waiting up to @p timeout ticks. */
camera_handle_status_t camera_get_stream_frame(camera_handler_instance_t *instance,
                                               camera_stream_frame_t *frame,
                                               rt_int32_t timeout);

/**
 * @brief Test whether a native stream frame still names a readable generation.
 *
 * Returns RT_FALSE for stale/error events, an inactive stream, ISR callers, or
 * a backend without this operation. In HALF_FRAME mode this compares the
 * notified half's sequence with the latest HT/TC state: HALF_FIRST expires at
 * TC, and HALF_SECOND expires at the following HT. It does not poll DMA.
 * A true result does not reserve memory or make its CPU cache coherent.
 */
rt_bool_t camera_stream_frame_is_valid(camera_handler_instance_t *instance,
                                       const camera_stream_frame_t *frame);

/** @brief Stop stream and clear queue state after the driver confirms success. */
camera_handle_status_t camera_stop_stream(camera_handler_instance_t *instance);

/** @brief Apply pixformat/framesize/quality and cache active config. */
camera_handle_status_t camera_change_settings(camera_handler_instance_t *instance,
                                              const camera_capture_config_t *config);

/**
 * @brief Set sensor output rotation to 0 or 180 degrees while capture is stopped.
 *
 * Returns CAMERA_ERRORRESOURCE for active streaming/async capture or a driver
 * without this optional operation. Reapply after reopening or changing settings.
 */
camera_handle_status_t camera_set_rotation(camera_handler_instance_t *instance,
                                           uint16_t degrees);

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
