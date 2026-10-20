/**
 * @file arducam_fifo.c
 * @brief Arducam FIFO SPI data-bus adapter.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#include "arducam_fifo_controller.h"
#include "data_bus_adapter.h"

#include <bf0_hal.h>
#include <drv_spi.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <string.h>

#define DBG_TAG "cam.ardufifo"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define ARDUCAM_FIFO_BUS_ADAPTER_NAME "arducam_fifo"
#define ARDUCAM_FIFO_WORKER_PRIORITY (RT_THREAD_PRIORITY_HIGH + 2)
#define ARDUCAM_FIFO_WORKER_TICK 10U

/* The SPI pads belong to the board: bsp_pinmux.c muxes SCLK/DIO/DI/CS. Which
 * SPI instance the module sits on is picked in the Data bus menu
 * (CAMERA_ARDUCAM_SPI_BUS1/2 -> CAMERA_ARDUCAM_SPI_BUS_NAME); the device label,
 * clock rate, FIFO scratch size, worker stack and frame wait are backend
 * constants. */
#if !defined(CAMERA_ARDUCAM_SPI_BUS_NAME)
#error "the ArduCam FIFO SPI bus choice is missing (CAMERA_ARDUCAM_SPI_BUS_NAME)"
#endif

#define ARDUCAM_FIFO_SPI_BUS_NAME CAMERA_ARDUCAM_SPI_BUS_NAME
#define ARDUCAM_FIFO_SPI_DEVICE_NAME "arducam"
#define ARDUCAM_FIFO_SPI_MAX_HZ 8000000U
#define ARDUCAM_FIFO_SCRATCH_SIZE 8192U
#define ARDUCAM_FIFO_WORKER_STACK_SIZE 2048U
#define ARDUCAM_FIFO_FRAME_WAIT_MS 30000U

typedef struct
{
    struct rt_spi_device *spi;
    struct rt_semaphore *capture_sem;
    struct rt_mutex *lock;
    rt_thread_t worker;
    bus_frame_notify_callback_t callback;
    void *callback_data;
    uint8_t *capture_buffer;
    uint32_t capture_buffer_size;
    uint32_t expected_raw_size;
    uint32_t capture_generation;
    bus_capture_mode_t mode;
    rt_bool_t initialized;
    rt_bool_t running;
    rt_bool_t capture_active;
    rt_bool_t burst_open;
    rt_bool_t worker_exit;
} arducam_fifo_handle_t;

typedef struct
{
    arducam_fifo_handle_t *handle;
    uint32_t generation;
    uint8_t *data;
    uint32_t capacity;
    uint32_t size;
} arducam_fifo_writer_t;

static uint8_t s_arducam_fifo_scratch[ARDUCAM_FIFO_SCRATCH_SIZE]
    __attribute__((aligned(64)));
static arducam_fifo_handle_t s_arducam_fifo_handle;
static const bus_adapter_ops_t s_arducam_fifo_ops;
static int arducam_fifo_cancelled(void *context);
static bus_adapter_t s_arducam_fifo_adapter =
{
    .name = ARDUCAM_FIFO_BUS_ADAPTER_NAME,
    .type = BUS_TYPE_SPI,
    .ops = &s_arducam_fifo_ops,
    .priv = &s_arducam_fifo_handle,
};

static int arducam_fifo_chip_write(void *context, uint8_t reg, uint8_t value)
{
    arducam_fifo_writer_t *writer = context;
    arducam_fifo_handle_t *handle = writer->handle;
    uint8_t command[2] = {(uint8_t)(reg | 0x80U), value};

    return rt_spi_send(handle->spi, command, sizeof(command)) ==
           sizeof(command) ? 0 : -1;
}

static int arducam_fifo_chip_read(void *context, uint8_t reg, uint8_t *value)
{
    arducam_fifo_writer_t *writer = context;
    arducam_fifo_handle_t *handle = writer->handle;
    uint8_t command = reg & 0x7fU;

    return rt_spi_send_then_recv(handle->spi, &command, sizeof(command),
                                 value, 1U) == RT_EOK ? 0 : -1;
}

static int arducam_fifo_begin(void *context)
{
    arducam_fifo_writer_t *writer = context;
    arducam_fifo_handle_t *handle = writer->handle;
    uint8_t command = 0x3cU;
    struct rt_spi_message message =
    {
        .send_buf = &command,
        .length = 1U,
    };

    if (rt_spi_take_bus(handle->spi) != RT_EOK)
        return -1;
    if (rt_spi_take(handle->spi) != RT_EOK)
    {
        rt_spi_release_bus(handle->spi);
        return -1;
    }
    if (handle->spi->bus->ops->xfer(handle->spi, &message) != 1U)
    {
        rt_spi_release(handle->spi);
        rt_spi_release_bus(handle->spi);
        return -1;
    }
    handle->burst_open = RT_TRUE;
    return 0;
}

static int arducam_fifo_read(void *context, uint8_t *data, size_t size)
{
    arducam_fifo_writer_t *writer = context;
    arducam_fifo_handle_t *handle = writer->handle;
    struct rt_spi_message message =
    {
        .recv_buf = data,
        .length = size,
    };

    if (!handle->burst_open)
        return -1;
    return handle->spi->bus->ops->xfer(handle->spi, &message) == size ?
           0 : -1;
}

static int arducam_fifo_end_transfer(arducam_fifo_handle_t *handle)
{
    int result = 0;

    if (!handle->burst_open)
        return -1;
    if (rt_spi_release(handle->spi) != RT_EOK)
        result = -1;
    if (rt_spi_release_bus(handle->spi) != RT_EOK)
        result = -1;
    handle->burst_open = RT_FALSE;
    return result;
}

static int arducam_fifo_end(void *context)
{
    arducam_fifo_writer_t *writer = context;

    return arducam_fifo_end_transfer(writer->handle);
}

static void arducam_fifo_delay(void *context, uint32_t milliseconds)
{
    (void)context;
    rt_thread_mdelay(milliseconds);
}

static const arducam_fifo_controller_ops_t s_arducam_fifo_controller_ops =
{
    .chip_write = arducam_fifo_chip_write,
    .chip_read = arducam_fifo_chip_read,
    .fifo_begin = arducam_fifo_begin,
    .fifo_read = arducam_fifo_read,
    .fifo_end = arducam_fifo_end,
    .delay_ms = arducam_fifo_delay,
    .is_cancelled = arducam_fifo_cancelled,
};

static int arducam_fifo_write(void *context, const uint8_t *data, size_t size)
{
    arducam_fifo_writer_t *writer = context;
    arducam_fifo_handle_t *handle = writer->handle;
    int result = -1;

    if (rt_mutex_take(handle->lock, RT_WAITING_FOREVER) != RT_EOK)
        return -1;
    if (handle->running && handle->capture_active && !handle->worker_exit &&
        writer->generation == handle->capture_generation &&
        size <= writer->capacity - writer->size)
    {
        memcpy(writer->data + writer->size, data, size);
        writer->size += (uint32_t)size;
        result = 0;
    }
    rt_mutex_release(handle->lock);
    return result;
}

static int arducam_fifo_cancelled(void *context)
{
    arducam_fifo_writer_t *writer = context;
    arducam_fifo_handle_t *handle = writer->handle;
    int cancelled = 1;

    if (rt_mutex_take(handle->lock, RT_WAITING_FOREVER) == RT_EOK)
    {
        cancelled = !handle->running || !handle->capture_active ||
                    handle->worker_exit ||
                    writer->generation != handle->capture_generation;
        rt_mutex_release(handle->lock);
    }
    return cancelled;
}

static void arducam_fifo_worker(void *parameter)
{
    arducam_fifo_handle_t *handle = parameter;

    while (RT_TRUE)
    {
        uint32_t generation;
        uint32_t expected_raw_size;
        uint32_t captured_size = 0U;
        uint8_t *buffer;
        uint32_t buffer_size;
        bus_capture_mode_t mode;
        bus_frame_notify_callback_t callback;
        void *callback_data;
        arducam_fifo_writer_t writer;
        int result;

        if (rt_sem_take(handle->capture_sem, RT_WAITING_FOREVER) != RT_EOK)
            continue;
        if (rt_mutex_take(handle->lock, RT_WAITING_FOREVER) != RT_EOK)
            continue;
        if (handle->worker_exit)
        {
            rt_mutex_release(handle->lock);
            break;
        }
        if (!handle->running || !handle->capture_active)
        {
            rt_mutex_release(handle->lock);
            continue;
        }
        generation = handle->capture_generation;
        buffer = handle->capture_buffer;
        buffer_size = handle->capture_buffer_size;
        expected_raw_size = handle->expected_raw_size;
        mode = handle->mode;
        callback = handle->callback;
        callback_data = handle->callback_data;
        rt_mutex_release(handle->lock);

        writer.handle = handle;
        writer.generation = generation;
        writer.data = buffer;
        writer.capacity = buffer_size;
        writer.size = 0U;
        result = arducam_fifo_controller_capture(
            &s_arducam_fifo_controller_ops, &writer, mode, expected_raw_size,
            s_arducam_fifo_scratch, sizeof(s_arducam_fifo_scratch),
            arducam_fifo_write, &writer, &captured_size,
            ARDUCAM_FIFO_FRAME_WAIT_MS);

        if (rt_mutex_take(handle->lock, RT_WAITING_FOREVER) != RT_EOK)
            continue;
        if (generation != handle->capture_generation || !handle->running)
        {
            rt_mutex_release(handle->lock);
            continue;
        }
        handle->capture_active = RT_FALSE;
        rt_mutex_release(handle->lock);

        if (callback != RT_NULL)
            callback(buffer, result == ARDUCAM_FIFO_OK ? captured_size : 0U,
                     callback_data);
    }
    handle->worker = RT_NULL;
}

static int arducam_fifo_create_worker(arducam_fifo_handle_t *handle)
{
    handle->capture_sem = rt_sem_create("ardufifo", 0, RT_IPC_FLAG_FIFO);
    if (handle->capture_sem == RT_NULL)
        return BUS_ERR_HW;
    handle->lock = rt_mutex_create("ardufifo", RT_IPC_FLAG_PRIO);
    if (handle->lock == RT_NULL)
    {
        rt_sem_delete(handle->capture_sem);
        handle->capture_sem = RT_NULL;
        return BUS_ERR_HW;
    }
    handle->worker = rt_thread_create("ardufifo", arducam_fifo_worker,
                                      handle, ARDUCAM_FIFO_WORKER_STACK_SIZE,
                                      ARDUCAM_FIFO_WORKER_PRIORITY,
                                      ARDUCAM_FIFO_WORKER_TICK);
    if (handle->worker == RT_NULL)
    {
        rt_mutex_delete(handle->lock);
        rt_sem_delete(handle->capture_sem);
        handle->lock = RT_NULL;
        handle->capture_sem = RT_NULL;
        return BUS_ERR_HW;
    }
    rt_thread_startup(handle->worker);
    return BUS_OK;
}

static void arducam_fifo_destroy_worker(arducam_fifo_handle_t *handle)
{
    if (handle->worker != RT_NULL)
    {
        if (rt_mutex_take(handle->lock, RT_WAITING_FOREVER) == RT_EOK)
        {
            handle->worker_exit = RT_TRUE;
            handle->running = RT_FALSE;
            handle->capture_active = RT_FALSE;
            handle->capture_generation++;
            rt_mutex_release(handle->lock);
        }
        rt_sem_release(handle->capture_sem);
        while (handle->worker != RT_NULL)
            rt_thread_mdelay(1U);
    }
    if (handle->lock != RT_NULL)
    {
        rt_mutex_delete(handle->lock);
        handle->lock = RT_NULL;
    }
    if (handle->capture_sem != RT_NULL)
    {
        rt_sem_delete(handle->capture_sem);
        handle->capture_sem = RT_NULL;
    }
}

static int arducam_fifo_init_spi(arducam_fifo_handle_t *handle)
{
    struct rt_spi_configuration config =
    {
        .mode = RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_MASTER,
        .data_width = 8U,
        .max_hz = ARDUCAM_FIFO_SPI_MAX_HZ,
        .frameMode = RT_SPI_MOTO,
    };

    if (rt_device_find(ARDUCAM_FIFO_SPI_DEVICE_NAME) == RT_NULL &&
        rt_hw_spi_device_attach(ARDUCAM_FIFO_SPI_BUS_NAME,
                                ARDUCAM_FIFO_SPI_DEVICE_NAME) != RT_EOK)
        return BUS_ERR_HW;
    handle->spi = (struct rt_spi_device *)rt_device_find(
        ARDUCAM_FIFO_SPI_DEVICE_NAME);
    if (handle->spi == RT_NULL ||
        rt_device_open((rt_device_t)handle->spi,
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DMA_RX |
                       RT_DEVICE_FLAG_DMA_TX) != RT_EOK ||
        rt_spi_configure(handle->spi, &config) != RT_EOK)
        return BUS_ERR_HW;
    return BUS_OK;
}

static void arducam_fifo_deinit_spi(arducam_fifo_handle_t *handle)
{
    if (handle->burst_open)
        arducam_fifo_end_transfer(handle);
    if (handle->spi != RT_NULL)
        rt_device_close((rt_device_t)handle->spi);
    handle->spi = RT_NULL;
}

static int arducam_fifo_configure(bus_adapter_t *self,
                                  const bus_adapter_config_t *config)
{
    arducam_fifo_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL || config == RT_NULL ||
        (config->mode != BUS_CAPTURE_MODE_JPEG &&
         config->mode != BUS_CAPTURE_MODE_RGB565))
        return BUS_ERR_INVALID;
    handle = self->priv;
    if (handle->capture_active)
        return BUS_ERR_HW;
    handle->mode = config->mode;
    return BUS_OK;
}

static int arducam_fifo_init(bus_adapter_t *self)
{
    arducam_fifo_handle_t *handle;
    bus_frame_notify_callback_t callback;
    void *callback_data;
    bus_capture_mode_t mode;
    arducam_fifo_writer_t probe_context = {0};
    int result;

    if (self == RT_NULL || self->priv == RT_NULL)
        return BUS_ERR_INVALID;
    handle = self->priv;
    if (handle->initialized)
        return BUS_OK;
    callback = handle->callback;
    callback_data = handle->callback_data;
    mode = handle->mode;
    memset(handle, 0, sizeof(*handle));
    handle->callback = callback;
    handle->callback_data = callback_data;
    handle->mode = mode;

    result = arducam_fifo_create_worker(handle);
    if (result != BUS_OK)
        return result;
    result = arducam_fifo_init_spi(handle);
    if (result != BUS_OK)
        goto error;
    probe_context.handle = handle;
    result = arducam_fifo_controller_probe(&s_arducam_fifo_controller_ops,
                                           &probe_context);
    if (result != ARDUCAM_FIFO_OK)
    {
        LOG_E("probe failed: %d", result);
        result = BUS_ERR_HW;
        goto error;
    }
    handle->initialized = RT_TRUE;
    LOG_I("initialized: %s at %u Hz", ARDUCAM_FIFO_SPI_BUS_NAME,
          (unsigned int)ARDUCAM_FIFO_SPI_MAX_HZ);
    return BUS_OK;

error:
    arducam_fifo_deinit_spi(handle);
    arducam_fifo_destroy_worker(handle);
    return result;
}

static int arducam_fifo_deinit(bus_adapter_t *self)
{
    arducam_fifo_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL)
        return BUS_ERR_INVALID;
    handle = self->priv;
    if (!handle->initialized)
        return BUS_OK;
    arducam_fifo_destroy_worker(handle);
    arducam_fifo_deinit_spi(handle);
    memset(handle, 0, sizeof(*handle));
    return BUS_OK;
}

static int arducam_fifo_start(bus_adapter_t *self)
{
    arducam_fifo_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL)
        return BUS_ERR_INVALID;
    handle = self->priv;
    if (handle->lock == RT_NULL ||
        rt_mutex_take(handle->lock, RT_WAITING_FOREVER) != RT_EOK)
        return BUS_ERR_HW;
    if (!handle->initialized)
    {
        rt_mutex_release(handle->lock);
        return BUS_ERR_HW;
    }
    handle->running = RT_TRUE;
    rt_mutex_release(handle->lock);
    return BUS_OK;
}

static int arducam_fifo_stop(bus_adapter_t *self)
{
    arducam_fifo_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL)
        return BUS_ERR_INVALID;
    handle = self->priv;
    if (handle->lock != RT_NULL &&
        rt_mutex_take(handle->lock, RT_WAITING_FOREVER) == RT_EOK)
    {
        handle->running = RT_FALSE;
        handle->capture_active = RT_FALSE;
        handle->capture_generation++;
        rt_mutex_release(handle->lock);
    }
    return BUS_OK;
}

static int arducam_fifo_set_callback(bus_adapter_t *self,
                                     bus_frame_notify_callback_t callback,
                                     void *user_data)
{
    arducam_fifo_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL)
        return BUS_ERR_INVALID;
    handle = self->priv;
    handle->callback = callback;
    handle->callback_data = user_data;
    return BUS_OK;
}

static int arducam_fifo_arm_capture(bus_adapter_t *self, void *buffer,
                                    uint32_t size)
{
    arducam_fifo_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL || buffer == RT_NULL ||
        size == 0U)
        return BUS_ERR_INVALID;
    handle = self->priv;
    if (rt_mutex_take(handle->lock, RT_WAITING_FOREVER) != RT_EOK)
        return BUS_ERR_HW;
    if (!handle->initialized || handle->capture_active ||
        (handle->mode == BUS_CAPTURE_MODE_RGB565 &&
         (handle->expected_raw_size == 0U ||
          size < handle->expected_raw_size)))
    {
        rt_mutex_release(handle->lock);
        return BUS_ERR_HW;
    }
    if (!handle->running)
        handle->running = RT_TRUE;
    handle->capture_buffer = buffer;
    handle->capture_buffer_size = size;
    handle->capture_active = RT_TRUE;
    handle->capture_generation++;
    rt_mutex_release(handle->lock);
    rt_sem_release(handle->capture_sem);
    return BUS_OK;
}

static int arducam_fifo_abort_capture(bus_adapter_t *self)
{
    arducam_fifo_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL)
        return BUS_ERR_INVALID;
    handle = self->priv;
    if (rt_mutex_take(handle->lock, RT_WAITING_FOREVER) != RT_EOK)
        return BUS_ERR_HW;
    handle->capture_active = RT_FALSE;
    handle->capture_generation++;
    rt_mutex_release(handle->lock);
    return BUS_OK;
}

static int arducam_fifo_set_pingpong_size(bus_adapter_t *self,
                                          uint32_t size)
{
    arducam_fifo_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL || size < 2U ||
        (size & 1U) != 0U)
        return BUS_ERR_INVALID;
    handle = self->priv;
    handle->expected_raw_size = size / 2U;
    return BUS_OK;
}

static int arducam_fifo_set_mode(bus_adapter_t *self, bus_capture_mode_t mode)
{
    bus_adapter_config_t config = {.mode = mode};

    return arducam_fifo_configure(self, &config);
}

static void arducam_fifo_dump_state(bus_adapter_t *self)
{
    arducam_fifo_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL)
        return;
    handle = self->priv;
    LOG_I("bus=%s running=%u mode=%u active=%u raw=%u",
          ARDUCAM_FIFO_SPI_BUS_NAME, handle->running, handle->mode,
          handle->capture_active, (unsigned int)handle->expected_raw_size);
}

static const bus_adapter_ops_t s_arducam_fifo_ops =
{
    .config = arducam_fifo_configure,
    .init = arducam_fifo_init,
    .deinit = arducam_fifo_deinit,
    .start = arducam_fifo_start,
    .stop = arducam_fifo_stop,
    .set_frame_notify_callback = arducam_fifo_set_callback,
    .start_capture = arducam_fifo_arm_capture,
    .rearm_capture = arducam_fifo_arm_capture,
    .abort_capture = arducam_fifo_abort_capture,
    .set_pingpong_size = arducam_fifo_set_pingpong_size,
    .set_mode = arducam_fifo_set_mode,
    .dump_state = arducam_fifo_dump_state,
};

static int arducam_fifo_register(void)
{
    int result = bus_adapter_register(&s_arducam_fifo_adapter);

    if (result != BUS_OK)
        LOG_E("adapter register failed: %d", result);
    return result;
}
INIT_BOARD_EXPORT(arducam_fifo_register);
