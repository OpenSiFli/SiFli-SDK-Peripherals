/**
 * @file    main.c
 * @brief   BF30A2 Camera Example using RT-Thread Standard Device Driver
 *
 * This example demonstrates how to use the BF30A2 camera with the
 * RT-Thread standard device driver interface. Features include:
 * - Live camera preview on LVGL display
 * - Photo capture and storage to PSRAM
 * - Photo export via UART
 * - FPS display
 * - Button-controlled state machine
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2024
 */

/*============================================================================*/
/*                              INCLUDES                                      */
/*============================================================================*/

#include <stdio.h>
#include <string.h>

#include "rtthread.h"
#define DBG_TAG           "main"
#define DBG_LVL           DBG_LOG
#include <rtdbg.h>

#include "bf0_hal.h"
#include "drv_io.h"
#include "board.h"

/* LVGL headers */
#include "littlevgl2rtt.h"
#include "lvgl.h"

/* Application headers */
#include "button.h"
#include "drv_bf30a2.h"
#include "mem_section.h"

/*============================================================================*/
/*                         HARDWARE CONFIGURATION                             */
/*============================================================================*/

/* Button GPIO pins */
#define KEY1_PIN                    34
#define KEY2_PIN                    11

/*============================================================================*/
/*                         PSRAM CONFIGURATION                                */
/*============================================================================*/

#define PSRAM_HEAP_SIZE             (512 * 1024)

static uint8_t psram_heap_pool[PSRAM_HEAP_SIZE] L2_RET_BSS_SECT(psram_heap_pool);
static struct rt_memheap psram_memheap;
static uint8_t psram_heap_initialized = 0;
static uint32_t psram_heap_used       = 0;

/* Photo storage in PSRAM */
static uint8_t  *psram_photo_buffer    = NULL;
static uint32_t  psram_photo_size      = 0;
static uint32_t  psram_photo_width     = 0;
static uint32_t  psram_photo_height    = 0;
static uint32_t  psram_photo_timestamp = 0;
static uint8_t   psram_photo_valid     = 0;

/*============================================================================*/
/*                           TYPE DEFINITIONS                                 */
/*============================================================================*/

typedef enum
{
    APP_STATE_DEFAULT = 0,
    APP_STATE_CAPTURE,
    APP_STATE_PHOTO,
} app_state_t;

/*============================================================================*/
/*                          GLOBAL VARIABLES                                  */
/*============================================================================*/

/* Camera device handle */
static rt_device_t cam_device = RT_NULL;

/* Application state */
static volatile app_state_t app_state    = APP_STATE_DEFAULT;
static volatile uint8_t     key1_pressed = 0;
static volatile uint8_t     key2_pressed = 0;

/* LVGL screen objects */
static lv_obj_t *scr_default = NULL;
static lv_obj_t *scr_capture = NULL;
static lv_obj_t *scr_photo   = NULL;

/* LVGL image objects */
static lv_obj_t *cam_img   = NULL;
static lv_obj_t *photo_img = NULL;
static lv_obj_t *fps_label = NULL;

/* Image descriptors */
static lv_image_dsc_t cam_img_dsc;
static lv_image_dsc_t photo_img_dsc;

/* Camera state */
static volatile uint8_t frame_updated = 0;
static uint8_t *rgb565_ptr           = NULL;

/*============================================================================*/
/*                       PSRAM HEAP MANAGEMENT                                */
/*============================================================================*/

static int psram_heap_init(void)
{
    rt_err_t ret;

    if (psram_heap_initialized)
    {
        return 0;
    }

    ret = rt_memheap_init(&psram_memheap, "psram_heap",
                          (void *)psram_heap_pool, PSRAM_HEAP_SIZE);
    if (ret != RT_EOK)
    {
        LOG_E("PSRAM heap init failed");
        return -1;
    }

    psram_heap_initialized = 1;
    psram_heap_used = 0;
    LOG_I("PSRAM heap initialized: %d KB", PSRAM_HEAP_SIZE / 1024);

    return 0;
}

static void *psram_heap_malloc(uint32_t size)
{
    void *ptr;

    if (!psram_heap_initialized)
    {
        LOG_E("PSRAM heap not initialized");
        return NULL;
    }

    ptr = rt_memheap_alloc(&psram_memheap, size);
    if (ptr)
    {
        psram_heap_used += size;
    }

    return ptr;
}

static void psram_heap_free(void *p, uint32_t size)
{
    if (p)
    {
        rt_memheap_free(p);
        if (psram_heap_used >= size)
        {
            psram_heap_used -= size;
        }
    }
}

/*============================================================================*/
/*                         PHOTO STORAGE                                      */
/*============================================================================*/

static int psram_save_photo(const uint8_t *data, uint32_t size,
                            uint32_t width, uint32_t height)
{
    if (!data || size == 0)
    {
        LOG_E("Invalid photo data");
        return -1;
    }

    if (!psram_heap_initialized)
    {
        if (psram_heap_init() != 0)
        {
            return -1;
        }
    }

    /* Free existing photo if present */
    if (psram_photo_buffer)
    {
        psram_heap_free(psram_photo_buffer, psram_photo_size);
        psram_photo_buffer = NULL;
        psram_photo_valid = 0;
    }

    /* Allocate new buffer */
    psram_photo_buffer = (uint8_t *)psram_heap_malloc(size);
    if (!psram_photo_buffer)
    {
        LOG_E("Failed to allocate PSRAM for photo (%d bytes)", size);
        return -1;
    }

    /* Copy photo data */
    memcpy(psram_photo_buffer, data, size);

    /* Update metadata */
    psram_photo_size      = size;
    psram_photo_width     = width;
    psram_photo_height    = height;
    psram_photo_timestamp = rt_tick_get();
    psram_photo_valid     = 1;

    LOG_I("Photo saved to PSRAM: %dx%d, %d bytes, addr=0x%08X",
          width, height, size, (uint32_t)psram_photo_buffer);

    return 0;
}

static int psram_get_photo(uint8_t **data, uint32_t *size,
                           uint32_t *width, uint32_t *height)
{
    if (!psram_photo_valid || !psram_photo_buffer)
    {
        LOG_W("No valid photo in PSRAM");
        return -1;
    }

    if (data)   *data   = psram_photo_buffer;
    if (size)   *size   = psram_photo_size;
    if (width)  *width  = psram_photo_width;
    if (height) *height = psram_photo_height;

    return 0;
}

static void psram_clear_photo(void)
{
    if (psram_photo_buffer)
    {
        psram_heap_free(psram_photo_buffer, psram_photo_size);
        psram_photo_buffer = NULL;
    }

    psram_photo_size      = 0;
    psram_photo_width     = 0;
    psram_photo_height    = 0;
    psram_photo_timestamp = 0;
    psram_photo_valid     = 0;

    LOG_I("PSRAM photo cleared");
}

/*============================================================================*/
/*                       FRAME CALLBACK                                       */
/*============================================================================*/

/**
 * @brief Frame ready callback from camera device
 */
static void on_frame_ready(rt_device_t dev, rt_uint32_t frame_num,
                          rt_uint8_t *buffer, rt_uint32_t size, void *user_data)
{
    if (buffer)
    {
        rgb565_ptr = buffer;
        frame_updated = 1;
    }
}

/*============================================================================*/
/*                       CAMERA CONTROL                                       */
/*============================================================================*/

/**
 * @brief Initialize camera device
 */
static rt_err_t camera_init(void)
{
    rt_err_t ret;

    /* Find or register camera device */
    cam_device = rt_device_find(BF30A2_DEVICE_NAME);
    if (cam_device == RT_NULL)
    {
        LOG_I("Registering BF30A2 device...");
        ret = bf30a2_device_register();
        if (ret != RT_EOK)
        {
            LOG_E("Failed to register BF30A2 device");
            return ret;
        }
        cam_device = rt_device_find(BF30A2_DEVICE_NAME);
    }

    if (cam_device == RT_NULL)
    {
        LOG_E("Camera device not found");
        return -RT_ENOSYS;
    }

    /* Initialize device */
    ret = rt_device_init(cam_device);
    if (ret != RT_EOK)
    {
        LOG_E("Camera init failed");
        return ret;
    }

    /* Open device */
    ret = rt_device_open(cam_device, RT_DEVICE_FLAG_RDONLY);
    if (ret != RT_EOK)
    {
        LOG_E("Camera open failed");
        return ret;
    }

    /* Set frame callback */
    bf30a2_callback_cfg_t cb_cfg =
    {
        .callback = on_frame_ready,
        .user_data = RT_NULL
    };
    rt_device_control(cam_device, BF30A2_CMD_SET_CALLBACK, &cb_cfg);

    LOG_I("Camera initialized successfully");

    return RT_EOK;
}

/**
 * @brief Start camera capture
 */
static rt_err_t camera_start_capture(void)
{
    rt_err_t ret;
    int timeout;

    if (cam_device == RT_NULL)
    {
        return -RT_ERROR;
    }

    ret = rt_device_control(cam_device, BF30A2_CMD_START, RT_NULL);
    if (ret != RT_EOK)
    {
        LOG_E("Camera start failed");
        return ret;
    }

    /* Wait for first frame */
    timeout = 100;
    while (!rgb565_ptr && timeout-- > 0)
    {
        rt_thread_mdelay(10);
    }

    if (!rgb565_ptr)
    {
        /* Try to get buffer directly */
        bf30a2_buffer_t buf;
        rt_device_control(cam_device, BF30A2_CMD_GET_BUFFER, &buf);
        rgb565_ptr = buf.data;
    }

    if (!rgb565_ptr)
    {
        LOG_E("Failed to get RGB565 buffer");
        return -RT_ERROR;
    }

    LOG_I("Camera capture started, buffer=0x%08X", (uint32_t)rgb565_ptr);

    return RT_EOK;
}

/**
 * @brief Stop camera capture
 */
static void camera_stop_capture(void)
{
    if (cam_device != RT_NULL)
    {
        rt_device_control(cam_device, BF30A2_CMD_STOP, RT_NULL);
        LOG_I("Camera capture stopped");
    }
}

/**
 * @brief Get current FPS
 */
static float camera_get_fps(void)
{
    float fps = 0.0f;
    if (cam_device != RT_NULL)
    {
        rt_device_control(cam_device, BF30A2_CMD_GET_FPS, &fps);
    }
    return fps;
}

/*============================================================================*/
/*                         PHOTO OPERATIONS                                   */
/*============================================================================*/

/**
 * @brief Export photo via UART from PSRAM
 */
static void export_photo_via_uart(void)
{
    uint8_t  *data;
    uint32_t  size;
    uint32_t  width;
    uint32_t  height;
    uint32_t  i;

    if (psram_get_photo(&data, &size, &width, &height) != 0)
    {
        LOG_E("No photo in PSRAM to export");
        return;
    }

    LOG_I("========================================");
    LOG_I("Exporting photo from PSRAM via UART...");
    LOG_I("Format: RGB565, Size: %dx%d", width, height);
    LOG_I("Total bytes: %d", size);
    LOG_I("PSRAM addr: 0x%08X", (uint32_t)data);
    LOG_I("========================================");

    rt_kprintf("\n===PHOTO_START===\n");
    rt_kprintf("WIDTH:%d\n", width);
    rt_kprintf("HEIGHT:%d\n", height);
    rt_kprintf("FORMAT:RGB565\n");
    rt_kprintf("SIZE:%d\n", size);
    rt_kprintf("SOURCE:PSRAM\n");
    rt_kprintf("===DATA_BEGIN===\n");

    for (i = 0; i < size; i++)
    {
        rt_kprintf("%02X", data[i]);
        if ((i + 1) % 32 == 0)
        {
            rt_kprintf("\n");
            if ((i + 1) % 1024 == 0)
            {
                rt_thread_mdelay(5);
            }
        }
    }

    rt_kprintf("\n===DATA_END===\n");
    rt_kprintf("===PHOTO_END===\n\n");

    LOG_I("Photo export completed!");
}

/**
 * @brief Capture current frame as photo
 */
static void take_photo(void)
{
    if (!rgb565_ptr)
    {
        LOG_E("Cannot take photo: buffer not ready");
        return;
    }

    camera_stop_capture();

    LOG_I("Photo taken! Saving to PSRAM...");

    if (psram_save_photo(rgb565_ptr, BF30A2_DEFAULT_FRAME_SIZE,
                         BF30A2_DEFAULT_WIDTH, BF30A2_DEFAULT_HEIGHT) != 0)
    {
        LOG_E("Failed to save photo to PSRAM");
        return;
    }

    LOG_I("Photo saved to PSRAM successfully!");
}

/*============================================================================*/
/*                         BUTTON HANDLING                                    */
/*============================================================================*/

static void button_event_handler(int32_t pin, button_action_t action)
{
    if (action != BUTTON_CLICKED)
    {
        return;
    }

    LOG_I("Button pressed: pin=%d", pin);

    if (pin == KEY1_PIN)
    {
        key1_pressed = 1;
    }
    else if (pin == KEY2_PIN)
    {
        key2_pressed = 1;
    }
}

static rt_err_t buttons_init(void)
{
    button_cfg_t cfg;
    int32_t id;

    HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA11, GPIO_A11, PIN_PULLDOWN, 1);

    /* Initialize KEY1 */
    cfg.pin            = KEY1_PIN;
    cfg.active_state   = BUTTON_ACTIVE_HIGH;
    cfg.mode           = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;

    id = button_init(&cfg);
    if (id < 0)
    {
        LOG_E("KEY1 init failed");
        return -RT_ERROR;
    }
    if (button_enable(id) != SF_EOK)
    {
        LOG_E("KEY1 enable failed");
        return -RT_ERROR;
    }

    /* Initialize KEY2 */
    cfg.pin            = KEY2_PIN;
    cfg.active_state   = BUTTON_ACTIVE_HIGH;
    cfg.mode           = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;

    id = button_init(&cfg);
    if (id < 0)
    {
        LOG_E("KEY2 init failed");
        return -RT_ERROR;
    }
    if (button_enable(id) != SF_EOK)
    {
        LOG_E("KEY2 enable failed");
        return -RT_ERROR;
    }

    LOG_I("Buttons initialized: KEY1=PA%d, KEY2=PA%d", KEY1_PIN, KEY2_PIN);

    return RT_EOK;
}

/*============================================================================*/
/*                         LVGL UI SCREENS                                    */
/*============================================================================*/

static void create_default_screen(void)
{
    lv_obj_t *title;
    lv_obj_t *subtitle;
    lv_obj_t *hint;

    scr_default = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_default, lv_color_hex(0x000000), 0);

    /* Title label */
    title = lv_label_create(scr_default);
    lv_label_set_text(title, "SiFli Camera Example");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -30);

    /* Subtitle label */
    subtitle = lv_label_create(scr_default);
    lv_label_set_text(subtitle, "BF30A2 (RT-Thread Driver)");
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0x00FF00), 0);
    lv_obj_align(subtitle, LV_ALIGN_CENTER, 0, 10);

    /* Hint label */
    hint = lv_label_create(scr_default);
    lv_label_set_text(hint, "Press KEY1 to start");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x888888), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -40);
}

static void create_capture_screen(void)
{
    lv_obj_t *hint;

    scr_capture = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_capture, lv_color_hex(0x000000), 0);

    /* Camera image */
    cam_img = lv_image_create(scr_capture);

    memset(&cam_img_dsc, 0, sizeof(cam_img_dsc));
    cam_img_dsc.header.magic  = LV_IMAGE_HEADER_MAGIC;
    cam_img_dsc.header.cf     = LV_COLOR_FORMAT_RGB565;
    cam_img_dsc.header.w      = BF30A2_DEFAULT_WIDTH;
    cam_img_dsc.header.h      = BF30A2_DEFAULT_HEIGHT;
    cam_img_dsc.header.stride = BF30A2_DEFAULT_WIDTH * 2;
    cam_img_dsc.data_size     = BF30A2_DEFAULT_FRAME_SIZE;
    cam_img_dsc.data          = NULL;

    lv_image_set_src(cam_img, &cam_img_dsc);
    lv_image_set_scale(cam_img, 308);
    lv_obj_align(cam_img, LV_ALIGN_TOP_MID, 0, 50);

    /* FPS label */
    fps_label = lv_label_create(scr_capture);
    lv_label_set_text(fps_label, "FPS: --");
    lv_obj_set_style_text_font(fps_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(fps_label, lv_color_hex(0x00FF00), 0);
    lv_obj_align(fps_label, LV_ALIGN_BOTTOM_MID, 0, -20);

    /* Hint label */
    hint = lv_label_create(scr_capture);
    lv_label_set_text(hint, "K1:Photo K2:Back");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0xFFFF00), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -5);
}

static void create_photo_screen(void)
{
    lv_obj_t *title;
    lv_obj_t *hint;

    scr_photo = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_photo, lv_color_hex(0x000000), 0);

    /* Photo image */
    photo_img = lv_image_create(scr_photo);

    memset(&photo_img_dsc, 0, sizeof(photo_img_dsc));
    photo_img_dsc.header.magic  = LV_IMAGE_HEADER_MAGIC;
    photo_img_dsc.header.cf     = LV_COLOR_FORMAT_RGB565;
    photo_img_dsc.header.w      = BF30A2_DEFAULT_WIDTH;
    photo_img_dsc.header.h      = BF30A2_DEFAULT_HEIGHT;
    photo_img_dsc.header.stride = BF30A2_DEFAULT_WIDTH * 2;
    photo_img_dsc.data_size     = BF30A2_DEFAULT_FRAME_SIZE;
    photo_img_dsc.data          = NULL;

    lv_image_set_src(photo_img, &photo_img_dsc);
    lv_image_set_scale(photo_img, 308);
    lv_obj_align(photo_img, LV_ALIGN_TOP_MID, 0, 50);

    /* Title label */
    title = lv_label_create(scr_photo);
    lv_label_set_text(title, "Recorded");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF0000), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

    /* Hint label */
    hint = lv_label_create(scr_photo);
    lv_label_set_text(hint, "K2:Back to viewfinder");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0xFFFF00), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -5);
}

/*============================================================================*/
/*                       STATE MACHINE                                        */
/*============================================================================*/

static void switch_to_default_state(void)
{
    LOG_I("Switching to DEFAULT state");

    camera_stop_capture();

    app_state = APP_STATE_DEFAULT;
    lv_screen_load(scr_default);
}

static void switch_to_capture_state(void)
{
    LOG_I("Switching to CAPTURE state");

    if (cam_device == RT_NULL)
    {
        if (camera_init() != RT_EOK)
        {
            LOG_E("Camera init failed");
            return;
        }
    }

    if (camera_start_capture() != RT_EOK)
    {
        LOG_E("Camera start failed");
        return;
    }

    if (rgb565_ptr)
    {
        cam_img_dsc.data = rgb565_ptr;
        lv_image_set_src(cam_img, &cam_img_dsc);
    }

    app_state = APP_STATE_CAPTURE;
    lv_screen_load(scr_capture);
}

static void switch_to_photo_state(void)
{
    uint8_t  *photo_data;
    uint32_t  photo_size;
    uint32_t  photo_width;
    uint32_t  photo_height;

    LOG_I("Switching to PHOTO state");

    if (psram_get_photo(&photo_data, &photo_size,
                        &photo_width, &photo_height) == 0)
    {
        photo_img_dsc.data = photo_data;
        lv_image_set_src(photo_img, &photo_img_dsc);
        lv_obj_invalidate(photo_img);
        LOG_I("Displaying photo from PSRAM");
    }
    else
    {
        LOG_W("No photo in PSRAM, using camera buffer");
        if (rgb565_ptr)
        {
            photo_img_dsc.data = rgb565_ptr;
            lv_image_set_src(photo_img, &photo_img_dsc);
            lv_obj_invalidate(photo_img);
        }
    }

    app_state = APP_STATE_PHOTO;
    lv_screen_load(scr_photo);
}

static void process_key_events(void)
{
    /* Process KEY1 */
    if (key1_pressed)
    {
        key1_pressed = 0;

        switch (app_state)
        {
        case APP_STATE_DEFAULT:
            switch_to_capture_state();
            break;

        case APP_STATE_CAPTURE:
            take_photo();
            switch_to_photo_state();
            break;

        case APP_STATE_PHOTO:
            /* No action */
            break;
        }
    }

    /* Process KEY2 */
    if (key2_pressed)
    {
        key2_pressed = 0;

        switch (app_state)
        {
        case APP_STATE_DEFAULT:
            /* No action */
            break;

        case APP_STATE_CAPTURE:
            switch_to_default_state();
            break;

        case APP_STATE_PHOTO:
            switch_to_capture_state();
            break;
        }
    }
}

/*============================================================================*/
/*                         FPS DISPLAY                                        */
/*============================================================================*/

static void update_fps_display(void)
{
    static uint32_t last_update = 0;
    uint32_t now;
    float fps;
    char buf[32];

    now = rt_tick_get_millisecond();

    if (now - last_update >= 500)
    {
        last_update = now;

        if (fps_label && app_state == APP_STATE_CAPTURE)
        {
            fps = camera_get_fps();
            snprintf(buf, sizeof(buf), "FPS: %.1f", fps);
            lv_label_set_text(fps_label, buf);
        }
    }
}

/*============================================================================*/
/*                           MAIN FUNCTION                                    */
/*============================================================================*/

int main(void)
{
    rt_err_t ret;
    rt_uint32_t ms;

    LOG_I("========================================");
    LOG_I("   BF30A2 Camera Photo Example");
    LOG_I("   Using RT-Thread Device Driver");
    LOG_I("   Platform: SF32LB52");
    LOG_I("========================================");

    /* Initialize PSRAM heap */
    ret = psram_heap_init();
    if (ret != 0)
    {
        LOG_W("PSRAM heap init failed, photo storage disabled");
    }

    /* Initialize buttons */
    ret = buttons_init();
    if (ret != RT_EOK)
    {
        LOG_E("Buttons init failed");
        return ret;
    }

    /* Initialize LVGL */
    ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        LOG_E("LVGL init failed");
        return ret;
    }
    LOG_I("LVGL initialized");

    /* Create UI screens */
    create_default_screen();
    create_capture_screen();
    create_photo_screen();
    LOG_I("UI screens created");

    /* Set initial state */
    lv_screen_load(scr_default);
    app_state = APP_STATE_DEFAULT;

    LOG_I("========================================");
    LOG_I("   System Ready");
    LOG_I("   KEY1: Start/Photo");
    LOG_I("   KEY2: Back");
    LOG_I("========================================");

    /* Main loop */
    while (1)
    {
        process_key_events();

        if (app_state == APP_STATE_CAPTURE)
        {
            if (frame_updated && cam_img && rgb565_ptr)
            {
                frame_updated = 0;

                if (cam_img_dsc.data != rgb565_ptr)
                {
                    cam_img_dsc.data = rgb565_ptr;
                    lv_image_set_src(cam_img, &cam_img_dsc);
                }

                lv_obj_invalidate(cam_img);
            }

            update_fps_display();
        }

        ms = lv_task_handler();
        rt_thread_mdelay(ms > 0 ? ms : 5);
    }

    return RT_EOK;
}

/*============================================================================*/
/*                        SHELL COMMANDS                                      */
/*============================================================================*/

/**
 * @brief Export photo via UART (shell command)
 */
static void psram_export(int argc, char **argv)
{
    export_photo_via_uart();
}
MSH_CMD_EXPORT(psram_export, Export photo from PSRAM via UART);

/**
 * @brief Show PSRAM photo information (shell command)
 */
static void psram_photo_info(int argc, char **argv)
{
    if (!psram_photo_valid)
    {
        LOG_I("No photo in PSRAM");
        return;
    }

    LOG_I("=== PSRAM Photo Info ===");
    LOG_I("  Valid: Yes");
    LOG_I("  Size: %dx%d", psram_photo_width, psram_photo_height);
    LOG_I("  Data size: %d bytes", psram_photo_size);
    LOG_I("  Address: 0x%08X", (uint32_t)psram_photo_buffer);
    LOG_I("  Timestamp: %d ticks", psram_photo_timestamp);
    LOG_I("========================");
}
MSH_CMD_EXPORT(psram_photo_info, Show PSRAM photo information);

/**
 * @brief Clear photo from PSRAM (shell command)
 */
static void psram_photo_clear(int argc, char **argv)
{
    psram_clear_photo();
}
MSH_CMD_EXPORT(psram_photo_clear, Clear photo from PSRAM);

/**
 * @brief Show PSRAM heap status (shell command)
 */
static void psram_heap_status(int argc, char **argv)
{
    LOG_I("=== PSRAM Heap Status ===");
    LOG_I("  Initialized: %s", psram_heap_initialized ? "Yes" : "No");
    LOG_I("  Total: %d bytes (%d KB)", PSRAM_HEAP_SIZE, PSRAM_HEAP_SIZE / 1024);
    LOG_I("  Used: %d bytes (%d KB)", psram_heap_used, psram_heap_used / 1024);
    LOG_I("  Free: %d bytes (%d KB)",
          PSRAM_HEAP_SIZE - psram_heap_used,
          (PSRAM_HEAP_SIZE - psram_heap_used) / 1024);
    LOG_I("  Photo valid: %s", psram_photo_valid ? "Yes" : "No");
    LOG_I("=========================");
}
MSH_CMD_EXPORT(psram_heap_status, Show PSRAM heap status);

/**
 * @brief Show application status (shell command)
 */
static void app_status(int argc, char **argv)
{
    const char *state_str[] = {"DEFAULT", "CAPTURE", "PHOTO"};
    bf30a2_status_info_t cam_status;

    LOG_I("=== Application Status ===");
    LOG_I("App state: %s", state_str[app_state]);
    LOG_I("Camera device: %s", cam_device ? "Opened" : "Not opened");
    LOG_I("RGB565 buffer: 0x%08X", (uint32_t)rgb565_ptr);
    LOG_I("PSRAM photo valid: %d", psram_photo_valid);

    if (cam_device)
    {
        rt_device_control(cam_device, BF30A2_CMD_GET_STATUS, &cam_status);
        LOG_I("Camera state: %s",
              cam_status.state == BF30A2_STATUS_RUNNING ? "Running" : "Idle");
        LOG_I("FPS: %.1f", cam_status.fps);
        LOG_I("Frames: %d", cam_status.complete_frames);
        LOG_I("Errors: %d", cam_status.error_count);
    }

    LOG_I("==========================");
}
MSH_CMD_EXPORT(app_status, Show application status);

/**
 * @brief Camera device test commands
 */
static void cam_test(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: cam_test <cmd>\n");
        rt_kprintf("  init   - Initialize camera\n");
        rt_kprintf("  start  - Start capture\n");
        rt_kprintf("  stop   - Stop capture\n");
        rt_kprintf("  status - Show status\n");
        rt_kprintf("  export - Export via UART\n");
        rt_kprintf("  read   - Read frame to buffer\n");
        return;
    }

    if (strcmp(argv[1], "init") == 0)
    {
        camera_init();
    }
    else if (strcmp(argv[1], "start") == 0)
    {
        if (cam_device)
        {
            rt_device_control(cam_device, BF30A2_CMD_START, RT_NULL);
        }
        else
        {
            rt_kprintf("Camera not initialized\n");
        }
    }
    else if (strcmp(argv[1], "stop") == 0)
    {
        if (cam_device)
        {
            rt_device_control(cam_device, BF30A2_CMD_STOP, RT_NULL);
        }
    }
    else if (strcmp(argv[1], "status") == 0)
    {
        if (cam_device)
        {
            bf30a2_status_info_t status;
            bf30a2_info_t info;

            rt_device_control(cam_device, BF30A2_CMD_GET_INFO, &info);
            rt_device_control(cam_device, BF30A2_CMD_GET_STATUS, &status);

            rt_kprintf("=== Camera Info ===\n");
            rt_kprintf("Chip ID: 0x%04X\n", info.chip_id);
            rt_kprintf("Resolution: %dx%d\n", info.width, info.height);
            rt_kprintf("Frame size: %d bytes\n", info.frame_size);
            rt_kprintf("State: %s\n",
                      status.state == BF30A2_STATUS_RUNNING ? "Running" : "Idle");
            rt_kprintf("FPS: %.1f\n", status.fps);
            rt_kprintf("Frames: %d\n", status.complete_frames);
            rt_kprintf("Errors: %d\n", status.error_count);
            rt_kprintf("===================\n");
        }
        else
        {
            rt_kprintf("Camera not initialized\n");
        }
    }
    else if (strcmp(argv[1], "export") == 0)
    {
        if (cam_device)
        {
            rt_device_control(cam_device, BF30A2_CMD_EXPORT_UART, RT_NULL);
        }
    }
    else if (strcmp(argv[1], "read") == 0)
    {
        if (cam_device)
        {
            static uint8_t test_buf[100];
            rt_size_t len = rt_device_read(cam_device, 0, test_buf, sizeof(test_buf));
            rt_kprintf("Read %d bytes from camera\n", len);
            if (len > 0)
            {
                rt_kprintf("First 16 bytes: ");
                for (int i = 0; i < 16 && i < len; i++)
                {
                    rt_kprintf("%02X ", test_buf[i]);
                }
                rt_kprintf("\n");
            }
        }
    }
    else
    {
        rt_kprintf("Unknown command: %s\n", argv[1]);
    }
}
MSH_CMD_EXPORT(cam_test, Camera device test commands);

/*============================================================================*/
/*                     END OF FILE                                            */
/*============================================================================*/
