#include "camera_keys.h"

#include <button.h>
#include <drv_gpio.h>
#include <rtthread.h>
#include <sf_type.h>

static camera_key_handler_t camera_shutter_handler;
static camera_key_handler_t camera_album_handler;
static camera_key_handler_t camera_next_handler;

/* The carrier's hardware key (KEY1, PA34 on the HDK boards) is the shutter; the
 * ADC keys below cover the keypad carriers. */
static void camera_keys_button_handler(int32_t pin, button_action_t action)
{
    (void)pin;
    if ((action == BUTTON_CLICKED) && (camera_shutter_handler != RT_NULL))
        camera_shutter_handler();
}

static void camera_keys_adc_handler(uint8_t group, int32_t key,
                                    button_action_t action)
{
    (void)group;
    if (action != BUTTON_CLICKED)
        return;
    if ((key == 0) && (camera_shutter_handler != RT_NULL))
        camera_shutter_handler();
    else if ((key == 1) && (camera_album_handler != RT_NULL))
        camera_album_handler();
    else if ((key == 2) && (camera_next_handler != RT_NULL))
        camera_next_handler();
}

int camera_keys_init(camera_key_handler_t shutter_handler,
                     camera_key_handler_t album_handler,
                     camera_key_handler_t next_handler)
{
    adc_button_handler_t handlers[ADC_BUTTON_GROUP1_MAX_NUM];
    button_cfg_t config = {0};
    int32_t button_id;
    uint8_t index;

    if ((shutter_handler == RT_NULL) || (album_handler == RT_NULL) ||
        (next_handler == RT_NULL))
        return -1;

    config.pin = GET_PIN(1, 34);
    config.active_state = BUTTON_ACTIVE_HIGH;
    config.mode = PIN_MODE_INPUT;
    config.button_handler = camera_keys_button_handler;
    config.name = "camera";
    button_id = button_init(&config);
    if (button_id < 0)
        return -1;

    for (index = 0; index < ADC_BUTTON_GROUP1_MAX_NUM; index++)
        handlers[index] = camera_keys_adc_handler;
    if (button_bind_adc_button(button_id, 0, ADC_BUTTON_GROUP1_MAX_NUM,
                               handlers) != SF_EOK)
        return -1;

    camera_shutter_handler = shutter_handler;
    camera_album_handler = album_handler;
    camera_next_handler = next_handler;
    if (button_enable(button_id) != SF_EOK)
    {
        camera_shutter_handler = RT_NULL;
        camera_album_handler = RT_NULL;
        camera_next_handler = RT_NULL;
        return -1;
    }
    return 0;
}
