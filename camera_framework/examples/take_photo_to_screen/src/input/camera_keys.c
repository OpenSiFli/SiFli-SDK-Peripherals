#include "camera_keys.h"

#include <button.h>
#include <drv_gpio.h>
#include <rtthread.h>
#include <sf_type.h>

/*
 * KEY1 (PA34 on the HDK carriers) is a plain GPIO key: the button library
 * samples it, sees a BUTTON_CLICKED and calls the handler registered here.
 *
 * The ADC key path is only usable on a carrier whose ADC channel is wired to a
 * resistor ladder. The HDK boards are not: their configured ADC button group
 * reads the battery monitor (ADC device "bat1", channel 6), so the voltage is
 * the pack voltage instead of a key divider. Worse, button_bind_adc_button()
 * replaces the GPIO button's own handler with the ADC group handler, which is
 * why pressing KEY1 used to print "Unknown pin:0,34,<action>" and take no
 * photo - the GPIO click was resolved as an ADC key whose voltage matched no
 * threshold. Keep the ADC path available (set CAMERA_KEYS_USE_ADC_KEYS to 1
 * for a keypad carrier) but bind it only when the hardware really has it.
 */
#ifndef CAMERA_KEYS_USE_ADC_KEYS
#define CAMERA_KEYS_USE_ADC_KEYS 0
#endif

static camera_key_handler_t camera_shutter_handler;
static camera_key_handler_t camera_album_handler;
static camera_key_handler_t camera_next_handler;

/* The carrier's hardware key (KEY1, PA34 on the HDK boards) is the shutter. */
static void camera_keys_button_handler(int32_t pin, button_action_t action)
{
    (void)pin;
    if ((action == BUTTON_CLICKED) && (camera_shutter_handler != RT_NULL))
        camera_shutter_handler();
}

#if CAMERA_KEYS_USE_ADC_KEYS
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
#endif

int camera_keys_init(camera_key_handler_t shutter_handler,
                     camera_key_handler_t album_handler,
                     camera_key_handler_t next_handler)
{
#if CAMERA_KEYS_USE_ADC_KEYS
    adc_button_handler_t handlers[ADC_BUTTON_GROUP1_MAX_NUM];
    uint8_t index;
#endif
    button_cfg_t config = {0};
    int32_t button_id;

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

#if CAMERA_KEYS_USE_ADC_KEYS
    for (index = 0; index < ADC_BUTTON_GROUP1_MAX_NUM; index++)
        handlers[index] = camera_keys_adc_handler;
    /* Note: this replaces the GPIO handler registered above, so it is only
     * correct on a carrier whose ADC channel belongs to the key ladder. */
    if (button_bind_adc_button(button_id, 0, ADC_BUTTON_GROUP1_MAX_NUM,
                               handlers) != SF_EOK)
        return -1;
#endif

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
