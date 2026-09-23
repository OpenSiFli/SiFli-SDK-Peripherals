#include <rtthread.h>

#include "app/camera_app.h"
#include "input/camera_keys.h"

#if defined(USING_BUTTON_LIB)
/* Album browsing and the next-photo key are not part of this example (yet), so
 * the two remaining key slots stay silent. */
static void camera_album_noop(void)
{
}

static void camera_next_noop(void)
{
}

/* The key library wants a void handler; the photo request returns a status. */
static void camera_shutter_handler(void)
{
    if (camera_app_take_photo() != 0)
        rt_kprintf("camera: photo ignored, preview is not running\n");
}
#endif

int main(void)
{
    rt_kprintf("take_photo_to_screen: preview to LCD via camera framework\n");
    if (camera_app_start() != 0)
        return -1;

#if defined(USING_BUTTON_LIB)
    /* KEY1 (PA34) and the first ADC key take a photo, see
     * src/input/camera_keys.c; "camera_photo" does the same from the console. */
    if (camera_keys_init(camera_shutter_handler, camera_album_noop,
                         camera_next_noop) != 0)
        rt_kprintf("camera: key init failed\n");
#endif
    return 0;
}
