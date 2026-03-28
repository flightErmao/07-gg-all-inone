#include <rtthread.h>

#include "led_status.h"

int led_status_set_mode(led_status_mode_t mode)
{
    RT_UNUSED(mode);
    return RT_EOK;
}
