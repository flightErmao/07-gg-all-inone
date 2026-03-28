#include <rtthread.h>

#include "app_init.h"
#include "usb_mode_manager.h"

int app_init_run(void)
{
    return usb_mode_manager_start();
}
