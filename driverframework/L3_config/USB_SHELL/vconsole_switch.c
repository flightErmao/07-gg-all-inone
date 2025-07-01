#include <rtthread.h>
#include <vconsole.h>

int usb_change_shell(void)
{
    static rt_device_t usb_device = RT_NULL;
    usb_device = rt_device_find("vcom");
    if (usb_device == RT_NULL)
    {
        rt_kprintf("not found vcom device!\n");
        return -1;
    }
    vconsole_switch(usb_device);
    return 0;
}

INIT_FS_EXPORT(usb_change_shell);