#ifndef APPLICATIONS_USB_MODE_MANAGER_H
#define APPLICATIONS_USB_MODE_MANAGER_H

#include <rtthread.h>

typedef enum
{
    USB_APP_MODE_CDC = 0,
    USB_APP_MODE_MSC,
    USB_APP_MODE_SWITCHING,
} usb_app_mode_t;

int usb_mode_manager_start(void);
int enter_msc_mode(void);
int enter_cdc_mode(void);
usb_app_mode_t usb_mode_manager_current_mode(void);
const char *usb_mode_manager_mode_name(usb_app_mode_t mode);
rt_bool_t usb_mode_manager_is_sd_ready(void);
rt_bool_t usb_mode_manager_is_test_ready(void);

#endif
