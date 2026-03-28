#ifndef APPLICATIONS_01_CMD_DEVICE_CONTROL_H
#define APPLICATIONS_01_CMD_DEVICE_CONTROL_H

#include <rtthread.h>

typedef enum
{
    CMD_DEVICE_USB_MODE_NONE = 0,
    CMD_DEVICE_USB_MODE_CDC,
    CMD_DEVICE_USB_MODE_MSC,
} cmd_device_usb_mode_t;

int cmd_device_control_report_status(void);
int cmd_device_control_switch_usb(cmd_device_usb_mode_t usb_mode);

#endif
