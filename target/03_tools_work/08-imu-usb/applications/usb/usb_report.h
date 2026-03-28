#ifndef APPLICATIONS_USB_REPORT_H
#define APPLICATIONS_USB_REPORT_H

#include <rtthread.h>

int usb_report_status(char *buffer, rt_size_t size);
int usb_report_ack_result(const char *cmd, const char *status, const char *action, char *buffer, rt_size_t size);
int usb_report_switch_ack(const char *cmd, usb_app_mode_t current_mode, usb_app_mode_t target_mode, char *buffer, rt_size_t size);

#endif
