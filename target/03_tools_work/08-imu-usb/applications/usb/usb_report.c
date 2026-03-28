#include <rtthread.h>

#include "usb_mode_manager.h"
#include "usb_report.h"

int usb_report_status(char *buffer, rt_size_t size)
{
    const char *sd_state;
    const char *test_state;

    if ((buffer == RT_NULL) || (size == 0))
    {
        return -RT_ERROR;
    }

    sd_state = usb_mode_manager_is_sd_ready() ? "ready" : "not_ready";
    test_state = usb_mode_manager_is_test_ready() ? "ready" : "not_ready";

    rt_snprintf(buffer,
                size,
                "STATUS mode=%s sd=%s test=%s\r\n",
                usb_mode_manager_mode_name(usb_mode_manager_current_mode()),
                sd_state,
                test_state);
    return RT_EOK;
}

int usb_report_ack_result(const char *cmd, const char *status, const char *action, char *buffer, rt_size_t size)
{
    if ((cmd == RT_NULL) || (status == RT_NULL) || (buffer == RT_NULL) || (size == 0))
    {
        return -RT_ERROR;
    }

    if (action == RT_NULL)
    {
        action = "";
    }

    rt_snprintf(buffer, size, "RESULT cmd=%s status=%s action=%s\r\n", cmd, status, action);
    return RT_EOK;
}

int usb_report_switch_ack(const char *cmd, usb_app_mode_t current_mode, usb_app_mode_t target_mode, char *buffer, rt_size_t size)
{
    if ((cmd == RT_NULL) || (buffer == RT_NULL) || (size == 0))
    {
        return -RT_ERROR;
    }

    rt_snprintf(buffer,
                size,
                "ACK cmd=%s received current=%s target=%s\r\n",
                cmd,
                usb_mode_manager_mode_name(current_mode),
                usb_mode_manager_mode_name(target_mode));
    return RT_EOK;
}
