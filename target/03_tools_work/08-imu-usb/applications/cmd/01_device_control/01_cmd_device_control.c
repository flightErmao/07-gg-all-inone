#include <rtthread.h>
#include <finsh.h>

#include "01_cmd_device_control.h"
#include "usb_mode_manager.h"
#include "usb_report.h"

static void cmd_device_control_print_line(const char *line)
{
    if (line != RT_NULL)
    {
        rt_kprintf("%s", line);
    }
}

int cmd_device_control_report_status(void)
{
    char line[96];

    rt_kprintf("ACK cmd=status received\r\n");
    if (usb_report_status(line, sizeof(line)) == RT_EOK)
    {
        cmd_device_control_print_line(line);
    }
    if (usb_report_ack_result("status", "ok", RT_NULL, line, sizeof(line)) == RT_EOK)
    {
        cmd_device_control_print_line(line);
    }
    return RT_EOK;
}

int cmd_device_control_switch_usb(cmd_device_usb_mode_t usb_mode)
{
    char line[128];
    const char *cmd_name;
    const char *action_name;
    usb_app_mode_t target_mode;

    if (usb_mode == CMD_DEVICE_USB_MODE_MSC)
    {
        cmd_name = "enter_msc";
        action_name = "reboot_to_cdc_msc";
        target_mode = USB_APP_MODE_MSC;
    }
    else if (usb_mode == CMD_DEVICE_USB_MODE_CDC)
    {
        cmd_name = "enter_cdc";
        action_name = "reboot_to_cdc";
        target_mode = USB_APP_MODE_CDC;
    }
    else
    {
        return -RT_ERROR;
    }

    if (usb_report_switch_ack(cmd_name,
                              usb_mode_manager_current_mode(),
                              target_mode,
                              line,
                              sizeof(line)) == RT_EOK)
    {
        cmd_device_control_print_line(line);
    }

    if (target_mode == USB_APP_MODE_MSC)
    {
        enter_msc_mode();
    }
    else
    {
        enter_cdc_mode();
    }

    if (usb_report_ack_result(cmd_name,
                              "accepted",
                              action_name,
                              line,
                              sizeof(line)) == RT_EOK)
    {
        cmd_device_control_print_line(line);
    }

    return RT_EOK;
}

static int status(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return cmd_device_control_report_status();
}
MSH_CMD_EXPORT(status, show compact status for host tool);

static int enter_msc(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return cmd_device_control_switch_usb(CMD_DEVICE_USB_MODE_MSC);
}
MSH_CMD_EXPORT(enter_msc, protocol-friendly alias for CDC+MSC switching);

static int enter_cdc(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return cmd_device_control_switch_usb(CMD_DEVICE_USB_MODE_CDC);
}
MSH_CMD_EXPORT(enter_cdc, protocol-friendly alias for CDC-only switching);
