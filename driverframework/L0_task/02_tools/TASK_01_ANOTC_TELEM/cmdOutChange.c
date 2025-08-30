#include <rtthread.h>
#include <string.h>
#include "deviceManager.h"

/* Switch telemetry output device: target in {"uart1", "uart2", "usb"(alias "vcom")} */
static int anotc_telem_switch_output(const char *target) {
  if (target == RT_NULL) {
    return -RT_EINVAL;
  }

  char next_name[8] = {0};
  if (!strcmp(target, "usb")) {
    strncpy(next_name, "vcom", sizeof(next_name) - 1);
  } else if (!strcmp(target, "uart1")) {
    strncpy(next_name, "uart1", sizeof(next_name) - 1);
  } else if (!strcmp(target, "uart2")) {
    strncpy(next_name, "uart2", sizeof(next_name) - 1);
  } else {
    rt_kprintf("未知设备: %s\n", target);
    return -RT_ERROR;
  }

  if (dev_anotc_telem_) {
    rt_device_close(dev_anotc_telem_);
    dev_anotc_telem_ = RT_NULL;
  }

  /* Re-init device with new name */
  if (task_dev_init(next_name) != RT_EOK) {
    rt_kprintf("切换到 %s 失败\n", next_name);
    return -RT_ERROR;
  }

  rt_kprintf("anotc 输出已切换到: %s\n", next_name);

  return RT_EOK;
}

/* Shell command: anotc out <uart1|uart2|usb> */
static int cmdAtkpChange(int argc, char **argv) {
  if (argc < 2) {
    rt_kprintf("anotc 命令用法:\n");
    rt_kprintf("  anotc out <uart1|uart2|usb>  - 切换输出设备 (默认uart1)\n");
    return 0;
  }

  if (!rt_strcmp(argv[1], "out")) {
    if (argc < 3) {
      rt_kprintf("缺少设备参数，支持: uart1|uart2|usb\n");
      return -1;
    }
    return anotc_telem_switch_output(argv[2]);
  }

  rt_kprintf("未知子命令: %s\n", argv[1]);
  rt_kprintf("使用 'anotc' 查看帮助\n");
  return -1;
}

MSH_CMD_EXPORT_ALIAS(cmdAtkpChange, cmdAtkpChange, anotc telem output switch command);