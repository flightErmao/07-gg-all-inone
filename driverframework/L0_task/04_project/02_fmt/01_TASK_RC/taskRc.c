#include <rtdevice.h>
#include <rtthread.h>
#include "taskRc.h"
#include "rtconfig.h"
#include "rc.h"

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

static rt_device_t rc_test_dev = RT_NULL;
static rt_uint16_t rc_channels[MAX_RC_CHANNEL_NUM] = {0};
static rt_uint32_t rc_timestamp = 0;

void rcTestTask(void* param) {
  rt_uint16_t channel_mask = 0xFF;

  while (1) {
    if (rc_test_dev != RT_NULL) {
      rt_size_t size = rt_device_read(rc_test_dev, channel_mask, rc_channels, 16);
      if (size > 0) {
        rc_timestamp = rt_tick_get();
      } else {
        rt_kprintf("Read RC data failed!\n");
        rt_thread_mdelay(1000);
      }
    } else {
      rt_kprintf("RC device not found!\n");
      rt_thread_mdelay(1000);
    }
  }
}

static int taskRcTestInit(void) {
  rt_err_t ret = RT_EOK;
  char rc_name[RT_NAME_MAX];

  rt_strncpy(rc_name, PROJECT_FMT_TASK01_RC_DEVICE_DEFAULT, RT_NAME_MAX);

  rc_test_dev = rt_device_find(rc_name);
  if (!rc_test_dev) {
    rt_kprintf("Find RC device %s failed!\n", rc_name);
    return -RT_ERROR;
  }

  ret = rt_device_open(rc_test_dev, RT_DEVICE_FLAG_RDONLY);
  if (ret != RT_EOK) {
    rt_kprintf("Open RC device %s failed!\n", rc_name);
    return ret;
  }

  rt_thread_t thread =
      rt_thread_create("L0_fmt_rc", rcTestTask, RT_NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  if (thread != RT_NULL) {
    rt_thread_startup(thread);
    rt_kprintf("RC test task started on %s\n", rc_name);
  } else {
    rt_kprintf("Create RC test thread failed!\n");
    ret = -RT_ERROR;
  }

  return ret;
}

rt_uint16_t* getRcChannels(void) { return rc_channels; }

#ifdef PROJECT_FMT_TASK01_RC_EN
INIT_APP_EXPORT(taskRcTestInit);
#endif