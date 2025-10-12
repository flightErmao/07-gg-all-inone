#include "barometer.h"
#include "string.h"
#include "rtconfig.h"
#include "mcnBaroShow.h"

#define DBG_TAG "task_baro"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

rt_device_t dev_sensor_baro = RT_NULL;
baro_report_t baro_report = {0};

static void task_dev_init(void) {
  const char* device_name = TASK_BARO_DEVICE_NAME;
  rt_device_t dev_temp = rt_device_find(device_name);
  if (dev_temp) {
    rt_device_open(dev_temp, RT_DEVICE_OFLAG_RDWR);
    dev_sensor_baro = dev_temp;
  }
}

static void baro_task_read_data(void) {
  if (dev_sensor_baro) {
    // memset(&baro_report, 0, sizeof(baro_report));
    rt_device_read(dev_sensor_baro, BARO_RD_REPORT, (void*)&baro_report, 1);
    mcnBaroReportPublish(&baro_report);
  } else {
    static int not_found_cnt = 0;
    not_found_cnt++;
    if (not_found_cnt > 30) {
      not_found_cnt = 0;
      rt_kprintf("baro device '%s' not found!\n", TASK_BARO_DEVICE_NAME);
    }
  }
}

static void baro_thread_entry(void* parameter) {
  task_dev_init();
  mcnBaroReportInit();
  while (1) {
    baro_task_read_data();
    rt_thread_mdelay(32);
  }
}

static int task_thread_init(void) {
  rt_align(RT_ALIGN_SIZE) static rt_uint8_t task_stack_baro[THREAD_STACK_SIZE];
  static struct rt_thread task_tid_baro;
  rt_thread_init(&task_tid_baro, "tBaro", baro_thread_entry, RT_NULL, task_stack_baro, THREAD_STACK_SIZE,
                 THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_baro);
  return 0;
}

#ifdef WORK_TASK_BARO_REPORT_EN
INIT_APP_EXPORT(task_thread_init);
#endif
