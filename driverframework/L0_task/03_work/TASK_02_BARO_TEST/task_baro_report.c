#include "barometer.h"
#include "string.h"
#define DBG_TAG "task_baro"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

rt_align(RT_ALIGN_SIZE) static rt_uint8_t task_stack_baro[THREAD_STACK_SIZE];
static struct rt_thread task_tid_baro;

#ifdef BARO_REPORT_USING_SPL16_001
rt_device_t dev_sensor_baro_spl16_001 = RT_NULL;
baro_report_t baro_report_spl16_001 = {0};
#endif
#ifdef BARO_REPORT_USING_DPS368
rt_device_t dev_sensor_baro_dps368 = RT_NULL;
baro_report_t baro_report_dps368 = {0};
#endif
#ifdef BARO_REPORT_USING_SPA06_003
rt_device_t dev_sensor_baro_spa06_003 = RT_NULL;
baro_report_t baro_report_spa06_003 = {0};
#endif

// ANOTC发送功能已移至atkpsendlog.c中

static void task_dev_init(void) {
  rt_device_t dev_temp = RT_NULL;

#ifdef BARO_REPORT_USING_SPL16_001
  dev_temp = rt_device_find("spl16_01");
  if (dev_temp) {
    rt_device_open(dev_temp, RT_DEVICE_OFLAG_RDWR);
    dev_sensor_baro_spl16_001 = dev_temp;
  }
#endif

#ifdef BARO_REPORT_USING_DPS368
  dev_temp = RT_NULL;
  dev_temp = rt_device_find(SENSOR_NAME_DPS368);
  if (dev_temp) {
    rt_device_open(dev_temp, RT_DEVICE_OFLAG_RDWR);
    dev_sensor_baro_dps368 = dev_temp;
  }
#endif

#ifdef BARO_REPORT_USING_SPA06_003
  dev_temp = RT_NULL;
  dev_temp = rt_device_find(SENSOR_SPA06_003_DEVICE_NAME);
  if (dev_temp) {
    rt_device_open(dev_temp, RT_DEVICE_OFLAG_RDWR);
    dev_sensor_baro_spa06_003 = dev_temp;
  }
#endif
}

static void baro_task_read_data(void) {
#ifdef BARO_REPORT_USING_SPL16_001
  if (dev_sensor_baro_spl16_001) {
    memset(&baro_report_spl16_001, 0, sizeof(baro_report_spl16_001));
    rt_device_read(dev_sensor_baro_spl16_001, 1, (void *)&baro_report_spl16_001, 1);
  } else {
    static int spl16_001_not_found = 0;
    spl16_001_not_found++;
    if (spl16_001_not_found > 30) {
      spl16_001_not_found = 0;
      rt_kprintf("spl16_001 device not found!\n");
    }
  }
#endif

#ifdef BARO_REPORT_USING_DPS368
  if (dev_sensor_baro_dps368) {
    // memset(&baro_report_dps368, 0, sizeof(baro_report_dps368));
    rt_device_read(dev_sensor_baro_dps368, 1, (void *)&baro_report_dps368, 1);
  } else {
    static int dps368_not_found = 0;
    dps368_not_found++;
    if (dps368_not_found > 30) {
      dps368_not_found = 0;
      rt_kprintf("dps368 device not found!\n");
    }
  }
#endif

#ifdef BARO_REPORT_USING_SPA06_003
  if (dev_sensor_baro_spa06_003) {
    memset(&baro_report_spa06_003, 0, sizeof(baro_report_spa06_003));
    rt_device_read(dev_sensor_baro_spa06_003, 1, (void *)&baro_report_spa06_003, 1);
  }
#endif
}

static void baro_task_printf_data(void) {
#ifdef TASK_CONFIG_BARO_PRINTF

#ifdef BARO_REPORT_USING_SPL16_001
  if (dev_sensor_baro_spl16_001) {
    printf("SPL16_001: pressure_Pa %.2f\n", baro_report_spl16_001.pressure_Pa);
  }
#endif

#ifdef BARO_REPORT_USING_DPS368
  if (dev_sensor_baro_dps368) {
    printf("DPS368: pressure_Pa %.2f\n", baro_report_dps368.pressure_Pa);
  }
#endif

#endif
}
static void baro_thread_entry(void *parameter) {
  task_dev_init();
  while (1) {
    baro_task_read_data();
    baro_task_printf_data();
    rt_thread_mdelay(32);
  }
}

static int task_thread_init(void) {
  rt_thread_init(&task_tid_baro, "task_Baro", baro_thread_entry, RT_NULL, task_stack_baro, THREAD_STACK_SIZE,
                 THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_baro);
  return 0;
}

#ifdef WORK_TASK_BARO_REPORT_EN
INIT_APP_EXPORT(task_thread_init);
#endif
