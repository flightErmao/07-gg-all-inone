#include "mag.h"
#include "string.h"
#include "rtconfig.h"
#include "mcnMagShow.h"
#include "timestamp.h"

#define DBG_TAG "task_mag"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

/* Timer period in milliseconds */
#define MAG_TIMER_PERIOD_MS 20

/* Event flag */
#define MAG_EVENT_TIMER (1u << 0)

static rt_device_t dev_sensor_mag = RT_NULL;
static mag_report_t mag_report = {0};
static rt_timer_t mag_timer = RT_NULL;
static rt_event_t mag_event = RT_NULL;
static float mag_lsb = 0.1f; /* Default LSB value */

/* Timer callback function */
static void mag_timer_callback(void* parameter) {
  RT_UNUSED(parameter);
  if (mag_event != RT_NULL) {
    rt_event_send(mag_event, MAG_EVENT_TIMER);
  }
}

/* Initialize magnetometer device */
static void task_dev_init(void) {
  const char* device_name = TASK_MAG_DEVICE_NAME;
  rt_device_t dev_temp = rt_device_find(device_name);
  if (dev_temp) {
    rt_device_open(dev_temp, RT_DEVICE_OFLAG_RDWR);
    dev_sensor_mag = dev_temp;
    
    /* Get LSB from device configuration */
    if (dev_sensor_mag != RT_NULL) {
      mag_dev_t mag_dev = (mag_dev_t)dev_sensor_mag;
      if (mag_dev->config.lsb > 0.0f) {
        mag_lsb = mag_dev->config.lsb;
        LOG_I("Device config: range=%dG, ODR=%dHz, LSB=%.3f uT/LSB",
              mag_dev->config.range_g,
              mag_dev->config.odr_hz,
              mag_lsb);
      } else {
        LOG_W("Device LSB not set, using default: %.3f uT/LSB", mag_lsb);
      }
    }
  }
}

/* Read magnetometer data */
static void mag_task_read_data(void) {
  if (dev_sensor_mag) {
    mag_report_t raw_report = {0};
    rt_size_t size = rt_device_read(dev_sensor_mag, MAG_RD_REPORT, (void*)&raw_report, 1);
    
    if (size > 0) {
      /* Get timestamp using timestamp_micros() and convert to milliseconds */
      uint32_t timestamp_us = timestamp_micros();
      raw_report.timestamp_ms = timestamp_us / 1000;
      
      /* Publish raw data (LSB values) to mag_raw_data topic */
      mcnMagRawDataPublish(&raw_report);
      
      /* Apply LSB scaling: raw values are in LSB units, scale to uT */
      mag_report.timestamp_ms = raw_report.timestamp_ms;
      mag_report.value_x = raw_report.value_x * mag_lsb;
      mag_report.value_y = raw_report.value_y * mag_lsb;
      mag_report.value_z = raw_report.value_z * mag_lsb;
      
      mcnMagReportPublish(&mag_report);
    }
  } else {
    static int not_found_cnt = 0;
    not_found_cnt++;
    if (not_found_cnt > 30) {
      not_found_cnt = 0;
      LOG_W("mag device '%s' not found!", TASK_MAG_DEVICE_NAME);
    }
  }
}

/* Worker thread entry */
static void mag_thread_entry(void* parameter) {
  RT_UNUSED(parameter);
  rt_uint32_t received = 0;
  
  task_dev_init();
  mcnMagReportInit();
  mcnMagRawDataInit();
  
  /* Create event */
  mag_event = rt_event_create("mag_evt", RT_IPC_FLAG_FIFO);
  if (mag_event == RT_NULL) {
    LOG_E("create mag event failed");
    return;
  }
  
  /* Create timer */
  mag_timer = rt_timer_create("mag_tmr", 
                               mag_timer_callback, 
                               RT_NULL,
                               rt_tick_from_millisecond(MAG_TIMER_PERIOD_MS),
                               RT_TIMER_FLAG_PERIODIC);
  if (mag_timer == RT_NULL) {
    LOG_E("create mag timer failed");
    rt_event_delete(mag_event);
    return;
  }
  
  /* Start timer */
  if (rt_timer_start(mag_timer) != RT_EOK) {
    LOG_E("start mag timer failed");
    rt_timer_delete(mag_timer);
    rt_event_delete(mag_event);
    return;
  }
  
  LOG_I("mag task started, timer period: %d ms", MAG_TIMER_PERIOD_MS);
  
  while (1) {
    /* Wait for timer event */
    if (rt_event_recv(mag_event,
                      MAG_EVENT_TIMER,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_FOREVER,
                      &received) == RT_EOK) {
      mag_task_read_data();
    }
  }
}

/* Initialize task thread */
static int task_thread_init(void) {
  rt_align(RT_ALIGN_SIZE) static rt_uint8_t task_stack_mag[THREAD_STACK_SIZE];
  static struct rt_thread task_tid_mag;
  rt_thread_init(&task_tid_mag, "tMag", mag_thread_entry, RT_NULL, task_stack_mag, THREAD_STACK_SIZE,
                 THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_mag);
  return 0;
}

#ifdef WORK_TASK_MAG_REPORT_EN
INIT_APP_EXPORT(task_thread_init);
#endif

