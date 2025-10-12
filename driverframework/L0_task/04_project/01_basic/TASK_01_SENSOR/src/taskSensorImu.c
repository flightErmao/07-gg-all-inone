#include "rtthread.h"
#include <rtdevice.h>
#include "imu.h"
#include "biasGyro.h"
#include "filterLpf2p.h"
#include "imuProcess.h"
#include "rtconfig.h"
#include "aMlogSensorImu.h"
#include "aMcnSensorImu.h"

#define SENSORS_MPU6500_BUFF_LEN 14
static rt_device_t dev_sensor_imu = RT_NULL;

#ifdef PROJECT_MINIFLY_TASK_SENSOR_TIMER_TRIGGER_EN
#define SENSOR_EVENT_FLAG_TRIGGER (1u << 0)
static struct rt_event sensor_event;
static rt_timer_t sensor_timer = RT_NULL;
static void sensor_timer_cb(void *parameter) {
  RT_UNUSED(parameter);
  rt_event_send(&sensor_event, SENSOR_EVENT_FLAG_TRIGGER);
}
#endif

static void deviceInit(void) {
  rt_device_t dev_temp = RT_NULL;

#ifdef PROJECT_MINIFLY_TASK_SENSOR_IMU_NAME
  dev_temp = rt_device_find(PROJECT_MINIFLY_TASK_SENSOR_IMU_NAME);
#else
  dev_temp = rt_device_find(SENSOR_NAME_MPU6500_MINIFLY);
#endif
  if (dev_temp) {
    rt_device_open(dev_temp, RT_DEVICE_OFLAG_RDWR);
    dev_sensor_imu = dev_temp;
    static imu_dev_t imu_ptr = RT_NULL;
    imu_ptr = (imu_dev_t)dev_temp;
    if (imu_ptr && imu_ptr->ops) {
      float acc_g_per_lsb = imu_ptr->config.acc_scale_factor;      // g/lsb
      float gyro_deg_per_lsb = imu_ptr->config.gyro_scale_factor;  // dps/lsb
      sensorsProcess_set_lsb(acc_g_per_lsb, gyro_deg_per_lsb);
    }
  }
}

static void rtosToolsInit(void) {

#ifdef PROJECT_MINIFLY_TASK_SENSOR_TIMER_TRIGGER_EN
  rt_event_init(&sensor_event, "sns_evt", RT_IPC_FLAG_PRIO);
  if (sensor_timer == RT_NULL) {
    /* Default 10ms period; adjust if needed */
    rt_tick_t period_ticks = rt_tick_from_millisecond(1);
    sensor_timer = rt_timer_create("sns_tmr", sensor_timer_cb, RT_NULL, period_ticks,
                                   RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (sensor_timer) {
      rt_timer_start(sensor_timer);
    } else {
      rt_kprintf("Failed to create sensor timer\n");
    }
  }
#endif
}

static void sensor_minifly_thread_entry(void *parameter) {
  deviceInit();
  rtosToolsInit();
  filterInitLpf2AccGyro();
  sensorsBiasObjInit();
  initImuRotationDir();
  mlogImuInit();

  uint8_t sensor_buffer[SENSORS_MPU6500_BUFF_LEN] = {0};
  sensorData_t sensors_data = {0};

  while (1) {
#ifdef PROJECT_MINIFLY_TASK_SENSOR_TIMER_TRIGGER_EN
    {
      rt_uint32_t received = 0;
      rt_event_recv(&sensor_event, SENSOR_EVENT_FLAG_TRIGGER, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                    RT_WAITING_FOREVER, &received);
    }
#endif
    if (dev_sensor_imu) {
      int rb = rt_device_read(dev_sensor_imu, 0, sensor_buffer, SENSORS_MPU6500_BUFF_LEN);
      if (rb == SENSORS_MPU6500_BUFF_LEN) {
        uint32_t timestamp = rt_tick_get();
        sensors_data = processAccGyroMeasurements(sensor_buffer);
        sensors_data.timestamp = timestamp;
        mcnSensorImuPublish(&sensors_data);
        mlogImuPushData(timestamp);
      } else {
        static int err_cnt = 0;
        if (++err_cnt % 100 == 0) {
          rt_kprintf("imu read fail %d\n", err_cnt);
        }
      }
    } else {
      static int not_found = 0;
      if (++not_found % 30 == 0) {
        rt_kprintf("imu not found\n");
      }
      rt_thread_mdelay(100);
    }
  }
}

static int taskSensorThreadAutoStart(void) {
#define THREAD_PRIORITY 4
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5
  static struct rt_thread task_tid_sensor_imu;
  static rt_uint8_t task_stack_sensor_imu[THREAD_STACK_SIZE];
  rt_thread_init(&task_tid_sensor_imu, "imu", sensor_minifly_thread_entry, RT_NULL, task_stack_sensor_imu,
                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_sensor_imu);
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK_SENSOR_EN
INIT_APP_EXPORT(taskSensorThreadAutoStart);
#endif