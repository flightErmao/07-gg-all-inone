#include "rtthread.h"
#include <rtdevice.h>
#include "imu.h"
#include "biasGyro.h"
#include "filterLpf2p.h"
#include "filterNotch2p.h"
#include "imuProcess.h"
#include "rtconfig.h"
#include "aMlogSensorImu.h"
#include "aMcnSensorImu.h"
#include "debugPin.h"
#include "timestamp.h"

#ifdef PROJECT_MINIFLY_TASK_SENSOR_HWTIMER_TRIGGER_EN
#include "hardwareTimerSensor.h"
#endif

#define SENSORS_MPU6500_BUFF_LEN 14
static rt_device_t dev_sensor_imu = RT_NULL;

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

static void sensor_imu_thread_entry(void* parameter) {
  deviceInit();

#ifdef PROJECT_MINIFLY_TASK_SENSOR_FILTER_EN
  // Initialize notch filters with configured parameters
#ifdef PROJECT_MINIFLY_TASK_SENSOR_NOTCH_GYRO_1_EN
  filterInitNotchGyro(1000.0f, 
                      (float)PROJECT_MINIFLY_TASK_SENSOR_NOTCH_GYRO_1_FREQ, 
                      (float)PROJECT_MINIFLY_TASK_SENSOR_NOTCH_GYRO_1_Q / 10.0f);
#endif
#ifdef PROJECT_MINIFLY_TASK_SENSOR_NOTCH_GYRO_2_EN
  filterInitNotchGyro2(1000.0f, 
                       (float)PROJECT_MINIFLY_TASK_SENSOR_NOTCH_GYRO_2_FREQ, 
                       (float)PROJECT_MINIFLY_TASK_SENSOR_NOTCH_GYRO_2_Q / 10.0f);
#endif
#ifdef PROJECT_MINIFLY_TASK_SENSOR_NOTCH_GYRO_3_EN
  filterInitNotchGyro3(1000.0f, 
                       (float)PROJECT_MINIFLY_TASK_SENSOR_NOTCH_GYRO_3_FREQ, 
                       (float)PROJECT_MINIFLY_TASK_SENSOR_NOTCH_GYRO_3_Q / 10.0f);
#endif
#ifdef PROJECT_MINIFLY_TASK_SENSOR_NOTCH_ACC_EN
  filterInitNotchAcc(1000.0f, 
                     (float)PROJECT_MINIFLY_TASK_SENSOR_NOTCH_ACC_FREQ, 
                     (float)PROJECT_MINIFLY_TASK_SENSOR_NOTCH_ACC_Q / 10.0f);
#endif
  filterInitLpf2AccGyro();
#endif
  
  sensorsBiasObjInit();
  initImuRotationDir();
  mlogImuInit();

  uint8_t sensor_buffer[SENSORS_MPU6500_BUFF_LEN] = {0};
  sensorData_t sensors_data = {0};

  while (1) {
#ifdef PROJECT_MINIFLY_TASK_SENSOR_HWTIMER_TRIGGER_EN
    /* Wait for hardware timer event to trigger sensor reading */
    hardwareTimerSensorRecvEvent(RT_WAITING_FOREVER);
#endif
    if (dev_sensor_imu) {
      DEBUG_PIN_DEBUG0_HIGH();
      int rb = rt_device_read(dev_sensor_imu, 0, sensor_buffer, SENSORS_MPU6500_BUFF_LEN);
      if (rb == SENSORS_MPU6500_BUFF_LEN) {
        uint32_t timestamp = timestamp_micros();
        sensors_data = processAccGyroMeasurements(sensor_buffer);
        sensors_data.timestamp = timestamp;
        mcnSensorImuPublish(&sensors_data);
        mlogImuPushData(timestamp);
        DEBUG_PIN_DEBUG0_LOW();
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
  rt_thread_init(&task_tid_sensor_imu, "imu", sensor_imu_thread_entry, RT_NULL, task_stack_sensor_imu,
                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_sensor_imu);
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK_SENSOR_EN
INIT_APP_EXPORT(taskSensorThreadAutoStart);
#endif