#include "rtthread.h"
#include "rtdbg.h"
#include "mpu6500.hpp"
#include "task_anotc_telem.h"
#include "imu.h"
#include <rtdevice.h>

#define DBG_TAG "task_sensor_minifly"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

static struct rt_thread task_tid_sensor_minifly;
static rt_uint8_t task_stack_sensor_minifly[THREAD_STACK_SIZE];

static rt_device_t dev_sensor_imu = RT_NULL;

static void task_dev_init(void) {
  rt_device_t dev_temp = RT_NULL;

#ifdef TASK_SENSOR_MINIFLY_IMU_NAME
  dev_temp = rt_device_find(TASK_SENSOR_MINIFLY_IMU_NAME);
#else
  dev_temp = rt_device_find(SENSOR_NAME_MPU6500_MINIFLY);
#endif
  if (dev_temp) {
    rt_device_open(dev_temp, RT_DEVICE_OFLAG_RDWR);
    dev_sensor_imu = dev_temp;
  }
}

static void sensor_minifly_thread_entry(void *parameter) {
  RT_UNUSED(parameter);

  task_dev_init();

  // 原始寄存器起始地址：加速度 X 高字节
  const rt_off_t reg_start = 0x3B;  // MPU6500_RA_ACCEL_XOUT_H

  while (1) {
    if (dev_sensor_imu) {
      uint8_t raw[14] = {0};
      int rb = rt_device_read(dev_sensor_imu, reg_start, raw, 14);
      if (rb == 14) {
        int16_t ax = ((uint16_t)raw[0] << 8) | raw[1];
        int16_t ay = ((uint16_t)raw[2] << 8) | raw[3];
        int16_t az = ((uint16_t)raw[4] << 8) | raw[5];
        // int16_t temp_raw = ((uint16_t)raw[6] << 8) | raw[7];
        int16_t gx = ((uint16_t)raw[8] << 8) | raw[9];
        int16_t gy = ((uint16_t)raw[10] << 8) | raw[11];
        int16_t gz = ((uint16_t)raw[12] << 8) | raw[13];

        sendUserDatafloat6(IMU_DATA, (float)ax, (float)ay, (float)az, (float)gx, (float)gy, (float)gz);
      } else {
        static int err_cnt = 0;
        if (++err_cnt % 100 == 0) {
          LOG_W("imu read fail %d", err_cnt);
        }
      }
      // 不主动延时；mpu6500 的 read 自身阻塞在数据就绪中断
    } else {
      static int not_found = 0;
      if (++not_found % 200 == 0) {
        LOG_W("imu device not found, retrying...");
      }
      task_dev_init();
      rt_thread_mdelay(20);
    }
  }
}

static void task_thread_init(void) {
  rt_thread_init(&task_tid_sensor_minifly, "t_sensor6", sensor_minifly_thread_entry, RT_NULL, task_stack_sensor_minifly,
                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_sensor_minifly);
}

#ifdef TASK_05_SENSOR_MINIFLY
INIT_APP_EXPORT(task_thread_init);
#endif