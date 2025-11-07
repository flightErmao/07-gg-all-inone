#include "taskImuPub.h"

#ifdef PROJECT_PX4_TASK_IMU_PUB_EN

#include <rtconfig.h>
#include <rtthread.h>

#ifdef PROJECT_PX4_TASK_IMU_PUB_DEBUG
#define LOG_LVL LOG_LVL_DBG
#else
#define LOG_LVL LOG_LVL_INFO
#endif
#define LOG_TAG "imu.pub"
#include <ulog.h>

#include "bmi270_px4_wrapper.h"
#include "taskImuPub_mcn.h"
#include "sensorsTypes.h"

#ifndef PROJECT_PX4_TASK_IMU_PUB_STACK_SIZE
#define PROJECT_PX4_TASK_IMU_PUB_STACK_SIZE 4096
#endif

#ifndef PROJECT_PX4_TASK_IMU_PUB_PRIORITY
#define PROJECT_PX4_TASK_IMU_PUB_PRIORITY 6
#endif

#ifndef PROJECT_PX4_TASK_IMU_PUB_TIMESLICE
#define PROJECT_PX4_TASK_IMU_PUB_TIMESLICE 5
#endif

static rt_thread_t imu_pub_thread = RT_NULL;

static void imu_pub_thread_entry(void *parameter) {
  RT_UNUSED(parameter);

  if (px4ImuMcnInit() != RT_EOK) {
    LOG_E("mc init failed");
    return;
  }

  if (px4_bmi270_init(RT_NULL) != RT_EOK) {
    LOG_E("bmi270 init failed");
    return;
  }

  uint32_t wait_count = 0;
  while (!px4_bmi270_is_ready()) {
    rt_thread_mdelay(10);
    if (++wait_count > 200) {
      LOG_E("bmi270 ready timeout");
      break;
    }
  }

  LOG_I("bmi270 ready state: %d", px4_bmi270_is_ready());

  while (1) {
    px4ImuMcnWait();

#ifdef PROJECT_PX4_TASK_IMU_PUB_DEBUG
    sensorData_t data = {0};
    if (px4ImuMcnAcquire(&data) == RT_EOK) {
      LOG_D("acc[%d %d %d] gyro[%d %d %d]", data.acc_raw.x, data.acc_raw.y, data.acc_raw.z,
            data.gyro_raw.x, data.gyro_raw.y, data.gyro_raw.z);
    }
#endif
  }
}

int taskImuPubInit(void) {
  if (imu_pub_thread != RT_NULL) {
    return 0;
  }

  imu_pub_thread = rt_thread_create("imu_pub", imu_pub_thread_entry, RT_NULL,
                                    PROJECT_PX4_TASK_IMU_PUB_STACK_SIZE,
                                    PROJECT_PX4_TASK_IMU_PUB_PRIORITY,
                                    PROJECT_PX4_TASK_IMU_PUB_TIMESLICE);

  if (imu_pub_thread == RT_NULL) {
    LOG_E("create imu thread fail");
    return -RT_ERROR;
  }

  rt_thread_startup(imu_pub_thread);
  LOG_I("imu publish task started");
  return RT_EOK;
}

INIT_APP_EXPORT(taskImuPubInit);

#else

int taskImuPubInit(void) {
  return 0;
}

#endif  // PROJECT_PX4_TASK_IMU_PUB_EN


