#include "taskImuPub_mcn.h"

#include <rtthread.h>

#define LOG_TAG "imu.pub.mcn"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

MCN_DEFINE(px4_imu, sizeof(sensorData_t));

static McnNode_t px4_imu_sub_node = RT_NULL;
static rt_sem_t px4_imu_event = RT_NULL;

int px4ImuMcnInit(void) {
  if (px4_imu_sub_node != RT_NULL) {
    return 0;
  }

  if (px4_imu_event == RT_NULL) {
    px4_imu_event = rt_sem_create("imu_evt", 0, RT_IPC_FLAG_FIFO);
    if (px4_imu_event == RT_NULL) {
      LOG_E("create imu event fail");
      return -RT_ERROR;
    }
  }

  px4_imu_sub_node = mcn_subscribe(MCN_HUB(px4_imu), px4_imu_event, RT_NULL);
  if (px4_imu_sub_node == RT_NULL) {
    LOG_E("subscribe imu topic fail");
    return -RT_ERROR;
  }

  LOG_I("px4 imu mcn ready");
  return RT_EOK;
}

int px4ImuMcnPublish(const sensorData_t *sensor_data) {
  if (sensor_data == RT_NULL) {
    return -RT_EINVAL;
  }

  return mcn_publish(MCN_HUB(px4_imu), sensor_data);
}

int px4ImuMcnAcquire(sensorData_t *sensor_data) {
  if (sensor_data == RT_NULL || px4_imu_sub_node == RT_NULL) {
    return -RT_EINVAL;
  }

  return mcn_copy(MCN_HUB(px4_imu), px4_imu_sub_node, sensor_data);
}

void px4ImuMcnWait(void) {
  if (px4_imu_sub_node == RT_NULL) {
    return;
  }

  mcn_poll_sync(px4_imu_sub_node, RT_WAITING_FOREVER);
}


