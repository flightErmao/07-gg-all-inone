#include "gyro_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "gyro_mcn"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "uMCN.h"
#include "imu_mcn.h"
#include "gyro_mcn.h"
}

#include <cstring>

/* 定义滤波后陀螺仪数据话题（在本文件内完成定义与发布） */
MCN_DEFINE(gyro, sizeof(gyro_filtered_msg_t));

// MCN echo 函数（参考 accgyro_spi_bmi270.cpp 中的 imu_raw_echo）
static int gyro_filtered_echo(void* parameter) {
  gyro_filtered_msg_t gyro_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &gyro_data) != RT_EOK) {
    return -1;
  }

  LOG_I("%lu pid:%.2f %.2f %.2f att:%.2f %.2f %.2f",
      gyro_data.seq, 
      gyro_data.gyro_filtered_for_pid[0], gyro_data.gyro_filtered_for_pid[1], gyro_data.gyro_filtered_for_pid[2],
      gyro_data.gyro_filtered_for_attitude[0], gyro_data.gyro_filtered_for_attitude[1], gyro_data.gyro_filtered_for_attitude[2]);
  return 0;
}

// 订阅 IMU MCN 主题
rt_err_t gyro::subscribeImu() {
  // 创建 MCN 事件信号量（用于 mcn_poll_sync）
  if (imu_event_ == RT_NULL) {
    imu_event_ = rt_sem_create("gyro_evt", 0, RT_IPC_FLAG_FIFO);
    if (imu_event_ == RT_NULL) {
      LOG_E("create imu event semaphore failed");
      return -RT_ERROR;
    }
  }

  // 订阅 gyro_raw MCN 节点（传入 event 用于同步等待）
  gyro_node_ = mcn_subscribe(MCN_HUB(gyro_raw), imu_event_, RT_NULL);
  if (gyro_node_ == RT_NULL) {
    LOG_E("subscribe gyro_raw topic failed");
    if (imu_event_ != RT_NULL) {
      rt_sem_delete(imu_event_);
      imu_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }
  LOG_I("Subscribed to gyro_raw MCN topic");
  return RT_EOK;
}

// 发布 gyro_filtered MCN 主题
rt_err_t gyro::advertiseGyroFiltered() {
  // 获取 gyro MCN hub（用于发布滤波后的数据）
  gyro_filtered_hub_ = MCN_HUB(gyro);
  if (gyro_filtered_hub_ == nullptr) {
    LOG_E("get gyro hub failed");
    return -RT_ERROR;
  }

  // 激活 gyro MCN 主题（必须调用，否则 mcn_publish 会失败）
  // 参考 accgyro_spi_bmi270.cpp:544 的 imu 主题激活方式
  rt_err_t advertise_ret = mcn_advertise(gyro_filtered_hub_, gyro_filtered_echo);
  if (advertise_ret != RT_EOK && advertise_ret != -RT_EBUSY) {
    // RT_EOK: 成功激活
    // -RT_EBUSY: 已经激活过了（可以忽略）
    // 其他值: 激活失败（内存不足等）
    LOG_E("gyro_filtered advertise failed: %d", advertise_ret);
    gyro_filtered_hub_ = nullptr;
    return advertise_ret;
  }
  LOG_I("gyro MCN topic advertised");
  return RT_EOK;
}

// MCN 初始化函数
rt_err_t gyro::initMcn() {
  // 订阅 IMU MCN 主题
  rt_err_t ret = subscribeImu();
  if (ret != RT_EOK) {
    return ret;
  }

  // 发布 gyro_filtered MCN 主题
  ret = advertiseGyroFiltered();
  if (ret != RT_EOK) {
    // 如果发布失败，清理订阅
    cleanupMcnSubscriptions();
    return ret;
  }

  return RT_EOK;
}

// 清理 MCN 订阅
void gyro::cleanupMcnSubscriptions() {
  if (gyro_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(gyro_raw), gyro_node_);
    gyro_node_ = RT_NULL;
  }

  if (imu_event_ != RT_NULL) {
    rt_sem_delete(imu_event_);
    imu_event_ = RT_NULL;
  }
}

// 发布滤波后的陀螺仪数据到 MCN
void gyro::publishGyroFiltered(rt_uint32_t seq) {
  if (gyro_filtered_hub_ == nullptr) {
    return;
  }

  gyro_filtered_msg_t filtered_msg;
  std::memcpy(filtered_msg.gyro_filtered_for_pid, gyro_adcf_, sizeof(gyro_adcf_));
  std::memcpy(filtered_msg.gyro_filtered_for_attitude, gyroFilteredDownsampled_, sizeof(gyroFilteredDownsampled_));
  filtered_msg.seq = seq;

  rt_err_t publish_result = mcn_publish(gyro_filtered_hub_, &filtered_msg);
  if (publish_result != RT_EOK) {
    LOG_E("Failed to publish gyro_filtered data: %d", publish_result);
  }
}
