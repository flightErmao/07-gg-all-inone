#include "attitude_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "attitude_mcn"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "uMCN.h"
#include "imu_mcn.h"
#include "gyro_mcn.h"
#include "acc_mcn.h"
#include "attitude_mcn.h"
#ifdef PROJECT_BF_ACC_EN
#include "../acc/inc/acc_mcn.h"
#endif
#include "timestamp.h"
}

#include <cstring>

/* 定义姿态数据话题（在本文件内完成定义与发布） */
MCN_DEFINE(att, sizeof(attitude_msg_t));

// MCN echo 函数（参考 gyro_mcn.cpp 中的 gyro_filtered_echo）
static int attitude_echo(void* parameter) {
  attitude_msg_t attitude_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &attitude_data) != RT_EOK) {
    return -1;
  }

  LOG_I("%lu deg:%.2f %.2f %.2f", attitude_data.seq, attitude_data.values[0], attitude_data.values[1],
        attitude_data.values[2]);
  return 0;
}

// MCN 初始化函数
rt_err_t AttitudeBf::initMcn() {
  // 创建 MCN 事件信号量（用于 mcn_poll_sync）
  if (imu_event_ == RT_NULL) {
    imu_event_ = rt_sem_create("att_imu_evt", 0, RT_IPC_FLAG_FIFO);
    if (imu_event_ == RT_NULL) {
      LOG_E("create imu event semaphore failed");
      return -RT_ERROR;
    }
  }

  // 订阅 imu MCN 节点（传入 event 用于同步等待）
  imu_node_ = mcn_subscribe(MCN_HUB(imu), imu_event_, RT_NULL);
  if (imu_node_ == RT_NULL) {
    LOG_E("subscribe imu topic failed");
    if (imu_event_ != RT_NULL) {
      rt_sem_delete(imu_event_);
      imu_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }
  LOG_I("Subscribed to imu MCN topic");

  // 订阅 gyro_filtered MCN 节点（可选，用于更精确的姿态估计）
  if (gyro_event_ == RT_NULL) {
    gyro_event_ = rt_sem_create("att_gyro_evt", 0, RT_IPC_FLAG_FIFO);
    if (gyro_event_ == RT_NULL) {
      LOG_E("create gyro event semaphore failed");
      // 继续执行，gyro 订阅是可选的
    } else {
      gyro_node_ = mcn_subscribe(MCN_HUB(gyro), gyro_event_, RT_NULL);
      if (gyro_node_ == RT_NULL) {
        LOG_W("subscribe gyro topic failed (optional)");
        rt_sem_delete(gyro_event_);
        gyro_event_ = RT_NULL;
      } else {
        LOG_I("Subscribed to gyro MCN topic (optional)");
      }
    }
  }

#ifdef PROJECT_BF_ACC_EN
  // 订阅 acc_filtered MCN 节点（用于使用处理后的加速度计数据）
  if (acc_event_ == RT_NULL) {
    acc_event_ = rt_sem_create("att_acc_evt", 0, RT_IPC_FLAG_FIFO);
    if (acc_event_ == RT_NULL) {
      LOG_E("create acc event semaphore failed");
      // 继续执行，acc 订阅是可选的（如果没有 acc 模块，会使用 imu 的原始数据）
    } else {
      acc_node_ = mcn_subscribe(MCN_HUB(acc), acc_event_, RT_NULL);
      if (acc_node_ == RT_NULL) {
        LOG_W("subscribe acc topic failed (optional)");
        rt_sem_delete(acc_event_);
        acc_event_ = RT_NULL;
      } else {
        LOG_I("Subscribed to acc MCN topic (optional)");
      }
    }
  }
#endif

  // 获取 attitude MCN hub（用于发布姿态数据）
  attitude_hub_ = MCN_HUB(att);
  if (attitude_hub_ == nullptr) {
    LOG_E("get attitude hub failed");
    if (imu_node_ != RT_NULL) {
      mcn_unsubscribe(MCN_HUB(imu), imu_node_);
      imu_node_ = RT_NULL;
    }
    if (imu_event_ != RT_NULL) {
      rt_sem_delete(imu_event_);
      imu_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }

  // 激活 attitude MCN 主题（必须调用，否则 mcn_publish 会失败）
  rt_err_t advertise_ret = mcn_advertise(attitude_hub_, attitude_echo);
  if (advertise_ret != RT_EOK && advertise_ret != -RT_EBUSY) {
    // RT_EOK: 成功激活
    // -RT_EBUSY: 已经激活过了（可以忽略）
    // 其他值: 激活失败（内存不足等）
    LOG_E("attitude advertise failed: %d", advertise_ret);
    if (imu_node_ != RT_NULL) {
      mcn_unsubscribe(MCN_HUB(imu), imu_node_);
      imu_node_ = RT_NULL;
    }
    if (imu_event_ != RT_NULL) {
      rt_sem_delete(imu_event_);
      imu_event_ = RT_NULL;
    }
    return advertise_ret;
  }
  LOG_I("attitude MCN topic advertised");

  return RT_EOK;
}

// 清理 MCN 订阅
void AttitudeBf::cleanupMcnSubscriptions() {
  if (imu_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(imu), imu_node_);
    imu_node_ = RT_NULL;
  }

  if (imu_event_ != RT_NULL) {
    rt_sem_delete(imu_event_);
    imu_event_ = RT_NULL;
  }

  if (gyro_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(gyro), gyro_node_);
    gyro_node_ = RT_NULL;
  }

  if (gyro_event_ != RT_NULL) {
    rt_sem_delete(gyro_event_);
    gyro_event_ = RT_NULL;
  }

#ifdef PROJECT_BF_ACC_EN
  if (acc_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(acc), acc_node_);
    acc_node_ = RT_NULL;
  }

  if (acc_event_ != RT_NULL) {
    rt_sem_delete(acc_event_);
    acc_event_ = RT_NULL;
  }
#endif
}

// 发布姿态数据到 MCN
void AttitudeBf::publishAttitude(const imu_raw_msg_t* imu_data) {
  if (imu_data == nullptr || attitude_hub_ == nullptr) {
    return;
  }

  attitude_msg_t attitude_msg;
  
  // 填充姿态数据
  for (int i = 0; i < 3; i++) {
    // raw: 单位十分之一度（centidegrees），与 Betaflight attitude.raw 一致
    attitude_msg.raw[i] = static_cast<int16_t>(attitude_values_[i] * 10.0f);
    // values: 单位度（degrees），与 Betaflight attitude.values 一致
    attitude_msg.values[i] = attitude_values_[i];
  }
  
  attitude_msg.seq = imu_data->seq;
  attitude_msg.timestamp = timestamp_micros();

  rt_err_t publish_result = mcn_publish(attitude_hub_, &attitude_msg);
  if (publish_result != RT_EOK) {
    LOG_E("Failed to publish attitude data: %d", publish_result);
  }
}

