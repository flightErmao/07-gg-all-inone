#include "acc_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "acc_mcn"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "uMCN.h"
#include "imu_mcn.h"
#include "acc_mcn.h"
#include "timestamp.h"
}

#include <cstring>

/* 定义处理后的加速度计数据话题（在本文件内完成定义与发布） */
MCN_DEFINE(acc, sizeof(acc_filtered_msg_t));

// MCN echo 函数
static int acc_filtered_echo(void* parameter) {
  acc_filtered_msg_t acc_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &acc_data) != RT_EOK) {
    return -1;
  }

  LOG_I("%lu f:%.2f %.2f %.2f r:%.2f %.2f %.2f", acc_data.seq,
        acc_data.acc_filtered[0], acc_data.acc_filtered[1], acc_data.acc_filtered[2], 
        acc_data.acc_adc[0], acc_data.acc_adc[1], acc_data.acc_adc[2]);
  return 0;
}

// MCN 初始化函数
rt_err_t AccBf::initMcn() {
  // 创建 MCN 事件信号量（用于 mcn_poll_sync）
  if (imu_event_ == RT_NULL) {
    imu_event_ = rt_sem_create("acc_evt", 0, RT_IPC_FLAG_FIFO);
    if (imu_event_ == RT_NULL) {
      LOG_E("create imu event semaphore failed");
      return -RT_ERROR;
    }
  }

  // 订阅 acc_raw MCN 节点（传入 event 用于同步等待）
  imu_node_ = mcn_subscribe(MCN_HUB(acc_raw), imu_event_, RT_NULL);
  if (imu_node_ == RT_NULL) {
    LOG_E("subscribe acc_raw topic failed");
    if (imu_event_ != RT_NULL) {
      rt_sem_delete(imu_event_);
      imu_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }
  LOG_I("Subscribed to acc_raw MCN topic");

  // 获取 acc MCN hub（用于发布处理后的数据）
  acc_filtered_hub_ = MCN_HUB(acc);
  if (acc_filtered_hub_ == nullptr) {
    LOG_E("get acc hub failed");
    if (imu_node_ != RT_NULL) {
      mcn_unsubscribe(MCN_HUB(acc_raw), imu_node_);
      imu_node_ = RT_NULL;
    }
    if (imu_event_ != RT_NULL) {
      rt_sem_delete(imu_event_);
      imu_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }

  // 激活 acc MCN 主题（必须调用，否则 mcn_publish 会失败）
  rt_err_t advertise_ret = mcn_advertise(acc_filtered_hub_, acc_filtered_echo);
  if (advertise_ret != RT_EOK && advertise_ret != -RT_EBUSY) {
    // RT_EOK: 成功激活
    // -RT_EBUSY: 已经激活过了（可以忽略）
    // 其他值: 激活失败（内存不足等）
    LOG_E("acc_filtered advertise failed: %d", advertise_ret);
    if (imu_node_ != RT_NULL) {
      mcn_unsubscribe(MCN_HUB(acc_raw), imu_node_);
      imu_node_ = RT_NULL;
    }
    if (imu_event_ != RT_NULL) {
      rt_sem_delete(imu_event_);
      imu_event_ = RT_NULL;
    }
    return advertise_ret;
  }
  LOG_I("acc MCN topic advertised");

  return RT_EOK;
}

// 清理 MCN 订阅
void AccBf::cleanupMcnSubscriptions() {
  if (imu_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(acc_raw), imu_node_);
    imu_node_ = RT_NULL;
  }

  if (imu_event_ != RT_NULL) {
    rt_sem_delete(imu_event_);
    imu_event_ = RT_NULL;
  }
}

// 发布处理后的加速度计数据到 MCN
void AccBf::publishAccFiltered(const acc_raw_msg_t* acc_data) {
  if (acc_data == nullptr || acc_filtered_hub_ == nullptr) {
    return;
  }

  acc_filtered_msg_t filtered_msg;
  std::memcpy(filtered_msg.acc_filtered, acc_filtered_, sizeof(acc_filtered_));
  std::memcpy(filtered_msg.acc_adc, acc_adc_, sizeof(acc_adc_));
  filtered_msg.seq = acc_data->seq;
  filtered_msg.timestamp = timestamp_micros();

  rt_err_t publish_result = mcn_publish(acc_filtered_hub_, &filtered_msg);
  if (publish_result != RT_EOK) {
    LOG_E("Failed to publish acc_filtered data: %d", publish_result);
  }
}

