#include "rc_bf.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#include "uMCN.h"
#define LOG_TAG "rc_bf_mcn"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cstring>
#include "rc_setpoint_msg.h"

/* 定义 RC setpoint MCN 话题（在本文件内完成定义与发布） */
MCN_DEFINE(rc, sizeof(rc_setpoint_msg_t));

// MCN echo 函数（用于调试）- 调用 RcBf 成员函数
// 需要使用 C 链接以便 MCN 调用，但实际逻辑在对象成员函数中
extern "C" {
static int rc_setpoint_echo(void* parameter) {
  rc_setpoint_msg_t setpoint_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &setpoint_data) != RT_EOK) {
    return -1;
  }

  // Call RcBf member function to print debug information
  // This keeps the logic in the object while maintaining C linkage for MCN
  RcBf& rc = RcBf::instance();
  rc.echoSetpoint(&setpoint_data);

  return 0;
}
}

rt_err_t RcBf::initMcn() {
  // Get rc_setpoint MCN hub（用于发布 setpoint 数据）
  rc_setpoint_hub_ = MCN_HUB(rc);
  if (rc_setpoint_hub_ == nullptr) {
    LOG_E("get rc_setpoint hub failed");
    return -RT_ERROR;
  }

  // 激活 rc_setpoint MCN 主题（必须调用，否则 mcn_publish 会失败）
  rt_err_t advertise_ret = mcn_advertise(rc_setpoint_hub_, rc_setpoint_echo);
  if (advertise_ret != RT_EOK && advertise_ret != -RT_EBUSY) {
    LOG_E("rc_setpoint advertise failed: %d", advertise_ret);
    return advertise_ret;
  }
  LOG_I("rc MCN topic advertised");

  // Subscribe to rc MCN topic (for PID thread to use)
  rc_setpoint_event_ = rt_sem_create("rc_setpoint_evt", 0, RT_IPC_FLAG_FIFO);
  if (rc_setpoint_event_ == RT_NULL) {
    LOG_E("create rc_setpoint event semaphore failed");
    return -RT_ERROR;
  }

  rc_setpoint_node_ = mcn_subscribe(rc_setpoint_hub_, rc_setpoint_event_, RT_NULL);
  if (rc_setpoint_node_ == RT_NULL) {
    LOG_E("subscribe rc topic failed");
    if (rc_setpoint_event_ != RT_NULL) {
      rt_sem_delete(rc_setpoint_event_);
      rc_setpoint_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }
  LOG_I("Subscribed to rc MCN topic");

  return RT_EOK;
}

void RcBf::publishSetpointToMcn(uint32_t current_time_us) {
  // Publish rawSetpoint data to MCN for PID thread (avoid data tearing)
  if (rc_setpoint_hub_ != nullptr) {
    rc_setpoint_msg_t setpoint_msg;
    std::memcpy(setpoint_msg.rawSetpoint, rawSetpoint_, sizeof(rawSetpoint_));
    setpoint_msg.rcCommandThrottle = rc_command_[THROTTLE];
    std::memcpy(setpoint_msg.feedforward, feedforward_, sizeof(feedforward_));
    std::memcpy(setpoint_msg.rcDeflection, rcDeflection_, sizeof(rcDeflection_));
    std::memcpy(setpoint_msg.rcDeflectionAbs, rcDeflectionAbs_, sizeof(rcDeflectionAbs_));
    setpoint_msg.smoothedRxRateHz = smoothed_rx_rate_hz_;
    setpoint_msg.seq = seq_++;
    setpoint_msg.timestamp = current_time_us;

    rt_err_t publish_result = mcn_publish(rc_setpoint_hub_, &setpoint_msg);
    if (publish_result != RT_EOK) {
      LOG_E("Failed to publish rc_setpoint data: %d", publish_result);
    }
  }
}

void RcBf::echoSetpoint(const rc_setpoint_msg_t* setpoint_data) {
  // Print detailed debug information by default
  printDebugInfo(setpoint_data);

  // Always print basic info
  LOG_I("seq: %lu, rawSetpoint: %.2f, %.2f, %.2f, throttle: %.0f, rxRate: %.1f Hz", setpoint_data->seq,
        setpoint_data->rawSetpoint[0], setpoint_data->rawSetpoint[1], setpoint_data->rawSetpoint[2],
        setpoint_data->rcCommandThrottle, setpoint_data->smoothedRxRateHz);
}

