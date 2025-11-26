#include "rc_class.hpp"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#include "uMCN.h"
#define LOG_TAG "rc_bf_mcn"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cstring>
#include "rc_mcn.h"
#include "rc_aux.h"
#include "rc_smooth.h"  // For THROTTLE definition

/* 定义 RC setpoint MCN 话题（在本文件内完成定义与发布） */
MCN_DEFINE(rc, sizeof(rc_command_msg_t));

/* 定义 RC auxiliary channels MCN 话题（在本文件内完成定义与发布） */
MCN_DEFINE(aux, sizeof(rc_aux_msg_t));

// MCN echo 函数（用于调试）- 调用 RcBf 成员函数
// 需要使用 C 链接以便 MCN 调用，但实际逻辑在对象成员函数中
extern "C" {
__attribute__((used)) static int echo_rc_command(void* parameter) {
  rc_command_msg_t rc_command;

  if (mcn_copy_from_hub((McnHub*)parameter, &rc_command) != RT_EOK) {
    return -1;
  }

  // Call RcBf member function to print debug information
  // This keeps the logic in the object while maintaining C linkage for MCN
  RcBf& rc = RcBf::instance();
  rc.echoSetpoint(&rc_command);

  return 0;
}

__attribute__((used)) static int echo_rc_aux(void* parameter) {
  rc_aux_msg_t aux_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &aux_data) != RT_EOK) {
    return -1;
  }

  // Call RcBf member function to print debug information
  // This keeps the logic in the object while maintaining C linkage for MCN
  RcBf& rc = RcBf::instance();
  rc.echoAux(&aux_data);

  return 0;
}
}

rt_err_t RcBf::initMcn() {
  // Get rc_setpoint MCN hub（用于发布 setpoint 数据）
  rc_command_hub_ = MCN_HUB(rc);
  if (rc_command_hub_ == nullptr) {
    LOG_E("get rc_setpoint hub failed");
    return -RT_ERROR;
  }

  // 激活 rc_setpoint MCN 主题（必须调用，否则 mcn_publish 会失败）
  rt_err_t advertise_ret = mcn_advertise(rc_command_hub_, echo_rc_command);
  if (advertise_ret != RT_EOK && advertise_ret != -RT_EBUSY) {
    LOG_E("rc_setpoint advertise failed: %d", advertise_ret);
    return advertise_ret;
  }
  LOG_I("rc MCN topic advertised");

  // Get aux MCN hub（用于发布辅助通道数据）
  rc_aux_hub_ = MCN_HUB(aux);
  if (rc_aux_hub_ == nullptr) {
    LOG_E("get aux hub failed");
    return -RT_ERROR;
  }

  // 激活 aux MCN 主题（必须调用，否则 mcn_publish 会失败）
  rt_err_t advertise_aux_ret = mcn_advertise(rc_aux_hub_, echo_rc_aux);
  if (advertise_aux_ret != RT_EOK && advertise_aux_ret != -RT_EBUSY) {
    LOG_E("aux advertise failed: %d", advertise_aux_ret);
    return advertise_aux_ret;
  }
  LOG_I("aux MCN topic advertised");

  return RT_EOK;
}

void RcBf::publishRcCommandToMcn(uint32_t current_time_us) {
  // Publish rawSetpoint data to MCN for PID thread (avoid data tearing)
  // Only publish data needed by smoothing filter
  if (rc_command_hub_ != nullptr) {
    rc_command_msg_t rcCommand_msg;
    std::memcpy(rcCommand_msg.rawSetpoint, rawSetpoint_, sizeof(rawSetpoint_));
    rcCommand_msg.rcCommandThrottle = rc_command_[THROTTLE];
    std::memcpy(rcCommand_msg.feedforward, feedforward_, sizeof(feedforward_));
    rcCommand_msg.seq = seq_++;
    rcCommand_msg.timestamp = current_time_us;

    rt_err_t publish_result = mcn_publish(rc_command_hub_, &rcCommand_msg);
    if (publish_result != RT_EOK) {
      LOG_E("Failed to publish rc_setpoint data: %d", publish_result);
    }
  }
}

void RcBf::publishAuxChannelsToMcn(uint32_t current_time_us) {
  // Publish auxiliary channels data to MCN for PID thread and other consumers
  // Includes arm status and flight mode for PID and motor control
  if (rc_aux_hub_ != nullptr) {
    rc_aux_msg_t aux_msg;
    
    // Copy AUX channel values (channels 5-18, indices 4-17)
    uint8_t aux_count = 0;
    if (channel_count_ > 4) {
      aux_count = channel_count_ - 4;
      if (aux_count > MAX_AUX_CHANNEL_COUNT) {
        aux_count = MAX_AUX_CHANNEL_COUNT;
      }
      for (uint8_t i = 0; i < aux_count; i++) {
        aux_msg.aux_channels[i] = rc_data_[4 + i];  // Channels 5-18 (indices 4-17)
      }
    }
    // Zero out unused AUX channels
    for (uint8_t i = aux_count; i < MAX_AUX_CHANNEL_COUNT; i++) {
      aux_msg.aux_channels[i] = 0.0f;
    }

    // Get arm status and flight mode from RcControls
    RcControls& rc_controls = RcControls::instance();
    aux_msg.armed = rc_controls.isArmed() ? RC_ARMED_STATUS_ARMED : RC_ARMED_STATUS_DISARMED;
    aux_msg.flight_mode = rc_controls.getFlightMode();
    
    aux_msg.seq = seq_++;
    aux_msg.timestamp = current_time_us;

    rt_err_t publish_result = mcn_publish(rc_aux_hub_, &aux_msg);
    if (publish_result != RT_EOK) {
      LOG_E("Failed to publish aux data: %d", publish_result);
    }
  }
}

void RcBf::echoSetpoint(const rc_command_msg_t* rc_command) {
  // Print rawSetpoint, rcCommandThrottle, and feedforward from published message
  LOG_I("rawSetpoint: %.2f, %.2f, %.2f | throttle: %.0f | feedforward: %.2f, %.2f, %.2f", rc_command->rawSetpoint[0],
        rc_command->rawSetpoint[1], rc_command->rawSetpoint[2], rc_command->rcCommandThrottle,
        rc_command->feedforward[0], rc_command->feedforward[1], rc_command->feedforward[2]);
}

void RcBf::echoAux(const rc_aux_msg_t* aux_data) {
  // Get arming state and flight mode from RcControls
  RcControls& rc_controls = RcControls::instance();
  bool is_armed = rc_controls.isArmed();
  uint8_t flight_mode = rc_controls.getFlightMode();

  // Print auxiliary channels debug information (first 6 channels)
  LOG_I("arm: %s, mode: %u, aux: %.0f %.0f %.0f %.0f %.0f %.0f", is_armed ? "ARMED" : "DISARMED", flight_mode,
        aux_data->aux_channels[0], aux_data->aux_channels[1], aux_data->aux_channels[2], aux_data->aux_channels[3],
        aux_data->aux_channels[4], aux_data->aux_channels[5]);
}

