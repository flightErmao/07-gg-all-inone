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
#include "rc_aux_msg.h"
#include "rc_controls_bf.h"

/* 定义 RC setpoint MCN 话题（在本文件内完成定义与发布） */
MCN_DEFINE(rc, sizeof(rc_setpoint_msg_t));

/* 定义 RC auxiliary channels MCN 话题（在本文件内完成定义与发布） */
MCN_DEFINE(aux, sizeof(rc_aux_msg_t));

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

static int aux_echo(void* parameter) {
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

  // Get aux MCN hub（用于发布辅助通道数据）
  rc_aux_hub_ = MCN_HUB(aux);
  if (rc_aux_hub_ == nullptr) {
    LOG_E("get aux hub failed");
    return -RT_ERROR;
  }

  // 激活 aux MCN 主题（必须调用，否则 mcn_publish 会失败）
  rt_err_t advertise_aux_ret = mcn_advertise(rc_aux_hub_, aux_echo);
  if (advertise_aux_ret != RT_EOK && advertise_aux_ret != -RT_EBUSY) {
    LOG_E("aux advertise failed: %d", advertise_aux_ret);
    return advertise_aux_ret;
  }
  LOG_I("aux MCN topic advertised");

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

void RcBf::publishAuxChannelsToMcn(uint32_t current_time_us) {
  // Publish auxiliary channels data to MCN for PID thread and other consumers
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
    aux_msg.aux_channel_count = aux_count;
    aux_msg.rx_receiving_signal = rx_receiving_signal_ ? 1 : 0;
    aux_msg.rx_flight_channels_valid = rx_flight_channels_valid_ ? 1 : 0;
    aux_msg.seq = seq_++;
    aux_msg.timestamp = current_time_us;

    rt_err_t publish_result = mcn_publish(rc_aux_hub_, &aux_msg);
    if (publish_result != RT_EOK) {
      LOG_E("Failed to publish aux data: %d", publish_result);
    }
  }
}

void RcBf::echoSetpoint(const rc_setpoint_msg_t* setpoint_data) {
  // Get 4 channels raw data from RcBf instance (roll, pitch, yaw, throttle)
  // rc_data_ contains processed RC data after range scaling, failsafe, and constraints
  const float* rc_data = getRcData();
  // Get rcCommand data (roll, pitch, yaw, throttle)
  const float* rc_command = getRcCommandArray();
  
  // Print rcCommand and 4 channels raw data (roll, pitch, yaw, throttle) in one line
  LOG_I("rcCmd: %.2f, %.2f, %.2f, %.0f | rawCh: %.0f, %.0f, %.0f, %.0f", 
        rc_command[0], rc_command[1], rc_command[2], rc_command[3],
        rc_data[0], rc_data[1], rc_data[2], rc_data[3]);
}

void RcBf::echoAux(const rc_aux_msg_t* aux_data) {
  // Get arming state and flight mode from RcControls
  RcControls& rc_controls = RcControls::instance();
  bool is_armed = rc_controls.isArmed();
  uint8_t flight_mode = rc_controls.getFlightMode();
  
  // Print auxiliary channels debug information
  if (aux_data->aux_channel_count > 0) {
    // Print first few AUX channels (up to 6) in one line
    uint8_t print_count = aux_data->aux_channel_count > 6 ? 6 : aux_data->aux_channel_count;
    if (print_count >= 1) {
      if (print_count == 1) {
        LOG_I("arm: %s, mode: %u, rx_signal: %s, rx_valid: %s, aux: %.0f", 
              is_armed ? "ARMED" : "DISARMED", flight_mode,
              aux_data->rx_receiving_signal ? "OK" : "LOST", 
              aux_data->rx_flight_channels_valid ? "OK" : "INVALID",
              aux_data->aux_channels[0]);
      } else if (print_count == 2) {
        LOG_I("arm: %s, mode: %u, rx_signal: %s, rx_valid: %s, aux: %.0f %.0f", 
              is_armed ? "ARMED" : "DISARMED", flight_mode,
              aux_data->rx_receiving_signal ? "OK" : "LOST", 
              aux_data->rx_flight_channels_valid ? "OK" : "INVALID",
              aux_data->aux_channels[0], aux_data->aux_channels[1]);
      } else if (print_count == 3) {
        LOG_I("arm: %s, mode: %u, rx_signal: %s, rx_valid: %s, aux: %.0f %.0f %.0f", 
              is_armed ? "ARMED" : "DISARMED", flight_mode,
              aux_data->rx_receiving_signal ? "OK" : "LOST", 
              aux_data->rx_flight_channels_valid ? "OK" : "INVALID",
              aux_data->aux_channels[0], aux_data->aux_channels[1], aux_data->aux_channels[2]);
      } else if (print_count == 4) {
        LOG_I("arm: %s, mode: %u, rx_signal: %s, rx_valid: %s, aux: %.0f %.0f %.0f %.0f", 
              is_armed ? "ARMED" : "DISARMED", flight_mode,
              aux_data->rx_receiving_signal ? "OK" : "LOST", 
              aux_data->rx_flight_channels_valid ? "OK" : "INVALID",
              aux_data->aux_channels[0], aux_data->aux_channels[1], aux_data->aux_channels[2], aux_data->aux_channels[3]);
      } else if (print_count == 5) {
        LOG_I("arm: %s, mode: %u, rx_signal: %s, rx_valid: %s, aux: %.0f %.0f %.0f %.0f %.0f", 
              is_armed ? "ARMED" : "DISARMED", flight_mode,
              aux_data->rx_receiving_signal ? "OK" : "LOST", 
              aux_data->rx_flight_channels_valid ? "OK" : "INVALID",
              aux_data->aux_channels[0], aux_data->aux_channels[1], aux_data->aux_channels[2], 
              aux_data->aux_channels[3], aux_data->aux_channels[4]);
      } else {
        LOG_I("arm: %s, mode: %u, rx_signal: %s, rx_valid: %s, aux: %.0f %.0f %.0f %.0f %.0f %.0f%s", 
              is_armed ? "ARMED" : "DISARMED", flight_mode,
              aux_data->rx_receiving_signal ? "OK" : "LOST", 
              aux_data->rx_flight_channels_valid ? "OK" : "INVALID",
              aux_data->aux_channels[0], aux_data->aux_channels[1], aux_data->aux_channels[2], 
              aux_data->aux_channels[3], aux_data->aux_channels[4], aux_data->aux_channels[5],
              aux_data->aux_channel_count > 6 ? " ..." : "");
      }
    }
  } else {
    LOG_I("arm: %s, mode: %u, rx_signal: %s, rx_valid: %s", 
          is_armed ? "ARMED" : "DISARMED", flight_mode,
          aux_data->rx_receiving_signal ? "OK" : "LOST", 
          aux_data->rx_flight_channels_valid ? "OK" : "INVALID");
  }
}

