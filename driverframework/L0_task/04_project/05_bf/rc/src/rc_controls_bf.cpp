#include "rc_controls_bf.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#include "param.h"  // getParam function
#define LOG_TAG "rc_controls"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cstring>
#include <cmath>

// Singleton instance
RcControls& RcControls::instance() {
  static RcControls inst;
  return inst;
}

RcControls::RcControls()
    : armed_(false),
      initialized_(false),
      mincheck_(1050),
      maxcheck_(1900),
      midrc_(1500),
      rx_min_usec_(1000),
      rx_max_usec_(2000),
      arm_delay_ms_(500),
      stick_delay_ms_(50),
      stick_autorepeat_ms_(250),
      use_stick_arming_(true),
      auto_disarm_delay_(5),
      gyro_cal_on_first_arm_(0),
      arm_aux_channel_(5),
      arm_aux_threshold_low_(1400),
      arm_aux_threshold_high_(1600),
      mode_aux_channel_(6),
      mode_aux_threshold_low_(1400),
      mode_aux_threshold_high_(1600),
      aux_channel_count_(0),
      flight_mode_(0),
      rc_sticks_(0),
      rc_delay_ms_(0),
      do_not_repeat_(false),
      rc_disarm_ticks_(0),
      aux_arm_channel_high_(false),
      aux_arm_throttle_ready_(false),
      aux_arm_error_logged_(false) {
  std::memset(aux_channels_, 0, sizeof(aux_channels_));
}

RcControls::~RcControls() {
  // Nothing to do
}

int RcControls::init() {
  if (initialized_) {
    LOG_W("RcControls already initialized");
    return 0;
  }

  // Load parameters
  loadParameters();

  // Initialize to disarmed state
  armed_ = false;
  flight_mode_ = 0;

  initialized_ = true;
  LOG_I("RcControls initialized");
  return 0;
}

void RcControls::loadParameters() {
  // Load thresholds
  getParam("rc_mincheck", &mincheck_, sizeof(mincheck_));
  getParam("rc_maxcheck", &maxcheck_, sizeof(maxcheck_));
  getParam("rc_midrc", &midrc_, sizeof(midrc_));
  getParam("rc_rx_min_usec", &rx_min_usec_, sizeof(rx_min_usec_));
  getParam("rc_rx_max_usec", &rx_max_usec_, sizeof(rx_max_usec_));

  // Load arming parameters
  getParam("rc_arm_delay_ms", &arm_delay_ms_, sizeof(arm_delay_ms_));
  getParam("rc_stick_delay_ms", &stick_delay_ms_, sizeof(stick_delay_ms_));
  getParam("rc_stick_autorepeat_ms", &stick_autorepeat_ms_, sizeof(stick_autorepeat_ms_));
  uint8_t use_stick_arming = 1;
  getParam("rc_use_stick_arming", &use_stick_arming, sizeof(use_stick_arming));
  use_stick_arming_ = (use_stick_arming != 0);
  getParam("rc_auto_disarm_delay", &auto_disarm_delay_, sizeof(auto_disarm_delay_));
  getParam("rc_gyro_cal_on_first_arm", &gyro_cal_on_first_arm_, sizeof(gyro_cal_on_first_arm_));

  // Load AUX channel configuration for arming
  getParam("rc_arm_aux_channel", &arm_aux_channel_, sizeof(arm_aux_channel_));
  getParam("rc_arm_aux_threshold_low", &arm_aux_threshold_low_, sizeof(arm_aux_threshold_low_));
  getParam("rc_arm_aux_threshold_high", &arm_aux_threshold_high_,
           sizeof(arm_aux_threshold_high_));

  // Load AUX channel configuration for flight mode
  getParam("rc_mode_aux_channel", &mode_aux_channel_, sizeof(mode_aux_channel_));
  getParam("rc_mode_aux_threshold_low", &mode_aux_threshold_low_,
           sizeof(mode_aux_threshold_low_));
  getParam("rc_mode_aux_threshold_high", &mode_aux_threshold_high_,
           sizeof(mode_aux_threshold_high_));

  LOG_I("RcControls parameters loaded:");
  LOG_I("  mincheck=%u, maxcheck=%u, midrc=%u", mincheck_, maxcheck_, midrc_);
  LOG_I("  arm_delay_ms=%u, stick_delay_ms=%u", arm_delay_ms_, stick_delay_ms_);
  LOG_I("  use_stick_arming=%d", use_stick_arming_ ? 1 : 0);
  LOG_I("  arm_aux_channel=%u, threshold=[%u, %u]", arm_aux_channel_,
        arm_aux_threshold_low_, arm_aux_threshold_high_);
  LOG_I("  mode_aux_channel=%u, threshold=[%u, %u]", mode_aux_channel_,
        mode_aux_threshold_low_, mode_aux_threshold_high_);
}

void RcControls::updateAuxChannels(const float* rc_data, uint8_t channel_count) {
  // Update AUX channel values (channels 5-18, indices 4-17)
  aux_channel_count_ = 0;
  if (channel_count > 4) {
    uint8_t aux_count = channel_count - 4;
    if (aux_count > 14) {
      aux_count = 14;  // Max 14 AUX channels
    }
    for (uint8_t i = 0; i < aux_count; i++) {
      aux_channels_[i] = rc_data[4 + i];  // Channels 5-18 (indices 4-17)
    }
    aux_channel_count_ = aux_count;
  }
}

void RcControls::processRcStickPositions(const float* rc_data, uint32_t current_time_us) {
  if (!initialized_ || rc_data == nullptr) {
    return;
  }

  // Update AUX channels first
  updateAuxChannels(rc_data, 18);  // Assume max 18 channels

  // Check stick positions (for stick commands)
  uint8_t stTmp = 0;
  for (int i = 0; i < 4; i++) {
    stTmp >>= 2;
    if (rc_data[i] > mincheck_) {
      stTmp |= 0x80;  // check for MIN
    }
    if (rc_data[i] < maxcheck_) {
      stTmp |= 0x40;  // check for MAX
    }
  }

  // Update delay if sticks are held in same position
  if (stTmp == rc_sticks_) {
    // Get delta time (simplified - should use actual delta time)
    static uint32_t last_time_us = 0;
    uint32_t delta_us = current_time_us - last_time_us;
    if (delta_us > 0 && delta_us < 100000) {  // Sanity check
      rc_delay_ms_ += delta_us / 1000;
      if (rc_delay_ms_ < 0) {
        rc_delay_ms_ = 0;  // Prevent overflow
      }
    }
    last_time_us = current_time_us;
  } else {
    rc_delay_ms_ = 0;
    do_not_repeat_ = false;
  }
  rc_sticks_ = stTmp;

  // Process arming/disarming
  if (use_stick_arming_) {
    processStickArming(rc_data, current_time_us);
  } else {
    processAuxArming(rc_data, current_time_us);
  }

  // Process flight mode switching
  processFlightMode(rc_data);

  // TODO: Add more stick commands (calibration, PID profile switching, etc.)
}

void RcControls::processStickArming(const float* rc_data, uint32_t current_time_us) {
  // Disarm: throttle down + yaw left
  if (rc_sticks_ == (THR_LO + YAW_LO + PIT_CE + ROL_CE)) {
    if (rc_delay_ms_ >= arm_delay_ms_ && !do_not_repeat_) {
      do_not_repeat_ = true;
      if (armed_) {
        LOG_I("Disarming via sticks");
        armed_ = false;
      } else {
        // TODO: Play beeper tone
        // TODO: Reset arming disabled flags
      }
    }
    return;
  }

  // Arm: throttle down + yaw right
  if (rc_sticks_ == (THR_LO + YAW_HI + PIT_CE + ROL_CE)) {
    if (rc_delay_ms_ >= arm_delay_ms_ && !do_not_repeat_) {
      do_not_repeat_ = true;
      if (!armed_) {
        LOG_I("Arming via sticks");
        // TODO: Check arming conditions (gyro ready, etc.)
        armed_ = true;
      } else {
        // TODO: Reset arming disabled flags
      }
    }
    return;
  }

  // Reset trying to arm if sticks are not in arming position
  // TODO: Reset arming state
}

void RcControls::processAuxArming(const float* rc_data, uint32_t current_time_us) {
  if (arm_aux_channel_ == 0) {
    return;  // AUX arming disabled
  }

  // Convert channel number to index (channel 5 = index 4, etc.)
  uint8_t aux_index = arm_aux_channel_ - 1;
  if (aux_index < 4 || aux_index >= 18) {
    return;  // Invalid channel
  }
  aux_index -= 4;  // Convert to AUX index (0-13)

  if (aux_index >= aux_channel_count_) {
    return;  // Channel not available
  }

  // Get AUX channel value
  float aux_value = aux_channels_[aux_index];

  // Check throttle status (throttle is at index 3)
  bool throttle_low = (rc_data != nullptr && rc_data[3] < mincheck_);

  // Arm/Disarm based on channel value relative to midrc
  // Arm sequence: throttle must be low first, then channel value > midrc
  // If channel value > midrc: ARM
  // If channel value <= midrc: DISARM
  if (aux_value > midrc_) {
    // Channel value is > midrc
    aux_arm_channel_high_ = true;
    rc_disarm_ticks_ = 0;
    
    if (!armed_) {
      // Check if throttle was ready (low) before channel value became > midrc
      if (aux_arm_throttle_ready_ && throttle_low) {
        // Both conditions met: throttle was low first, then channel value > midrc
        LOG_I("Arming via AUX channel %u (value=%.0f > midrc=%u, throttle at minimum)",
              arm_aux_channel_, aux_value, midrc_);
        // TODO: Check arming conditions
        armed_ = true;
        aux_arm_throttle_ready_ = false;  // Reset after arming
        aux_arm_error_logged_ = false;    // Reset error flag after successful arming
      } else {
        // Channel value > midrc but throttle not ready or not low
        if (!aux_arm_error_logged_) {
          if (!aux_arm_throttle_ready_) {
            // Throttle was not low when channel value became > midrc
            LOG_E("Arming failed: AUX channel %u > midrc but throttle was not at minimum first (throttle=%.0f, mincheck=%u). "
                  "Please lower throttle first, then move channel back to <= midrc, then move channel to > midrc.",
                  arm_aux_channel_, rc_data != nullptr ? rc_data[3] : 0.0f, mincheck_);
            aux_arm_error_logged_ = true;
          } else if (!throttle_low) {
            // Throttle was ready but now not low
            LOG_E("Arming failed: AUX channel %u > midrc but throttle not at minimum (throttle=%.0f, mincheck=%u). "
                  "Please move channel back to <= midrc, then move channel to > midrc again.",
                  arm_aux_channel_, rc_data != nullptr ? rc_data[3] : 0.0f, mincheck_);
            aux_arm_error_logged_ = true;
          }
        }
      }
    } else {
      // Already armed, reset flags
      aux_arm_throttle_ready_ = false;
    }
  } else {
    // Channel value <= midrc
    aux_arm_channel_high_ = false;
    aux_arm_error_logged_ = false;  // Reset error flag when channel goes back to <= midrc
    
    // Update throttle ready state: throttle must be low when channel <= midrc
    if (throttle_low) {
      // Throttle is low and channel <= midrc, ready for next arm attempt
      aux_arm_throttle_ready_ = true;
    } else {
      // Throttle not low, not ready for arming
      aux_arm_throttle_ready_ = false;
    }
    
    // TODO: Reset trying to arm
    if (armed_) {
      rc_disarm_ticks_++;
      if (rc_disarm_ticks_ > 3) {
        // Require three duplicate disarm values in a row before we disarm
        LOG_I("Disarming via AUX channel %u (value=%.0f <= midrc=%u)", arm_aux_channel_, aux_value, midrc_);
        armed_ = false;
      }
    }
  }
}

void RcControls::processFlightMode(const float* rc_data) {
  if (mode_aux_channel_ == 0) {
    return;  // Flight mode switching disabled
  }

  // Convert channel number to index
  uint8_t aux_index = mode_aux_channel_ - 1;
  if (aux_index < 4 || aux_index >= 18) {
    return;  // Invalid channel
  }
  aux_index -= 4;  // Convert to AUX index

  if (aux_index >= aux_channel_count_) {
    return;  // Channel not available
  }

  float aux_value = aux_channels_[aux_index];

  // Determine flight mode based on AUX channel value
  // Mode 0: 角速度模式 (Rate/Acro mode) - low position (<= threshold_low)
  // Mode 1: 角度模式 (Angle mode) - middle position (between thresholds)
  // Mode 2: 高度模式 (Altitude/Horizon mode) - high position (>= threshold_high)
  if (aux_value >= mode_aux_threshold_high_) {
    flight_mode_ = 2;  // High position = mode 2 (高度模式)
  } else if (aux_value <= mode_aux_threshold_low_) {
    flight_mode_ = 0;  // Low position = mode 0 (角速度模式)
  } else {
    flight_mode_ = 1;  // Middle position = mode 1 (角度模式)
  }
}

bool RcControls::isAuxChannelActive(uint8_t channel, uint16_t threshold_low,
                                     uint16_t threshold_high) const {
  if (channel >= aux_channel_count_) {
    return false;
  }

  float aux_value = aux_channels_[channel];
  return (aux_value >= threshold_low && aux_value <= threshold_high);
}

RcControls::ThrottleStatus RcControls::calculateThrottleStatus(
    const float* rc_data) const {
  if (rc_data[3] < mincheck_) {  // THROTTLE is index 3
    return ThrottleStatus::LOW;
  }
  return ThrottleStatus::HIGH;
}

