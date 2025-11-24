#include "rc_smooth.h"
#include "../pid/inc/pid_class.h"  // For pid_setpoint_msg_t

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#include "filter.h"  // pt3Filter functions
#include "param.h"   // getParam function
#ifdef PROJECT_BF_RC_DEBUG_PIN_EN
#include "debugPin.h"  // DEBUG_PIN macros
#endif
#define LOG_TAG "rc_smoothing"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cmath>
#include <cstring>

// Singleton instance
RcSmoothingFilter& RcSmoothingFilter::instance() {
  static RcSmoothingFilter inst;
  return inst;
}

RcSmoothingFilter::RcSmoothingFilter()
    : smoothed_throttle_(0.0f), initialized_(false) {
  std::memset(&smoothing_data_, 0, sizeof(smoothing_data_));
  std::memset(&last_rc_setpoint_msg_, 0, sizeof(last_rc_setpoint_msg_));
  std::memset(setpoint_rate_, 0, sizeof(setpoint_rate_));
  std::memset(feedforward_smoothed_, 0, sizeof(feedforward_smoothed_));
}

RcSmoothingFilter::~RcSmoothingFilter() {
  // Nothing to do
}

rt_err_t RcSmoothingFilter::init(float target_looptime_s, float smoothed_rx_rate_hz) {
  if (initialized_) {
    LOG_W("RcSmoothingFilter already initialized");
    return RT_EOK;
  }

  float dt = target_looptime_s;

  // Load RC smoothing parameters
  float setpoint_cutoff_setting = 0.0f;
  float throttle_cutoff_setting = 0.0f;
  float auto_factor_rpy = 0.0f;
  float auto_factor_throttle = 0.0f;
  uint8_t smoothing_enabled = 1;

  // Load from parameters if available
  getParam("rc_smoothing_setpoint_cutoff", &setpoint_cutoff_setting, sizeof(setpoint_cutoff_setting));
  getParam("rc_smoothing_throttle_cutoff", &throttle_cutoff_setting, sizeof(throttle_cutoff_setting));
  getParam("rc_smoothing_auto_factor_rpy", &auto_factor_rpy, sizeof(auto_factor_rpy));
  getParam("rc_smoothing_auto_factor_throttle", &auto_factor_throttle, sizeof(auto_factor_throttle));
  getParam("rc_smoothing_enabled", &smoothing_enabled, sizeof(smoothing_enabled));

  // Calculate auto smoothness factors (Betaflight formula)
  // autoSmoothnessFactor = 1.5 / (1.0 + (auto_factor / 10.0))
  float autoSmoothnessFactorSetpoint = 1.5f / (1.0f + (auto_factor_rpy / 10.0f));
  float autoSmoothnessFactorThrottle = 1.5f / (1.0f + (auto_factor_throttle / 10.0f));

  // Initialize smoothing data
  smoothing_data_.setpointCutoffSetting = setpoint_cutoff_setting;
  smoothing_data_.throttleCutoffSetting = throttle_cutoff_setting;
  smoothing_data_.autoSmoothnessFactorSetpoint = autoSmoothnessFactorSetpoint;
  smoothing_data_.autoSmoothnessFactorThrottle = autoSmoothnessFactorThrottle;
  smoothing_data_.enabled = (smoothing_enabled != 0);

  // Initialize cutoff frequencies (will be updated dynamically if setpointCutoffSetting is 0)
  float initial_setpoint_cutoff_hz =
      (setpoint_cutoff_setting > 0.0f) ? setpoint_cutoff_setting : RC_SMOOTHING_CUTOFF_HZ;
  float initial_throttle_cutoff_hz =
      (throttle_cutoff_setting > 0.0f) ? throttle_cutoff_setting : initial_setpoint_cutoff_hz;
  smoothing_data_.setpointCutoffFrequency = initial_setpoint_cutoff_hz;
  smoothing_data_.throttleCutoffFrequency = initial_throttle_cutoff_hz;

  // Initialize pt3Filter for setpoint smoothing
  // filterSetpoint[ROLL/PITCH/YAW] use setpoint cutoff, filterSetpoint[THROTTLE] uses throttle cutoff
  float pt3k_sp = pt3FilterGain(initial_setpoint_cutoff_hz, dt);
  float pt3k_thr = pt3FilterGain(initial_throttle_cutoff_hz, dt);

  for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
    if (i == THROTTLE) {
      pt3FilterInit(&smoothing_data_.filterSetpoint[i], pt3k_thr);
    } else {
      pt3FilterInit(&smoothing_data_.filterSetpoint[i], pt3k_sp);
    }
  }

  // Initialize pt3Filter for RC deflection smoothing (ROLL and PITCH only, used in Horizon mode)
  for (int i = 0; i < RP_AXIS_COUNT; i++) {
    pt3FilterInit(&smoothing_data_.filterRcDeflection[i], pt3k_sp);
  }

  // Initialize pt3Filter for feedforward smoothing (ROLL, PITCH, YAW)
  for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
    pt3FilterInit(&smoothing_data_.filterFeedforward[i], pt3k_sp);
  }

  initialized_ = true;

  LOG_I(
      "RC smoothing initialized: enabled=%d, setpoint_cutoff=%.1f, throttle_cutoff=%.1f, auto_factor_rpy=%.1f, "
      "auto_factor_throttle=%.1f",
      smoothing_data_.enabled, setpoint_cutoff_setting, throttle_cutoff_setting, auto_factor_rpy,
      auto_factor_throttle);

  return RT_EOK;
}

void RcSmoothingFilter::updateFilterCutoffs(float target_looptime_s, float smoothed_rx_rate_hz) {
  if (!smoothing_data_.enabled || !initialized_) {
    return;
  }
  
  const float min_cutoff_hz = 15.0f;
  float dt = target_looptime_s;
  
  // Calculate setpoint cutoff (auto if setting is 0, otherwise use manual value)
  float setpoint_cutoff = smoothing_data_.setpointCutoffSetting == 0
      ? std::max(min_cutoff_hz, smoothed_rx_rate_hz * smoothing_data_.autoSmoothnessFactorSetpoint)
      : smoothing_data_.setpointCutoffSetting;
  
  // Calculate throttle cutoff (auto if setting is 0, otherwise use manual value)
  float throttle_cutoff = smoothing_data_.throttleCutoffSetting == 0
      ? std::max(min_cutoff_hz, smoothed_rx_rate_hz * smoothing_data_.autoSmoothnessFactorThrottle)
      : smoothing_data_.throttleCutoffSetting;
  
  // Update filter cutoffs
  float pt3k_sp = pt3FilterGain(setpoint_cutoff, dt);
  float pt3k_thr = pt3FilterGain(throttle_cutoff, dt);

  // Update setpoint filters for ROLL, PITCH, YAW (use setpoint cutoff)
  for (int i = FD_ROLL; i <= FD_YAW; i++) {
    pt3FilterUpdateCutoff(&smoothing_data_.filterSetpoint[i], pt3k_sp);
  }

  // Update throttle filter (use throttle cutoff)
  pt3FilterUpdateCutoff(&smoothing_data_.filterSetpoint[THROTTLE], pt3k_thr);

  // Update feedforward filters for ROLL, PITCH, YAW (use setpoint cutoff)
  for (int i = FD_ROLL; i <= FD_YAW; i++) {
    pt3FilterUpdateCutoff(&smoothing_data_.filterFeedforward[i], pt3k_sp);
  }

  // Update RC deflection filters for ROLL and PITCH (use setpoint cutoff)
  for (int i = 0; i < RP_AXIS_COUNT; i++) {
    pt3FilterUpdateCutoff(&smoothing_data_.filterRcDeflection[i], pt3k_sp);
  }

  smoothing_data_.setpointCutoffFrequency = setpoint_cutoff;
  smoothing_data_.throttleCutoffFrequency = throttle_cutoff;
}

void RcSmoothingFilter::processFilter(const rc_command_msg_t* rc_command_msg, pid_setpoint_msg_t* pid_setpoint_out) {
  if (!initialized_ || pid_setpoint_out == nullptr) {
    return;
  }
#ifdef PROJECT_BF_RC_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG1_LOW();  // Debug pin: RC task execution end
#endif
  // Use cached data if rc_command_msg is null (for PID thread frequency update)
  const rc_command_msg_t* msg_to_use = rc_command_msg;
  if (msg_to_use == nullptr) {
    msg_to_use = &last_rc_setpoint_msg_;
    // If no cached data yet, skip processing
    if (msg_to_use->seq == 0 && msg_to_use->timestamp == 0) {
      return;
    }
  } else {
    // Cache the new data
    std::memcpy(&last_rc_setpoint_msg_, rc_command_msg, sizeof(rc_command_msg_t));
  }

  // Prepare output data
  uint32_t output_timestamp = msg_to_use->timestamp;
  uint32_t output_seq = msg_to_use->seq;

  if (!smoothing_data_.enabled) {
    // If smoothing is disabled, just copy rawSetpoint to setpointRate
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      setpoint_rate_[axis] = msg_to_use->rawSetpoint[axis];
      feedforward_smoothed_[axis] = msg_to_use->feedforward[axis];
    }
    smoothed_throttle_ = msg_to_use->rcCommandThrottle;
  } else {
    // Prepare data to smooth (from MCN data to avoid data tearing)
    float rx_data_to_smooth[PRIMARY_CHANNEL_COUNT];
    for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
      if (i == THROTTLE) {
        rx_data_to_smooth[i] = msg_to_use->rcCommandThrottle;
      } else {
        rx_data_to_smooth[i] = msg_to_use->rawSetpoint[i];
      }
    }

    // Apply pt3Filter smoothing for setpoint (ROLL, PITCH, YAW, THROTTLE)
    for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
      float smoothed = pt3FilterApply(&smoothing_data_.filterSetpoint[i], rx_data_to_smooth[i]);
      
      if (i == THROTTLE) {
        smoothed_throttle_ = smoothed;
      } else {
        // Output: setpointRate[ROLL/PITCH/YAW]
        setpoint_rate_[i] = smoothed;
      }
    }

    // Apply pt3Filter smoothing for feedforward (ROLL, PITCH, YAW)
    // Same as Betaflight: feedforwardSmoothed[axis] = pt3FilterApply(&rcSmoothingData.filterFeedforward[axis],
    // feedforwardRaw[axis]);
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      feedforward_smoothed_[axis] =
          pt3FilterApply(&smoothing_data_.filterFeedforward[axis], msg_to_use->feedforward[axis]);
    }
  }

  // Write filtered setpoint directly to PID singleton
  // Copy filtered setpoint rates [roll, pitch, yaw]
  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    pid_setpoint_out->rate[axis] = setpoint_rate_[axis];
  }
  // Copy smoothed feedforward values (same as Betaflight: use feedforwardSmoothed)
  std::memcpy(pid_setpoint_out->feedforward, feedforward_smoothed_, sizeof(feedforward_smoothed_));
  // Copy smoothed throttle value
  pid_setpoint_out->smoothed_throttle = smoothed_throttle_;
  // Copy timestamp and sequence
  pid_setpoint_out->timestamp = output_timestamp;
  pid_setpoint_out->seq = output_seq;
#ifdef PROJECT_BF_RC_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG0_LOW();  // Debug pin: RC task execution end
#endif
}

float RcSmoothingFilter::getSetpointRate(int axis) const {
  if (axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return setpoint_rate_[axis];
  }
  return 0.0f;
}

float RcSmoothingFilter::constrainf(float x, float min, float max) const {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

