#ifndef RC_SMOOTHING_FILTER_H__
#define RC_SMOOTHING_FILTER_H__

#include <rtthread.h>
#include <cstdint>
#include <cstring>

extern "C" {
#include "rc_mcn.h"  // For rc_command_msg_t
#include "filter.h"  // pt3Filter_t
#include "param.h"
}

// Forward declaration (defined in pid_class.h)
struct pid_setpoint_msg_t;

// Constants
#define PRIMARY_CHANNEL_COUNT 4  // Roll, Pitch, Yaw, Throttle
#define RP_AXIS_COUNT 2          // Roll and Pitch only
#define XYZ_AXIS_COUNT 3         // Roll, Pitch, Yaw
#define FD_ROLL 0
#define FD_PITCH 1
#define FD_YAW 2
#define THROTTLE 3
#define RC_SMOOTHING_CUTOFF_HZ 3200.0f  // 3.2kHz cutoff for smoothing filter

// RC smoothing filter data structure
struct rcSmoothingFilter_t {
  pt3Filter_t filterSetpoint[PRIMARY_CHANNEL_COUNT];  // For setpoint smoothing [ROLL, PITCH, YAW, THROTTLE]
  pt3Filter_t filterRcDeflection[RP_AXIS_COUNT];      // For RC deflection smoothing [ROLL, PITCH] (used in Horizon mode)
  pt3Filter_t filterFeedforward[XYZ_AXIS_COUNT];      // For feedforward smoothing [ROLL, PITCH, YAW]
  float setpointCutoffFrequency;
  float throttleCutoffFrequency;
  float setpointCutoffSetting;
  float throttleCutoffSetting;
  float autoSmoothnessFactorSetpoint;
  float autoSmoothnessFactorThrottle;
  bool enabled;
};

class RcSmoothingFilter {
 public:
  // Singleton pattern
  static RcSmoothingFilter& instance();

  RcSmoothingFilter();
  ~RcSmoothingFilter();

  // Initialize smoothing filters (called from RC thread)
  rt_err_t init(float target_looptime_s, float smoothed_rx_rate_hz);

  // Update filter cutoffs based on RX rate (called from RC thread)
  void updateFilterCutoffs(float target_looptime_s, float smoothed_rx_rate_hz);

  // Process RC smoothing filter: rcCommand[THROTTLE] or rawSetpoint[ROLL/PITCH/YAW] → smoothed output
  // Input: rc_command_msg_t from MCN (to avoid data tearing between threads)
  // Output: pid_setpoint_msg_t (filtered setpoint for PID)
  // Called from PID thread at high frequency (8kHz)
  void processFilter(const rc_command_msg_t* rc_command_msg, pid_setpoint_msg_t* pid_setpoint_out);

  // Get last cached rc_setpoint message (for smoothing filter when no new data)
  const rc_command_msg_t* getLastRcSetpointMsg() const { return &last_rc_setpoint_msg_; }

  // Get smoothed setpoint rate for a specific axis
  float getSetpointRate(int axis) const;

  // Get smoothed throttle command
  float getSmoothedThrottle() const { return smoothed_throttle_; }

  // Get throttle cutoff frequency
  float getThrottleCutoffFrequency() const { return smoothing_data_.throttleCutoffFrequency; }

 private:
  RcSmoothingFilter(const RcSmoothingFilter&) = delete;
  RcSmoothingFilter& operator=(const RcSmoothingFilter&) = delete;

  // Initialize smoothing filters (internal)
  void initFilters(float dt);

  // Helper: constrain float
  float constrainf(float x, float min, float max) const;

  // RC smoothing filter data
  rcSmoothingFilter_t smoothing_data_;

  // Last cached rc_setpoint message (for smoothing filter when no new data)
  rc_command_msg_t last_rc_setpoint_msg_;

  // Smoothed setpoint rates [roll, pitch, yaw]
  float setpoint_rate_[XYZ_AXIS_COUNT];

  // Smoothed throttle command
  float smoothed_throttle_;

  // Smoothed feedforward values [roll, pitch, yaw]
  float feedforward_smoothed_[XYZ_AXIS_COUNT];

  // Initialization flag
  bool initialized_;
};

#endif /* RC_SMOOTHING_FILTER_H__ */

