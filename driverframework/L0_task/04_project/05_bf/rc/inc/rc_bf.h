#ifndef RC_BF_H__
#define RC_BF_H__

#include <rtthread.h>
#include <uMCN.h>
#include <cstdint>
#include <cmath>
#include <cstring>

extern "C" {
#include "pid_setpoint_msg.h"
#include "timestamp.h"
#include "taskRc.h"  // pilot_cmd_bus_t
#include "param.h"
#include "filter.h"  // pt3Filter_t
}

// Constants from rx.h
#define MAX_SUPPORTED_RC_CHANNEL_COUNT 18
#define NON_AUX_CHANNEL_COUNT 4
#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE (PWM_RANGE_MAX - PWM_RANGE_MIN)
#define PWM_RANGE_MIDDLE (PWM_RANGE_MIN + (PWM_RANGE / 2))
#define PWM_PULSE_MIN 750
#define PWM_PULSE_MAX 2250
#define PPM_RCVR_TIMEOUT 0

// Flight dynamics indices
#define FD_ROLL 0
#define FD_PITCH 1
#define FD_YAW 2
#define XYZ_AXIS_COUNT 3
#define PRIMARY_CHANNEL_COUNT 4  // Roll, Pitch, Yaw, Throttle

// Channel indices
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3

// RC smoothing filter data structure
struct rcSmoothingFilter_t {
  pt3Filter_t filterSetpoint[PRIMARY_CHANNEL_COUNT];  // For setpoint smoothing
  float setpointCutoffFrequency;
  float throttleCutoffFrequency;
  float setpointCutoffSetting;
  float throttleCutoffSetting;
  float autoSmoothnessFactorSetpoint;
  float autoSmoothnessFactorThrottle;
  bool enabled;
};

// Rate profile structure
struct rateProfile_t {
  float rcRates[XYZ_AXIS_COUNT];    // RC rates (0-200) [roll, pitch, yaw]
  float rcExpo[XYZ_AXIS_COUNT];     // RC expo (0-100) [roll, pitch, yaw]
  float rates[XYZ_AXIS_COUNT];      // Super rates (0-100) [roll, pitch, yaw]
  float rate_limit[XYZ_AXIS_COUNT]; // Rate limits (deg/s) [roll, pitch, yaw]
};

// RC Controls configuration
struct rcControlsConfig_t {
  float deadband;       // Roll/Pitch deadband
  float yaw_deadband;   // Yaw deadband
  int8_t yaw_control_reversed;  // Yaw control reversed flag
};

// Channel range configuration
struct rxChannelRangeConfig_t {
  uint16_t min;
  uint16_t max;
};

// Failsafe channel configuration
struct rxFailsafeChannelConfig_t {
  uint8_t mode;  // 0=auto, 1=hold, 2=set
  uint8_t step;
};

class RcBf {
 public:
  // Singleton pattern
  static RcBf& instance();

  RcBf();
  ~RcBf();

  rt_err_t init();
  
  // Get setpoint rate for PID (smoothed if enabled)
  float getSetpointRate(int axis) const;
  
  // Get feedforward value
  float getFeedforward(int axis) const;
  
  // Get max RC rate
  float getMaxRcRate(int axis) const;
  
  // Get RC command value
  float getRcCommand(int channel) const;
  
  // Get RC data (for external access)
  const float* getRcData() const { return rc_data_; }
  const float* getRcRaw() const { return rc_raw_; }
  const float* getRcCommandArray() const { return rc_command_; }
  
  // PID task functions (1-4kHz)
  // Process RC command: rcCommand[] → applyRates() → rawSetpoint[]
  void processRcCommand(uint32_t current_time_us);
  
  // Process RC smoothing filter: rcCommand[THROTTLE] or rawSetpoint[ROLL/PITCH/YAW] → smoothed output
  // Output: rcCommand[THROTTLE] (in-place update), setpointRate[ROLL/PITCH/YAW] (new array)
  void processRcSmoothingFilter();

 private:
  RcBf(const RcBf&) = delete;
  RcBf& operator=(const RcBf&) = delete;

  // Thread entry point
  static void rcThreadEntry(void* parameter);
  
  // Initialize RC device
  rt_err_t initRcDevice();
  
  // Read raw RC channels from device
  void readRawRcChannels();
  
  // Apply range scaling: raw value -> rcRaw[]
  void applyRangeScaling();
  
  // Apply failsafe and constraints: rcRaw[] -> rcData[]
  void applyFailsafeAndConstraints(uint32_t current_time_us);
  
  // Convert to control command: rcData[] -> rcCommand[] (RX task, 100-200Hz)
  void updateRcCommands();
  
  // Initialize rate profile from parameters
  void initRateProfile();
  
  // Initialize RC controls config from parameters
  void initRcControlsConfig();
  
  // Initialize channel range configs
  void initChannelRangeConfigs();
  
  // Initialize channel mapping
  void initChannelMapping();
  
  // Parse channel mapping string (e.g., "AETR1234")
  void parseChannelMapping(const char* map_string);
  
  // Initialize failsafe configs
  void initFailsafeConfigs();
  
  // Apply rates calculation (Betaflight style)
  float applyBetaflightRates(int axis, float rcCommandf, float rcCommandfAbs) const;
  
  // Update RC refresh rate
  void updateRcRefreshRate(uint32_t current_time_us, bool rxReceivingSignal);
  
  // Initialize smoothing filters (pt3Filter)
  void initSmoothingFilters();
  
  // Update smoothing filter cutoffs based on RX rate
  void updateSmoothingFilterCutoffs();
  
  // Helper: apply deadband
  float applyDeadband(float value, float deadband) const;
  
  // Helper: constrain float
  float constrainf(float x, float min, float max) const;
  
  // Helper: scale range
  float scaleRangef(float x, float in_min, float in_max, float out_min, float out_max) const;
  
  // Helper: check if pulse is valid
  bool isPulseValid(uint16_t pulseDuration) const;
  
  // Rate profile
  rateProfile_t rate_profile_;
  
  // RC Controls configuration
  rcControlsConfig_t rc_controls_config_;
  
  // RC command dividers for normalization (adjusted by deadband)
  float rcCommandDivider_;      // For roll/pitch: 500.0f - deadband
  float rcCommandYawDivider_;   // For yaw: 500.0f - yaw_deadband
  
  // Channel range configurations
  rxChannelRangeConfig_t channel_range_configs_[NON_AUX_CHANNEL_COUNT];
  
  // Channel mapping: maps logical channel (A/E/T/R/1/2/3/4...) to physical channel index
  // rcmap_[logical_channel] = physical_channel_index
  // For example, if map_string is "AETR1234":
  //   rcmap_[0] = 0 (A -> physical channel 0)
  //   rcmap_[1] = 1 (E -> physical channel 1)
  //   rcmap_[2] = 2 (T -> physical channel 2)
  //   rcmap_[3] = 3 (R -> physical channel 3)
  //   rcmap_[4] = 4 (1 -> physical channel 4)
  //   ...
  uint8_t rcmap_[MAX_SUPPORTED_RC_CHANNEL_COUNT];
  
  // Failsafe configurations
  rxFailsafeChannelConfig_t failsafe_configs_[MAX_SUPPORTED_RC_CHANNEL_COUNT];
  
  // Raw RC channel data from device (PWM values, typically 1000-2000)
  uint16_t rc_raw_channels_[MAX_SUPPORTED_RC_CHANNEL_COUNT];
  
  // Processed RC data after range scaling
  float rc_raw_[MAX_SUPPORTED_RC_CHANNEL_COUNT];
  
  // RC data after failsafe and constraints
  float rc_data_[MAX_SUPPORTED_RC_CHANNEL_COUNT];
  
  // RC command after deadband [roll, pitch, yaw, throttle]
  // Note: rc_command_[THROTTLE] is updated in-place by processRcSmoothingFilter()
  float rc_command_[PRIMARY_CHANNEL_COUNT];
  
  // Raw setpoint from RC input (before smoothing)
  float rawSetpoint_[XYZ_AXIS_COUNT];
  
  // Smoothed setpoint rate (after smoothing filter)
  float setpointRate_[XYZ_AXIS_COUNT];
  
  // RC deflection [-1.0, 1.0]
  float rcDeflection_[XYZ_AXIS_COUNT];
  float rcDeflectionAbs_[XYZ_AXIS_COUNT];
  
  // Max RC rate for each axis
  float maxRcRate_[XYZ_AXIS_COUNT];
  
  // Feedforward data
  float feedforward_[XYZ_AXIS_COUNT];
  float prevSetpoint_[XYZ_AXIS_COUNT];
  
  // RC refresh rate tracking
  uint32_t last_rc_time_us_;
  uint32_t previous_rx_interval_us_;
  uint16_t current_rx_interval_us_;
  float current_rx_rate_hz_;
  float smoothed_rx_rate_hz_;
  bool is_rx_rate_valid_;
  bool rx_receiving_signal_;
  bool rx_flight_channels_valid_;
  
  // RC smoothing filter data
  rcSmoothingFilter_t rc_smoothing_data_;
  
  // Flag to indicate new RC data available for smoothing
  bool is_rc_data_new_;
  
  // Thread handle
  rt_thread_t rc_thread_;
  struct rt_thread rc_thread_obj_;
  rt_uint8_t* rc_thread_stack_;
  bool thread_inited_;
  
  // RC device handle
  rt_device_t rc_device_;
  
  // Channel count
  uint8_t channel_count_;
  
  // Valid signal timeout for each channel
  uint32_t valid_rx_signal_timeout_[MAX_SUPPORTED_RC_CHANNEL_COUNT];
  
  // RC loss count
  uint16_t rc_loss_count_;
  
  // Sequence counter
  uint32_t seq_;
  
  // Target loop time (for smoothing filter calculation)
  float target_looptime_s_;  // in seconds
};

#endif /* RC_BF_H__ */

