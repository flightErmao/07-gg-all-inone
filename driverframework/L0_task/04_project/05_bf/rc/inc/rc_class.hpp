#ifndef RC_BF_HPP__
#define RC_BF_HPP__

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
}

// Forward declarations
struct rc_command_msg_t;
struct rc_aux_msg_t;

// Simple lowpass filter structure (same as in pid_bf.hpp)
struct SimpleLowpass {
  float state;
  float alpha;
  bool enabled;
};

#define FD_ROLL 0
#define FD_PITCH 1
#define FD_YAW 2
#define XYZ_AXIS_COUNT 3
#define PRIMARY_CHANNEL_COUNT 4  // Roll, Pitch, Yaw, Throttle

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

class RcBf {
 public:
  // Singleton pattern
  static RcBf& instance();

  RcBf();
  ~RcBf();

  rt_err_t init();
  void updateRcCommand(uint32_t current_time_us);
  
  // Get setpoint rate for PID (smoothed if enabled)
  float getSetpointRate(int axis) const;
  
  // Get feedforward value
  float getFeedforward(int axis) const;
  
  // Get max RC rate
  float getMaxRcRate(int axis) const;

  // MCN related functions
  rt_err_t initMcn();
  void publishSetpointToMcn(uint32_t current_time_us);
  void publishAuxChannelsToMcn(uint32_t current_time_us);
  void echoSetpoint(const rc_command_msg_t* setpoint_data);
  void echoAux(const rc_aux_msg_t* aux_data);

  // Get RC data and command arrays (for echo functions)
  const float* getRcData() const { return rc_data_; }
  const float* getRcCommandArray() const { return rc_command_; }

 private:
  RcBf(const RcBf&) = delete;
  RcBf& operator=(const RcBf&) = delete;

  // Thread entry point
  static void rcThreadEntry(void* parameter);
  
  // Read raw RC channels from device
  void readRawRcChannels();
  
  // Update RC commands (apply deadband)
  void updateRcCommands();
  
  // Process RC command (calculate setpoint rates)
  void processRcCommand(uint32_t current_time_us);
  
  // Initialize rate profile from parameters
  void initRateProfile();
  
  // Initialize RC controls config from parameters
  void initRcControlsConfig();
  
  // Apply rates calculation (Betaflight style)
  float applyBetaflightRates(int axis, float rcCommandf, float rcCommandfAbs) const;
  
  // Update RC refresh rate
  void updateRcRefreshRate(uint32_t current_time_us, bool rxReceivingSignal);
  
  // Process RC smoothing filter (3.2k Hz)
  void processRcSmoothingFilter();
  
  // Initialize smoothing filters
  void initSmoothingFilters();
  
  // Helper: lowpass filter initialization
  void lowpassInit(SimpleLowpass* filter, float cutoff_hz, float dt);
  
  // Helper: lowpass filter application
  float lowpassApply(SimpleLowpass* filter, float input);
  
  // Helper: apply deadband
  float applyDeadband(float value, float deadband) const;
  
  // Rate profile
  rateProfile_t rate_profile_;
  
  // RC Controls configuration
  rcControlsConfig_t rc_controls_config_;
  
  // Raw RC channel data (PWM values, typically 1000-2000)
  uint16_t rc_raw_channels_[16];
  bool rc_data_new_;
  bool rx_receiving_signal_;
  
  // RC command after deadband [roll, pitch, yaw, throttle] (normalized to -500..500 or 1000..2000)
  float rc_command_[PRIMARY_CHANNEL_COUNT];
  
  // Raw setpoint from RC input (before smoothing)
  float rawSetpoint_[XYZ_AXIS_COUNT];
  
  // Smoothed setpoint rate (after 3.2k filter)
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
  
  // RC smoothing filters (3.2k Hz cutoff)
  SimpleLowpass smoothing_filter_[PRIMARY_CHANNEL_COUNT];  // For setpoint smoothing
  
  // Thread handle
  rt_thread_t rc_thread_;
  struct rt_thread rc_thread_obj_;
  static constexpr size_t RC_THREAD_STACK_SIZE = 2048;
  rt_uint8_t rc_thread_stack_[RC_THREAD_STACK_SIZE];
  bool thread_inited_;
  
  // MCN subscription and publication
  rt_sem_t rc_event_;
  McnNode_t rc_node_;
  McnHub_t pid_setpoint_hub_;
  
  // MCN hubs and nodes for RC setpoint and aux channels
  McnHub_t rc_setpoint_hub_;
  McnHub_t rc_aux_hub_;
  rt_sem_t rc_setpoint_event_;
  McnNode_t rc_setpoint_node_;

  // Sequence counter
  uint32_t seq_;

  // Target loop time (for smoothing filter calculation)
  float target_looptime_s_;  // in seconds
  
  // RC data arrays
  float rc_data_[16];  // Processed RC channel data
  uint8_t channel_count_;  // Number of RC channels
  bool rx_flight_channels_valid_;  // Flight channels valid status
  bool is_rc_data_new_;  // Flag indicating new RC data available
};

#endif /* RC_BF_HPP__ */
