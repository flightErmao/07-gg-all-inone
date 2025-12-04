#ifndef RC_BF_HPP__
#define RC_BF_HPP__

#include <rtthread.h>
#include <uMCN.h>
#include <cstdint>
#include <cmath>
#include <cstring>

extern "C" {
#include "rc_mcn.h"
#include "timestamp.h"
#include "taskRc.h"  // pilot_cmd_bus_t
#include "param.h"
}

#include "rc_mlog.h"

// Forward declarations (rc_command_msg_t and rc_aux_msg_t are typedefs defined in rc_mcn.h)

// Simple lowpass filter structure (shared with pid_class.h)
#ifndef SIMPLE_LOWPASS_DEFINED
#define SIMPLE_LOWPASS_DEFINED
struct SimpleLowpass {
  float state;
  float alpha;
  bool enabled;
};
#endif

#define FD_ROLL 0
#define FD_PITCH 1
#define FD_YAW 2
#define XYZ_AXIS_COUNT 3
#define PRIMARY_CHANNEL_COUNT 4  // Roll, Pitch, Yaw, Throttle
#define ROLL 0
#define PITCH 1
#define YAW 2
#define NON_AUX_CHANNEL_COUNT 4  // Roll, Pitch, Yaw, Throttle

// Channel range configuration
struct rxChannelRangeConfig_t {
  uint16_t min;
  uint16_t max;
};

// Channel mapping array type
typedef uint8_t rcmap_t[16];

// Failsafe configuration
struct rxFailsafeConfig_t {
  uint8_t mode;  // 0=auto, 1=hold
  uint8_t step;  // Step value for failsafe
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

class RcBf {
 public:
  // Singleton pattern
  static RcBf& instance();

  RcBf();
  ~RcBf();

  rt_err_t init();
  void updateRcCommand(uint32_t current_time_us);

  // MCN related functions
  rt_err_t initMcn();
  void publishRcCommandToMcn(uint32_t current_time_us);
  void publishAuxChannelsToMcn(uint32_t current_time_us);
  void echoSetpoint(const rc_command_msg_t* setpoint_data);
  void echoAux(const rc_aux_msg_t* aux_data);

  // Mlog related functions
  rt_err_t initMlog();
  void pushRcDataToMlog(const bf_mlog::rc_mlog_data_t* data);

  // Get RC data and command arrays (for echo functions)
  const float* getRcData() const { return rc_data_; }
  const float* getRcCommandArray() const { return rc_command_; }

  // Get max RC rate for a specific axis (deg/s)
  // This is the maximum angle rate that can be achieved with full stick deflection
  float getMaxRcRate(int axis) const;

  // Print debug information
  void printDebugInfo(const rc_command_msg_t* setpoint_msg) const;

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
  
  // Initialize RC device
  rt_err_t initRcDevice();
  
  // Initialize channel range configurations
  void initChannelRangeConfigs();
  
  // Initialize channel mapping
  void initChannelMapping();
  
  // Parse channel mapping string
  void parseChannelMapping(const char* map_string);
  
  // Initialize failsafe configurations
  void initFailsafeConfigs();
  
  // Apply range scaling
  void applyRangeScaling();
  
  // Apply failsafe and constraints
  void applyFailsafeAndConstraints(uint32_t current_time_us);
  
  // Apply rates calculation (Betaflight style)
  float applyBetaflightRates(int axis, float rcCommandf, float rcCommandfAbs) const;
  
  // Helper: constrain float
  float constrainf(float x, float min, float max) const;
  
  // Helper: scale range
  float scaleRangef(float x, float in_min, float in_max, float out_min, float out_max) const;
  
  // Helper: check if pulse is valid
  bool isPulseValid(uint16_t pulseDuration) const;
  
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
  McnHub_t rc_command_hub_;
  McnHub_t rc_aux_hub_;
  rt_sem_t rc_setpoint_event_;

  // Sequence counter
  uint32_t seq_;

  // Target loop time (for smoothing filter calculation)
  float target_looptime_s_;  // in seconds
  
  // RC data arrays
  float rc_data_[16];  // Processed RC channel data
  uint8_t channel_count_;  // Number of RC channels
  bool is_rc_data_new_;  // Flag indicating new RC data available
  
  // RC device handle
  rt_device_t rc_device_;
  
  // Command dividers for normalization
  float rcCommandDivider_;
  float rcCommandYawDivider_;
  
  // Channel range configurations
  rxChannelRangeConfig_t channel_range_configs_[16];
  
  // Channel mapping (logical to physical)
  rcmap_t rcmap_;
  
  // Failsafe configurations
  rxFailsafeConfig_t failsafe_configs_[16];
  
  // RC loss count
  uint32_t rc_loss_count_;
  
  // Valid RX signal timeout (per channel)
  uint32_t valid_rx_signal_timeout_[16];
  
  // Raw RC data (before processing)
  uint16_t rc_raw_[16];
};

#endif /* RC_BF_HPP__ */
