#include "rc_class.hpp"
#include "rc_smooth.h"
#include "rc_aux.h"

#ifdef PROJECT_BF_PID_EN
#include "../pid/inc/pid_class.h"
#endif

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#include <rtdevice.h>
#include "timestamp.h"
#include "filter.h"  // pt3Filter functions
#include "param.h"   // getParam function
#include "../common/inc/init_sync.h"  // For initSyncNotify
#ifdef TASK_TOOL_02_SD_MLOG
#include "taskMlog.h"  // task_mlog_start_logging, task_mlog_stop_logging
#endif
#ifdef PROJECT_BF_RC_DEBUG_PIN_EN
#include "debugPin.h"
#endif
#define LOG_TAG "rc"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cmath>
#include <cstring>

// Constants
#define MAX_INVALID_PULSE_TIME_MS 300
#define RXLOSS_TRIGGER_INTERVAL_US (150 * 1000)  // 150ms
#define RX_INTERVAL_MIN_US 800
#define RX_INTERVAL_MAX_US 65500
#define RC_RATE_INCREMENTAL 14.54f
#define RC_SMOOTHING_CUTOFF_HZ 3200.0f  // 3.2kHz cutoff for smoothing filter
#define PPM_RCVR_TIMEOUT 0  // PPM receiver timeout value (invalid pulse)

// Helper macros
#define RXFAIL_STEP_TO_CHANNEL_VALUE(step) (PWM_PULSE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_RXFAIL_STEP(channelValue) \
  ((constrainf(channelValue, PWM_PULSE_MIN, PWM_PULSE_MAX) - PWM_PULSE_MIN) / 25)

// Helper function: constrain float value
static inline float constrainf(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

// Constants
#define MAX_SUPPORTED_RC_CHANNEL_COUNT 16
#define ROLL 0
#define PITCH 1
#define YAW 2

// Singleton instance
RcBf& RcBf::instance() {
  static RcBf instance_obj;
  return instance_obj;
}

// Constructor
RcBf::RcBf()
    : rate_profile_(),
      rc_controls_config_(),
      rc_data_new_(false),
      last_rc_time_us_(0),
      previous_rx_interval_us_(0),
      current_rx_interval_us_(0),
      current_rx_rate_hz_(100.0f),
      smoothed_rx_rate_hz_(100.0f),
      is_rx_rate_valid_(false),
      rc_thread_(RT_NULL),
      thread_inited_(false),
      rc_command_hub_(nullptr),
      rc_setpoint_event_(RT_NULL),
      seq_(0),
      target_looptime_s_(0.0003125f),  // 3.2kHz default
      channel_count_(16),              // Default to 16 channels
      is_rc_data_new_(false),
      rc_device_(RT_NULL),
      rcCommandDivider_(500.0f),
      rcCommandYawDivider_(500.0f),
      rc_loss_count_(0),
      rc_arm_control_(0),
      prev_armed_status_(false),
      prev_rc_command_throttle_(PWM_RANGE_MIN) {  // Initialize to minimum throttle
  std::memset(rc_raw_channels_, 0, sizeof(rc_raw_channels_));
  std::memset(rc_data_, 0, sizeof(rc_data_));
  std::memset(rc_command_, 0, sizeof(rc_command_));
  std::memset(rawSetpoint_, 0, sizeof(rawSetpoint_));
  std::memset(rcDeflection_, 0, sizeof(rcDeflection_));
  std::memset(rcDeflectionAbs_, 0, sizeof(rcDeflectionAbs_));
  std::memset(maxRcRate_, 0, sizeof(maxRcRate_));
  std::memset(feedforward_, 0, sizeof(feedforward_));
  std::memset(prevSetpoint_, 0, sizeof(prevSetpoint_));
  std::memset(&rc_thread_obj_, 0, sizeof(rc_thread_obj_));
  std::memset(rc_thread_stack_, 0, sizeof(rc_thread_stack_));
  std::memset(channel_range_configs_, 0, sizeof(channel_range_configs_));
  std::memset(rcmap_, 0, sizeof(rcmap_));
  std::memset(failsafe_configs_, 0, sizeof(failsafe_configs_));
  std::memset(valid_rx_signal_timeout_, 0, sizeof(valid_rx_signal_timeout_));
  std::memset(rc_raw_, 0, sizeof(rc_raw_));

  // Initialize default values
  for (int i = 0; i < 16; i++) {
    rc_data_[i] = PWM_RANGE_MIDDLE;  // Default to middle
    valid_rx_signal_timeout_[i] = MAX_INVALID_PULSE_TIME_MS;
  }
  rc_data_[THROTTLE] = PWM_RANGE_MIN;  // Throttle starts at minimum
}

RcBf::~RcBf() {
  if (thread_inited_ && rc_thread_ != RT_NULL) {
    rt_thread_delete(rc_thread_);
  }
  // rc_thread_stack_ is a fixed-size array, no need to delete
}

rt_err_t RcBf::init() {
  // Initialize configurations
  initChannelRangeConfigs();
  initChannelMapping();
  initFailsafeConfigs();
  initRateProfile();
  initRcControlsConfig();

  initRcDevice();

  // Initialize RC smoothing filter singleton
  // Note: This is called from RC thread initialization
  RcSmoothingFilter& smoothing_filter = RcSmoothingFilter::instance();
  rt_err_t ret = smoothing_filter.init(target_looptime_s_, smoothed_rx_rate_hz_);
  if (ret != RT_EOK) {
    LOG_E("RC smoothing filter init failed");
    return ret;
  }

  // Pass smoothing filter pointer to PidBf singleton
#ifdef PROJECT_BF_PID_EN
  PidBf& pid = PidBf::instance();
  pid.setRcSmoothingFilter(&smoothing_filter);
  LOG_I("RC smoothing filter pointer passed to PidBf");
#endif

  // Initialize RC controls singleton
  RcControls& rc_controls = RcControls::instance();
  ret = rc_controls.init();
  if (ret != RT_EOK) {
    LOG_E("RC controls init failed");
    return ret;
  }
  LOG_I("RC controls initialized");

  // Initialize MCN
  ret = initMcn();
  if (ret != RT_EOK) {
    LOG_E("MCN init failed");
    return ret;
  }

  // Initialize mlog
  ret = initMlog();
  if (ret != RT_EOK) {
    LOG_E("Mlog init failed");
    return ret;
  }

  // Read mlog_rc_arm_control parameter
  getParam("mlog_rc_arm_control", &rc_arm_control_, sizeof(rc_arm_control_));
  LOG_I("mlog_rc_arm_control: %u", rc_arm_control_);

  // Initialize thread (rc_thread_stack_ is a fixed-size array, not dynamically allocated)
  size_t stack_size = RC_THREAD_STACK_SIZE;
  
  // Initialize thread
  ret = rt_thread_init(&rc_thread_obj_, "rc", rcThreadEntry, this, rc_thread_stack_, stack_size,
                       PROJECT_BF_RC_THREAD_PRIORITY, PROJECT_BF_RC_THREAD_TIMESLICE);

  if (ret != RT_EOK) {
    LOG_E("RC thread init failed: %d", ret);
    return ret;
  }
  
  rc_thread_ = &rc_thread_obj_;
  thread_inited_ = true;
  
  // Start thread
  ret = rt_thread_startup(rc_thread_);
  if (ret != RT_EOK) {
    LOG_E("RC thread startup failed: %d", ret);
    thread_inited_ = false;
    return ret;
  }
  
  // 通知 RC 模块初始化完成
  rt_err_t notify_ret = initSyncNotify(INIT_SYNC_RC);
  if (notify_ret != RT_EOK) {
    LOG_E("RC initSyncNotify failed: %d", notify_ret);
  } else {
    LOG_I("RC initialization completed and notified");
  }
  
  LOG_I("RcBf initialized successfully");
  return RT_EOK;
}

rt_err_t RcBf::initRcDevice() {
  char rc_name[RT_NAME_MAX];
  
  // Get device name from config
#ifndef PROJECT_BF_RC_DEVICE_NAME
  const char* default_name = "sbus";
#else
  const char* default_name = PROJECT_BF_RC_DEVICE_NAME;
#endif
  
  rt_strncpy(rc_name, default_name, RT_NAME_MAX);
  
  rc_device_ = rt_device_find(rc_name);
  if (rc_device_ == RT_NULL) {
    LOG_E("Find RC device %s failed!", rc_name);
    return -RT_ERROR;
  }
  
  rt_err_t ret = rt_device_open(rc_device_, RT_DEVICE_FLAG_RDONLY);
  if (ret != RT_EOK) {
    LOG_E("Open RC device %s failed!", rc_name);
    return ret;
  }
  
  LOG_I("RC device %s opened successfully", rc_name);
  return RT_EOK;
}

void RcBf::initChannelRangeConfigs() {
  // Initialize with default values first
  for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
    channel_range_configs_[i].min = PWM_RANGE_MIN;
    channel_range_configs_[i].max = PWM_RANGE_MAX;
  }
  
  // Load channel range configurations from parameters
  // Each channel has its own range parameter: {min, max}
  // Parameter names: rc_channel_range_roll, rc_channel_range_pitch, rc_channel_range_yaw, rc_channel_range_throttle
  
  // Roll channel range
  rxChannelRangeConfig_t roll_range;
  if (getParam("rc_channel_range_roll", &roll_range, sizeof(rxChannelRangeConfig_t)) == RT_EOK) {
    channel_range_configs_[ROLL].min = roll_range.min;
    channel_range_configs_[ROLL].max = roll_range.max;
    LOG_I("Loaded rc_channel_range_roll: min=%u, max=%u", roll_range.min, roll_range.max);
  } else {
    LOG_W("Failed to load rc_channel_range_roll, using default");
  }
  
  // Pitch channel range
  rxChannelRangeConfig_t pitch_range;
  if (getParam("rc_channel_range_pitch", &pitch_range, sizeof(rxChannelRangeConfig_t)) == RT_EOK) {
    channel_range_configs_[PITCH].min = pitch_range.min;
    channel_range_configs_[PITCH].max = pitch_range.max;
    LOG_I("Loaded rc_channel_range_pitch: min=%u, max=%u", pitch_range.min, pitch_range.max);
  } else {
    LOG_W("Failed to load rc_channel_range_pitch, using default");
  }
  
  // Yaw channel range
  rxChannelRangeConfig_t yaw_range;
  if (getParam("rc_channel_range_yaw", &yaw_range, sizeof(rxChannelRangeConfig_t)) == RT_EOK) {
    channel_range_configs_[YAW].min = yaw_range.min;
    channel_range_configs_[YAW].max = yaw_range.max;
    LOG_I("Loaded rc_channel_range_yaw: min=%u, max=%u", yaw_range.min, yaw_range.max);
  } else {
    LOG_W("Failed to load rc_channel_range_yaw, using default");
  }
  
  // Throttle channel range
  rxChannelRangeConfig_t throttle_range;
  if (getParam("rc_channel_range_throttle", &throttle_range, sizeof(rxChannelRangeConfig_t)) == RT_EOK) {
    channel_range_configs_[THROTTLE].min = throttle_range.min;
    channel_range_configs_[THROTTLE].max = throttle_range.max;
    LOG_I("Loaded rc_channel_range_throttle: min=%u, max=%u", throttle_range.min, throttle_range.max);
  } else {
    LOG_W("Failed to load rc_channel_range_throttle, using default");
  }
}

void RcBf::initChannelMapping() {
  // Initialize with default mapping (no remapping)
  for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
    rcmap_[i] = i;  // Default: logical channel = physical channel
  }
  
  // Load channel mapping string from parameters
  char map_string[19];  // 18 chars + null terminator
  if (getParam("rc_channel_map", map_string, sizeof(map_string)) == RT_EOK) {
    map_string[sizeof(map_string) - 1] = '\0';  // Ensure null termination
    parseChannelMapping(map_string);
    LOG_I("Loaded rc_channel_map: %s", map_string);
  } else {
    LOG_W("Failed to load rc_channel_map, using default (no remapping)");
  }
}

void RcBf::parseChannelMapping(const char* map_string) {
  if (map_string == nullptr || map_string[0] == '\0') {
    return;
  }
  
  // Channel letters: A = Aileron (Roll), E = Elevator (Pitch), T = Throttle, R = Rudder (Yaw)
  // Numbers: 1,2,3,4... = Aux channels
  const char rcChannelLetters[] = "AERT12345678abcdefgh";
  
  // Reset mapping to default (no remapping)
  for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
    rcmap_[i] = i;
  }
  
  // Parse the mapping string
  for (int physical_channel = 0; physical_channel < MAX_SUPPORTED_RC_CHANNEL_COUNT && map_string[physical_channel] != '\0'; physical_channel++) {
    char c = map_string[physical_channel];
    const char* s = std::strchr(rcChannelLetters, c);
    
    if (s != nullptr) {
      int logical_channel = s - rcChannelLetters;
      if (logical_channel < MAX_SUPPORTED_RC_CHANNEL_COUNT) {
        rcmap_[logical_channel] = physical_channel;
      }
    }
  }
}

void RcBf::initFailsafeConfigs() {
  // Default failsafe: auto for flight channels, hold for aux channels
  for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
    failsafe_configs_[i].mode = (i < NON_AUX_CHANNEL_COUNT) ? 0 : 1;  // 0=auto, 1=hold
    failsafe_configs_[i].step = (i == THROTTLE)
        ? CHANNEL_VALUE_TO_RXFAIL_STEP(PWM_RANGE_MIN)
        : CHANNEL_VALUE_TO_RXFAIL_STEP(PWM_RANGE_MIDDLE);
  }
}

void RcBf::initRateProfile() {
  // Default Betaflight-style rates
  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    rate_profile_.rcRates[axis] = 100.0f;  // Default RC rate
    rate_profile_.rcExpo[axis] = 0.0f;     // No expo by default
    rate_profile_.rates[axis] = 0.0f;      // No super rate by default
    rate_profile_.rate_limit[axis] = 720.0f; // Default 720 deg/s limit
  }

  // Load rc_rate array [roll, pitch, yaw] from parameters
  float rc_rate_array[XYZ_AXIS_COUNT];
  if (getParam("rc_rate", rc_rate_array, sizeof(rc_rate_array)) == RT_EOK) {
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      rate_profile_.rcRates[axis] = rc_rate_array[axis];
    }
    LOG_I("Loaded rc_rate: [%.1f, %.1f, %.1f]", rc_rate_array[FD_ROLL], rc_rate_array[FD_PITCH], rc_rate_array[FD_YAW]);
  } else {
    LOG_W("Failed to load rc_rate, using default");
  }

  // Load rc_expo array [roll, pitch, yaw] from parameters
  float rc_expo_array[XYZ_AXIS_COUNT];
  if (getParam("rc_expo", rc_expo_array, sizeof(rc_expo_array)) == RT_EOK) {
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      rate_profile_.rcExpo[axis] = rc_expo_array[axis];
    }
    LOG_I("Loaded rc_expo: [%.1f, %.1f, %.1f]", rc_expo_array[FD_ROLL], rc_expo_array[FD_PITCH], rc_expo_array[FD_YAW]);
  } else {
    LOG_W("Failed to load rc_expo, using default");
  }

  // Load rc_super_rate array [roll, pitch, yaw] from parameters
  float rc_super_rate_array[XYZ_AXIS_COUNT];
  if (getParam("rc_super_rate", rc_super_rate_array, sizeof(rc_super_rate_array)) == RT_EOK) {
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      rate_profile_.rates[axis] = rc_super_rate_array[axis];
    }
    LOG_I("Loaded rc_super_rate: [%.1f, %.1f, %.1f]", rc_super_rate_array[FD_ROLL], rc_super_rate_array[FD_PITCH],
          rc_super_rate_array[FD_YAW]);
  } else {
    LOG_W("Failed to load rc_super_rate, using default");
  }

  // Load rc_rate_limit array [roll, pitch, yaw] from parameters
  float rc_rate_limit_array[XYZ_AXIS_COUNT];
  if (getParam("rc_rate_limit", rc_rate_limit_array, sizeof(rc_rate_limit_array)) == RT_EOK) {
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      rate_profile_.rate_limit[axis] = rc_rate_limit_array[axis];
    }
    LOG_I("Loaded rc_rate_limit: [%.1f, %.1f, %.1f]", rc_rate_limit_array[FD_ROLL], rc_rate_limit_array[FD_PITCH],
          rc_rate_limit_array[FD_YAW]);
  } else {
    LOG_W("Failed to load rc_rate_limit, using default");
  }

  // Calculate max RC rates for each axis
  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    maxRcRate_[axis] = applyBetaflightRates(axis, 1.0f, 1.0f);
  }
}

void RcBf::initRcControlsConfig() {
  rc_controls_config_.deadband = 0.0f;
  rc_controls_config_.yaw_deadband = 0.0f;
  rc_controls_config_.yaw_control_reversed = 0;
  
  // Load from parameters if available
  getParam("rc_deadband", &rc_controls_config_.deadband, sizeof(rc_controls_config_.deadband));
  getParam("rc_yaw_deadband", &rc_controls_config_.yaw_deadband, sizeof(rc_controls_config_.yaw_deadband));
  
  // Calculate command dividers for normalization (adjusted by deadband)
  // These are used when normalizing rcCommand to [-1.0, 1.0] range
  rcCommandDivider_ = 500.0f - rc_controls_config_.deadband;
  rcCommandYawDivider_ = 500.0f - rc_controls_config_.yaw_deadband;
  
  // Ensure dividers are positive and not too small
  if (rcCommandDivider_ < 1.0f) {
    rcCommandDivider_ = 500.0f;
  }
  if (rcCommandYawDivider_ < 1.0f) {
    rcCommandYawDivider_ = 500.0f;
  }
}

void RcBf::readRawRcChannels() {
  if (rc_device_ == RT_NULL) {
    return;
  }
  
  // Read raw channels from device into temporary buffer
  uint16_t raw_channels_temp[MAX_SUPPORTED_RC_CHANNEL_COUNT];
  rt_uint16_t channel_mask = 0xFFFF;
  rt_size_t size = rt_device_read(rc_device_, channel_mask, raw_channels_temp, 
                                   MAX_SUPPORTED_RC_CHANNEL_COUNT * sizeof(uint16_t));
  
  if (size > 0) {
    rc_loss_count_ = 0;
    
    // Apply channel mapping: map logical channels to physical channels
    // rcmap_[logical] = physical, so we need to reverse: rc_raw_channels_[logical] = raw_channels_temp[physical]
    for (int logical_channel = 0; logical_channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; logical_channel++) {
      uint8_t physical_channel = rcmap_[logical_channel];
      if (physical_channel < MAX_SUPPORTED_RC_CHANNEL_COUNT) {
        rc_raw_channels_[logical_channel] = raw_channels_temp[physical_channel];
      } else {
        rc_raw_channels_[logical_channel] = 0;  // Invalid mapping
      }
    }
  } else {
    rc_loss_count_++;
  }
}

void RcBf::applyRangeScaling() {
  // Step 1: 原始值 → rcRaw[]（range 缩放）
  for (int channel = 0; channel < channel_count_; channel++) {
    float sample = static_cast<float>(rc_raw_channels_[channel]);
    
    // Apply range scaling for flight channels
    if (channel < NON_AUX_CHANNEL_COUNT) {
      const rxChannelRangeConfig_t* range = &channel_range_configs_[channel];
      if (sample != PPM_RCVR_TIMEOUT) {
        sample = this->scaleRangef(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
      }
    }
    
    rc_raw_[channel] = sample;
  }
}

void RcBf::applyFailsafeAndConstraints(uint32_t current_time_us) {
  // Step 2: rcRaw[] → rcData[]（failsafe + 约束）
  uint32_t current_time_ms = current_time_us / 1000;
  bool rx_receiving_signal = (rc_loss_count_ <= 10);
  
  for (int channel = 0; channel < channel_count_; channel++) {
    float sample = rc_raw_[channel];
    bool this_channel_valid = rx_receiving_signal && this->isPulseValid(static_cast<uint16_t>(sample));
    
    if (this_channel_valid) {
      valid_rx_signal_timeout_[channel] = current_time_ms + MAX_INVALID_PULSE_TIME_MS;
    }
    
    // Apply failsafe logic
    if (!rx_receiving_signal) {
      // Signal lost - apply failsafe
      const rxFailsafeConfig_t* failsafe = &failsafe_configs_[channel];
      
      if (failsafe->mode == 0) {  // Auto
        if (channel == THROTTLE) {
          sample = PWM_RANGE_MIN;
        } else if (channel < NON_AUX_CHANNEL_COUNT) {
          sample = PWM_RANGE_MIDDLE;
        } else {
          sample = rc_data_[channel];  // Hold last value
        }
      } else if (failsafe->mode == 1) {  // Hold
        sample = rc_data_[channel];  // Hold last value
      } else if (failsafe->mode == 2) {  // Set
        sample = RXFAIL_STEP_TO_CHANNEL_VALUE(failsafe->step);
      }
    } else if (!this_channel_valid) {
      // Channel invalid but signal present
      if (current_time_ms < valid_rx_signal_timeout_[channel]) {
        // First 300ms - hold last value
        sample = rc_data_[channel];
      } else {
        // After 300ms - apply failsafe
        const rxFailsafeConfig_t* failsafe = &failsafe_configs_[channel];
        if (failsafe->mode == 2) {
          sample = RXFAIL_STEP_TO_CHANNEL_VALUE(failsafe->step);
        } else {
          sample = rc_data_[channel];  // Hold
        }
      }
    }
    
    // Constrain to valid range
    sample = constrainf(sample, PWM_PULSE_MIN, PWM_PULSE_MAX);
    rc_data_[channel] = sample;
  }
}

void RcBf::updateRcCommands() {
  // RX Task (100-200Hz): rcData[] → rcCommand[]（转换为控制命令）
  const float midrc = PWM_RANGE_MIDDLE;
  
  // Process roll, pitch, yaw
  for (int axis = 0; axis < 3; axis++) {
    float rc = constrainf(rc_data_[axis] - midrc, -500.0f, 500.0f);
    float rc_deadband = 0.0f;

    if (axis == FD_ROLL || axis == FD_PITCH) {
      rc_deadband = rc_controls_config_.deadband;
    } else {
      rc_deadband = rc_controls_config_.yaw_deadband;
      if (rc_controls_config_.yaw_control_reversed) {
        rc = -rc;
      }
    }

    rc_command_[axis] = applyDeadband(rc, rc_deadband) + midrc;  // Convert back to 1000-2000 range
  }
  
  // Process throttle
  float throttle_min = PWM_RANGE_MIN;
  float throttle_max = PWM_RANGE_MAX;
  float throttle = constrainf(rc_data_[THROTTLE], throttle_min, throttle_max);

  // Apply throttle rate limiting (limit sudden increases)
  throttle = applyThrottleRateLimit(throttle);

  rc_command_[THROTTLE] = throttle;
  
  // Mark new data available for PID task
  is_rc_data_new_ = true;
}

void RcBf::processRcCommand(uint32_t current_time_us) {
  if (!is_rc_data_new_) {
    return;  // No new data, skip processing
  }

  // Update smoothing filter cutoffs if needed (now handled by RcSmoothingFilter singleton)
  RcSmoothingFilter& smoothing_filter = RcSmoothingFilter::instance();
  smoothing_filter.updateFilterCutoffs(target_looptime_s_, smoothed_rx_rate_hz_);

  // Calculate setpoint rates from rcCommand
  // Normalize rcCommand to [-1.0, 1.0] range using dividers adjusted by deadband
  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    float rc_commandf = 0.0f;
    
    if (axis == FD_ROLL) {
      rc_commandf = (rc_command_[FD_ROLL] - PWM_RANGE_MIDDLE) / rcCommandDivider_;
    } else if (axis == FD_PITCH) {
      rc_commandf = (rc_command_[FD_PITCH] - PWM_RANGE_MIDDLE) / rcCommandDivider_;
    } else if (axis == FD_YAW) {
      rc_commandf = (rc_command_[FD_YAW] - PWM_RANGE_MIDDLE) / rcCommandYawDivider_;
    }
    
    rc_commandf = constrainf(rc_commandf, -1.0f, 1.0f);
    rcDeflection_[axis] = rc_commandf;
    const float rc_commandf_abs = std::abs(rc_commandf);
    rcDeflectionAbs_[axis] = rc_commandf_abs;
    
    // Apply rates to get angle rate: rcCommand[] → applyRates() → rawSetpoint[]
    float angle_rate = applyBetaflightRates(axis, rc_commandf, rc_commandf_abs);

    // Limit setpoint (match Betaflight: -1.0f * rate_limit to 1.0f * rate_limit)
    rawSetpoint_[axis] =
        constrainf(angle_rate, -1.0f * rate_profile_.rate_limit[axis], 1.0f * rate_profile_.rate_limit[axis]);

    // Calculate feedforward (setpoint delta)
    float setpoint_delta = rawSetpoint_[axis] - prevSetpoint_[axis];
    feedforward_[axis] = setpoint_delta * smoothed_rx_rate_hz_;
    prevSetpoint_[axis] = rawSetpoint_[axis];
  }

  // Publish rawSetpoint data to MCN for PID thread (avoid data tearing)
  publishRcCommandToMcn(current_time_us);

  // Mark data as processed
  is_rc_data_new_ = false;
}

void RcBf::updateRcRefreshRate(uint32_t current_time_us, bool rxReceivingSignal) {
  if (rxReceivingSignal && last_rc_time_us_ > 0) {
    uint32_t delta = current_time_us - last_rc_time_us_;
    current_rx_interval_us_ = static_cast<uint16_t>(
        constrainf(static_cast<float>(delta), RX_INTERVAL_MIN_US, RX_INTERVAL_MAX_US));
    current_rx_rate_hz_ = 1e6f / current_rx_interval_us_;
    is_rx_rate_valid_ = (delta == current_rx_interval_us_);
    
    // Smooth the rate
    const float smoothing_factor = 0.1f;
    smoothed_rx_rate_hz_ += smoothing_factor * (current_rx_rate_hz_ - smoothed_rx_rate_hz_);
  }
  last_rc_time_us_ = current_time_us;
}

float RcBf::applyThrottleRateLimit(float throttle) {
  // Calculate throttle increase
  float throttle_increase = throttle - prev_rc_command_throttle_;

  // Limit maximum increase per cycle
  if (throttle_increase > THROTTLE_RATE_LIMIT) {
    throttle = prev_rc_command_throttle_ + THROTTLE_RATE_LIMIT;
    // Calculate original requested value inline to avoid unused variable warning if LOG_E is disabled
    // LOG_E("Throttle rate limit applied: prev=%.1f, requested=%.1f, increase=%.1f, limited to %.1f (max
    // increase=%.1f)",
    //       prev_rc_command_throttle_, throttle + throttle_increase - THROTTLE_RATE_LIMIT, throttle_increase, throttle,
    //       THROTTLE_RATE_LIMIT);
  }

  // Update previous throttle value
  prev_rc_command_throttle_ = throttle;

  return throttle;
}

float RcBf::applyBetaflightRates(int axis, float rcCommandf, float rcCommandfAbs) const {
  float rc_commandf = rcCommandf;
  
  // Apply expo
  if (rate_profile_.rcExpo[axis] > 0.0f) {
    const float expof = rate_profile_.rcExpo[axis] / 100.0f;
    rc_commandf = rc_commandf * rc_commandf * rc_commandf * expof + 
                  rc_commandf * (1.0f - expof);
  }
  
  // Calculate RC rate
  float rc_rate_raw = rate_profile_.rcRates[axis] / 100.0f;
  float rc_rate = rc_rate_raw;
  if (rc_rate > 2.0f) {
    float rc_rate_incremental = RC_RATE_INCREMENTAL * (rc_rate - 2.0f);
    rc_rate += rc_rate_incremental;
  }
  
  // Calculate angle rate
  float angle_rate = 200.0f * rc_rate * rc_commandf;
  
  // Apply super rate
  if (rate_profile_.rates[axis] > 0.0f) {
    float super_rate_percent = rate_profile_.rates[axis] / 100.0f;
    float denominator = 1.0f - (rcCommandfAbs * super_rate_percent);
    float rc_superfactor = 1.0f / constrainf(denominator, 0.01f, 1.00f);
    angle_rate *= rc_superfactor;
  }

  return angle_rate;
}

float RcBf::getMaxRcRate(int axis) const {
  if (axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return maxRcRate_[axis];
  }
  return 0.0f;
}

float RcBf::constrainf(float x, float min, float max) const {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

float RcBf::scaleRangef(float x, float in_min, float in_max, float out_min, float out_max) const {
  if (in_max == in_min) return out_min;
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

bool RcBf::isPulseValid(uint16_t pulseDuration) const {
  return (pulseDuration >= PWM_PULSE_MIN && pulseDuration <= PWM_PULSE_MAX);
}

float RcBf::applyDeadband(float value, float deadband) const {
  if (std::abs(value) < deadband) {
    return 0.0f;
  }
  if (value > 0.0f) {
    return value - deadband;
  } else {
    return value + deadband;
  }
}

// Process auxiliary channels and publish to MCN
// Combines RcControls processing and MCN publishing
void RcBf::processAuxChannelsAndPublish(uint32_t current_time_us) {
  // Process auxiliary channels with RcControls (including failsafe protection)
  // This handles stick positions, arming/disarming, and flight mode switching
  RcControls& rc_controls = RcControls::instance();
  rc_controls.processRcStickPositions(rc_data_, current_time_us);
  
  // Publish auxiliary channels data to MCN (after failsafe protection)
  publishAuxChannelsToMcn(current_time_us);
}

// Handle mlog start/stop based on arm status change
void RcBf::handleMlogArmControl() {
  if (rc_arm_control_ != 1) {
    return;  // Mlog arm control disabled
  }
  
  RcControls& rc_controls = RcControls::instance();
  bool current_armed = rc_controls.isArmed();
  
  // Check if this is the first run (prev_armed_status_ is uninitialized)
  // On first run, initialize prev_armed_status_ to current state without triggering action
  static bool first_run = true;
  if (first_run) {
    prev_armed_status_ = current_armed;
    first_run = false;
    return;
  }
  
  // Check if arm status changed
  if (prev_armed_status_ != current_armed) {
    if (current_armed) {
      // Disarmed -> Armed: Start mlog
#ifdef TASK_TOOL_02_SD_MLOG
      task_mlog_start_logging(NULL);
      LOG_I("RC arm detected, mlog started");
#endif
    } else {
      // Armed -> Disarmed: Stop mlog
#ifdef TASK_TOOL_02_SD_MLOG
      task_mlog_stop_logging();
      LOG_I("RC disarm detected, mlog stopped");
#endif
    }
    prev_armed_status_ = current_armed;
  }
}

// Prepare and push RC data to mlog
void RcBf::prepareAndPushRcMlogData(uint32_t current_time_us) {
  bf_mlog::rc_mlog_data_t mlog_data;
  
  // Note: seq_ was incremented in both publishRcCommandToMcn and publishAuxChannelsToMcn
  // So use seq_ - 2 to match the seq from publishRcCommandToMcn
  mlog_data.seq = seq_ - 2;
  mlog_data.timestamp = current_time_us;
  
  // Copy first 6 channels of raw RC data
  for (int i = 0; i < 6; i++) {
    mlog_data.raw_channels[i] = rc_data_[i];
  }
  
  // Copy rawSetpoint rates
  std::memcpy(mlog_data.rawSetpoint, rawSetpoint_, sizeof(mlog_data.rawSetpoint));
  
  // Copy throttle and related data
  mlog_data.rcCommandThrottle = rc_command_[THROTTLE];
  mlog_data.rx_rate_hz = smoothed_rx_rate_hz_;
  mlog_data.rc_raw_throttle = rc_raw_channels_[THROTTLE];
  
  // Get throttle cutoff frequency from smoothing filter
  RcSmoothingFilter& smoothing_filter = RcSmoothingFilter::instance();
  mlog_data.throttle_cutoff = smoothing_filter.getThrottleCutoffFrequency();
  
  // Get arm status and flight mode from RcControls
  RcControls& rc_controls = RcControls::instance();
  mlog_data.armed = rc_controls.isArmed() ? RC_ARMED_STATUS_ARMED : RC_ARMED_STATUS_DISARMED;
  mlog_data.flight_mode = rc_controls.getFlightMode();
  
  // Push to mlog
  pushRcDataToMlog(&mlog_data);
}

// Thread entry point - RX Task (100-200Hz)
void RcBf::rcThreadEntry(void* parameter) {
  RcBf* instance = static_cast<RcBf*>(parameter);
  if (instance == nullptr) {
    return;
  }
  
  LOG_I("RC thread started (RX task, 100-200Hz)");
  
  // Target frequency: 100-200Hz (5-10ms interval)
  const uint32_t target_interval_us = 5000;  // 5ms = 200Hz

  while (true) {
    uint32_t current_time_us = timestamp_micros();
    
    // ========== Module 1: Read and Process Raw RC Channels ==========
    instance->readRawRcChannels();

#ifdef PROJECT_BF_RC_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG3_HIGH();  // Debug pin: RC task execution start (monitor RC task frequency ~100-200Hz)
#endif
    
    instance->applyRangeScaling();
    instance->applyFailsafeAndConstraints(current_time_us);
    instance->updateRcCommands();  // rcData[] → rcCommand[]
    
    // ========== Module 2: Update RC Refresh Rate ==========
    bool rx_receiving_signal = (instance->rc_loss_count_ <= 10);
    instance->updateRcRefreshRate(current_time_us, rx_receiving_signal);

    // ========== Module 3: Process RC Command and Publish Setpoint ==========
    // This calculates rawSetpoint[] from rcCommand[] and publishes to MCN for PID thread
    instance->processRcCommand(current_time_us);

    // ========== Module 4: Process Auxiliary Channels and Publish ==========
    // Combines RcControls processing and MCN publishing
    instance->processAuxChannelsAndPublish(current_time_us);

    // ========== Module 5: Handle Mlog Arm Control ==========
    // Start/stop mlog based on arm status change (if enabled)
    instance->handleMlogArmControl();

    // ========== Module 6: Prepare and Push RC Data to Mlog ==========
    // Record raw channels, rawSetpoint, throttle, armed, flight_mode
    instance->prepareAndPushRcMlogData(current_time_us);

#ifdef PROJECT_BF_RC_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG3_LOW();  // Debug pin: RC task execution end
#endif

    // ========== Rate Control ==========
    // Maintain 100-200Hz
    uint32_t elapsed = timestamp_micros() - current_time_us;
    if (elapsed < target_interval_us) {
      rt_thread_mdelay((target_interval_us - elapsed) / 1000);
    }
  }
}

// RT-Thread auto initialization wrapper
#ifdef PROJECT_BF_RC_EN
extern "C" {
static int rc_bf_init_wrapper(void) {
  RcBf& instance = RcBf::instance();
  rt_err_t ret = instance.init();
  if (ret == RT_EOK) {
    LOG_I("RcBf auto-init success");
  } else {
    LOG_E("RcBf auto-init failed: %d", ret);
  }
  return (int)ret;
}
INIT_APP_EXPORT(rc_bf_init_wrapper);
}
#endif

