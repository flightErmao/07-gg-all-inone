#include "rc_bf.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#include <rtdevice.h>
#include "timestamp.h"
#include "filter.h"  // pt3Filter functions
#include "param.h"   // getParam function
#define LOG_TAG "rc_bf"
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

// Helper macros
#define RXFAIL_STEP_TO_CHANNEL_VALUE(step) (PWM_PULSE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_RXFAIL_STEP(channelValue) \
  ((constrainf(channelValue, PWM_PULSE_MIN, PWM_PULSE_MAX) - PWM_PULSE_MIN) / 25)

// Singleton instance
RcBf& RcBf::instance() {
  static RcBf instance_obj;
  return instance_obj;
}

// Constructor
RcBf::RcBf()
    : rc_thread_(RT_NULL),
      rc_thread_stack_(nullptr),
      thread_inited_(false),
      rcCommandDivider_(500.0f),       // Will be updated in initRcControlsConfig()
      rcCommandYawDivider_(500.0f),   // Will be updated in initRcControlsConfig()
      rc_device_(RT_NULL),
      last_rc_time_us_(0),
      previous_rx_interval_us_(0),
      current_rx_interval_us_(0),
      current_rx_rate_hz_(100.0f),
      smoothed_rx_rate_hz_(100.0f),
      is_rx_rate_valid_(false),
      rx_receiving_signal_(false),
      rx_flight_channels_valid_(false),
      is_rc_data_new_(false),
      channel_count_(MAX_SUPPORTED_RC_CHANNEL_COUNT),
      rc_loss_count_(0),
      seq_(0),
      target_looptime_s_(0.000125f) {  // 8kHz default
  std::memset(rc_raw_channels_, 0, sizeof(rc_raw_channels_));
  std::memset(rc_raw_, 0, sizeof(rc_raw_));
  std::memset(rc_data_, 0, sizeof(rc_data_));
  std::memset(rc_command_, 0, sizeof(rc_command_));
  std::memset(rawSetpoint_, 0, sizeof(rawSetpoint_));
  std::memset(setpointRate_, 0, sizeof(setpointRate_));
  std::memset(rcDeflection_, 0, sizeof(rcDeflection_));
  std::memset(rcDeflectionAbs_, 0, sizeof(rcDeflectionAbs_));
  std::memset(maxRcRate_, 0, sizeof(maxRcRate_));
  std::memset(feedforward_, 0, sizeof(feedforward_));
  std::memset(prevSetpoint_, 0, sizeof(prevSetpoint_));
  std::memset(valid_rx_signal_timeout_, 0, sizeof(valid_rx_signal_timeout_));
  std::memset(&rc_thread_obj_, 0, sizeof(rc_thread_obj_));
  std::memset(&rate_profile_, 0, sizeof(rate_profile_));
  std::memset(&rc_controls_config_, 0, sizeof(rc_controls_config_));
  std::memset(channel_range_configs_, 0, sizeof(channel_range_configs_));
  std::memset(rcmap_, 0, sizeof(rcmap_));
  std::memset(failsafe_configs_, 0, sizeof(failsafe_configs_));
  std::memset(&rc_smoothing_data_, 0, sizeof(rc_smoothing_data_));
  
  // Initialize default values
  for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
    rc_data_[i] = PWM_RANGE_MIDDLE;  // Default to middle
    valid_rx_signal_timeout_[i] = MAX_INVALID_PULSE_TIME_MS;
  }
  rc_data_[THROTTLE] = PWM_RANGE_MIN;  // Throttle starts at minimum
}

RcBf::~RcBf() {
  if (thread_inited_ && rc_thread_ != RT_NULL) {
    rt_thread_delete(rc_thread_);
  }
  if (rc_thread_stack_ != nullptr) {
    delete[] rc_thread_stack_;
  }
  if (rc_device_ != RT_NULL) {
    rt_device_close(rc_device_);
  }
}

rt_err_t RcBf::init() {
  // Initialize configurations
  initChannelRangeConfigs();
  initChannelMapping();
  initFailsafeConfigs();
  initRateProfile();
  initRcControlsConfig();
  initSmoothingFilters();
  
  // Initialize RC device
  rt_err_t ret = initRcDevice();
  if (ret != RT_EOK) {
    LOG_E("RC device init failed");
    return ret;
  }
  
  // Allocate thread stack
#ifndef CONFIG_PROJECT_BF_RC_THREAD_STACK_SIZE
#define CONFIG_PROJECT_BF_RC_THREAD_STACK_SIZE 2048
#endif
  size_t stack_size = CONFIG_PROJECT_BF_RC_THREAD_STACK_SIZE;
  rc_thread_stack_ = new rt_uint8_t[stack_size];
  if (rc_thread_stack_ == nullptr) {
    LOG_E("Failed to allocate thread stack");
    return -RT_ENOMEM;
  }
  std::memset(rc_thread_stack_, 0, stack_size);
  
  // Initialize thread
  ret = rt_thread_init(&rc_thread_obj_, "rc_bf", rcThreadEntry, this,
                       rc_thread_stack_, stack_size,
#ifndef CONFIG_PROJECT_BF_RC_THREAD_PRIORITY
                       RT_THREAD_PRIORITY_MAX / 2,
#else
                       CONFIG_PROJECT_BF_RC_THREAD_PRIORITY,
#endif
#ifndef CONFIG_PROJECT_BF_RC_THREAD_TIMESLICE
                       5
#else
                       CONFIG_PROJECT_BF_RC_THREAD_TIMESLICE
#endif
                       );
  
  if (ret != RT_EOK) {
    LOG_E("RC thread init failed: %d", ret);
    delete[] rc_thread_stack_;
    rc_thread_stack_ = nullptr;
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
  
  LOG_I("RcBf initialized successfully");
  return RT_EOK;
}

rt_err_t RcBf::initRcDevice() {
  char rc_name[RT_NAME_MAX];
  
  // Get device name from config
#ifndef CONFIG_PROJECT_BF_RC_DEVICE_NAME
  const char* default_name = "rc";
#else
  const char* default_name = CONFIG_PROJECT_BF_RC_DEVICE_NAME;
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
  
  // Load from parameters if available
  float rc_rate_roll = rate_profile_.rcRates[FD_ROLL];
  if (getParam("rc_rate_roll", &rc_rate_roll, sizeof(rc_rate_roll)) == RT_EOK) {
    rate_profile_.rcRates[FD_ROLL] = rc_rate_roll;
  }
  
  float rc_rate_pitch = rate_profile_.rcRates[FD_PITCH];
  if (getParam("rc_rate_pitch", &rc_rate_pitch, sizeof(rc_rate_pitch)) == RT_EOK) {
    rate_profile_.rcRates[FD_PITCH] = rc_rate_pitch;
  }
  
  float rc_rate_yaw = rate_profile_.rcRates[FD_YAW];
  if (getParam("rc_rate_yaw", &rc_rate_yaw, sizeof(rc_rate_yaw)) == RT_EOK) {
    rate_profile_.rcRates[FD_YAW] = rc_rate_yaw;
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

void RcBf::initSmoothingFilters() {
  float dt = target_looptime_s_;
  float cutoff_hz = RC_SMOOTHING_CUTOFF_HZ;
  
  // Initialize pt3Filter for each channel
  float k = pt3FilterGain(cutoff_hz, dt);
  for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
    pt3FilterInit(&rc_smoothing_data_.filterSetpoint[i], k);
  }
  
  // Initialize smoothing data
  rc_smoothing_data_.setpointCutoffFrequency = cutoff_hz;
  rc_smoothing_data_.throttleCutoffFrequency = cutoff_hz;
  rc_smoothing_data_.setpointCutoffSetting = cutoff_hz;
  rc_smoothing_data_.throttleCutoffSetting = cutoff_hz;
  rc_smoothing_data_.autoSmoothnessFactorSetpoint = 0.15f;  // Default auto factor
  rc_smoothing_data_.autoSmoothnessFactorThrottle = 0.15f;
  rc_smoothing_data_.enabled = true;
}

void RcBf::updateSmoothingFilterCutoffs() {
  if (!rc_smoothing_data_.enabled) {
    return;
  }
  
  const float min_cutoff_hz = 15.0f;
  float dt = target_looptime_s_;
  
  // Calculate setpoint cutoff (auto if setting is 0, otherwise use manual value)
  float setpoint_cutoff = rc_smoothing_data_.setpointCutoffSetting == 0
      ? std::max(min_cutoff_hz, smoothed_rx_rate_hz_ * rc_smoothing_data_.autoSmoothnessFactorSetpoint)
      : rc_smoothing_data_.setpointCutoffSetting;
  
  // Calculate throttle cutoff (auto if setting is 0, otherwise use manual value)
  float throttle_cutoff = rc_smoothing_data_.throttleCutoffSetting == 0
      ? std::max(min_cutoff_hz, smoothed_rx_rate_hz_ * rc_smoothing_data_.autoSmoothnessFactorThrottle)
      : rc_smoothing_data_.throttleCutoffSetting;
  
  // Update filter cutoffs
  float pt3k_sp = pt3FilterGain(setpoint_cutoff, dt);
  float pt3k_thr = pt3FilterGain(throttle_cutoff, dt);
  
  // Update setpoint filters for ROLL, PITCH, YAW
  for (int i = FD_ROLL; i <= FD_YAW; i++) {
    pt3FilterUpdateCutoff(&rc_smoothing_data_.filterSetpoint[i], pt3k_sp);
  }
  
  // Update throttle filter
  pt3FilterUpdateCutoff(&rc_smoothing_data_.filterSetpoint[THROTTLE], pt3k_thr);
  
  rc_smoothing_data_.setpointCutoffFrequency = setpoint_cutoff;
  rc_smoothing_data_.throttleCutoffFrequency = throttle_cutoff;
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
    rx_receiving_signal_ = true;
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
    if (rc_loss_count_ > 10) {
      rx_receiving_signal_ = false;
    }
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
        sample = scaleRangef(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
      }
    }
    
    rc_raw_[channel] = sample;
  }
}

void RcBf::applyFailsafeAndConstraints(uint32_t current_time_us) {
  // Step 2: rcRaw[] → rcData[]（failsafe + 约束）
  uint32_t current_time_ms = current_time_us / 1000;
  
  rx_flight_channels_valid_ = rx_receiving_signal_;
  
  for (int channel = 0; channel < channel_count_; channel++) {
    float sample = rc_raw_[channel];
    bool this_channel_valid = rx_flight_channels_valid_ && isPulseValid(sample);
    
    if (this_channel_valid) {
      valid_rx_signal_timeout_[channel] = current_time_ms + MAX_INVALID_PULSE_TIME_MS;
    }
    
    // Apply failsafe logic
    if (!rx_receiving_signal_) {
      // Signal lost - apply failsafe
      const rxFailsafeChannelConfig_t* failsafe = &failsafe_configs_[channel];
      
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
        if (channel < NON_AUX_CHANNEL_COUNT) {
          rx_flight_channels_valid_ = false;
        }
        const rxFailsafeChannelConfig_t* failsafe = &failsafe_configs_[channel];
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
    
    if (axis == ROLL || axis == PITCH) {
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
  rc_command_[THROTTLE] = throttle;
  
  // Mark new data available for PID task
  is_rc_data_new_ = true;
}

// PID Task (1-4kHz): Process RC command and calculate setpoint rates
void RcBf::processRcCommand(uint32_t current_time_us) {
  if (!is_rc_data_new_) {
    return;  // No new data, skip processing
  }
  
  // Update smoothing filter cutoffs if needed
  updateSmoothingFilterCutoffs();
  
  // Calculate setpoint rates from rcCommand
  // Normalize rcCommand to [-1.0, 1.0] range using dividers adjusted by deadband
  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    float rc_commandf = 0.0f;
    
    if (axis == FD_ROLL) {
      rc_commandf = (rc_command_[ROLL] - PWM_RANGE_MIDDLE) / rcCommandDivider_;
    } else if (axis == FD_PITCH) {
      rc_commandf = (rc_command_[PITCH] - PWM_RANGE_MIDDLE) / rcCommandDivider_;
    } else if (axis == FD_YAW) {
      rc_commandf = (rc_command_[YAW] - PWM_RANGE_MIDDLE) / rcCommandYawDivider_;
    }
    
    rc_commandf = constrainf(rc_commandf, -1.0f, 1.0f);
    rcDeflection_[axis] = rc_commandf;
    const float rc_commandf_abs = std::abs(rc_commandf);
    rcDeflectionAbs_[axis] = rc_commandf_abs;
    
    // Apply rates to get angle rate: rcCommand[] → applyRates() → rawSetpoint[]
    float angle_rate = applyBetaflightRates(axis, rc_commandf, rc_commandf_abs);
    
    // Limit setpoint
    rawSetpoint_[axis] = constrainf(angle_rate, -rate_profile_.rate_limit[axis], 
                                     rate_profile_.rate_limit[axis]);
    
    // Calculate feedforward (setpoint delta)
    float setpoint_delta = rawSetpoint_[axis] - prevSetpoint_[axis];
    feedforward_[axis] = setpoint_delta * smoothed_rx_rate_hz_;
    prevSetpoint_[axis] = rawSetpoint_[axis];
  }
  
  // Mark data as processed
  is_rc_data_new_ = false;
}

// PID Task (1-4kHz): Process RC smoothing filter
// Input: rcCommand[THROTTLE] or rawSetpoint[ROLL/PITCH/YAW]
// Output: rcCommand[THROTTLE] (in-place update), setpointRate[ROLL/PITCH/YAW] (new array)
void RcBf::processRcSmoothingFilter() {
  if (!rc_smoothing_data_.enabled) {
    // If smoothing is disabled, just copy rawSetpoint to setpointRate
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      setpointRate_[axis] = rawSetpoint_[axis];
    }
    return;
  }
  
  // Prepare data to smooth
  float rx_data_to_smooth[PRIMARY_CHANNEL_COUNT];
  for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
    if (i == THROTTLE) {
      rx_data_to_smooth[i] = rc_command_[THROTTLE];
    } else {
      rx_data_to_smooth[i] = rawSetpoint_[i];
    }
  }
  
  // Apply pt3Filter smoothing
  for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
    float smoothed = pt3FilterApply(&rc_smoothing_data_.filterSetpoint[i], rx_data_to_smooth[i]);
    
    if (i == THROTTLE) {
      // Output: rcCommand[THROTTLE] (in-place update)
      rc_command_[THROTTLE] = smoothed;
    } else {
      // Output: setpointRate[ROLL/PITCH/YAW] (new array)
      setpointRate_[i] = smoothed;
    }
  }
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

float RcBf::applyBetaflightRates(int axis, float rcCommandf, float rcCommandfAbs) const {
  float rc_commandf = rcCommandf;
  
  // Apply expo
  if (rate_profile_.rcExpo[axis] > 0.0f) {
    const float expof = rate_profile_.rcExpo[axis] / 100.0f;
    rc_commandf = rc_commandf * rc_commandf * rc_commandf * expof + 
                  rc_commandf * (1.0f - expof);
  }
  
  // Calculate RC rate
  float rc_rate = rate_profile_.rcRates[axis] / 100.0f;
  if (rc_rate > 2.0f) {
    rc_rate += RC_RATE_INCREMENTAL * (rc_rate - 2.0f);
  }
  
  // Calculate angle rate
  float angle_rate = 200.0f * rc_rate * rc_commandf;
  
  // Apply super rate
  if (rate_profile_.rates[axis] > 0.0f) {
    const float rc_superfactor = 1.0f / 
        constrainf(1.0f - (rcCommandfAbs * (rate_profile_.rates[axis] / 100.0f)), 0.01f, 1.00f);
    angle_rate *= rc_superfactor;
  }
  
  return angle_rate;
}

float RcBf::getSetpointRate(int axis) const {
  if (axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return setpointRate_[axis];
  }
  return 0.0f;
}

float RcBf::getFeedforward(int axis) const {
  if (axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return feedforward_[axis];
  }
  return 0.0f;
}

float RcBf::getMaxRcRate(int axis) const {
  if (axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return maxRcRate_[axis];
  }
  return 720.0f;
}

float RcBf::getRcCommand(int channel) const {
  if (channel >= 0 && channel < PRIMARY_CHANNEL_COUNT) {
    return rc_command_[channel];  // Return smoothed value (THROTTLE) or original (ROLL/PITCH/YAW)
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
    
    // RX Task: Read raw channels and process to rcCommand[]
    instance->readRawRcChannels();
    instance->applyRangeScaling();
    instance->applyFailsafeAndConstraints(current_time_us);
    instance->updateRcCommands();  // rcData[] → rcCommand[]
    
    // Update refresh rate
    instance->updateRcRefreshRate(current_time_us, instance->rx_receiving_signal_);
    
    // Rate control - maintain 100-200Hz
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

