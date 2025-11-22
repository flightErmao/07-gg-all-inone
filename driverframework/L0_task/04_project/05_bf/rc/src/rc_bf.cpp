#include "rc_bf.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#include <rtdevice.h>
#include "timestamp.h"
#include "filter.h"  // pt3Filter functions
#include "param.h"   // getParam function
#ifdef PROJECT_BF_RC_DEBUG_PIN_EN
#include "debugPin.h"
#endif
#define LOG_TAG "rc_bf"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "uMCN.h"
}

#include <cmath>
#include <cstring>
#include "rc_setpoint_msg.h"
// #include "rc_setpoint_debug.h"

/* 定义 RC setpoint MCN 话题（在本文件内完成定义与发布） */
MCN_DEFINE(rc, sizeof(rc_setpoint_msg_t));

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
}

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
    : rcCommandDivider_(500.0f),     // Will be updated in initRcControlsConfig()
      rcCommandYawDivider_(500.0f),  // Will be updated in initRcControlsConfig()
      last_rc_time_us_(0),
      previous_rx_interval_us_(0),
      current_rx_interval_us_(0),
      current_rx_rate_hz_(100.0f),
      smoothed_rx_rate_hz_(100.0f),
      is_rx_rate_valid_(false),
      rx_receiving_signal_(false),
      rx_flight_channels_valid_(false),
      is_rc_data_new_(false),
      rc_thread_(RT_NULL),
      rc_thread_stack_(nullptr),
      thread_inited_(false),
      rc_device_(RT_NULL),
      channel_count_(MAX_SUPPORTED_RC_CHANNEL_COUNT),
      rc_loss_count_(0),
      seq_(0),
      target_looptime_s_(0.000125f),  // 8kHz default
      rc_setpoint_hub_(nullptr),
      rc_setpoint_event_(RT_NULL),
      rc_setpoint_node_(RT_NULL) {
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
  std::memset(&last_rc_setpoint_msg_, 0, sizeof(last_rc_setpoint_msg_));

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

  // Initialize RC device
  rt_err_t ret = initRcDevice();
  if (ret != RT_EOK) {
    LOG_E("RC device init failed");
    return ret;
  }

  // Allocate thread stack
  size_t stack_size = PROJECT_BF_RC_THREAD_STACK_SIZE;
  rc_thread_stack_ = new rt_uint8_t[stack_size];
  if (rc_thread_stack_ == nullptr) {
    LOG_E("Failed to allocate thread stack");
    return -RT_ENOMEM;
  }
  std::memset(rc_thread_stack_, 0, stack_size);
  
  // Initialize thread
  ret = rt_thread_init(&rc_thread_obj_, "rc_bf", rcThreadEntry, this, rc_thread_stack_, stack_size,
                       PROJECT_BF_RC_THREAD_PRIORITY, PROJECT_BF_RC_THREAD_TIMESLICE);

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

void RcBf::initSmoothingFilters() {
  float dt = target_looptime_s_;

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
  rc_smoothing_data_.setpointCutoffSetting = setpoint_cutoff_setting;
  rc_smoothing_data_.throttleCutoffSetting = throttle_cutoff_setting;
  rc_smoothing_data_.autoSmoothnessFactorSetpoint = autoSmoothnessFactorSetpoint;
  rc_smoothing_data_.autoSmoothnessFactorThrottle = autoSmoothnessFactorThrottle;
  rc_smoothing_data_.enabled = (smoothing_enabled != 0);

  // Initialize cutoff frequencies (will be updated dynamically if setpointCutoffSetting is 0)
  float initial_setpoint_cutoff_hz =
      (setpoint_cutoff_setting > 0.0f) ? setpoint_cutoff_setting : RC_SMOOTHING_CUTOFF_HZ;
  float initial_throttle_cutoff_hz =
      (throttle_cutoff_setting > 0.0f) ? throttle_cutoff_setting : initial_setpoint_cutoff_hz;
  rc_smoothing_data_.setpointCutoffFrequency = initial_setpoint_cutoff_hz;
  rc_smoothing_data_.throttleCutoffFrequency = initial_throttle_cutoff_hz;

  // Initialize pt3Filter for setpoint smoothing
  // filterSetpoint[ROLL/PITCH/YAW] use setpoint cutoff, filterSetpoint[THROTTLE] uses throttle cutoff
  float pt3k_sp = pt3FilterGain(initial_setpoint_cutoff_hz, dt);
  float pt3k_thr = pt3FilterGain(initial_throttle_cutoff_hz, dt);

  for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
    if (i == THROTTLE) {
      pt3FilterInit(&rc_smoothing_data_.filterSetpoint[i], pt3k_thr);
    } else {
      pt3FilterInit(&rc_smoothing_data_.filterSetpoint[i], pt3k_sp);
    }
  }

  // Initialize pt3Filter for RC deflection smoothing (ROLL and PITCH only, used in Horizon mode)
  for (int i = 0; i < RP_AXIS_COUNT; i++) {
    pt3FilterInit(&rc_smoothing_data_.filterRcDeflection[i], pt3k_sp);
  }

  // Initialize pt3Filter for feedforward smoothing (ROLL, PITCH, YAW)
  for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
    pt3FilterInit(&rc_smoothing_data_.filterFeedforward[i], pt3k_sp);
  }

  LOG_I(
      "RC smoothing initialized: enabled=%d, setpoint_cutoff=%.1f, throttle_cutoff=%.1f, auto_factor_rpy=%.1f, "
      "auto_factor_throttle=%.1f",
      rc_smoothing_data_.enabled, setpoint_cutoff_setting, throttle_cutoff_setting, auto_factor_rpy,
      auto_factor_throttle);
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

  // Update setpoint filters for ROLL, PITCH, YAW (use setpoint cutoff)
  for (int i = FD_ROLL; i <= FD_YAW; i++) {
    pt3FilterUpdateCutoff(&rc_smoothing_data_.filterSetpoint[i], pt3k_sp);
  }

  // Update throttle filter (use throttle cutoff)
  pt3FilterUpdateCutoff(&rc_smoothing_data_.filterSetpoint[THROTTLE], pt3k_thr);

  // Update feedforward filters for ROLL, PITCH, YAW (use setpoint cutoff)
  for (int i = FD_ROLL; i <= FD_YAW; i++) {
    pt3FilterUpdateCutoff(&rc_smoothing_data_.filterFeedforward[i], pt3k_sp);
  }

  // Update RC deflection filters for ROLL and PITCH (use setpoint cutoff)
  for (int i = 0; i < RP_AXIS_COUNT; i++) {
    pt3FilterUpdateCutoff(&rc_smoothing_data_.filterRcDeflection[i], pt3k_sp);
  }

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

    // Limit setpoint (match Betaflight: -1.0f * rate_limit to 1.0f * rate_limit)
    rawSetpoint_[axis] =
        constrainf(angle_rate, -1.0f * rate_profile_.rate_limit[axis], 1.0f * rate_profile_.rate_limit[axis]);

    // Calculate feedforward (setpoint delta)
    float setpoint_delta = rawSetpoint_[axis] - prevSetpoint_[axis];
    feedforward_[axis] = setpoint_delta * smoothed_rx_rate_hz_;
    prevSetpoint_[axis] = rawSetpoint_[axis];
  }

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

  // Mark data as processed
  is_rc_data_new_ = false;
}

// PID Task (1-4kHz): Process RC smoothing filter
// Input: rc_setpoint_msg_t from MCN (to avoid data tearing between threads)
// Output: rcCommand[THROTTLE] (in-place update), setpointRate[ROLL/PITCH/YAW] (new array)
void RcBf::processRcSmoothingFilter(const rc_setpoint_msg_t* setpoint_msg) {
  // Use cached data if setpoint_msg is null (for PID thread frequency update)
  const rc_setpoint_msg_t* msg_to_use = setpoint_msg;
  if (msg_to_use == nullptr) {
    msg_to_use = &last_rc_setpoint_msg_;
    // If no cached data yet, skip processing
    if (msg_to_use->seq == 0 && msg_to_use->timestamp == 0) {
      return;
    }
  } else {
    // Cache the new data
    std::memcpy(&last_rc_setpoint_msg_, setpoint_msg, sizeof(rc_setpoint_msg_t));
  }

  if (!rc_smoothing_data_.enabled) {
    // If smoothing is disabled, just copy rawSetpoint to setpointRate
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      setpointRate_[axis] = msg_to_use->rawSetpoint[axis];
    }
    // Update throttle command from MCN data
    rc_command_[THROTTLE] = msg_to_use->rcCommandThrottle;
    // Update feedforward and deflection from MCN data
    std::memcpy(feedforward_, msg_to_use->feedforward, sizeof(feedforward_));
    std::memcpy(rcDeflection_, msg_to_use->rcDeflection, sizeof(rcDeflection_));
    std::memcpy(rcDeflectionAbs_, msg_to_use->rcDeflectionAbs, sizeof(rcDeflectionAbs_));
    return;
  }

  // Prepare data to smooth (from MCN data to avoid data tearing)
  float rx_data_to_smooth[PRIMARY_CHANNEL_COUNT];
  for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
    if (i == THROTTLE) {
      rx_data_to_smooth[i] = msg_to_use->rcCommandThrottle;
    } else {
      rx_data_to_smooth[i] = msg_to_use->rawSetpoint[i];
    }
  }

#ifdef PROJECT_BF_RC_DEBUG_PIN_EN
  DEBUG_PIN_DEBUG1_HIGH();  // Debug pin: processRcSmoothingFilter start (monitor PID task call frequency ~8kHz)
#endif

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

  // Update feedforward and deflection from MCN data
  std::memcpy(feedforward_, msg_to_use->feedforward, sizeof(feedforward_));
  std::memcpy(rcDeflection_, msg_to_use->rcDeflection, sizeof(rcDeflection_));
  std::memcpy(rcDeflectionAbs_, msg_to_use->rcDeflectionAbs, sizeof(rcDeflectionAbs_));

#ifdef PROJECT_BF_RC_DEBUG_PIN_EN
  DEBUG_PIN_DEBUG1_LOW();  // Debug pin: processRcSmoothingFilter end
#endif
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

void RcBf::echoSetpoint(const rc_setpoint_msg_t* setpoint_data) {
  // Print detailed debug information by default
  printDebugInfo(setpoint_data);

  // Always print basic info
  LOG_I("seq: %lu, rawSetpoint: %.2f, %.2f, %.2f, throttle: %.0f, rxRate: %.1f Hz", setpoint_data->seq,
        setpoint_data->rawSetpoint[0], setpoint_data->rawSetpoint[1], setpoint_data->rawSetpoint[2],
        setpoint_data->rcCommandThrottle, setpoint_data->smoothedRxRateHz);
}

void RcBf::printDebugInfo(const rc_setpoint_msg_t* setpoint_msg) const {
  if (setpoint_msg == nullptr) {
    return;
  }

  LOG_I("=== RC Debug Info ===");
  LOG_I("Raw channels: Roll=%d, Pitch=%d, Yaw=%d, Throttle=%d", (int)rc_data_[ROLL], (int)rc_data_[PITCH],
        (int)rc_data_[YAW], (int)rc_data_[THROTTLE]);
  LOG_I("rcCommand: Roll=%.1f, Pitch=%.1f, Yaw=%.1f, Throttle=%.1f", rc_command_[ROLL], rc_command_[PITCH],
        rc_command_[YAW], rc_command_[THROTTLE]);
  LOG_I("Channel Range Config - Roll: [%u, %u], Pitch: [%u, %u], Yaw: [%u, %u], Throttle: [%u, %u]",
        channel_range_configs_[ROLL].min, channel_range_configs_[ROLL].max, channel_range_configs_[PITCH].min,
        channel_range_configs_[PITCH].max, channel_range_configs_[YAW].min, channel_range_configs_[YAW].max,
        channel_range_configs_[THROTTLE].min, channel_range_configs_[THROTTLE].max);
  LOG_I("Dividers: rcCommandDivider=%.1f, rcCommandYawDivider=%.1f (calculated from deadband)", rcCommandDivider_,
        rcCommandYawDivider_);
  LOG_I("Rate Profile - rcRates: [%.1f, %.1f, %.1f], rcExpo: [%.1f, %.1f, %.1f]", rate_profile_.rcRates[FD_ROLL],
        rate_profile_.rcRates[FD_PITCH], rate_profile_.rcRates[FD_YAW], rate_profile_.rcExpo[FD_ROLL],
        rate_profile_.rcExpo[FD_PITCH], rate_profile_.rcExpo[FD_YAW]);
  LOG_I("Rate Profile - rates (super): [%.1f, %.1f, %.1f], rate_limit: [%.1f, %.1f, %.1f]",
        rate_profile_.rates[FD_ROLL], rate_profile_.rates[FD_PITCH], rate_profile_.rates[FD_YAW],
        rate_profile_.rate_limit[FD_ROLL], rate_profile_.rate_limit[FD_PITCH], rate_profile_.rate_limit[FD_YAW]);
  LOG_I("Note: Base angle_rate = 200.0 * rc_rate * rc_commandf");
  LOG_I("      - 200.0 is a FIXED constant from Betaflight (not adjustable)");
  LOG_I("      - To get higher rates, increase rcRates parameter (default 100)");
  LOG_I("      - Example: rcRates=360 gives max angle_rate = 200 * 3.6 * 1.0 = 720 deg/s");

  // Print detailed calculation for each axis
  const char* axis_names[] = {"Roll", "Pitch", "Yaw"};
  const int channel_idx[] = {ROLL, PITCH, YAW};

  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    LOG_I("--- %s Axis ---", axis_names[axis]);
    LOG_I("  rc_data[%s]=%.1f (raw input, mapped from [%u, %u] -> [1000, 2000])", axis_names[axis],
          rc_data_[channel_idx[axis]], channel_range_configs_[channel_idx[axis]].min,
          channel_range_configs_[channel_idx[axis]].max);
    LOG_I("  rc_command[%s]=%.1f (after deadband, should be in [1000, 2000])", axis_names[axis],
          rc_command_[channel_idx[axis]]);

    // Calculate rc_commandf (same as in processRcCommand)
    float rc_commandf = 0.0f;
    if (axis == FD_ROLL) {
      rc_commandf = (rc_command_[ROLL] - PWM_RANGE_MIDDLE) / rcCommandDivider_;
    } else if (axis == FD_PITCH) {
      rc_commandf = (rc_command_[PITCH] - PWM_RANGE_MIDDLE) / rcCommandDivider_;
    } else if (axis == FD_YAW) {
      rc_commandf = (rc_command_[YAW] - PWM_RANGE_MIDDLE) / rcCommandYawDivider_;
    }
    rc_commandf = constrainf(rc_commandf, -1.0f, 1.0f);
    float rc_commandf_abs = std::abs(rc_commandf);

    // Calculate theoretical max rc_commandf based on channel range
    float max_possible_rc = std::max((channel_range_configs_[channel_idx[axis]].max - PWM_RANGE_MIDDLE) /
                                         (axis == FD_YAW ? rcCommandYawDivider_ : rcCommandDivider_),
                                     std::abs((channel_range_configs_[channel_idx[axis]].min - PWM_RANGE_MIDDLE) /
                                              (axis == FD_YAW ? rcCommandYawDivider_ : rcCommandDivider_)));

    LOG_I("  rc_commandf=%.4f (normalized from rc_command, range: [-1.0, 1.0], abs=%.4f)", rc_commandf,
          rc_commandf_abs);
    LOG_I("    -> Max possible rc_commandf with current range = %.4f (should be ~1.0 for full range)", max_possible_rc);

    // Recalculate angle_rate to show before limit (same calculation as in processRcCommand)
    float angle_rate = applyBetaflightRates(axis, rc_commandf, rc_commandf_abs);
    float angle_rate_limited =
        constrainf(angle_rate, -1.0f * rate_profile_.rate_limit[axis], 1.0f * rate_profile_.rate_limit[axis]);

    LOG_I("  angle_rate=%.2f deg/s (before limit, calculated)", angle_rate);
    LOG_I("  rate_limit=%.1f deg/s", rate_profile_.rate_limit[axis]);
    LOG_I("  rawSetpoint=%.2f deg/s (final, limited by rate_limit, from msg)", setpoint_msg->rawSetpoint[axis]);

    // Show if angle_rate was limited
    if (std::abs(angle_rate) > rate_profile_.rate_limit[axis]) {
      LOG_I("    -> angle_rate was limited from %.2f to %.2f deg/s", angle_rate, angle_rate_limited);
    }

    // Warning if rc_commandf is not reaching full range
    if (rc_commandf_abs < 0.95f) {
      LOG_W("  WARNING: rc_commandf (%.4f) is not reaching ±1.0! Channel range needs calibration.", rc_commandf);
      LOG_W("  Problem: rc_data[%s]=%.1f is not mapped to full [1000, 2000] range", axis_names[axis],
            rc_data_[channel_idx[axis]]);
      LOG_W("  Solution: Set rc_channel_range_%s parameter to actual min/max values",
            (axis == FD_ROLL ? "roll" : (axis == FD_PITCH ? "pitch" : "yaw")));
      LOG_W("            Example: If roll channel range is [1068, 1932], set: par set 27 0 1068 1932");
      LOG_W("            This will map [1068, 1932] -> [1000, 2000], allowing rc_commandf to reach ±1.0");
    }
  }
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

#ifdef PROJECT_BF_RC_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG2_HIGH();  // Debug pin: RC task execution start (monitor RC task frequency ~100-200Hz)
#endif
    instance->applyRangeScaling();
    instance->applyFailsafeAndConstraints(current_time_us);
    instance->updateRcCommands();  // rcData[] → rcCommand[]
    
    // Update refresh rate
    instance->updateRcRefreshRate(current_time_us, instance->rx_receiving_signal_);

    // Process RC command and publish to MCN (in RC thread)
    // This calculates rawSetpoint[] from rcCommand[] and publishes to MCN for PID thread
    instance->processRcCommand(current_time_us);

#ifdef PROJECT_BF_RC_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG2_LOW();  // Debug pin: RC task execution end
#endif

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

