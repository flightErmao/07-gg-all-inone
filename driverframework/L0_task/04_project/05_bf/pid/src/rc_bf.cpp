#include "rc_bf.hpp"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "rc_bf"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "pid_setpoint_msg.h"
#include "param.h"
}

#include <cmath>
#include <cstring>

#define CLAMPF(x, min, max) ((x) < (min) ? (min) : (((x) > (max)) ? (max) : (x)))
#define RC_RATE_INCREMENTAL 14.54f
#define RX_INTERVAL_MIN_US 800
#define RX_INTERVAL_MAX_US 65500

// Helper functions
namespace {
float power3(float x) {
  return x * x * x;
}

float constrainf(float x, float min, float max) {
  return CLAMPF(x, min, max);
}
}  // namespace

MCN_DECLARE(rc);

RcBf& RcBf::instance() {
  static RcBf instance_obj;
  return instance_obj;
}

RcBf::RcBf()
    : rc_data_ready_(false),
      last_rc_time_us_(0),
      current_rx_interval_us_(0),
      current_rx_rate_hz_(100.0f),
      smoothed_rx_rate_hz_(100.0f),
      rc_event_(RT_NULL),
      rc_node_(RT_NULL),
      pid_setpoint_hub_(nullptr),
      seq_(0) {
  std::memset(&rc_command_, 0, sizeof(rc_command_));
  std::memset(rawSetpoint_, 0, sizeof(rawSetpoint_));
  std::memset(rcDeflection_, 0, sizeof(rcDeflection_));
  std::memset(rcDeflectionAbs_, 0, sizeof(rcDeflectionAbs_));
  std::memset(maxRcRate_, 0, sizeof(maxRcRate_));
  std::memset(feedforward_, 0, sizeof(feedforward_));
  std::memset(prevSetpoint_, 0, sizeof(prevSetpoint_));
  std::memset(&rate_profile_, 0, sizeof(rate_profile_));
}

RcBf::~RcBf() = default;

rt_err_t RcBf::init() {
  // Create event semaphore
  if (rc_event_ == RT_NULL) {
    rc_event_ = rt_sem_create("rc_bf_evt", 0, RT_IPC_FLAG_FIFO);
    if (rc_event_ == RT_NULL) {
      LOG_E("create rc_bf event semaphore failed");
      return -RT_ERROR;
    }
  }

  // Subscribe to RC MCN topic
  rc_node_ = mcn_subscribe(MCN_HUB(rc), rc_event_, RT_NULL);
  if (rc_node_ == RT_NULL) {
    LOG_E("subscribe rc topic failed");
    if (rc_event_ != RT_NULL) {
      rt_sem_delete(rc_event_);
      rc_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }
  LOG_I("Subscribed to rc MCN topic");

  // Get pid_setpoint hub for publishing
  pid_setpoint_hub_ = MCN_HUB(pid_setpoint);
  if (pid_setpoint_hub_ == nullptr) {
    LOG_E("get pid_setpoint hub failed");
    if (rc_node_ != RT_NULL) {
      mcn_unsubscribe(MCN_HUB(rc), rc_node_);
      rc_node_ = RT_NULL;
    }
    if (rc_event_ != RT_NULL) {
      rt_sem_delete(rc_event_);
      rc_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }

  initRateProfile();

  LOG_I("RcBf initialized");
  return RT_EOK;
}

void RcBf::initRateProfile() {
  // Default Betaflight-style rates
  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    rate_profile_.rcRates[axis] = 100.0f;  // Default RC rate
    rate_profile_.rcExpo[axis] = 0.0f;     // No expo by default
    rate_profile_.rates[axis] = 0.0f;      // No super rate by default
    rate_profile_.rate_limit[axis] = 720.0f; // Default 720 deg/s limit
  }

  // Load RC rates from parameters
  float rc_rate_roll = rate_profile_.rcRates[FD_ROLL];
  if (getParam("rc_rate_roll", &rc_rate_roll, sizeof(rc_rate_roll)) == RT_EOK) {
    rate_profile_.rcRates[FD_ROLL] = rc_rate_roll;
    LOG_I("Loaded rc_rate_roll: %.1f", rc_rate_roll);
  }

  float rc_rate_pitch = rate_profile_.rcRates[FD_PITCH];
  if (getParam("rc_rate_pitch", &rc_rate_pitch, sizeof(rc_rate_pitch)) == RT_EOK) {
    rate_profile_.rcRates[FD_PITCH] = rc_rate_pitch;
    LOG_I("Loaded rc_rate_pitch: %.1f", rc_rate_pitch);
  }

  float rc_rate_yaw = rate_profile_.rcRates[FD_YAW];
  if (getParam("rc_rate_yaw", &rc_rate_yaw, sizeof(rc_rate_yaw)) == RT_EOK) {
    rate_profile_.rcRates[FD_YAW] = rc_rate_yaw;
    LOG_I("Loaded rc_rate_yaw: %.1f", rc_rate_yaw);
  }

  // Load RC expo from parameters
  float rc_expo_roll = rate_profile_.rcExpo[FD_ROLL];
  if (getParam("rc_expo_roll", &rc_expo_roll, sizeof(rc_expo_roll)) == RT_EOK) {
    rate_profile_.rcExpo[FD_ROLL] = rc_expo_roll;
    LOG_I("Loaded rc_expo_roll: %.1f", rc_expo_roll);
  }

  float rc_expo_pitch = rate_profile_.rcExpo[FD_PITCH];
  if (getParam("rc_expo_pitch", &rc_expo_pitch, sizeof(rc_expo_pitch)) == RT_EOK) {
    rate_profile_.rcExpo[FD_PITCH] = rc_expo_pitch;
    LOG_I("Loaded rc_expo_pitch: %.1f", rc_expo_pitch);
  }

  float rc_expo_yaw = rate_profile_.rcExpo[FD_YAW];
  if (getParam("rc_expo_yaw", &rc_expo_yaw, sizeof(rc_expo_yaw)) == RT_EOK) {
    rate_profile_.rcExpo[FD_YAW] = rc_expo_yaw;
    LOG_I("Loaded rc_expo_yaw: %.1f", rc_expo_yaw);
  }

  // Load super rates from parameters
  float rc_super_rate_roll = rate_profile_.rates[FD_ROLL];
  if (getParam("rc_super_rate_roll", &rc_super_rate_roll, sizeof(rc_super_rate_roll)) == RT_EOK) {
    rate_profile_.rates[FD_ROLL] = rc_super_rate_roll;
    LOG_I("Loaded rc_super_rate_roll: %.1f", rc_super_rate_roll);
  }

  float rc_super_rate_pitch = rate_profile_.rates[FD_PITCH];
  if (getParam("rc_super_rate_pitch", &rc_super_rate_pitch, sizeof(rc_super_rate_pitch)) == RT_EOK) {
    rate_profile_.rates[FD_PITCH] = rc_super_rate_pitch;
    LOG_I("Loaded rc_super_rate_pitch: %.1f", rc_super_rate_pitch);
  }

  float rc_super_rate_yaw = rate_profile_.rates[FD_YAW];
  if (getParam("rc_super_rate_yaw", &rc_super_rate_yaw, sizeof(rc_super_rate_yaw)) == RT_EOK) {
    rate_profile_.rates[FD_YAW] = rc_super_rate_yaw;
    LOG_I("Loaded rc_super_rate_yaw: %.1f", rc_super_rate_yaw);
  }

  // Load rate limits from parameters
  float rc_rate_limit_roll = rate_profile_.rate_limit[FD_ROLL];
  if (getParam("rc_rate_limit_roll", &rc_rate_limit_roll, sizeof(rc_rate_limit_roll)) == RT_EOK) {
    rate_profile_.rate_limit[FD_ROLL] = rc_rate_limit_roll;
    LOG_I("Loaded rc_rate_limit_roll: %.1f", rc_rate_limit_roll);
  }

  float rc_rate_limit_pitch = rate_profile_.rate_limit[FD_PITCH];
  if (getParam("rc_rate_limit_pitch", &rc_rate_limit_pitch, sizeof(rc_rate_limit_pitch)) == RT_EOK) {
    rate_profile_.rate_limit[FD_PITCH] = rc_rate_limit_pitch;
    LOG_I("Loaded rc_rate_limit_pitch: %.1f", rc_rate_limit_pitch);
  }

  float rc_rate_limit_yaw = rate_profile_.rate_limit[FD_YAW];
  if (getParam("rc_rate_limit_yaw", &rc_rate_limit_yaw, sizeof(rc_rate_limit_yaw)) == RT_EOK) {
    rate_profile_.rate_limit[FD_YAW] = rc_rate_limit_yaw;
    LOG_I("Loaded rc_rate_limit_yaw: %.1f", rc_rate_limit_yaw);
  }

  // Calculate max RC rates for each axis
  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    maxRcRate_[axis] = applyBetaflightRates(axis, 1.0f, 1.0f);
  }

  LOG_I("Rate profile initialized - Roll: %.1f/%.1f/%.1f/%.1f, Pitch: %.1f/%.1f/%.1f/%.1f, Yaw: %.1f/%.1f/%.1f/%.1f",
        rate_profile_.rcRates[FD_ROLL], rate_profile_.rcExpo[FD_ROLL], 
        rate_profile_.rates[FD_ROLL], rate_profile_.rate_limit[FD_ROLL],
        rate_profile_.rcRates[FD_PITCH], rate_profile_.rcExpo[FD_PITCH],
        rate_profile_.rates[FD_PITCH], rate_profile_.rate_limit[FD_PITCH],
        rate_profile_.rcRates[FD_YAW], rate_profile_.rcExpo[FD_YAW],
        rate_profile_.rates[FD_YAW], rate_profile_.rate_limit[FD_YAW]);
}

float RcBf::applyBetaflightRates(int axis, float rcCommandf, float rcCommandfAbs) {
  // Apply expo
  if (rate_profile_.rcExpo[axis] > 0.0f) {
    const float expof = rate_profile_.rcExpo[axis] / 100.0f;
    rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1.0f - expof);
  }

  // Calculate RC rate
  float rcRate = rate_profile_.rcRates[axis] / 100.0f;
  if (rcRate > 2.0f) {
    rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
  }

  // Calculate angle rate
  float angleRate = 200.0f * rcRate * rcCommandf;

  // Apply super rate
  if (rate_profile_.rates[axis] > 0.0f) {
    const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (rate_profile_.rates[axis] / 100.0f)), 0.01f, 1.00f));
    angleRate *= rcSuperfactor;
  }

  return angleRate;
}

void RcBf::updateRcRefreshRate(uint32_t current_time_us) {
  if (last_rc_time_us_ > 0) {
    uint32_t delta = current_time_us - last_rc_time_us_;
    current_rx_interval_us_ = CLAMPF(delta, RX_INTERVAL_MIN_US, RX_INTERVAL_MAX_US);
    current_rx_rate_hz_ = 1e6f / current_rx_interval_us_;

    // Smooth the rate
    const float smoothing_factor = 0.1f;
    smoothed_rx_rate_hz_ += smoothing_factor * (current_rx_rate_hz_ - smoothed_rx_rate_hz_);
  }
  last_rc_time_us_ = current_time_us;
}

void RcBf::processStickInput(uint32_t current_time_us) {
  if (!rc_data_ready_) {
    return;
  }

  updateRcRefreshRate(current_time_us);

  // Convert stick inputs from -1.0 to 1.0 range to setpoint rates
  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    float rcCommandf = 0.0f;

    // Map axis to stick inputs
    if (axis == FD_ROLL) {
      rcCommandf = rc_command_.stick_roll;
    } else if (axis == FD_PITCH) {
      rcCommandf = rc_command_.stick_pitch;
    } else if (axis == FD_YAW) {
      rcCommandf = rc_command_.stick_yaw;
    }

    // Clamp to valid range
    rcCommandf = constrainf(rcCommandf, -1.0f, 1.0f);
    rcDeflection_[axis] = rcCommandf;
    const float rcCommandfAbs = std::abs(rcCommandf);
    rcDeflectionAbs_[axis] = rcCommandfAbs;

    // Apply rates to get angle rate
    float angleRate = applyBetaflightRates(axis, rcCommandf, rcCommandfAbs);

    // Limit setpoint
    rawSetpoint_[axis] = constrainf(angleRate, -rate_profile_.rate_limit[axis], rate_profile_.rate_limit[axis]);

    // Calculate feedforward (setpoint delta)
    float setpointDelta = rawSetpoint_[axis] - prevSetpoint_[axis];
    feedforward_[axis] = setpointDelta * smoothed_rx_rate_hz_;
    prevSetpoint_[axis] = rawSetpoint_[axis];
  }

  rc_data_ready_ = false;
}

void RcBf::processRcCommand(uint32_t current_time_us) {
  // Poll for new RC data
  if (rc_node_ != RT_NULL && mcn_poll_sync(rc_node_, 0) == RT_TRUE) {
    if (mcn_copy(MCN_HUB(rc), rc_node_, &rc_command_) == RT_EOK) {
      rc_data_ready_ = true;
    }
  }

  // Process stick input to generate setpoint
  processStickInput(current_time_us);

  // Publish setpoint message
  if (pid_setpoint_hub_ != nullptr) {
    pid_setpoint_msg_t setpoint_msg;
    setpoint_msg.timestamp = current_time_us;
    setpoint_msg.seq = seq_++;

    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      setpoint_msg.rate[axis] = rawSetpoint_[axis];
      setpoint_msg.feedforward[axis] = feedforward_[axis];
    }

    mcn_publish(pid_setpoint_hub_, &setpoint_msg);
  }
}

float RcBf::getSetpointRate(int axis) {
  if (axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return rawSetpoint_[axis];
  }
  return 0.0f;
}

float RcBf::getFeedforward(int axis) {
  if (axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return feedforward_[axis];
  }
  return 0.0f;
}

float RcBf::getMaxRcRate(int axis) {
  if (axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return maxRcRate_[axis];
  }
  return 720.0f;
}

// RT-Thread auto initialization wrapper
#ifdef PROJECT_BF_PID_EN
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

