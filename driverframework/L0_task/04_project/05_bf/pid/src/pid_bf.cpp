#include "pid_bf.hpp"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "pid_bf"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "pid_setpoint_msg.h"
#include "pid_output_msg.h"
#include "bfPidParam.h"
}

#include <cmath>
#include <cstring>

// PID constants from ref/pid.h
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f
#define FEEDFORWARD_SCALE 0.013754f
#define PIDSUM_LIMIT 500.0f
#define PIDSUM_LIMIT_YAW 400.0f

// Helper macros
#define CLAMPF(x, min, max) ((x) < (min) ? (min) : (((x) > (max)) ? (max) : (x)))

namespace {

constexpr float PI_F = 3.14159265358979323846f;

void lowpassInit(SimpleLowpass* filter, float cutoff_hz, float dt) {
  if (!filter) {
    return;
  }
  if (cutoff_hz <= 0.0f || dt <= 0.0f) {
    filter->enabled = false;
    filter->alpha = 1.0f;
    filter->state = 0.0f;
    return;
  }
  const float rc = 1.0f / (2.0f * PI_F * cutoff_hz);
  filter->alpha = dt / (rc + dt);
  filter->enabled = true;
  filter->state = 0.0f;
}

float lowpassApply(SimpleLowpass* filter, float input) {
  if (!filter || !filter->enabled) {
    if (filter) {
      filter->state = input;
    }
    return input;
  }
  filter->state += filter->alpha * (input - filter->state);
  return filter->state;
}

}  // namespace

/* 定义 PID 消息 MCN */
MCN_DEFINE(pid_setpoint, sizeof(pid_setpoint_msg_t));
MCN_DEFINE(pid_output, sizeof(pid_output_msg_t));

// Thread configuration removed - using taskPid.cpp main thread instead

PidBf& PidBf::instance() {
  static PidBf instance_obj;
  return instance_obj;
}

PidBf::PidBf()
    : gyro_filtered_event_(RT_NULL),
      gyro_filtered_node_(RT_NULL),
      setpoint_event_(RT_NULL),
      setpoint_node_(RT_NULL),
      pid_output_hub_(nullptr),
      gyro_data_ready_(false),
      setpoint_data_ready_(false),
      rc_smoothing_filter_(nullptr),
      target_looptime_us_(0) {
  std::memset(&gyro_filtered_data_, 0, sizeof(gyro_filtered_data_));
  std::memset(&setpoint_data_, 0, sizeof(setpoint_data_));
  std::memset(&pid_runtime_, 0, sizeof(pid_runtime_));
  std::memset(pid_data_, 0, sizeof(pid_data_));
  std::memset(&pid_profile_, 0, sizeof(pid_profile_));

  pid_runtime_.pidStabilisationEnabled = true;
  pid_runtime_.dT = 0.0f;
  pid_runtime_.pidFrequency = 0.0f;
  for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
    pid_runtime_.previousPidSetpoint[axis] = 0.0f;
    dterm_lpf_[axis].state = 0.0f;
    dterm_lpf_[axis].alpha = 1.0f;
    dterm_lpf_[axis].enabled = false;
  }
  yaw_pterm_lpf_.state = 0.0f;
  yaw_pterm_lpf_.alpha = 1.0f;
  yaw_pterm_lpf_.enabled = false;

  pid_profile_.pidSumLimit = PIDSUM_LIMIT;
  pid_profile_.pidSumLimitYaw = PIDSUM_LIMIT_YAW;
  pid_profile_.itermWindup = 80.0f;
  pid_profile_.dtermLpfHz = 120.0f;
  pid_profile_.yawLpfHz = 90.0f;
}

PidBf::~PidBf() = default;

rt_err_t PidBf::init() {
  // Note: Thread initialization removed - using taskPid.cpp main thread instead

  if (gyro_filtered_event_ == RT_NULL) {
    gyro_filtered_event_ = rt_sem_create("pid_gyro_evt", 0, RT_IPC_FLAG_FIFO);
    if (gyro_filtered_event_ == RT_NULL) {
      LOG_E("create gyro_filtered event semaphore failed");
      return -RT_ERROR;
    }
  }

  gyro_filtered_node_ = mcn_subscribe(MCN_HUB(gyro), gyro_filtered_event_, RT_NULL);
  if (gyro_filtered_node_ == RT_NULL) {
    LOG_E("subscribe gyro topic failed");
    if (gyro_filtered_event_ != RT_NULL) {
      rt_sem_delete(gyro_filtered_event_);
      gyro_filtered_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }
  LOG_I("Subscribed to gyro MCN topic");

  // Note: setpoint data is now directly written by processRcSmoothingFilter() in subTaskRcCommand()
  // No need to subscribe to pid_setpoint MCN topic anymore
  setpoint_node_ = RT_NULL;
  setpoint_event_ = RT_NULL;

  pid_output_hub_ = MCN_HUB(pid_output);
  if (pid_output_hub_ == nullptr) {
    LOG_E("get pid_output hub failed");
    if (gyro_filtered_node_ != RT_NULL) {
      mcn_unsubscribe(MCN_HUB(gyro), gyro_filtered_node_);
      gyro_filtered_node_ = RT_NULL;
    }
    if (gyro_filtered_event_ != RT_NULL) {
      rt_sem_delete(gyro_filtered_event_);
      gyro_filtered_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }

  uint8_t pid_process_denom = 1;
  if (getParam("pid_process_denom", &pid_process_denom, sizeof(pid_process_denom)) != RT_EOK) {
    pid_process_denom = 1;
  }

  target_looptime_us_ = 125;  // 默认 8kHz
  if (pid_process_denom > 1) {
    target_looptime_us_ *= pid_process_denom;
  }

  pid_runtime_.dT = target_looptime_us_ * 1e-6f;
  pid_runtime_.pidFrequency = (pid_runtime_.dT > 0.0f) ? 1.0f / pid_runtime_.dT : 0.0f;

  LOG_I("Target looptime: %u us, dT: %.6f s, frequency: %.1f Hz", target_looptime_us_, pid_runtime_.dT,
        pid_runtime_.pidFrequency);

  initConfig();
  initFilters();

  LOG_I("PidBf initialized (no thread - using taskPid.cpp main thread)");
  return RT_EOK;
}

void PidBf::initConfig() {
  auto loadAxis = [&](int axis, const float* src) {
    pid_profile_.pid[axis].P = src[0];
    pid_profile_.pid[axis].I = src[1];
    pid_profile_.pid[axis].D = src[2];
    pid_profile_.pid[axis].F = 0.0f;
    pid_profile_.pid[axis].S = 0.0f;
  };

  float pid_rate_roll[3] = {45.0f, 80.0f, 30.0f};
  if (getParam("pid_rate_roll", pid_rate_roll, sizeof(pid_rate_roll)) != RT_EOK) {
    LOG_W("Using default PID Roll values");
  } else {
    LOG_I("PID Roll: P=%.1f, I=%.1f, D=%.1f", pid_rate_roll[0], pid_rate_roll[1], pid_rate_roll[2]);
  }
  loadAxis(FD_ROLL, pid_rate_roll);

  float pid_rate_pitch[3] = {47.0f, 84.0f, 34.0f};
  if (getParam("pid_rate_pitch", pid_rate_pitch, sizeof(pid_rate_pitch)) != RT_EOK) {
    LOG_W("Using default PID Pitch values");
  } else {
    LOG_I("PID Pitch: P=%.1f, I=%.1f, D=%.1f", pid_rate_pitch[0], pid_rate_pitch[1], pid_rate_pitch[2]);
  }
  loadAxis(FD_PITCH, pid_rate_pitch);

  float pid_rate_yaw[3] = {45.0f, 80.0f, 0.0f};
  if (getParam("pid_rate_yaw", pid_rate_yaw, sizeof(pid_rate_yaw)) != RT_EOK) {
    LOG_W("Using default PID Yaw values");
  } else {
    LOG_I("PID Yaw: P=%.1f, I=%.1f, D=%.1f", pid_rate_yaw[0], pid_rate_yaw[1], pid_rate_yaw[2]);
  }
  loadAxis(FD_YAW, pid_rate_yaw);

  float pid_rate_roll_i_limit = pid_profile_.pidSumLimit;
  if (getParam("pid_rate_roll_i_limit", &pid_rate_roll_i_limit, sizeof(pid_rate_roll_i_limit)) == RT_EOK) {
    pid_profile_.pidSumLimit = pid_rate_roll_i_limit;
  }
  float pid_rate_yaw_i_limit = pid_profile_.pidSumLimitYaw;
  if (getParam("pid_rate_yaw_i_limit", &pid_rate_yaw_i_limit, sizeof(pid_rate_yaw_i_limit)) == RT_EOK) {
    pid_profile_.pidSumLimitYaw = pid_rate_yaw_i_limit;
  }

  float iterm_windup = pid_profile_.itermWindup;
  if (getParam("pid_iterm_windup", &iterm_windup, sizeof(iterm_windup)) == RT_EOK) {
    pid_profile_.itermWindup = iterm_windup;
  }

  float dterm_lpf_hz = pid_profile_.dtermLpfHz;
  if (getParam("pid_dterm_lpf_hz", &dterm_lpf_hz, sizeof(dterm_lpf_hz)) == RT_EOK) {
    pid_profile_.dtermLpfHz = dterm_lpf_hz;
  }
  float yaw_lpf_hz = pid_profile_.yawLpfHz;
  if (getParam("pid_yaw_lpf_hz", &yaw_lpf_hz, sizeof(yaw_lpf_hz)) == RT_EOK) {
    pid_profile_.yawLpfHz = yaw_lpf_hz;
  }

  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    const pidf_t& pidf = pid_profile_.pid[axis];
    pid_runtime_.pidCoefficient[axis].Kp = pidf.P * PTERM_SCALE;
    pid_runtime_.pidCoefficient[axis].Ki = pidf.I * ITERM_SCALE;
    pid_runtime_.pidCoefficient[axis].Kd = pidf.D * DTERM_SCALE;
    pid_runtime_.pidCoefficient[axis].Kf = pidf.F * FEEDFORWARD_SCALE * 0.01f;
  }

  pid_runtime_.itermLimit = 0.01f * pid_profile_.itermWindup * pid_profile_.pidSumLimit;
  pid_runtime_.itermLimitYaw = 0.01f * pid_profile_.itermWindup * pid_profile_.pidSumLimitYaw;

  LOG_I("PID Sum Limits: Roll/Pitch=%.1f, Yaw=%.1f, I windup=%.1f%%", pid_profile_.pidSumLimit,
        pid_profile_.pidSumLimitYaw, pid_profile_.itermWindup);
}

void PidBf::initFilters() {
  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    lowpassInit(&dterm_lpf_[axis], pid_profile_.dtermLpfHz, pid_runtime_.dT);
  }
  lowpassInit(&yaw_pterm_lpf_, pid_profile_.yawLpfHz, pid_runtime_.dT);

  LOG_I("PID filters initialized: dterm LPF=%.1f Hz, yaw LPF=%.1f Hz", pid_profile_.dtermLpfHz, pid_profile_.yawLpfHz);
}

// Thread entry and loop functions removed - logic moved to processPidController()
// This function is called from subTaskPidController in taskPid.cpp

void PidBf::processPidController(uint32_t current_time_us) {
  // Poll for new gyro data (non-blocking)
  if (mcn_poll_sync(gyro_filtered_node_, RT_WAITING_FOREVER) == RT_TRUE) {
    if (mcn_copy(MCN_HUB(gyro), gyro_filtered_node_, &gyro_filtered_data_) == RT_EOK) {
      gyro_data_ready_ = true;
    }
  }

  // Note: setpoint data is now directly written by processRcSmoothingFilter() in subTaskRcCommand()
  // No need to subscribe to pid_setpoint MCN topic anymore
  // setpoint_data_ready_ is set to true by subTaskRcCommand() after calling processRcSmoothingFilter()

  // Process PID controller if gyro data is ready and setpoint data is ready
  if (gyro_data_ready_ && setpoint_data_ready_) {
    pidController(current_time_us);
    gyro_data_ready_ = false;
    setpoint_data_ready_ = false;  // Reset flag, will be set again by subTaskRcCommand()
  }
}

void PidBf::pidController(uint32_t current_time_us) {
  static float previousGyroRateDterm[XYZ_AXIS_COUNT] = {0.0f};

  if (!pid_runtime_.pidStabilisationEnabled) {
    return;
  }

  float gyroRate[XYZ_AXIS_COUNT];
  std::memcpy(gyroRate, gyro_filtered_data_.gyro_filtered, sizeof(gyroRate));

  pid_output_msg_t output_msg;
  output_msg.timestamp = current_time_us;
  output_msg.seq = gyro_filtered_data_.seq;

  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    const float currentSetpoint = getSetpointRate(axis);
    const float errorRate = currentSetpoint - gyroRate[axis];

    float pTerm = pid_runtime_.pidCoefficient[axis].Kp * errorRate;
    if (axis == FD_YAW) {
      pTerm = lowpassApply(&yaw_pterm_lpf_, pTerm);
    }
    pid_data_[axis].P = pTerm;

    const float itermLimit = (axis == FD_YAW) ? pid_runtime_.itermLimitYaw : pid_runtime_.itermLimit;
    const float iTermChange = pid_runtime_.pidCoefficient[axis].Ki * pid_runtime_.dT * errorRate;
    pid_data_[axis].I = CLAMPF(pid_data_[axis].I + iTermChange, -itermLimit, itermLimit);

    float gyroForD = lowpassApply(&dterm_lpf_[axis], gyroRate[axis]);
    float delta = -(gyroForD - previousGyroRateDterm[axis]) * pid_runtime_.pidFrequency;
    previousGyroRateDterm[axis] = gyroForD;
    pid_data_[axis].D = pid_runtime_.pidCoefficient[axis].Kd * delta;

    float pidSetpointDelta = (currentSetpoint - pid_runtime_.previousPidSetpoint[axis]) * pid_runtime_.pidFrequency;
    pid_runtime_.previousPidSetpoint[axis] = currentSetpoint;

    const float ffFromProfile = pid_runtime_.pidCoefficient[axis].Kf * pidSetpointDelta;
    const float ffFromMessage = FEEDFORWARD_SCALE * getFeedforward(axis);
    pid_data_[axis].F = ffFromProfile + ffFromMessage;
    pid_data_[axis].S = 0.0f;

    const float pidSumLimit = (axis == FD_YAW) ? pid_profile_.pidSumLimitYaw : pid_profile_.pidSumLimit;
    pid_data_[axis].Sum =
        CLAMPF(pid_data_[axis].P + pid_data_[axis].I + pid_data_[axis].D + pid_data_[axis].F, -pidSumLimit, pidSumLimit);

    output_msg.pid_sum[axis] = pid_data_[axis].Sum;
    output_msg.pid_p[axis] = pid_data_[axis].P;
    output_msg.pid_i[axis] = pid_data_[axis].I;
    output_msg.pid_d[axis] = pid_data_[axis].D;
    output_msg.pid_f[axis] = pid_data_[axis].F;
  }

  if (pid_output_hub_ != nullptr) {
    mcn_publish(pid_output_hub_, &output_msg);
  }
}

float PidBf::getSetpointRate(int axis) {
  if (setpoint_data_ready_ && axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return setpoint_data_.rate[axis];
  }
  return 0.0f;
}

float PidBf::getFeedforward(int axis) {
  if (setpoint_data_ready_ && axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return setpoint_data_.feedforward[axis];
  }
  return 0.0f;
}

float PidBf::getMaxRcRate(int axis) {
  (void)axis;
  return 720.0f;
}

// RT-Thread 自动初始化包装函数
// NOTE: Disabled - using taskPid.cpp main thread instead
// #ifdef PROJECT_BF_PID_EN
// extern "C" {
// static int pid_bf_init_wrapper(void) {
//   PidBf& instance = PidBf::instance();
//   rt_err_t ret = instance.init();
//   if (ret == RT_EOK) {
//     LOG_I("PidBf auto-init success");
//   } else {
//     LOG_E("PidBf auto-init failed: %d", ret);
//   }
//   return (int)ret;
// }
// INIT_APP_EXPORT(pid_bf_init_wrapper);
// }
// #endif
#
