#include "pid_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "pid_bf"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "gyro_mcn.h"
#include "pid_mcn.h"
#include "bfPidParam.h"
#include "rc_mcn.h"  // For MCN_DECLARE(rc), rc_command_msg_t, rc_aux_msg_t
#ifdef PROJECT_BF_PID_DEBUG_PIN_EN
#include "debugPin.h"
#endif
}

#include "rc_smooth.h"  // For RcSmoothingFilter

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

/* PID 消息 MCN 定义已移动到 pid_mcn.cpp */

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
      rc_aux_node_(RT_NULL),
      rc_command_node_(RT_NULL),
      rc_command_data_valid_(false),
      rc_smoothing_filter_(nullptr),
      target_looptime_us_(0),
      main_thread_(RT_NULL),
      main_thread_inited_(false) {
  std::memset(&gyro_filtered_data_, 0, sizeof(gyro_filtered_data_));
  std::memset(&setpoint_data_, 0, sizeof(setpoint_data_));
  std::memset(&rc_command_data_, 0, sizeof(rc_command_data_));
  std::memset(&rc_command_data_cached_, 0, sizeof(rc_command_data_cached_));
  std::memset(&pid_runtime_, 0, sizeof(pid_runtime_));
  std::memset(pid_data_, 0, sizeof(pid_data_));
  std::memset(&pid_profile_, 0, sizeof(pid_profile_));
  std::memset(&main_thread_obj_, 0, sizeof(main_thread_obj_));
  std::memset(main_thread_stack_, 0, sizeof(main_thread_stack_));

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

PidBf::~PidBf() { cleanupMcnSubscriptions(); }

rt_err_t PidBf::init() {
  // Note: Thread initialization removed - using taskPid.cpp main thread instead

  // 初始化MCN订阅
  rt_err_t ret = initMcnSubscriptions();
  if (ret != RT_EOK) {
    LOG_E("MCN subscriptions initialization failed");
    return ret;
  }

  // Note: setpoint data is now directly written by processRcSmoothingFilter() in subTaskRcCommand()
  // No need to subscribe to pid_setpoint MCN topic anymore
  setpoint_node_ = RT_NULL;
  setpoint_event_ = RT_NULL;

  uint8_t pid_process_denom = 1;
  if (getParam("pid_process_denom", &pid_process_denom, sizeof(pid_process_denom)) != RT_EOK) {
    pid_process_denom = 1;
  }

  target_looptime_us_ = 312;  // 默认 3.2kHz
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

  // Load PID sum limit (限制 P+I+D+F 的总和，单位：deg/s)
  // 注意：这不是I项的单独限制，而是整个PID输出的限制
  // I项的windup限制会基于此值计算：itermLimit = pidSumLimit * itermWindup / 100
  float pid_sum_limit = pid_profile_.pidSumLimit;
  if (getParam("pid_sum_limit", &pid_sum_limit, sizeof(pid_sum_limit)) == RT_EOK) {
    pid_profile_.pidSumLimit = pid_sum_limit;
  }
  float pid_sum_limit_yaw = pid_profile_.pidSumLimitYaw;
  if (getParam("pid_sum_limit_yaw", &pid_sum_limit_yaw, sizeof(pid_sum_limit_yaw)) == RT_EOK) {
    pid_profile_.pidSumLimitYaw = pid_sum_limit_yaw;
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

  // Calculate I term windup limit based on PID sum limit
  // I项windup限制 = PID sum限制 * I项windup百分比 / 100
  // 例如：如果pidSumLimit=500, itermWindup=80%，则itermLimit=500*0.8=400
  // 这确保I项不会超过PID sum限制的itermWindup百分比
  pid_runtime_.itermLimit = 0.01f * pid_profile_.itermWindup * pid_profile_.pidSumLimit;
  pid_runtime_.itermLimitYaw = 0.01f * pid_profile_.itermWindup * pid_profile_.pidSumLimitYaw;

  LOG_I("PID Sum Limits: Roll/Pitch=%.1f deg/s, Yaw=%.1f deg/s", pid_profile_.pidSumLimit,
        pid_profile_.pidSumLimitYaw);
  LOG_I("I Term Windup Limits: Roll/Pitch=%.1f deg/s (%.1f%% of sum), Yaw=%.1f deg/s (%.1f%% of sum)",
        pid_runtime_.itermLimit, pid_profile_.itermWindup, pid_runtime_.itermLimitYaw, pid_profile_.itermWindup);
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

// MCN相关方法已移动到 pid_mcn.cpp

void PidBf::processPidController(uint32_t current_time_us) {
  // 获取新的陀螺仪数据（阻塞 - 将等待直到数据可用）
  // 注意：RC设定值和辅助通道数据在subTaskRcCommand()中通过updateRcDataFromMcn()更新
  if (updateGyroDataFromMcn()) {
    // 陀螺仪数据准备就绪，设定值数据已由subTaskRcCommand()中的processRcSmoothingFilter()写入
    // 辅助通道数据已在subTaskRcCommand()中通过updateRcDataFromMcn()更新
    // 直接处理PID控制器
    pidController(current_time_us);
  }
}

void PidBf::pidController(uint32_t current_time_us) {
  static float previousGyroRateDterm[XYZ_AXIS_COUNT] = {0.0f};

  if (!pid_runtime_.pidStabilisationEnabled) {
    return;
  }

  // Check arm status from aux channels data
  // If disarmed, zero all PID outputs (Betaflight behavior)
  bool is_armed = (aux_channels_data_.armed == RC_ARMED_STATUS_ARMED);
  if (!is_armed) {
    // Disarmed: zero all PID outputs and reset I term
    pid_output_msg_t output_msg;
    output_msg.timestamp = current_time_us;
    output_msg.seq = gyro_filtered_data_.seq;

    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      pid_data_[axis].P = 0.0f;
      pid_data_[axis].I = 0.0f;  // Reset I term when disarmed
      pid_data_[axis].D = 0.0f;
      pid_data_[axis].F = 0.0f;
      pid_data_[axis].S = 0.0f;
      pid_data_[axis].Sum = 0.0f;

      output_msg.pid_sum[axis] = 0.0f;
      output_msg.pid_p[axis] = 0.0f;
      output_msg.pid_i[axis] = 0.0f;
      output_msg.pid_d[axis] = 0.0f;
      output_msg.pid_f[axis] = 0.0f;
    }

    // Get smoothed throttle from setpoint data (already filtered by RC smoothing filter)
    output_msg.smoothed_throttle = setpoint_data_.smoothed_throttle;

    // Reset previous setpoint and D term history
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      pid_runtime_.previousPidSetpoint[axis] = 0.0f;
      previousGyroRateDterm[axis] = 0.0f;
    }

    // Publish zero output
    publishPidOutput(output_msg);
    return;
  }

  // Armed: process PID controller
  // Check flight mode (currently only Rate mode is supported)
  // Mode 0 = Rate mode (角速度模式) - use rate PID
  // Mode 1 = Angle mode (角度模式) - not implemented yet
  // Mode 2 = Altitude mode (高度模式) - not implemented yet
  uint8_t flight_mode = aux_channels_data_.flight_mode;
  if (flight_mode != 0) {
    // Currently only Rate mode (mode 0) is supported
    // For other modes, zero output for now (will be implemented later)
    pid_output_msg_t output_msg;
    output_msg.timestamp = current_time_us;
    output_msg.seq = gyro_filtered_data_.seq;

    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      output_msg.pid_sum[axis] = 0.0f;
      output_msg.pid_p[axis] = 0.0f;
      output_msg.pid_i[axis] = 0.0f;
      output_msg.pid_d[axis] = 0.0f;
      output_msg.pid_f[axis] = 0.0f;
    }

    // Get smoothed throttle from setpoint data (already filtered by RC smoothing filter)
    output_msg.smoothed_throttle = setpoint_data_.smoothed_throttle;

    publishPidOutput(output_msg);
    return;
  }

  // Rate mode (角速度模式): process rate PID controller
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

    // Calculate feedforward component (same as Betaflight)
    // In Rate mode, use feedforward value from RC module (already contains setpoint change rate info)
    // The feedforward value from RC module is calculated based on setpointDelta * rxRate,
    // and has been smoothed and processed with boost, jitter attenuation, etc.
    float pidSetpointDelta = getFeedforward(axis);
    pid_runtime_.previousPidSetpoint[axis] = currentSetpoint;  // Still update for potential other uses

    // Apply feedforward gain (Kf already includes FEEDFORWARD_SCALE)
    pid_data_[axis].F = pid_runtime_.pidCoefficient[axis].Kf * pidSetpointDelta;
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

  // Get smoothed throttle from setpoint data (already filtered by RC smoothing filter)
  // Same as Betaflight: rcCommand[THROTTLE] is smoothed and stored in setpoint_data_
  output_msg.smoothed_throttle = setpoint_data_.smoothed_throttle;

  publishPidOutput(output_msg);
}

float PidBf::getSetpointRate(int axis) {
  if (axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return setpoint_data_.rate[axis];
  }
  return 0.0f;
}

float PidBf::getFeedforward(int axis) {
  if (axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return setpoint_data_.feedforward[axis];
  }
  return 0.0f;
}

float PidBf::getMaxRcRate(int axis) {
  (void)axis;
  return 720.0f;
}

void PidBf::subTaskRcCommand(uint32_t current_time_us) {
  // PID Task (8kHz): Process RC smoothing filter
  // Note: processRcCommand is now in RC thread (100-200Hz), data passed via MCN to avoid data tearing
  (void)current_time_us;

  // Get RC smoothing filter instance from PID
  RcSmoothingFilter* smoothing_filter = getRcSmoothingFilter();
  if (smoothing_filter == nullptr) {
    return;  // RC smoothing filter not initialized yet
  }

  // Step 1: Get RC command data from MCN (for smoothing filter)
  // This polls MCN topic using mcn_poll (non-blocking) and always returns valid data pointer
  // Returns new data if available, otherwise cached historical data
  // This ensures smoothing filter can process RC data at PID frequency (3.2kHz) even when RC frequency is only 65Hz
  const rc_command_msg_t* rc_command_msg = updateRcCommandFromMcn();

  // Step 2: Process smoothing filter with RC command data
  // Get PID setpoint data reference for direct write (filter will write filtered data here)
  pid_setpoint_msg_t* pid_setpoint_out = &getSetpointDataRef();

  // Process RC smoothing filter: rc_command_msg → filtered → pid_setpoint_out
  // rc_command_msg is always valid (new data or cached historical data)
  // Filter will process RC command data at PID frequency (3.2kHz) even when RC frequency is only 65Hz
  // The filtered setpoint is directly written to PID singleton's setpoint_data_ member
  smoothing_filter->processFilter(rc_command_msg, pid_setpoint_out);

  // Step 3: Get RC aux channels data from MCN
  // This polls MCN topic and updates internal aux_channels_data_ in PID module
  updateRcAuxFromMcn();
}

void PidBf::pidMainLoop() {
  LOG_I("PidMain loop started");

  while (true) {
#ifdef PROJECT_BF_PID_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG1_HIGH();  // Debug pin: PID task execution start (monitor PID task frequency ~3.2kHz)
#endif
    uint32_t current_time_us = timestamp_micros();
    subTaskRcCommand(current_time_us);
    processPidController(current_time_us);
#ifdef PROJECT_BF_PID_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG1_LOW();  // Debug pin: PID task execution end
#endif
  }
}

void PidBf::workerEntry(void* parameter) {
  auto* self = static_cast<PidBf*>(parameter);
  if (!self) {
    return;
  }
  self->pidMainLoop();
}

rt_err_t PidBf::startMainThread() {
  if (main_thread_inited_) {
    LOG_W("PidMain already initialized");
    return RT_EOK;
  }

  // Initialize main thread
  rt_err_t ret = rt_thread_init(&main_thread_obj_, "pid_main", workerEntry, this,
                                 main_thread_stack_, PROJECT_BF_PID_THREAD_STACK_SIZE,
                                 PROJECT_BF_PID_THREAD_PRIORITY,
                                 PROJECT_BF_PID_THREAD_TIMESLICE);

  if (ret != RT_EOK) {
    LOG_E("PidMain thread init failed: %d", ret);
    return ret;
  }

  main_thread_ = &main_thread_obj_;
  main_thread_inited_ = true;

  ret = rt_thread_startup(main_thread_);
  if (ret != RT_EOK) {
    LOG_E("PidMain thread startup failed: %d", ret);
    main_thread_inited_ = false;
    return ret;
  }

  return RT_EOK;
}

// RT-Thread 自动初始化包装函数
#ifdef PROJECT_BF_PID_EN
extern "C" {
static int pid_main_init_wrapper(void) {
  // Small delay to ensure RcBf is initialized first
  rt_thread_mdelay(10);
  
  PidBf& pid = PidBf::instance();
  rt_err_t ret = pid.init();
  if (ret != RT_EOK) {
    LOG_E("PidBf init failed: %d", ret);
    return (int)ret;
  }
  LOG_I("PidBf initialized successfully");

  ret = pid.startMainThread();
  if (ret == RT_EOK) {
    LOG_I("PidMain auto-init success");
  } else {
    LOG_E("PidMain auto-init failed: %d", ret);
  }
  return (int)ret;
}
INIT_APP_EXPORT(pid_main_init_wrapper);
}
#endif


