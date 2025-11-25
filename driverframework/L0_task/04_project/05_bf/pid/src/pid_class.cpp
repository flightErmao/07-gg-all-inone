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
#include "filter.h"  // For filter functions: pt1FilterInit, pt2FilterInit, pt3FilterInit, biquadFilterInit, etc.
#include "../common/inc/init_sync.h"  // For initSyncWait, initSyncNotify
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

  // Initialize filter function pointers (will be set in initFilters)
  pid_runtime_.dtermNotchApplyFn = nullFilterApply;
  pid_runtime_.dtermLowpassApplyFn = nullFilterApply;
  pid_runtime_.dtermLowpass2ApplyFn = nullFilterApply;
  pid_runtime_.ptermYawLowpassApplyFn = nullFilterApply;

  // Initialize Dynamic LPF (same as Betaflight)
  pid_runtime_.dynLpfFilter = DYN_LPF_NONE;
  pid_runtime_.dynLpfMin = 0;
  pid_runtime_.dynLpfMax = 0;
  pid_runtime_.dynLpfCurveExpo = 0;

#ifdef PROJECT_BF_PID_D_MAX_EN
  // Initialize D_MAX (same as Betaflight)
  for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
    std::memset(&pid_runtime_.dMaxRange[axis], 0, sizeof(pt2Filter_t));
    std::memset(&pid_runtime_.dMaxLowpass[axis], 0, sizeof(pt2Filter_t));
    pid_runtime_.dMaxPercent[axis] = 1.0f;
  }
  pid_runtime_.dMaxGyroGain = 0.0f;
  pid_runtime_.dMaxSetpointGain = 0.0f;
#endif

  // Initialize filter states
  for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
    pid_runtime_.previousPidSetpoint[axis] = 0.0f;
    pid_runtime_.previousGyroRateDterm[axis] = 0.0f;

    // Initialize dtermNotch (biquadFilter_t)
    std::memset(&pid_runtime_.dtermNotch[axis], 0, sizeof(biquadFilter_t));

    // Initialize dtermLowpass (union - initialize as pt1Filter)
    std::memset(&pid_runtime_.dtermLowpass[axis], 0, sizeof(dtermLowpass_t));

    // Initialize dtermLowpass2 (union - initialize as pt1Filter)
    std::memset(&pid_runtime_.dtermLowpass2[axis], 0, sizeof(dtermLowpass_t));
  }

  // Initialize ptermYawLowpass (pt1Filter_t)
  std::memset(&pid_runtime_.ptermYawLowpass, 0, sizeof(pt1Filter_t));

  // Legacy: SimpleLowpass for yaw P term (kept for backward compatibility)
  yaw_pterm_lpf_.state = 0.0f;
  yaw_pterm_lpf_.alpha = 1.0f;
  yaw_pterm_lpf_.enabled = false;

  // Initialize profile defaults
  pid_profile_.pidSumLimit = PIDSUM_LIMIT;
  pid_profile_.pidSumLimitYaw = PIDSUM_LIMIT_YAW;
  pid_profile_.itermWindup = 80.0f;

  // Dterm filter defaults (same as Betaflight)
  pid_profile_.dterm_notch_hz = 0;
  pid_profile_.dterm_notch_cutoff = 0;
  pid_profile_.dterm_lpf1_static_hz = 100;  // Betaflight default
  pid_profile_.dterm_lpf1_type = FILTER_PT1;
  pid_profile_.dterm_lpf2_static_hz = 0;  // Disabled by default
  pid_profile_.dterm_lpf2_type = FILTER_PT1;

  // Dynamic LPF defaults (same as Betaflight)
  pid_profile_.dterm_lpf1_dyn_min_hz = 0;  // Disabled by default
  pid_profile_.dterm_lpf1_dyn_max_hz = 0;  // Disabled by default
  pid_profile_.dterm_lpf1_dyn_expo = 5;    // Default expo (0.5)

#ifdef PROJECT_BF_PID_D_MAX_EN
  // D_MAX defaults (same as Betaflight)
  pid_profile_.d_max[FD_ROLL] = 40;   // Default D_MAX for roll
  pid_profile_.d_max[FD_PITCH] = 46;  // Default D_MAX for pitch
  pid_profile_.d_max[FD_YAW] = 0;     // Default D_MAX for yaw (disabled)
  pid_profile_.d_max_gain = 37;       // Default D_MAX gain
  pid_profile_.d_max_advance = 20;    // Default D_MAX advance
#endif

  // Yaw P term filter defaults
  pid_profile_.yaw_lowpass_hz = 90;
  pid_profile_.yawLpfHz = 90.0f;  // Legacy: kept for backward compatibility
}

PidBf::~PidBf() { cleanupMcnSubscriptions(); }

rt_err_t PidBf::init() {
  // Note: Thread initialization removed - using taskPid.cpp main thread instead
  // 注意：依赖 Gyro Filter 的初始化已移到 workerEntry 中，在线程调度器启动后执行

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

  // Load PID process denominator and calculate target looptime
  loadPidProcessDenom();

  initConfig();
  initFilters();

  // 注意：initSyncNotify(INIT_SYNC_PID) 已移到 workerEntry 中，在等待 Gyro Filter 之后

  LOG_I("PidBf initialized (no thread - using taskPid.cpp main thread)");
  return RT_EOK;
}

void PidBf::initConfig() {
  // Load PID parameters from param system
  loadPidParameters();
  
  // Calculate PID coefficients and limits from loaded parameters
  calculatePidCoefficients();
}

void PidBf::initFilters() {
  // Initialize all Dterm filters and Yaw P term filter (same pattern as Betaflight)
  // initDtermFilters() is now in pid_init.cpp
  initDtermFilters();

  // Initialize legacy Yaw P term SimpleLowpass filter (for backward compatibility)
  // This is only used if ptermYawLowpassApplyFn is nullFilterApply
  lowpassInit(&yaw_pterm_lpf_, pid_profile_.yawLpfHz, pid_runtime_.dT);

  LOG_I("PID filters initialized");
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
      pid_runtime_.previousGyroRateDterm[axis] = 0.0f;
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
      // Apply Yaw P term lowpass filter (same as Betaflight)
      if (pid_runtime_.ptermYawLowpassApplyFn != nullFilterApply) {
        pTerm = pid_runtime_.ptermYawLowpassApplyFn((filter_t*)&pid_runtime_.ptermYawLowpass, pTerm);
      } else {
        // Fallback to legacy SimpleLowpass for backward compatibility
      pTerm = lowpassApply(&yaw_pterm_lpf_, pTerm);
      }
    }
    pid_data_[axis].P = pTerm;

    const float itermLimit = (axis == FD_YAW) ? pid_runtime_.itermLimitYaw : pid_runtime_.itermLimit;
    const float iTermChange = pid_runtime_.pidCoefficient[axis].Ki * pid_runtime_.dT * errorRate;
    pid_data_[axis].I = CLAMPF(pid_data_[axis].I + iTermChange, -itermLimit, itermLimit);

    // Apply Dterm filters (same pattern as Betaflight)
    // Reference: ref/pid.c pidController()
    // Step 1: Get raw gyro rate
    float gyroRateDterm = gyroRate[axis];

    // Step 2: Apply Dterm notch filter (if enabled)
    gyroRateDterm = pid_runtime_.dtermNotchApplyFn((filter_t*)&pid_runtime_.dtermNotch[axis], gyroRateDterm);

    // Step 3: Apply 1st Dterm lowpass filter (if enabled)
    gyroRateDterm = pid_runtime_.dtermLowpassApplyFn((filter_t*)&pid_runtime_.dtermLowpass[axis], gyroRateDterm);

    // Step 4: Apply 2nd Dterm lowpass filter (if enabled)
    gyroRateDterm = pid_runtime_.dtermLowpass2ApplyFn((filter_t*)&pid_runtime_.dtermLowpass2[axis], gyroRateDterm);

    // Step 5: Calculate delta (rate change) for D term
    // Divide rate change by dT to get differential (ie dr/dt)
    // dT is fixed and calculated from the target PID loop time
    // This is done to avoid DTerm spikes that occur with dynamically
    // calculated deltaT whenever another task causes the PID
    // loop execution to be delayed (same as Betaflight)
    const float delta = -(gyroRateDterm - pid_runtime_.previousGyroRateDterm[axis]) * pid_runtime_.pidFrequency;
    pid_runtime_.previousGyroRateDterm[axis] = gyroRateDterm;

    // Step 6: Calculate D term (before D_MAX and TPA)
    float preTpaD = pid_runtime_.pidCoefficient[axis].Kd * delta;

#ifdef PROJECT_BF_PID_D_MAX_EN
    // Apply D_MAX multiplier (same as Betaflight)
    // Reference: ref/pid.c pidController()
    float dMaxMultiplier = 1.0f;
    if (pid_runtime_.dMaxPercent[axis] > 1.0f) {
      // Calculate D_MAX boost based on gyro rate change and setpoint change
      float dMaxGyroFactor = pt2FilterApply(&pid_runtime_.dMaxRange[axis], delta);
      dMaxGyroFactor = std::fabs(dMaxGyroFactor) * pid_runtime_.dMaxGyroGain;

      // Get setpoint delta for D_MAX calculation
      float pidSetpointDelta = getFeedforward(axis);  // Use feedforward as setpoint delta
      const float dMaxSetpointFactor = std::fabs(pidSetpointDelta) * pid_runtime_.dMaxSetpointGain;

      // Use the maximum of gyro factor and setpoint factor
      const float dMaxBoost = std::fmax(dMaxGyroFactor, dMaxSetpointFactor);

      // dMaxBoost starts at zero, and by 1.0 we get Dmax, but it can exceed 1.
      dMaxMultiplier += (pid_runtime_.dMaxPercent[axis] - 1.0f) * dMaxBoost;

      // Smooth the multiplier with PT2 filter
      dMaxMultiplier = pt2FilterApply(&pid_runtime_.dMaxLowpass[axis], dMaxMultiplier);

      // Limit the gain to the fraction that DMax is greater than Min
      dMaxMultiplier = std::fmin(dMaxMultiplier, pid_runtime_.dMaxPercent[axis]);
    }

    // Apply the gain that increases D towards Dmax
    preTpaD *= dMaxMultiplier;
#endif

    // Step 7: Apply TPA factor (currently always 1.0, can be implemented later)
    // For now, TPA is not implemented, so we use 1.0
    const float tpaFactor = 1.0f;  // TODO: Implement TPA if needed
    pid_data_[axis].D = preTpaD * tpaFactor;

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
  
  // 在线程调度器启动后，等待 Gyro Filter 初始化完成（PID 需要 gyro 数据）
  // 这部分初始化依赖其他线程状态，必须在线程入口函数中执行
  rt_err_t ret = initSyncWait(INIT_SYNC_GYRO_FILTER, 2000);  // 等待最多2秒
  if (ret != RT_EOK) {
    LOG_W("Gyro Filter not ready, continuing anyway (ret=%d)", ret);
  }
  
  // 通知 PID 初始化完成（在等待 Gyro Filter 之后）
  initSyncNotify(INIT_SYNC_PID);
  
  // 进入主循环
  self->pidMainLoop();
}

rt_err_t PidBf::startMainThread() {
  if (main_thread_inited_) {
    LOG_W("PidMain already initialized");
    return RT_EOK;
  }

  // Initialize main thread
  rt_err_t ret =
      rt_thread_init(&main_thread_obj_, "pid", workerEntry, this, main_thread_stack_, PROJECT_BF_PID_THREAD_STACK_SIZE,
                     PROJECT_BF_PID_THREAD_PRIORITY, PROJECT_BF_PID_THREAD_TIMESLICE);

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


