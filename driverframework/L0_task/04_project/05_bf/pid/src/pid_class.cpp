#include "pid_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "pid_bf"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "gyro_mcn.h"
#include "pid_mcn.h"
#include "pidParam.h"
#include "rc_mcn.h"  // For MCN_DECLARE(rc), rc_command_msg_t, rc_aux_msg_t
#include "filter.h"  // For filter functions: pt1FilterInit, pt2FilterInit, pt3FilterInit, biquadFilterInit, etc.
#include "../common/inc/init_sync.h"  // For initSyncWait, initSyncNotify
#ifdef PROJECT_BF_ATTITUDE_EN
#include "../attitude/inc/attitude_mcn.h"  // For attitude_msg_t
#endif
#ifdef PROJECT_BF_PID_DEBUG_PIN_EN
#include "debugPin.h"
#endif
}

#include "rc_smooth.h"  // For RcSmoothingFilter
#include "../rc/inc/rc_class.hpp"  // For RcBf::instance()

#include <cmath>
#include <cstring>

// PID constants from ref/pid.h (used in this file)
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f
#define FEEDFORWARD_SCALE 0.013754f
// Note: PIDSUM_LIMIT and PIDSUM_LIMIT_YAW moved to pid_init.cpp

// Helper macros
#define CLAMPF(x, min, max) ((x) < (min) ? (min) : (((x) > (max)) ? (max) : (x)))

namespace {

constexpr float PI_F = 3.14159265358979323846f;
constexpr float DEGREES_TO_RADIANS = PI_F / 180.0f;
constexpr float RADIANS_TO_DEGREES = 180.0f / PI_F;

}  // namespace

/* PID 消息 MCN 定义已移动到 pid_mcn.cpp */

// Thread configuration removed - using taskPid.cpp main thread instead

// Constructor, destructor, init functions moved to pid_init.cpp

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
  // Check flight mode
  // Mode 0 = Rate mode (角速度模式) - use rate PID
  // Mode 1 = Angle mode (角度模式) - use angle PID
  uint8_t flight_mode = aux_channels_data_.flight_mode;
  
#ifdef PROJECT_BF_ATTITUDE_EN
  // Update attitude data from MCN (non-blocking)
  // 即使没有新数据，也会返回 true（如果有历史数据）
  bool attitude_available = updateAttitudeDataFromMcn();
#else
  bool attitude_available = false;
#endif

  // Rate mode (角速度模式): process rate PID controller
  // Angle mode (角度模式): process angle PID controller (converts angle error to rate setpoint, then rate PID)
  // 参考 Betaflight：一旦订阅成功并获取到一次有效数据，就持续使用历史数据
  // 即使某次没有新数据更新，也继续使用历史数据，避免角度模式频繁退出
  bool is_angle_mode = (flight_mode == 1) && attitude_available;
  
  if (flight_mode == 1 && !attitude_available) {
    // Angle mode requested but attitude data not available
    // 只有在订阅失败或从未获取到数据时才会进入这里
    static uint32_t last_warn_time = 0;
    uint32_t current_time = rt_tick_get();
    // 避免频繁打印警告（每 1 秒打印一次）
    if (current_time - last_warn_time > RT_TICK_PER_SECOND) {
      LOG_W("Angle mode requested but attitude data not available, falling back to rate mode");
      last_warn_time = current_time;
    }
    is_angle_mode = false;
  }

  // Get gyro rates
  float gyroRate[XYZ_AXIS_COUNT];
  std::memcpy(gyroRate, gyro_filtered_data_.gyro_filtered_for_pid, sizeof(gyroRate));

  pid_output_msg_t output_msg;
  output_msg.timestamp = current_time_us;
  output_msg.seq = gyro_filtered_data_.seq;

  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    float currentSetpoint = getSetpointRate(axis);
    
#ifdef PROJECT_BF_ATTITUDE_EN
    // If in angle mode and this is roll or pitch axis, convert angle error to rate setpoint
    if (is_angle_mode && axis < FD_YAW) {
      pid_runtime_.axisInAngleMode[axis] = true;
      float horizonLevelStrength = 0.0f;  // Currently only angle mode, no horizon mode
      currentSetpoint = pidLevel(axis, currentSetpoint, horizonLevelStrength);
    } else {
      pid_runtime_.axisInAngleMode[axis] = false;
    }
    
    // Yaw axis: handle earth reference compensation in angle mode
    // Reference: Betaflight ref/pid.c pidController() around line 1350-1360
    // In angle mode, yaw always uses rate mode (no angle loop conversion)
    // But we apply earth reference attenuation to yaw setpoint when pitched or rolled
    if (is_angle_mode && axis == FD_YAW) {
      // Store yaw setpoint (before attenuation) for earth reference compensation in roll/pitch
      // This is the raw yaw rate setpoint from RC input
      pid_runtime_.angleYawSetpoint = currentSetpoint;
      
      // Apply earth reference attenuation to yaw setpoint when pitched or rolled
      // When the craft is tilted, reduce yaw responsiveness to prevent unwanted yaw rotation
      // The attenuation uses cosine of the maximum tilt angle (roll or pitch)
      // This naturally reduces yaw response as the craft tilts more
      float maxAngleTargetAbs = std::fmax(std::fabs(pid_runtime_.angleTarget[FD_ROLL]), 
                                         std::fabs(pid_runtime_.angleTarget[FD_PITCH]));
      
      // Attenuation factor: cos(angle) reduces from 1 to 0 as angle approaches 90 degrees
      // When craft is level (0°), cos(0°) = 1.0 (full yaw response)
      // When craft is tilted 90°, cos(90°) = 0.0 (no yaw response)
      currentSetpoint *= std::cos(maxAngleTargetAbs * DEGREES_TO_RADIANS);
    }
#endif

    // Save currentSetpoint snapshot before errorRate calculation (for logging/debugging)
    pid_runtime_.rateLoopDebug[axis].currentSetpoint = currentSetpoint;

    const float errorRate = currentSetpoint - gyroRate[axis];

    float pTerm = pid_runtime_.pidCoefficient[axis].Kp * errorRate;
    if (axis == FD_YAW) {
      // Apply Yaw P term lowpass filter (same as Betaflight)
      // ptermYawLowpassApplyFn is always initialized (yaw_lowpass_hz has default value)
      pTerm = pid_runtime_.ptermYawLowpassApplyFn((filter_t*)&pid_runtime_.ptermYawLowpass, pTerm);
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
#ifdef PROJECT_BF_ATTITUDE_EN
    float pidSetpointDelta = 0.0f;
    if (is_angle_mode && axis < FD_YAW && pid_runtime_.axisInAngleMode[axis]) {
      // In angle mode, feedforward is already applied in pidLevel() as angleFeedforward
      // So we set feedforward to zero here (same as Betaflight)
      // Reference: ref/pid.c pidController() line 1408-1410
      pidSetpointDelta = 0.0f;
    } else {
      // In Rate mode, use feedforward value from RC module (already contains setpoint change rate info)
      // The feedforward value from RC module is calculated based on setpointDelta * rxRate,
      // and has been smoothed and processed with boost, jitter attenuation, etc.
      pidSetpointDelta = getFeedforward(axis);
    }
#else
    // In Rate mode, use feedforward value from RC module
    float pidSetpointDelta = getFeedforward(axis);
#endif
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

  // Push PID data to mlog (record rate and angle loop setpoint/actual snapshots)
  bf_mlog::pid_mlog_data_t mlog_data;
  mlog_data.seq = output_msg.seq;
  mlog_data.timestamp = current_time_us;
  // Rate loop data: setpoint and actual rates
  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    mlog_data.rate_setpoint[axis] = pid_runtime_.rateLoopDebug[axis].currentSetpoint;
  }
  // Use gyro_filtered_for_pid (same as anotc) for actual rates
  std::memcpy(mlog_data.rate_actual, gyro_filtered_data_.gyro_filtered_for_pid, sizeof(mlog_data.rate_actual));
  
  // Angle loop data: setpoint and actual angles (roll and pitch only)
#ifdef PROJECT_BF_ATTITUDE_EN
  if (is_angle_mode) {
    // Get angle loop debug data for roll and pitch
    for (int axis = 0; axis < 2; axis++) {
      const pidRuntime_t::angleLoopDebug_t* debug = getAngleLoopDebug(axis);
      if (debug != nullptr) {
        mlog_data.angle_setpoint[axis] = debug->target;
        mlog_data.angle_actual[axis] = debug->current;
      } else {
        mlog_data.angle_setpoint[axis] = 0.0f;
        mlog_data.angle_actual[axis] = 0.0f;
      }
    }
  } else {
    // Rate mode: angle loop data is not used
    mlog_data.angle_setpoint[0] = 0.0f;
    mlog_data.angle_setpoint[1] = 0.0f;
    mlog_data.angle_actual[0] = 0.0f;
    mlog_data.angle_actual[1] = 0.0f;
  }
#else
  // No attitude support: angle loop data is not available
  mlog_data.angle_setpoint[0] = 0.0f;
  mlog_data.angle_setpoint[1] = 0.0f;
  mlog_data.angle_actual[0] = 0.0f;
  mlog_data.angle_actual[1] = 0.0f;
#endif

  pushPidDataToMlog(&mlog_data);
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
  // Return cached max RC rate (initialized once from RC module)
  // This avoids repeated singleton calls for better performance
  if (axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return max_rc_rate_[axis];
  }
  return 0.0f;
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

// Thread-related functions moved to pid_init.cpp

#ifdef PROJECT_BF_ATTITUDE_EN
// Get current angle from attitude data (degrees)
float PidBf::getCurrentAngle(int axis) const {
  if (axis >= 0 && axis < XYZ_AXIS_COUNT && attitude_data_valid_) {
    // Return angle in degrees (from attitude.values)
    return attitude_data_.values[axis];
  }
  return 0.0f;
}

// Calculate horizon level strength (for horizon mode, currently not used but kept for compatibility)
float PidBf::calcHorizonLevelStrength() const {
  // Currently only angle mode is supported, so return 0.0 (no horizon leveling)
  // This function is kept for future horizon mode support
  return 0.0f;
}

// Angle mode PID controller (pidLevel function)
// Reference: ref/pid.c pidLevel()
// Converts angle error to angle rate setpoint
float PidBf::pidLevel(int axis, float currentPidSetpoint, float horizonLevelStrength) {
  // Applies only to axes that are in Angle mode
  // We now use Acro Rates, transformed into the range +/- 1, to provide setpoints
  float angleLimit = pid_profile_.angle_limit;
  float angleFeedforward = 0.0f;
#ifdef PROJECT_BF_PID_DEBUG_PIN_EN
  if (axis == FD_ROLL) {
    DEBUG_PIN_DEBUG2_HIGH();
  }
#endif
  // if user changes rates profile, update the max setpoint for angle mode
  const float maxSetpointRateInv = 1.0f / getMaxRcRate(axis);

  // Calculate angle feedforward (if enabled)
  angleFeedforward = angleLimit * getFeedforward(axis) * pid_runtime_.angleFeedforwardGain * maxSetpointRateInv;
  // angle feedforward must be heavily filtered, at the PID loop rate, with limited user control over time constant
  // it MUST be very delayed to avoid early overshoot and being too aggressive
  angleFeedforward = pt3FilterApply(&pid_runtime_.angleFeedforwardPt3[axis], angleFeedforward);

  // Calculate angle target from RC input
  // use acro rates for the angle target in both horizon and angle modes, converted to -1 to +1 range using maxRate
  float angleTarget = angleLimit * currentPidSetpoint * maxSetpointRateInv;

  // Limit angle target
  angleTarget = CLAMPF(angleTarget, -angleLimit, angleLimit);

  // Get current angle from attitude data (degrees)
  // Note: Betaflight uses (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f
  // We use attitude.values[axis] directly (already in degrees)
  // TODO: Add angle trim support if needed (currently angle trim is 0)
  const float currentAngle = getCurrentAngle(axis);

  // Record debug data: target and current angle (before error calculation)
  pid_runtime_.angleLoopDebug[axis].target = angleTarget;
  pid_runtime_.angleLoopDebug[axis].current = currentAngle;

  // Calculate angle error
  const float errorAngle = angleTarget - currentAngle;

  // Calculate angle rate from error
  const float errorGain = errorAngle * pid_runtime_.angleGain;
  float angleRate = errorGain + angleFeedforward;

  // Record debug data: errorGain and feedforward (after calculation)
  pid_runtime_.angleLoopDebug[axis].errorGain = errorGain;
  pid_runtime_.angleLoopDebug[axis].feedforward = angleFeedforward;

  // Earth reference compensation (coordinate yaw turns)
  // minimise cross-axis wobble due to faster yaw responses than roll or pitch, and make co-ordinated yaw turns
  // by compensating for the effect of yaw on roll while pitched, and on pitch while rolled
  float sinAngle = std::sin(pid_runtime_.angleTarget[axis == FD_ROLL ? FD_PITCH : FD_ROLL] * DEGREES_TO_RADIANS);
  sinAngle *= (axis == FD_ROLL) ? -1.0f : 1.0f;  // must be negative for Roll
  angleRate += pid_runtime_.angleYawSetpoint * sinAngle * pid_runtime_.angleEarthRef;
  pid_runtime_.angleTarget[axis] =
      angleTarget;  // set target for alternate axis to current axis, for use in preceding calculation

  // smooth final angle rate output to clean up attitude signal steps (500hz), GPS steps (10 or 100hz), RC steps etc
  // this filter runs at ATTITUDE_CUTOFF_HZ, currently 50hz, so GPS roll may be a bit steppy
  angleRate = pt3FilterApply(&pid_runtime_.attitudeFilter[axis], angleRate);

#ifdef PROJECT_BF_PID_DEBUG_PIN_EN
  if (axis == FD_ROLL) {
    DEBUG_PIN_DEBUG2_LOW();
  }
#endif

  // For angle mode, return the angle rate directly
  // For horizon mode (not implemented yet), would crossfade Angle rate and Acro rate
  return angleRate;
}
#endif

#ifdef USE_DYN_LPF
// Forward declaration of dynLpfCutoffFreq (defined in pid_init.cpp)
extern float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo);

// Dynamic throttle curve function (same as Betaflight)
static float dynThrottle(float throttle) {
  return throttle * (1.0f - (throttle * throttle) / 3.0f) * 1.5f;
}

// Dynamic LPF Dterm update function (same as Betaflight)
void PidBf::dynLpfDTermUpdate(float throttle) {
  if (pid_runtime_.dynLpfFilter != DYN_LPF_NONE) {
    float cutoffFreq;
    if (pid_runtime_.dynLpfCurveExpo > 0) {
      cutoffFreq = dynLpfCutoffFreq(throttle, pid_runtime_.dynLpfMin, pid_runtime_.dynLpfMax, pid_runtime_.dynLpfCurveExpo);
    } else {
      cutoffFreq = std::fmax(dynThrottle(throttle) * pid_runtime_.dynLpfMax, pid_runtime_.dynLpfMin);
    }
    
    switch (pid_runtime_.dynLpfFilter) {
      case DYN_LPF_PT1:
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
          pt1FilterUpdateCutoff(&pid_runtime_.dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, pid_runtime_.dT));
        }
        break;
      case DYN_LPF_BIQUAD:
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
          biquadFilterUpdateLPF(&pid_runtime_.dtermLowpass[axis].biquadFilter, cutoffFreq, target_looptime_us_);
        }
        break;
      case DYN_LPF_PT2:
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
          pt2FilterUpdateCutoff(&pid_runtime_.dtermLowpass[axis].pt2Filter, pt2FilterGain(cutoffFreq, pid_runtime_.dT));
        }
        break;
      case DYN_LPF_PT3:
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
          pt3FilterUpdateCutoff(&pid_runtime_.dtermLowpass[axis].pt3Filter, pt3FilterGain(cutoffFreq, pid_runtime_.dT));
        }
        break;
      case DYN_LPF_NONE:
        // No update needed
        break;
    }
  }
}
#endif

// Get rate loop debug data for logging
const pidRuntime_t::rateLoopDebug_t* PidBf::getRateLoopDebug(int axis) const {
  if (axis >= 0 && axis < XYZ_AXIS_COUNT) {
    return &pid_runtime_.rateLoopDebug[axis];
  }
  return nullptr;
}

