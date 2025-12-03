#include "pid_class.h"
#include "../rc/inc/rc_class.hpp"  // For RcBf::instance()

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "pid_init"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "param.h"
#include "filter.h"  // For filter functions: pt1FilterInit, pt2FilterInit, pt3FilterInit, biquadFilterInit, etc.
#include "maths.h"   // For M_PIf (if needed)
#include "gyro_mcn.h"
#include "pid_mcn.h"
#include "rc_mcn.h"
#include "../common/inc/init_sync.h"
#ifdef PROJECT_BF_ATTITUDE_EN
#include "../attitude/inc/attitude_mcn.h"
#endif
#ifdef PROJECT_BF_PID_DEBUG_PIN_EN
#include "debugPin.h"
#endif
}

#include "rc_smooth.h"
#include <cmath>
#include <cstring>

namespace {
constexpr float PI_F = 3.14159265358979323846f;
}

// PID constants from ref/pid.h
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f
#define FEEDFORWARD_SCALE 0.013754f
#define PIDSUM_LIMIT 500.0f
#define PIDSUM_LIMIT_YAW 400.0f

// Dynamic LPF cutoff frequency calculation (same as Betaflight)
// Reference: ref/pid.c dynLpfCutoffFreq()
float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo) {
  const float expof = expo / 10.0f;
  const float curve = throttle * (1.0f - throttle) * expof + throttle;
  return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

void PidBf::initDefaults() {
  // Initialize profile defaults (same as Betaflight)
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

#ifdef PROJECT_BF_ATTITUDE_EN
  // Angle mode defaults (same as Betaflight)
  pid_profile_.angle_limit = 60;  // Max angle in degrees
  pid_profile_.angle_earth_ref = 100;  // Earth reference gain (0-100)
  pid_profile_.angle_p_gain = 50.0f;  // Angle mode P gain
  pid_profile_.angle_feedforward = 50.0f;  // Angle mode feedforward gain
  pid_profile_.angle_feedforward_smoothing_ms = 80;  // Time constant in milliseconds
#endif
}

void PidBf::initRuntime() {
  // Initialize runtime state
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

#ifdef PROJECT_BF_ATTITUDE_EN
  // Initialize angle mode runtime parameters
  pid_runtime_.angleGain = 0.0f;
  pid_runtime_.angleFeedforwardGain = 0.0f;
  pid_runtime_.angleTarget[0] = 0.0f;
  pid_runtime_.angleTarget[1] = 0.0f;
  pid_runtime_.angleYawSetpoint = 0.0f;
  pid_runtime_.angleEarthRef = 0.0f;
  pid_runtime_.axisInAngleMode[0] = false;
  pid_runtime_.axisInAngleMode[1] = false;
  pid_runtime_.axisInAngleMode[2] = false;
  
  // Initialize attitude filters (will be initialized in initFilters)
  std::memset(pid_runtime_.attitudeFilter, 0, sizeof(pid_runtime_.attitudeFilter));
  std::memset(pid_runtime_.angleFeedforwardPt3, 0, sizeof(pid_runtime_.angleFeedforwardPt3));
  
  // Initialize angle loop debug data
  for (int axis = 0; axis < RP_AXIS_COUNT; ++axis) {
    pid_runtime_.angleLoopDebug[axis].target = 0.0f;
    pid_runtime_.angleLoopDebug[axis].current = 0.0f;
    pid_runtime_.angleLoopDebug[axis].errorGain = 0.0f;
    pid_runtime_.angleLoopDebug[axis].feedforward = 0.0f;
  }
#endif
  
  // Initialize rate loop debug data
  for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
    pid_runtime_.rateLoopDebug[axis].currentSetpoint = 0.0f;
  }
}

void PidBf::loadPidParameters() {
  auto loadAxis = [&](int axis, const float* src) {
    pid_profile_.pid[axis].P = src[0];
    pid_profile_.pid[axis].I = src[1];
    pid_profile_.pid[axis].D = src[2];
    pid_profile_.pid[axis].F = 0.0f;
    pid_profile_.pid[axis].S = 0.0f;
  };

  // Load PID rate gains (Roll, Pitch, Yaw)
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
  if (getParam("pid_rate_sum_limit", &pid_sum_limit, sizeof(pid_sum_limit)) == RT_EOK) {
    pid_profile_.pidSumLimit = pid_sum_limit;
  }
  float pid_sum_limit_yaw = pid_profile_.pidSumLimitYaw;
  if (getParam("pid_rate_sum_limit_yaw", &pid_sum_limit_yaw, sizeof(pid_sum_limit_yaw)) == RT_EOK) {
    pid_profile_.pidSumLimitYaw = pid_sum_limit_yaw;
  }

  // Load PID filter parameters
  float iterm_windup = pid_profile_.itermWindup;
  if (getParam("pid_iterm_windup", &iterm_windup, sizeof(iterm_windup)) == RT_EOK) {
    pid_profile_.itermWindup = iterm_windup;
  }

  // Load Dterm filter parameters (same as Betaflight)
  uint16_t dterm_notch_hz = pid_profile_.dterm_notch_hz;
  if (getParam("pid_dterm_notch_hz", &dterm_notch_hz, sizeof(dterm_notch_hz)) == RT_EOK) {
    pid_profile_.dterm_notch_hz = dterm_notch_hz;
  }
  
  uint16_t dterm_notch_cutoff = pid_profile_.dterm_notch_cutoff;
  if (getParam("pid_dterm_notch_cutoff", &dterm_notch_cutoff, sizeof(dterm_notch_cutoff)) == RT_EOK) {
    pid_profile_.dterm_notch_cutoff = dterm_notch_cutoff;
  }
  
  uint16_t dterm_lpf1_static_hz = pid_profile_.dterm_lpf1_static_hz;
  if (getParam("pid_dterm_lpf1_static_hz", &dterm_lpf1_static_hz, sizeof(dterm_lpf1_static_hz)) == RT_EOK) {
    pid_profile_.dterm_lpf1_static_hz = dterm_lpf1_static_hz;
  }
  
  uint8_t dterm_lpf1_type = pid_profile_.dterm_lpf1_type;
  if (getParam("pid_dterm_lpf1_type", &dterm_lpf1_type, sizeof(dterm_lpf1_type)) == RT_EOK) {
    pid_profile_.dterm_lpf1_type = dterm_lpf1_type;
  }
  
  uint16_t dterm_lpf2_static_hz = pid_profile_.dterm_lpf2_static_hz;
  if (getParam("pid_dterm_lpf2_static_hz", &dterm_lpf2_static_hz, sizeof(dterm_lpf2_static_hz)) == RT_EOK) {
    pid_profile_.dterm_lpf2_static_hz = dterm_lpf2_static_hz;
  }
  
  uint8_t dterm_lpf2_type = pid_profile_.dterm_lpf2_type;
  if (getParam("pid_dterm_lpf2_type", &dterm_lpf2_type, sizeof(dterm_lpf2_type)) == RT_EOK) {
    pid_profile_.dterm_lpf2_type = dterm_lpf2_type;
  }
  
  // Load Dynamic LPF parameters (same as Betaflight)
  uint16_t dterm_lpf1_dyn_min_hz = pid_profile_.dterm_lpf1_dyn_min_hz;
  if (getParam("pid_dterm_lpf1_dyn_min_hz", &dterm_lpf1_dyn_min_hz, sizeof(dterm_lpf1_dyn_min_hz)) == RT_EOK) {
    pid_profile_.dterm_lpf1_dyn_min_hz = dterm_lpf1_dyn_min_hz;
  }
  
  uint16_t dterm_lpf1_dyn_max_hz = pid_profile_.dterm_lpf1_dyn_max_hz;
  if (getParam("pid_dterm_lpf1_dyn_max_hz", &dterm_lpf1_dyn_max_hz, sizeof(dterm_lpf1_dyn_max_hz)) == RT_EOK) {
    pid_profile_.dterm_lpf1_dyn_max_hz = dterm_lpf1_dyn_max_hz;
  }
  
  uint8_t dterm_lpf1_dyn_expo = pid_profile_.dterm_lpf1_dyn_expo;
  if (getParam("pid_dterm_lpf1_dyn_expo", &dterm_lpf1_dyn_expo, sizeof(dterm_lpf1_dyn_expo)) == RT_EOK) {
    pid_profile_.dterm_lpf1_dyn_expo = dterm_lpf1_dyn_expo;
  }
  
#ifdef PROJECT_BF_PID_D_MAX_EN
  // Load D_MAX parameters (same as Betaflight)
  uint8_t d_max[XYZ_AXIS_COUNT];
  std::memcpy(d_max, pid_profile_.d_max, sizeof(d_max));
  if (getParam("pid_d_max", d_max, sizeof(d_max)) == RT_EOK) {
    std::memcpy(pid_profile_.d_max, d_max, sizeof(d_max));
  }
  
  uint8_t d_max_gain = pid_profile_.d_max_gain;
  if (getParam("pid_d_max_gain", &d_max_gain, sizeof(d_max_gain)) == RT_EOK) {
    pid_profile_.d_max_gain = d_max_gain;
  }
  
  uint8_t d_max_advance = pid_profile_.d_max_advance;
  if (getParam("pid_d_max_advance", &d_max_advance, sizeof(d_max_advance)) == RT_EOK) {
    pid_profile_.d_max_advance = d_max_advance;
  }
#endif
  
  // Load Yaw P term filter parameters
  // Ensure yaw filter always has a default value (same as Betaflight)
  constexpr uint16_t YAW_LOWPASS_HZ_DEFAULT = 90;  // Default yaw lowpass frequency
  uint16_t yaw_lowpass_hz = pid_profile_.yaw_lowpass_hz;
  if (getParam("pid_yaw_lowpass_hz", &yaw_lowpass_hz, sizeof(yaw_lowpass_hz)) == RT_EOK) {
    pid_profile_.yaw_lowpass_hz = yaw_lowpass_hz;
    pid_profile_.yawLpfHz = (float)yaw_lowpass_hz;  // Update legacy value for backward compatibility
  }

  // Legacy: support old parameter name
  float yaw_lpf_hz = pid_profile_.yawLpfHz;
  if (getParam("pid_yaw_lpf_hz", &yaw_lpf_hz, sizeof(yaw_lpf_hz)) == RT_EOK) {
    pid_profile_.yawLpfHz = yaw_lpf_hz;
    pid_profile_.yaw_lowpass_hz = (uint16_t)yaw_lpf_hz;  // Update new value
  }

  // If yaw_lowpass_hz is 0 or not set, use default value (same as Betaflight)
  // This ensures yaw filter is always initialized
  if (pid_profile_.yaw_lowpass_hz == 0) {
    pid_profile_.yaw_lowpass_hz = YAW_LOWPASS_HZ_DEFAULT;
    pid_profile_.yawLpfHz = (float)YAW_LOWPASS_HZ_DEFAULT;
  }

#ifdef PROJECT_BF_ATTITUDE_EN
  // Load angle mode parameters (same as Betaflight)
  uint8_t angle_limit = pid_profile_.angle_limit;
  if (getParam("pid_angle_limit", &angle_limit, sizeof(angle_limit)) == RT_EOK) {
    pid_profile_.angle_limit = angle_limit;
  }

  uint8_t angle_earth_ref = pid_profile_.angle_earth_ref;
  if (getParam("pid_angle_earth_ref", &angle_earth_ref, sizeof(angle_earth_ref)) == RT_EOK) {
    pid_profile_.angle_earth_ref = angle_earth_ref;
  }

  float angle_p_gain = pid_profile_.angle_p_gain;
  if (getParam("pid_angle_p_gain", &angle_p_gain, sizeof(angle_p_gain)) == RT_EOK) {
    pid_profile_.angle_p_gain = angle_p_gain;
  }

  float angle_feedforward = pid_profile_.angle_feedforward;
  if (getParam("pid_angle_feedforward", &angle_feedforward, sizeof(angle_feedforward)) == RT_EOK) {
    pid_profile_.angle_feedforward = angle_feedforward;
  }

  uint8_t angle_feedforward_smoothing_ms = pid_profile_.angle_feedforward_smoothing_ms;
  if (getParam("pid_angle_feedforward_smoothing_ms", &angle_feedforward_smoothing_ms, sizeof(angle_feedforward_smoothing_ms)) == RT_EOK) {
    pid_profile_.angle_feedforward_smoothing_ms = angle_feedforward_smoothing_ms;
  }

  LOG_I("Angle mode parameters: limit=%u deg, earth_ref=%u, p_gain=%.2f, feedforward=%.2f, smoothing_ms=%u",
        pid_profile_.angle_limit, pid_profile_.angle_earth_ref,
        pid_profile_.angle_p_gain, pid_profile_.angle_feedforward,
        pid_profile_.angle_feedforward_smoothing_ms);
#endif
}

void PidBf::calculatePidCoefficients() {
  // Calculate PID coefficients from profile values
  // Apply scaling factors (same as Betaflight)
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

#ifdef PROJECT_BF_PID_D_MAX_EN
  // Calculate D_MAX percentage multipliers (same as Betaflight)
  // Reference: ref/pid_init.c pidInitConfig()
  // This must be called after calculatePidCoefficients() because it uses pid_profile_.pid[axis].D
  constexpr float D_MAX_GYRO_GAIN_FACTOR = 0.00008f;
  constexpr float D_MAX_SETPOINT_GAIN_FACTOR = 0.00008f;
  constexpr float D_MAX_LOWPASS_HZ = 35.0f;
  
  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    const uint8_t dMax = pid_profile_.d_max[axis];
    if ((pid_profile_.pid[axis].D > 0) && dMax > pid_profile_.pid[axis].D) {
      pid_runtime_.dMaxPercent[axis] = (float)dMax / pid_profile_.pid[axis].D;
      // fraction that Dmax is higher than D, eg if D is 8 and Dmax is 10, Dmax is 1.25 times bigger
    } else {
      pid_runtime_.dMaxPercent[axis] = 1.0f;
    }
  }
  
  const float dmaxLpfInv = 1.0f / D_MAX_LOWPASS_HZ; // lowpass included inversely in gain since stronger lowpass decreases peak effect
  pid_runtime_.dMaxGyroGain = D_MAX_GYRO_GAIN_FACTOR * pid_profile_.d_max_gain * dmaxLpfInv;
  pid_runtime_.dMaxSetpointGain = D_MAX_SETPOINT_GAIN_FACTOR * pid_profile_.d_max_advance * dmaxLpfInv;
  
  LOG_I("D_MAX: Roll=%u/%.1f, Pitch=%u/%.1f, Yaw=%u/%.1f, Gain=%.6f, Advance=%.6f", 
        pid_profile_.d_max[FD_ROLL], pid_runtime_.dMaxPercent[FD_ROLL],
        pid_profile_.d_max[FD_PITCH], pid_runtime_.dMaxPercent[FD_PITCH],
        pid_profile_.d_max[FD_YAW], pid_runtime_.dMaxPercent[FD_YAW],
        pid_runtime_.dMaxGyroGain, pid_runtime_.dMaxSetpointGain);
#endif

#ifdef PROJECT_BF_ATTITUDE_EN
  // Calculate angle mode gains (same as Betaflight)
  // Reference: ref/pid_init.c pidInitConfig()
  // Angle gain: divide by 10.0 (same as Betaflight: pid[PID_LEVEL].P / 10.0f)
  pid_runtime_.angleGain = pid_profile_.angle_p_gain / 10.0f;
  
  // Angle feedforward gain: divide by 100.0 (same as Betaflight: pid[PID_LEVEL].F / 100.0f)
  pid_runtime_.angleFeedforwardGain = pid_profile_.angle_feedforward / 100.0f;
  
  // Earth reference gain (0.0-1.0 from 0-100): divide by 100.0 (same as Betaflight: angle_earth_ref / 100.0f)
  pid_runtime_.angleEarthRef = pid_profile_.angle_earth_ref / 100.0f;
  
  LOG_I("Angle mode gains: angleGain=%.4f, feedforwardGain=%.4f, earthRef=%.2f", 
        pid_runtime_.angleGain, pid_runtime_.angleFeedforwardGain, pid_runtime_.angleEarthRef);
#endif
}

void PidBf::initDtermFilters() {
  // Initialize Dterm filters (same pattern as Betaflight)
  // Reference: ref/pid_init.c initPidFilters()
  
  // Get target looptime (must be loaded first via loadPidProcessDenom())
  uint32_t targetPidLooptime = target_looptime_us_;
  
  if (targetPidLooptime == 0 || pid_runtime_.dT <= 0.0f) {
    // No looptime set, disable all filters (same as Betaflight)
    pid_runtime_.dtermNotchApplyFn = nullFilterApply;
    pid_runtime_.dtermLowpassApplyFn = nullFilterApply;
    pid_runtime_.dtermLowpass2ApplyFn = nullFilterApply;
    pid_runtime_.ptermYawLowpassApplyFn = nullFilterApply;
    LOG_W("Dterm filters disabled (invalid looptime)");
    return;
  }

  const uint32_t pidFrequencyNyquist = (uint32_t)(pid_runtime_.pidFrequency / 2.0f);

  // Initialize Dterm notch filter
  uint16_t dTermNotchHz = 0;
  if (pid_profile_.dterm_notch_hz <= pidFrequencyNyquist) {
    dTermNotchHz = pid_profile_.dterm_notch_hz;
  } else {
    if (pid_profile_.dterm_notch_cutoff < pidFrequencyNyquist) {
      dTermNotchHz = pidFrequencyNyquist;
    } else {
      dTermNotchHz = 0;
    }
  }

  if (dTermNotchHz != 0 && pid_profile_.dterm_notch_cutoff != 0) {
    pid_runtime_.dtermNotchApplyFn = (filterApplyFnPtr)biquadFilterApply;
    const float notchQ = filterGetNotchQ((float)dTermNotchHz, (float)pid_profile_.dterm_notch_cutoff);
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
      biquadFilterInit(&pid_runtime_.dtermNotch[axis], (float)dTermNotchHz, targetPidLooptime, notchQ, FILTER_NOTCH, 1.0f);
    }
    LOG_I("Dterm notch filter initialized: %u Hz, Q=%.2f", dTermNotchHz, notchQ);
  } else {
    pid_runtime_.dtermNotchApplyFn = nullFilterApply;
  }

  // Initialize 1st Dterm Lowpass Filter
  uint16_t dterm_lpf1_init_hz = pid_profile_.dterm_lpf1_static_hz;
  
#ifdef USE_DYN_LPF
  // If dynamic LPF is enabled, use dyn_min_hz as initial frequency
  if (pid_profile_.dterm_lpf1_dyn_min_hz > 0) {
    dterm_lpf1_init_hz = pid_profile_.dterm_lpf1_dyn_min_hz;
  }
#endif
  
  if (dterm_lpf1_init_hz > 0) {
    switch (pid_profile_.dterm_lpf1_type) {
      case FILTER_PT1:
        pid_runtime_.dtermLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
          const float k = pt1FilterGain((float)dterm_lpf1_init_hz, pid_runtime_.dT);
          pt1FilterInit(&pid_runtime_.dtermLowpass[axis].pt1Filter, k);
        }
        LOG_I("Dterm LPF1 initialized: PT1, %u Hz", dterm_lpf1_init_hz);
        break;
        
      case FILTER_BIQUAD:
        if (pid_profile_.dterm_lpf1_static_hz < pidFrequencyNyquist) {
          pid_runtime_.dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApply;
          for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            biquadFilterInitLPF(&pid_runtime_.dtermLowpass[axis].biquadFilter, (float)dterm_lpf1_init_hz, targetPidLooptime);
          }
          LOG_I("Dterm LPF1 initialized: BIQUAD, %u Hz", dterm_lpf1_init_hz);
        } else {
          pid_runtime_.dtermLowpassApplyFn = nullFilterApply;
          LOG_W("Dterm LPF1 disabled: frequency >= Nyquist");
        }
        break;
        
      case FILTER_PT2:
        pid_runtime_.dtermLowpassApplyFn = (filterApplyFnPtr)pt2FilterApply;
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
          const float k = pt2FilterGain((float)dterm_lpf1_init_hz, pid_runtime_.dT);
          pt2FilterInit(&pid_runtime_.dtermLowpass[axis].pt2Filter, k);
        }
        LOG_I("Dterm LPF1 initialized: PT2, %u Hz", dterm_lpf1_init_hz);
        break;
        
      case FILTER_PT3:
        pid_runtime_.dtermLowpassApplyFn = (filterApplyFnPtr)pt3FilterApply;
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
          const float k = pt3FilterGain((float)dterm_lpf1_init_hz, pid_runtime_.dT);
          pt3FilterInit(&pid_runtime_.dtermLowpass[axis].pt3Filter, k);
        }
        LOG_I("Dterm LPF1 initialized: PT3, %u Hz", dterm_lpf1_init_hz);
        break;
        
      default:
        pid_runtime_.dtermLowpassApplyFn = nullFilterApply;
        LOG_W("Dterm LPF1 disabled: unknown filter type %u", pid_profile_.dterm_lpf1_type);
        break;
    }
  } else {
    pid_runtime_.dtermLowpassApplyFn = nullFilterApply;
  }

  // Initialize 2nd Dterm Lowpass Filter
  if (pid_profile_.dterm_lpf2_static_hz > 0) {
    switch (pid_profile_.dterm_lpf2_type) {
      case FILTER_PT1:
        pid_runtime_.dtermLowpass2ApplyFn = (filterApplyFnPtr)pt1FilterApply;
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
          const float k = pt1FilterGain((float)pid_profile_.dterm_lpf2_static_hz, pid_runtime_.dT);
          pt1FilterInit(&pid_runtime_.dtermLowpass2[axis].pt1Filter, k);
        }
        LOG_I("Dterm LPF2 initialized: PT1, %u Hz", pid_profile_.dterm_lpf2_static_hz);
        break;
        
      case FILTER_BIQUAD:
        if (pid_profile_.dterm_lpf2_static_hz < pidFrequencyNyquist) {
          pid_runtime_.dtermLowpass2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
          for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            biquadFilterInitLPF(&pid_runtime_.dtermLowpass2[axis].biquadFilter, (float)pid_profile_.dterm_lpf2_static_hz, targetPidLooptime);
          }
          LOG_I("Dterm LPF2 initialized: BIQUAD, %u Hz", pid_profile_.dterm_lpf2_static_hz);
        } else {
          pid_runtime_.dtermLowpass2ApplyFn = nullFilterApply;
          LOG_W("Dterm LPF2 disabled: frequency >= Nyquist");
        }
        break;
        
      case FILTER_PT2:
        pid_runtime_.dtermLowpass2ApplyFn = (filterApplyFnPtr)pt2FilterApply;
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
          const float k = pt2FilterGain((float)pid_profile_.dterm_lpf2_static_hz, pid_runtime_.dT);
          pt2FilterInit(&pid_runtime_.dtermLowpass2[axis].pt2Filter, k);
        }
        LOG_I("Dterm LPF2 initialized: PT2, %u Hz", pid_profile_.dterm_lpf2_static_hz);
        break;
        
      case FILTER_PT3:
        pid_runtime_.dtermLowpass2ApplyFn = (filterApplyFnPtr)pt3FilterApply;
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
          const float k = pt3FilterGain((float)pid_profile_.dterm_lpf2_static_hz, pid_runtime_.dT);
          pt3FilterInit(&pid_runtime_.dtermLowpass2[axis].pt3Filter, k);
        }
        LOG_I("Dterm LPF2 initialized: PT3, %u Hz", pid_profile_.dterm_lpf2_static_hz);
        break;
        
      default:
        pid_runtime_.dtermLowpass2ApplyFn = nullFilterApply;
        LOG_W("Dterm LPF2 disabled: unknown filter type %u", pid_profile_.dterm_lpf2_type);
        break;
    }
  } else {
    pid_runtime_.dtermLowpass2ApplyFn = nullFilterApply;
  }

  // Initialize Yaw P term Lowpass Filter
  // yaw_lowpass_hz is guaranteed to have a default value (set in loadPidParameters)
  // so the filter is always initialized (same as Betaflight)
  pid_runtime_.ptermYawLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
  const float k = pt1FilterGain((float)pid_profile_.yaw_lowpass_hz, pid_runtime_.dT);
  pt1FilterInit(&pid_runtime_.ptermYawLowpass, k);
  LOG_I("Yaw P term lowpass filter initialized: PT1, %u Hz", pid_profile_.yaw_lowpass_hz);

#ifdef USE_DYN_LPF
  // Initialize Dynamic LPF (same as Betaflight)
  if (pid_profile_.dterm_lpf1_dyn_min_hz > 0) {
    switch (pid_profile_.dterm_lpf1_type) {
      case FILTER_PT1:
        pid_runtime_.dynLpfFilter = DYN_LPF_PT1;
        break;
      case FILTER_BIQUAD:
        pid_runtime_.dynLpfFilter = DYN_LPF_BIQUAD;
        break;
      case FILTER_PT2:
        pid_runtime_.dynLpfFilter = DYN_LPF_PT2;
        break;
      case FILTER_PT3:
        pid_runtime_.dynLpfFilter = DYN_LPF_PT3;
        break;
      default:
        pid_runtime_.dynLpfFilter = DYN_LPF_NONE;
        break;
    }
  } else {
    pid_runtime_.dynLpfFilter = DYN_LPF_NONE;
  }
  pid_runtime_.dynLpfMin = pid_profile_.dterm_lpf1_dyn_min_hz;
  pid_runtime_.dynLpfMax = pid_profile_.dterm_lpf1_dyn_max_hz;
  pid_runtime_.dynLpfCurveExpo = pid_profile_.dterm_lpf1_dyn_expo;
#endif

#ifdef PROJECT_BF_PID_D_MAX_EN
  // Initialize D_MAX filters and parameters (same as Betaflight)
  // Reference: ref/pid_init.c initPidFilters()
  // Initialize the filters for all axis even if the d_max[axis] value is 0
  // Otherwise if the d_max_xxx parameters are ever added to in-flight adjustments
  // and transition from 0 to > 0 in flight the feature won't work because the filter wasn't initialized.
  constexpr float D_MAX_RANGE_HZ = 85.0f;    // PT2 lowpass input cutoff to peak D around propwash frequencies
  constexpr float D_MAX_LOWPASS_HZ = 35.0f;  // PT2 lowpass cutoff to smooth the boost effect
  
  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    const float k_range = pt2FilterGain(D_MAX_RANGE_HZ, pid_runtime_.dT);
    pt2FilterInit(&pid_runtime_.dMaxRange[axis], k_range);
    
    const float k_lowpass = pt2FilterGain(D_MAX_LOWPASS_HZ, pid_runtime_.dT);
    pt2FilterInit(&pid_runtime_.dMaxLowpass[axis], k_lowpass);
  }
  
  // Note: dMaxPercent, dMaxGyroGain, and dMaxSetpointGain are calculated in calculatePidCoefficients()
  // because they depend on pid_profile_.pid[axis].D which is loaded in loadPidParameters()
  // Only the filters are initialized here
  LOG_I("D_MAX filters initialized");
#endif
}

void PidBf::loadPidProcessDenom() {
  // Load PID process denominator (controls PID loop frequency)
  // pid_process_denom = 1: 3.2kHz (312us)
  // pid_process_denom = 2: 1.6kHz (624us)
  // pid_process_denom = 3: 1.07kHz (936us)
  // etc.
  uint8_t pid_process_denom = 1;
  if (getParam("pid_process_denom", &pid_process_denom, sizeof(pid_process_denom)) != RT_EOK) {
    pid_process_denom = 1;
  }

  // Calculate target looptime based on denominator
  target_looptime_us_ = 312;  // Default 3.2kHz
  if (pid_process_denom > 1) {
    target_looptime_us_ *= pid_process_denom;
  }

  // Update runtime parameters
  pid_runtime_.dT = target_looptime_us_ * 1e-6f;
  pid_runtime_.pidFrequency = (pid_runtime_.dT > 0.0f) ? 1.0f / pid_runtime_.dT : 0.0f;

  LOG_I("Target looptime: %u us, dT: %.6f s, frequency: %.1f Hz", target_looptime_us_, pid_runtime_.dT,
        pid_runtime_.pidFrequency);
}

void PidBf::initAngleModeFilters() {
#ifdef PROJECT_BF_ATTITUDE_EN
  // Initialize angle mode filters (same as Betaflight)
  // Reference: ref/pid_init.c pidInitFilters()
  constexpr float ATTITUDE_CUTOFF_HZ = 50.0f;  // Betaflight default
  const float k = pt3FilterGain(ATTITUDE_CUTOFF_HZ, pid_runtime_.dT);
  
  // Angle feedforward filter cutoff frequency (from parameter)
  // Use angle_feedforward_smoothing_ms parameter for cutoff frequency calculation
  const float angleCutoffHz = 1000.0f / (2.0f * PI_F * pid_profile_.angle_feedforward_smoothing_ms);
  const float k2 = pt3FilterGain(angleCutoffHz, pid_runtime_.dT);
  
  // Initialize attitude filter (PT3) for roll and pitch only
  for (int axis = 0; axis < 2; axis++) {
    pt3FilterInit(&pid_runtime_.attitudeFilter[axis], k);
    pt3FilterInit(&pid_runtime_.angleFeedforwardPt3[axis], k2);
  }
  // Initialize angle feedforward filter for yaw (though yaw doesn't use angle mode)
  pt3FilterInit(&pid_runtime_.angleFeedforwardPt3[FD_YAW], k2);
  
  pid_runtime_.angleYawSetpoint = 0.0f;
  
  LOG_I("Angle mode filters initialized: attitude_cutoff=%.1f Hz, feedforward_cutoff=%.1f Hz", 
        ATTITUDE_CUTOFF_HZ, angleCutoffHz);
#endif
}

// Singleton instance
PidBf& PidBf::instance() {
  static PidBf instance_obj;
  return instance_obj;
}

// Constructor
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
  // Initialize data structures to zero
  std::memset(&gyro_filtered_data_, 0, sizeof(gyro_filtered_data_));
  std::memset(&setpoint_data_, 0, sizeof(setpoint_data_));
  std::memset(&rc_command_data_, 0, sizeof(rc_command_data_));
  std::memset(&rc_command_data_cached_, 0, sizeof(rc_command_data_cached_));
  std::memset(&pid_runtime_, 0, sizeof(pid_runtime_));
  std::memset(pid_data_, 0, sizeof(pid_data_));
  std::memset(&pid_profile_, 0, sizeof(pid_profile_));
  std::memset(&main_thread_obj_, 0, sizeof(main_thread_obj_));
  std::memset(main_thread_stack_, 0, sizeof(main_thread_stack_));
  std::memset(max_rc_rate_, 0, sizeof(max_rc_rate_));

#ifdef PROJECT_BF_ATTITUDE_EN
  attitude_data_valid_ = false;
#endif

  // Initialize defaults and runtime state (moved to init functions)
  initDefaults();
  initRuntime();
}

// Destructor
PidBf::~PidBf() { cleanupMcnSubscriptions(); }

// Initialize PID module
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

  // Initialize mlog
  ret = initMlog();
  if (ret != RT_EOK) {
    LOG_E("Mlog init failed");
    return ret;
  }

  // 注意：initSyncNotify(INIT_SYNC_PID) 已移到 workerEntry 中，在等待 Gyro Filter 之后

  LOG_I("PidBf initialized (no thread - using taskPid.cpp main thread)");
  return RT_EOK;
}

// Initialize PID configuration
void PidBf::initConfig() {
  // Load PID parameters from param system
  loadPidParameters();
  
  // Calculate PID coefficients and limits from loaded parameters
  calculatePidCoefficients();
}

// Initialize PID filters
void PidBf::initFilters() {
  // Initialize all Dterm filters and Yaw P term filter (same pattern as Betaflight)
  // initDtermFilters() is now in pid_init.cpp
  initDtermFilters();

  // Initialize angle mode filters (moved to pid_init.cpp)
  initAngleModeFilters();

  LOG_I("PID filters initialized");
}

// Main thread loop
void PidBf::pidMainLoop() {
  LOG_I("PidMain loop started");

  while (true) {
#ifdef PROJECT_BF_PID_DEBUG_PIN_EN
    // DEBUG_PIN_DEBUG1_HIGH();  // Debug pin: PID task execution start (monitor PID task frequency ~3.2kHz)
#endif
    uint32_t current_time_us = timestamp_micros();
    subTaskRcCommand(current_time_us);
    processPidController(current_time_us);
#ifdef PROJECT_BF_PID_DEBUG_PIN_EN
    // DEBUG_PIN_DEBUG1_LOW();  // Debug pin: PID task execution end
#endif
  }
}

// Worker entry function
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
  
  // 等待 RC 模块初始化完成，然后缓存 max RC rates
  ret = initSyncWait(INIT_SYNC_RC, 2000);  // 等待最多2秒
  if (ret != RT_EOK) {
    LOG_W("RC not ready, using default max RC rates (ret=%d)", ret);
    // 使用默认值
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
      self->max_rc_rate_[axis] = 720.0f;  // 默认值
    }
  } else {
    // 从 RC 模块获取并缓存 max RC rates
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
      self->max_rc_rate_[axis] = RcBf::instance().getMaxRcRate(axis);
    }
    LOG_I("Cached max RC rates: Roll=%.1f, Pitch=%.1f, Yaw=%.1f deg/s",
          self->max_rc_rate_[0], self->max_rc_rate_[1], self->max_rc_rate_[2]);
  }
  
#ifdef PROJECT_BF_ATTITUDE_EN
  // 等待 Attitude 模块初始化完成（角度模式需要姿态数据）
  ret = initSyncWait(INIT_SYNC_ATTITUDE, 2000);  // 等待最多2秒
  if (ret != RT_EOK) {
    LOG_W("Attitude not ready, angle mode will be disabled (ret=%d)", ret);
  }
#endif
  
  // 通知 PID 初始化完成（在等待依赖模块之后）
  initSyncNotify(INIT_SYNC_PID);
  
  // 进入主循环
  self->pidMainLoop();
}

// Start main PID thread
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

