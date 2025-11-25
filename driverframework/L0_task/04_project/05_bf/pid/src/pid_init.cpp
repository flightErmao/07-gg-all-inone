#include "pid_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "pid_init"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "param.h"
#include "filter.h"  // For filter functions: pt1FilterInit, pt2FilterInit, pt3FilterInit, biquadFilterInit, etc.
#include "maths.h"   // For M_PIf (if needed)
}

// PID constants from ref/pid.h
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f
#define FEEDFORWARD_SCALE 0.013754f

// Dynamic LPF cutoff frequency calculation (same as Betaflight)
// Reference: ref/pid.c dynLpfCutoffFreq()
float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo) {
  const float expof = expo / 10.0f;
  const float curve = throttle * (1.0f - throttle) * expof + throttle;
  return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
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
  if (getParam("pid_sum_limit", &pid_sum_limit, sizeof(pid_sum_limit)) == RT_EOK) {
    pid_profile_.pidSumLimit = pid_sum_limit;
  }
  float pid_sum_limit_yaw = pid_profile_.pidSumLimitYaw;
  if (getParam("pid_sum_limit_yaw", &pid_sum_limit_yaw, sizeof(pid_sum_limit_yaw)) == RT_EOK) {
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
  if (pid_profile_.yaw_lowpass_hz == 0) {
    pid_runtime_.ptermYawLowpassApplyFn = nullFilterApply;
  } else {
    pid_runtime_.ptermYawLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
    const float k = pt1FilterGain((float)pid_profile_.yaw_lowpass_hz, pid_runtime_.dT);
    pt1FilterInit(&pid_runtime_.ptermYawLowpass, k);
    LOG_I("Yaw P term lowpass filter initialized: PT1, %u Hz", pid_profile_.yaw_lowpass_hz);
  }

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

