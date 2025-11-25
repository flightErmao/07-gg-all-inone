#include "pid_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "pid_param"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "param.h"
}

// PID constants from ref/pid.h
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f
#define FEEDFORWARD_SCALE 0.013754f

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

  float dterm_lpf_hz = pid_profile_.dtermLpfHz;
  if (getParam("pid_dterm_lpf_hz", &dterm_lpf_hz, sizeof(dterm_lpf_hz)) == RT_EOK) {
    pid_profile_.dtermLpfHz = dterm_lpf_hz;
  }
  
  float yaw_lpf_hz = pid_profile_.yawLpfHz;
  if (getParam("pid_yaw_lpf_hz", &yaw_lpf_hz, sizeof(yaw_lpf_hz)) == RT_EOK) {
    pid_profile_.yawLpfHz = yaw_lpf_hz;
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
}

