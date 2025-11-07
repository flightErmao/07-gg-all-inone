#include "rtthread.h"
#include <rtdevice.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "stateControl.h"
#include "pidMinifly.h"
#include "pidpx4.h"
#include "taskParam.h"
#include "timestamp.h"
#include "param.h"

#define UPARAM_DEBUG

#define LOG_TAG "stateCtrl"
#ifdef UPARAM_DEBUG
#define LOG_LVL LOG_LVL_DBG
#else
#define LOG_LVL LOG_LVL_WARNING
#endif
#include <ulog.h>

static float actualThrust_;
static attitude_t attitudeDesired_;
static attitude_t rateDesired_;
static control_t controlOutputLogged_;

/* PX4 风格速率控制器 */
static RateControlPX4 g_rate_ctrl_px4;

PidObject pidAngleRoll;
PidObject pidAnglePitch;
PidObject pidAngleYaw;

PidObject pidRateRoll;
PidObject pidRatePitch;
PidObject pidRateYaw;

typedef struct {
  float acro_r_max;
  float acro_p_max;
  float acro_y_max;
  float acro_expo;
  float acro_expo_y;
  float acro_supexpo;
  float acro_supexpo_y;
} manual_rate_cfg_t;

static manual_rate_cfg_t g_manual_rate_cfg = {0};
static float g_rate_integral_limit[3] = {0.f, 0.f, 0.f};
static float g_rate_ff_gain[3] = {0.f, 0.f, 0.f};
static float g_rate_extra_gain[3] = {0.f, 0.f, 0.f};
static float g_yaw_tq_cutoff = 0.f;

static float load_param_float(const char *name) {
  float value = 0.f;
  RT_ASSERT(getParam(name, &value, sizeof(value)) == RT_EOK);
  return value;
}

static inline int16_t pidOutLimit(float in) {
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

/* PX4 math: superexpo 曲线映射（stick ∈ [-1,1]）*/
static inline float superexpo_map(float stick_input, float expo, float supexpo) {
  if (expo <= 0.0f) {
    return stick_input;
  }
  float expo_stick = stick_input * (1.0f - expo) + stick_input * stick_input * stick_input * expo;
  if (supexpo <= 0.0f) {
    return expo_stick;
  }
  return expo_stick * (1.0f - supexpo) + expo_stick * expo_stick * expo_stick * supexpo;
}

/* 将 RC 输入映射为角速度期望（deg/s） */
static inline void manualRateFromRc(const setpoint_t* setpoint, attitude_t* rate_sp_out) {
  /* 读取 RC stick，假设已归一化到 [-1, 1]，若上游未归一化，则应在 RC 层做缩放 */
  float stick_roll = setpoint->attitude.roll;   /* [-1, 1] */
  float stick_pitch = setpoint->attitude.pitch; /* [-1, 1] */
  float stick_yaw = setpoint->attitude.yaw;     /* [-1, 1] */

  /* expo/superexpo 映射（与 PX4 一致，yaw 可使用独立 expo）*/
  float roll_cmd = superexpo_map(stick_roll, g_manual_rate_cfg.acro_expo, g_manual_rate_cfg.acro_supexpo);
  float pitch_cmd = superexpo_map(stick_pitch, g_manual_rate_cfg.acro_expo, g_manual_rate_cfg.acro_supexpo);
  float yaw_cmd = superexpo_map(stick_yaw, g_manual_rate_cfg.acro_expo_y, g_manual_rate_cfg.acro_supexpo_y);

  /* 最大角速率（deg/s）*/
  rate_sp_out->roll = roll_cmd * g_manual_rate_cfg.acro_r_max;
  rate_sp_out->pitch = pitch_cmd * g_manual_rate_cfg.acro_p_max;
  rate_sp_out->yaw = yaw_cmd * g_manual_rate_cfg.acro_y_max;

  // static int cnt = 0;
  // cnt++;
  // if (cnt > 10) {
  //   LOG_D("RC->Rate: [%.2f, %.2f, %.2f]", rate_sp_out->roll, rate_sp_out->pitch, rate_sp_out->yaw);
  //   cnt = 0;
  // }
}

void attitudeControlInit(float ratePidDt, float anglePidDt) {
  float angle_pid_roll_cfg[3];
  float angle_pid_pitch_cfg[3];
  float angle_pid_yaw_cfg[3];
  RT_ASSERT(getParam("angle_pid_roll", angle_pid_roll_cfg, sizeof(angle_pid_roll_cfg)) == RT_EOK);
  RT_ASSERT(getParam("angle_pid_pitch", angle_pid_pitch_cfg, sizeof(angle_pid_pitch_cfg)) == RT_EOK);
  RT_ASSERT(getParam("angle_pid_yaw", angle_pid_yaw_cfg, sizeof(angle_pid_yaw_cfg)) == RT_EOK);
  float angle_pid_roll_i_limit = load_param_float("angle_pid_roll_i_limit");
  float angle_pid_pitch_i_limit = load_param_float("angle_pid_pitch_i_limit");
  float angle_pid_yaw_i_limit = load_param_float("angle_pid_yaw_i_limit");

  float mc_rollrate_p = load_param_float("mc_rollrate_p");
  float mc_rollrate_i = load_param_float("mc_rollrate_i");
  float mc_rr_int_lim = load_param_float("mc_rr_int_lim");
  float mc_rollrate_d = load_param_float("mc_rollrate_d");
  float mc_rollrate_ff = load_param_float("mc_rollrate_ff");
  float mc_rollrate_k = load_param_float("mc_rollrate_k");

  float mc_pitchrate_p = load_param_float("mc_pitchrate_p");
  float mc_pitchrate_i = load_param_float("mc_pitchrate_i");
  float mc_pr_int_lim = load_param_float("mc_pr_int_lim");
  float mc_pitchrate_d = load_param_float("mc_pitchrate_d");
  float mc_pitchrate_ff = load_param_float("mc_pitchrate_ff");
  float mc_pitchrate_k = load_param_float("mc_pitchrate_k");

  float mc_yawrate_p = load_param_float("mc_yawrate_p");
  float mc_yawrate_i = load_param_float("mc_yawrate_i");
  float mc_yr_int_lim = load_param_float("mc_yr_int_lim");
  float mc_yawrate_d = load_param_float("mc_yawrate_d");
  float mc_yawrate_ff = load_param_float("mc_yawrate_ff");
  float mc_yawrate_k = load_param_float("mc_yawrate_k");
  g_yaw_tq_cutoff = load_param_float("mc_yaw_tq_cutoff");

  g_manual_rate_cfg.acro_r_max = load_param_float("mc_acro_r_max");
  g_manual_rate_cfg.acro_p_max = load_param_float("mc_acro_p_max");
  g_manual_rate_cfg.acro_y_max = load_param_float("mc_acro_y_max");
  g_manual_rate_cfg.acro_expo = load_param_float("mc_acro_expo");
  g_manual_rate_cfg.acro_expo_y = load_param_float("mc_acro_expo_y");
  g_manual_rate_cfg.acro_supexpo = load_param_float("mc_acro_supexpo");
  g_manual_rate_cfg.acro_supexpo_y = load_param_float("mc_acro_supexpo_y");

  g_rate_integral_limit[0] = mc_rr_int_lim;
  g_rate_integral_limit[1] = mc_pr_int_lim;
  g_rate_integral_limit[2] = mc_yr_int_lim;

  g_rate_ff_gain[0] = mc_rollrate_ff;
  g_rate_ff_gain[1] = mc_pitchrate_ff;
  g_rate_ff_gain[2] = mc_yawrate_ff;

  g_rate_extra_gain[0] = mc_rollrate_k;
  g_rate_extra_gain[1] = mc_pitchrate_k;
  g_rate_extra_gain[2] = mc_yawrate_k;

  pidInit_t pid_angle_roll = {.kp = angle_pid_roll_cfg[0], .ki = angle_pid_roll_cfg[1], .kd = angle_pid_roll_cfg[2]};
  pidInit_t pid_angle_pitch = {
      .kp = angle_pid_pitch_cfg[0], .ki = angle_pid_pitch_cfg[1], .kd = angle_pid_pitch_cfg[2]};
  pidInit_t pid_angle_yaw = {.kp = angle_pid_yaw_cfg[0], .ki = angle_pid_yaw_cfg[1], .kd = angle_pid_yaw_cfg[2]};
  pidInit_t pid_rate_roll = {.kp = mc_rollrate_p, .ki = mc_rollrate_i, .kd = mc_rollrate_d};
  pidInit_t pid_rate_pitch = {.kp = mc_pitchrate_p, .ki = mc_pitchrate_i, .kd = mc_pitchrate_d};
  pidInit_t pid_rate_yaw = {.kp = mc_yawrate_p, .ki = mc_yawrate_i, .kd = mc_yawrate_d};

  pidInit(&pidAngleRoll, 0, pid_angle_roll, anglePidDt);
  pidInit(&pidAnglePitch, 0, pid_angle_pitch, anglePidDt);
  pidInit(&pidAngleYaw, 0, pid_angle_yaw, anglePidDt);
  pidSetIntegralLimit(&pidAngleRoll, angle_pid_roll_i_limit);
  pidSetIntegralLimit(&pidAnglePitch, angle_pid_pitch_i_limit);
  pidSetIntegralLimit(&pidAngleYaw, angle_pid_yaw_i_limit);

  pidInit(&pidRateRoll, 0, pid_rate_roll, ratePidDt);
  pidInit(&pidRatePitch, 0, pid_rate_pitch, ratePidDt);
  pidInit(&pidRateYaw, 0, pid_rate_yaw, ratePidDt);
  pidSetIntegralLimit(&pidRateRoll, g_rate_integral_limit[0]);
  pidSetIntegralLimit(&pidRatePitch, g_rate_integral_limit[1]);
  pidSetIntegralLimit(&pidRateYaw, g_rate_integral_limit[2]);

  /* 初始化 PX4 速率控制器参数（沿用现有 Rate PID 的 P/I/D 配置） */
  rcpx4_init(&g_rate_ctrl_px4);
  float P[3] = {pid_rate_roll.kp, pid_rate_pitch.kp, pid_rate_yaw.kp};
  float I[3] = {pid_rate_roll.ki, pid_rate_pitch.ki, pid_rate_yaw.ki};
  float D[3] = {pid_rate_roll.kd, pid_rate_pitch.kd, pid_rate_yaw.kd};
  rcpx4_set_pid(&g_rate_ctrl_px4, P, I, D);
  rcpx4_set_integrator_limit(&g_rate_ctrl_px4, g_rate_integral_limit);
  rcpx4_set_ff(&g_rate_ctrl_px4, g_rate_ff_gain);
}

static float getDtForRatePid() {
  uint32_t now_time_us = timestamp_micros();
  static uint32_t last_time_us = 0;
  float dt_s = (now_time_us - last_time_us) / 1000000.0f;
  dt_s = fmaxf(fminf(dt_s, 0.01f), 0.0005f);
  last_time_us = now_time_us;
  return dt_s;
}

void ratePid(const Axis3f* actualRate, const attitude_t* desiredRate, const Axis3f* angularAccel, control_t* output) {
  float rate[3] = {actualRate->x, actualRate->y, actualRate->z};
  float rate_sp[3] = {desiredRate->roll, desiredRate->pitch, desiredRate->yaw};
  float angular_accel_vec[3] = {0.f, 0.f, 0.f};
  if (angularAccel != NULL) {
    angular_accel_vec[0] = angularAccel->x;
    angular_accel_vec[1] = angularAccel->y;
    angular_accel_vec[2] = angularAccel->z;
  }
  float torque[3] = {0.f, 0.f, 0.f};

  bool landed = (actualThrust_ < 5.0f);
  float dt = getDtForRatePid();
  rcpx4_update(&g_rate_ctrl_px4, rate, rate_sp, angular_accel_vec, dt, landed, torque);

  // static uint16_t cnt = 0;
  // cnt++;
  // if (cnt > 10) {
  //   LOG_D("dt = %.6f int = [%.2f, %.2f, %.2f]", dt, g_rate_ctrl_px4.rate_int[0], g_rate_ctrl_px4.rate_int[1],
  //         g_rate_ctrl_px4.rate_int[2]);
  //   cnt = 0;
  // }

  torque[0] += g_rate_extra_gain[0] * rate_sp[0];
  torque[1] += g_rate_extra_gain[1] * rate_sp[1];
  torque[2] += g_rate_extra_gain[2] * rate_sp[2];

  if (fabsf(torque[2]) < g_yaw_tq_cutoff) {
    torque[2] = 0.f;
  }

  output->roll = pidOutLimit(torque[0]);
  output->pitch = pidOutLimit(torque[1]);
  output->yaw = pidOutLimit(torque[2]);

  controlOutputLogged_.roll = output->roll;
  controlOutputLogged_.pitch = output->pitch;
  controlOutputLogged_.yaw = output->yaw;
}

void anglePid(const attitude_t* actualAngle, const attitude_t* desiredAngle, attitude_t* outDesiredRate) {
  outDesiredRate->roll = pidUpdate(&pidAngleRoll, desiredAngle->roll - actualAngle->roll);
  outDesiredRate->pitch = pidUpdate(&pidAnglePitch, desiredAngle->pitch - actualAngle->pitch);

  float yawError = desiredAngle->yaw - actualAngle->yaw;
  if (yawError > 180.0f)
    yawError -= 360.0f;
  else if (yawError < -180.0)
    yawError += 360.0f;
  outDesiredRate->yaw = pidUpdate(&pidAngleYaw, yawError);
}

void attitudeControllerResetRollAttitudePID(void) { pidReset(&pidAngleRoll); }
void attitudeControllerResetPitchAttitudePID(void) { pidReset(&pidAnglePitch); }

void attitudeResetAllPID(void) {
  pidReset(&pidAngleRoll);
  pidReset(&pidAnglePitch);
  pidReset(&pidAngleYaw);
  pidReset(&pidRateRoll);
  pidReset(&pidRatePitch);
  pidReset(&pidRateYaw);
  rcpx4_reset_integral(&g_rate_ctrl_px4);
}

void attitudePIDwriteToConfigParam(void) {
  /* Intentionally left unimplemented (same as original) */
}

/* PID debug data access functions */
void getAnglePidRollDebug(float* outP, float* outI, float* outD) {
  if (outP) *outP = pidAngleRoll.outP;
  if (outI) *outI = pidAngleRoll.outI;
  if (outD) *outD = pidAngleRoll.outD;
}

void getAnglePidPitchDebug(float* outP, float* outI, float* outD) {
  if (outP) *outP = pidAnglePitch.outP;
  if (outI) *outI = pidAnglePitch.outI;
  if (outD) *outD = pidAnglePitch.outD;
}

void getAnglePidYawDebug(float* outP, float* outI, float* outD) {
  if (outP) *outP = pidAngleYaw.outP;
  if (outI) *outI = pidAngleYaw.outI;
  if (outD) *outD = pidAngleYaw.outD;
}

void getRatePidRollDebug(float* outP, float* outI, float* outD) {
  if (outP) *outP = pidRateRoll.outP;
  if (outI) *outI = pidRateRoll.outI;
  if (outD) *outD = pidRateRoll.outD;
}

void getRatePidPitchDebug(float* outP, float* outI, float* outD) {
  if (outP) *outP = pidRatePitch.outP;
  if (outI) *outI = pidRatePitch.outI;
  if (outD) *outD = pidRatePitch.outD;
}

void getRatePidYawDebug(float* outP, float* outI, float* outD) {
  if (outP) *outP = pidRateYaw.outP;
  if (outI) *outI = pidRateYaw.outI;
  if (outD) *outD = pidRateYaw.outD;
}

void getPx4RatePidInt(float* rollOutInt, float* pitchOutInt, float* yawOutInt) {
  if (rollOutInt) *rollOutInt = g_rate_ctrl_px4.rate_int[0];
  if (pitchOutInt) *pitchOutInt = g_rate_ctrl_px4.rate_int[1];
  if (yawOutInt) *yawOutInt = g_rate_ctrl_px4.rate_int[2];
}

void stateControlInit(void) { attitudeControlInit(RATE_PID_DT, ANGLE_PID_DT); /*初始化姿态PID*/ }

static bool resetControl(const state_t *state, const setpoint_t *setpoint, control_t *control) {
  static uint16_t cnt = 0;

  if (setpoint->thrust < 5.f || !setpoint->armed) {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    control->thrust = 0;
    controlOutputLogged_ = *control;
    attitudeResetAllPID();
    attitudeDesired_.yaw = state->attitude.yaw;
    rcpx4_reset_integral(&g_rate_ctrl_px4);
    if (cnt++ > 1500) {
      cnt = 0;
      // configParamGiveSemaphore();
    }
    return true;
  } else {
    cnt = 0;
  }

  return false;
}

static void generateAttitudeDesired(const setpoint_t* setpoint, const uint32_t tick) {
  if (setpoint->fly_mode == FLYER_MODE_STABLIZE) {
    attitudeDesired_.roll = setpoint->attitude.roll;
    attitudeDesired_.pitch = setpoint->attitude.pitch;
  }

  attitudeDesired_.yaw += setpoint->attitude.yaw / RATE_250_HZ;
  if (attitudeDesired_.yaw > 180.0f) attitudeDesired_.yaw -= 360.0f;
  if (attitudeDesired_.yaw < -180.0f) attitudeDesired_.yaw += 360.0f;
}

void stateControl(const state_t* state, const setpoint_t* setpoint, control_t* control, const uint32_t tick) {
  if (resetControl(state, setpoint, control)) {
    return;
  } else {
    actualThrust_ = setpoint->thrust;
  }

  if (RATE_DO_EXECUTE(RATE_250_HZ, tick)) {
    generateAttitudeDesired(setpoint, tick);
    if (setpoint->fly_mode == FLYER_MODE_STABLIZE) {
      anglePid(&state->attitude, &attitudeDesired_, &rateDesired_);
    } else {
      manualRateFromRc(setpoint, &rateDesired_);
    }
  }

  if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
    ratePid(&state->gyro_filter, &rateDesired_, &state->angular_accel, control);
  }

  control->thrust = actualThrust_;
  controlOutputLogged_.thrust = control->thrust;
}

void getAngleDesired(attitude_t* get) {
  get->pitch = attitudeDesired_.pitch;
  get->roll = attitudeDesired_.roll;
  get->yaw = attitudeDesired_.yaw;
}

void getRateDesired(attitude_t *get) {
  get->pitch = rateDesired_.pitch;
  get->roll = rateDesired_.roll;
  get->yaw = rateDesired_.yaw;
}

void getControlOutput(control_t* out) {
  if (!out) {
    return;
  }
  *out = controlOutputLogged_;
}