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

static RateControlPX4 g_rate_ctrl_px4;
static manual_rate_cfg_t g_manual_rate_cfg = {0};
static float g_yaw_tq_cutoff = 0.f;
static float g_rate_integral_limit[3] = {0.f, 0.f, 0.f};
static float g_rate_extra_gain[3] = {0.f, 0.f, 0.f};

static inline int16_t pidOutLimit(float in) {
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

static void pidParamInit(void) {
  g_rate_ctrl_px4.gain_p[AXIS_ROLL] = load_param_float("mc_rollrate_p");
  g_rate_ctrl_px4.gain_i[AXIS_ROLL] = load_param_float("mc_rollrate_i");
  g_rate_integral_limit[AXIS_ROLL] = load_param_float("mc_rr_int_lim");
  g_rate_ctrl_px4.lim_int[AXIS_ROLL] = g_rate_integral_limit[AXIS_ROLL];
  g_rate_ctrl_px4.gain_d[AXIS_ROLL] = load_param_float("mc_rollrate_d");
  g_rate_ctrl_px4.gain_ff[AXIS_ROLL] = load_param_float("mc_rollrate_ff");
  g_rate_extra_gain[AXIS_ROLL] = load_param_float("mc_rollrate_k");

  g_rate_ctrl_px4.gain_p[AXIS_PITCH] = load_param_float("mc_pitchrate_p");
  g_rate_ctrl_px4.gain_i[AXIS_PITCH] = load_param_float("mc_pitchrate_i");
  g_rate_integral_limit[AXIS_PITCH] = load_param_float("mc_pr_int_lim");
  g_rate_ctrl_px4.lim_int[AXIS_PITCH] = g_rate_integral_limit[AXIS_PITCH];
  g_rate_ctrl_px4.gain_d[AXIS_PITCH] = load_param_float("mc_pitchrate_d");
  g_rate_ctrl_px4.gain_ff[AXIS_PITCH] = load_param_float("mc_pitchrate_ff");
  g_rate_extra_gain[AXIS_PITCH] = load_param_float("mc_pitchrate_k");

  g_rate_ctrl_px4.gain_p[AXIS_YAW] = load_param_float("mc_yawrate_p");
  g_rate_ctrl_px4.gain_i[AXIS_YAW] = load_param_float("mc_yawrate_i");
  g_rate_integral_limit[AXIS_YAW] = load_param_float("mc_yr_int_lim");
  g_rate_ctrl_px4.lim_int[AXIS_YAW] = g_rate_integral_limit[AXIS_YAW];
  g_rate_ctrl_px4.gain_d[AXIS_YAW] = load_param_float("mc_yawrate_d");
  g_rate_ctrl_px4.gain_ff[AXIS_YAW] = load_param_float("mc_yawrate_ff");
  g_rate_extra_gain[AXIS_YAW] = load_param_float("mc_yawrate_k");

  g_manual_rate_cfg.acro_r_max = load_param_float("mc_acro_r_max");
  g_manual_rate_cfg.acro_p_max = load_param_float("mc_acro_p_max");
  g_manual_rate_cfg.acro_y_max = load_param_float("mc_acro_y_max");
  g_manual_rate_cfg.acro_expo = load_param_float("mc_acro_expo");
  g_manual_rate_cfg.acro_expo_y = load_param_float("mc_acro_expo_y");
  g_manual_rate_cfg.acro_supexpo = load_param_float("mc_acro_supexpo");
  g_manual_rate_cfg.acro_supexpo_y = load_param_float("mc_acro_supexpo_y");

  g_yaw_tq_cutoff = load_param_float("mc_yaw_tq_cutoff");

  // 设置积分清零阈值：当误差绝对值小于此值时，清零积分（单位：deg/s）
  // 如果参数为0或未设置，则禁用此功能（阈值<=0时不启用）
  // float int_threshold[3] = {
  //     load_param_float("mc_rr_int_threshold"),  // roll轴积分清零阈值
  //     load_param_float("mc_pr_int_threshold"),  // pitch轴积分清零阈值
  //     load_param_float("mc_yr_int_threshold")   // yaw轴积分清零阈值
  // };

  float int_threshold[3] = {5.0f, 5.0f, 5.0f};

  rcpx4_set_integral_threshold(&g_rate_ctrl_px4, int_threshold);
}

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

static inline void manualRateFromRc(const setpoint_t* setpoint, attitude_t* rate_sp_out) {
  float stick_roll = setpoint->attitude.roll;
  float stick_pitch = setpoint->attitude.pitch;
  float stick_yaw = setpoint->attitude.yaw;

  float roll_cmd = superexpo_map(stick_roll, g_manual_rate_cfg.acro_expo, g_manual_rate_cfg.acro_supexpo);
  float pitch_cmd = superexpo_map(stick_pitch, g_manual_rate_cfg.acro_expo, g_manual_rate_cfg.acro_supexpo);
  float yaw_cmd = superexpo_map(stick_yaw, g_manual_rate_cfg.acro_expo_y, g_manual_rate_cfg.acro_supexpo_y);

  rate_sp_out->roll = roll_cmd * g_manual_rate_cfg.acro_r_max;
  rate_sp_out->pitch = pitch_cmd * g_manual_rate_cfg.acro_p_max;
  rate_sp_out->yaw = yaw_cmd * g_manual_rate_cfg.acro_y_max;
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
  float torque[3] = {0.f, 0.f, 0.f};
  bool landed = (actualThrust_ < 5.0f);
  float dt = getDtForRatePid();

  // 计算误差，用于判断饱和方向
  float rate_error[3];
  rate_error[0] = rate_sp[0] - rate[0];
  rate_error[1] = rate_sp[1] - rate[1];
  rate_error[2] = rate_sp[2] - rate[2];

  rcpx4_update(&g_rate_ctrl_px4, rate, rate_sp, angularAccel->axis, dt, landed, torque);
  if (fabsf(torque[2]) < g_yaw_tq_cutoff) {
    torque[2] = 0.f;
  }

  // 检测输出饱和并设置饱和标志（用于下一次调用的积分抗饱和）
  // 这是PX4标准的积分抗饱和机制：当输出达到限制且误差方向与饱和方向一致时，
  // 停止该方向的积分累积，防止积分windup
  for (int i = 0; i < 3; i++) {
    // 检查原始输出是否超过限制（饱和）
    bool is_saturated_pos = (torque[i] > (float)INT16_MAX);
    bool is_saturated_neg = (torque[i] < (float)(-INT16_MAX));

    // 当输出正饱和且误差为正时，设置正饱和标志（下一次调用时误差会被限制为<=0）
    // 这样可以防止积分在输出已经达到上限时继续累积
    if (is_saturated_pos && rate_error[i] > 0.f) {
      rcpx4_set_pos_sat_axis(&g_rate_ctrl_px4, i, 1);
    } else {
      rcpx4_set_pos_sat_axis(&g_rate_ctrl_px4, i, 0);
    }

    // 当输出负饱和且误差为负时，设置负饱和标志（下一次调用时误差会被限制为>=0）
    // 这样可以防止积分在输出已经达到下限时继续累积
    if (is_saturated_neg && rate_error[i] < 0.f) {
      rcpx4_set_neg_sat_axis(&g_rate_ctrl_px4, i, 1);
    } else {
      rcpx4_set_neg_sat_axis(&g_rate_ctrl_px4, i, 0);
    }
  }

  output->roll = pidOutLimit(torque[0]);
  output->pitch = pidOutLimit(torque[1]);
  output->yaw = pidOutLimit(torque[2]);
}

static void ResetPid(void) { rcpx4_reset_integral(&g_rate_ctrl_px4); }

/* PID debug data access functions */
void getAnglePidRollDebug(float* outP, float* outI, float* outD) {
  // if (outP) *outP = pidAngleRoll.outP;
  // if (outI) *outI = pidAngleRoll.outI;
  // if (outD) *outD = pidAngleRoll.outD;
}

void getAnglePidPitchDebug(float* outP, float* outI, float* outD) {
  // if (outP) *outP = pidAnglePitch.outP;
  // if (outI) *outI = pidAnglePitch.outI;
  // if (outD) *outD = pidAnglePitch.outD;
}

void getAnglePidYawDebug(float* outP, float* outI, float* outD) {
  // if (outP) *outP = pidAngleYaw.outP;
  // if (outI) *outI = pidAngleYaw.outI;
  // if (outD) *outD = pidAngleYaw.outD;
}

void getRatePidRollDebug(float* outP, float* outI, float* outD) {
  // if (outP) *outP = pidRateRoll.outP;
  // if (outI) *outI = pidRateRoll.outI;
  // if (outD) *outD = pidRateRoll.outD;
}

void getRatePidPitchDebug(float* outP, float* outI, float* outD) {
  // if (outP) *outP = pidRatePitch.outP;
  // if (outI) *outI = pidRatePitch.outI;
  // if (outD) *outD = pidRatePitch.outD;
}

void getRatePidYawDebug(float* outP, float* outI, float* outD) {
  // if (outP) *outP = pidRateYaw.outP;
  // if (outI) *outI = pidRateYaw.outI;
  // if (outD) *outD = pidRateYaw.outD;
}

void getPx4RatePidInt(float* rollOutInt, float* pitchOutInt, float* yawOutInt) {
  if (rollOutInt) *rollOutInt = g_rate_ctrl_px4.rate_int[0];
  if (pitchOutInt) *pitchOutInt = g_rate_ctrl_px4.rate_int[1];
  if (yawOutInt) *yawOutInt = g_rate_ctrl_px4.rate_int[2];
}

void stateControlInit(void) { pidParamInit(); }

static bool resetControl(const state_t* state, const setpoint_t* setpoint, control_t* control) {
  if (setpoint->thrust < 5.f || !setpoint->armed) {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    control->thrust = 0;
    rcpx4_reset_integral(&g_rate_ctrl_px4);
    return true;
  }
  return false;
}

__attribute__((unused)) static void generateAttitudeDesired(const setpoint_t* setpoint, const uint32_t tick) {
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
    // generateAttitudeDesired(setpoint, tick);
    if (setpoint->fly_mode == FLYER_MODE_STABLIZE) {
      // anglePid(&state->attitude, &attitudeDesired_, &rateDesired_);
    } else {
      manualRateFromRc(setpoint, &rateDesired_);
    }
  }

  if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
    ratePid(&state->gyro_filter, &rateDesired_, &state->angular_accel, control);
  }

  control->thrust = actualThrust_;
}

void getAngleDesired(attitude_t* get) {
  // get->pitch = attitudeDesired_.pitch;
  // get->roll = attitudeDesired_.roll;
  // get->yaw = attitudeDesired_.yaw;
}

void getRateDesired(attitude_t *get) {
  get->pitch = rateDesired_.pitch;
  get->roll = rateDesired_.roll;
  get->yaw = rateDesired_.yaw;
}

__attribute__((unused)) static void cmdPidParamReflash(int argc, char** argv) {
  ResetPid();
  pidParamInit();
  rt_kprintf("pid param reflash done.\n");
}

MSH_CMD_EXPORT_ALIAS(cmdPidParamReflash, cmdPidParamReflash, Reflash PID parameters from uParam);