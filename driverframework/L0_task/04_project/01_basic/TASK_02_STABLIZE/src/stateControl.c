#include "rtthread.h"
#include <rtdevice.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "stateControl.h"
#include "pidMinifly.h"
#include "pidpx4.h"
#include "taskParam.h"
/* 若未提供 PX4 acro 参数头文件，则提供本地默认值 */
#ifndef MC_ACRO_R_MAX
#define MC_ACRO_R_MAX      100.0f
#endif
#ifndef MC_ACRO_P_MAX
#define MC_ACRO_P_MAX      100.0f
#endif
#ifndef MC_ACRO_Y_MAX
#define MC_ACRO_Y_MAX      100.0f
#endif
#ifndef MC_ACRO_EXPO
#define MC_ACRO_EXPO       0.0f
#endif
#ifndef MC_ACRO_EXPO_Y
#define MC_ACRO_EXPO_Y     0.0f
#endif
#ifndef MC_ACRO_SUPEXPO
#define MC_ACRO_SUPEXPO    0.0f
#endif
#ifndef MC_ACRO_SUPEXPOY
#define MC_ACRO_SUPEXPOY   0.0f
#endif

#define PID_ANGLE_ROLL_INTEGRATION_LIMIT 30.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT 30.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT 180.0

#define PID_RATE_ROLL_INTEGRATION_LIMIT 500.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT 500.0
#define PID_RATE_YAW_INTEGRATION_LIMIT 50.0

static float actualThrust_;
static attitude_t attitudeDesired_;
static attitude_t rateDesired_;

/* PX4 风格速率控制器 */
static RateControlPX4 g_rate_ctrl_px4;

PidObject pidAngleRoll;
PidObject pidAnglePitch;
PidObject pidAngleYaw;

PidObject pidRateRoll;
PidObject pidRatePitch;
PidObject pidRateYaw;

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
  float roll_cmd = superexpo_map(stick_roll, MC_ACRO_EXPO, MC_ACRO_SUPEXPO);
  float pitch_cmd = superexpo_map(stick_pitch, MC_ACRO_EXPO, MC_ACRO_SUPEXPO);
  float yaw_cmd = superexpo_map(stick_yaw, MC_ACRO_EXPO_Y, MC_ACRO_SUPEXPOY);

  /* 最大角速率（deg/s）*/
  rate_sp_out->roll = roll_cmd * MC_ACRO_R_MAX;
  rate_sp_out->pitch = pitch_cmd * MC_ACRO_P_MAX;
  rate_sp_out->yaw = yaw_cmd * MC_ACRO_Y_MAX;
}

void attitudeControlInit(float ratePidDt, float anglePidDt) {
  configParam_t configParam;
  getConfigParam(&configParam);

  pidInit(&pidAngleRoll, 0, configParam.pidAngle.roll, anglePidDt);
  pidInit(&pidAnglePitch, 0, configParam.pidAngle.pitch, anglePidDt);
  pidInit(&pidAngleYaw, 0, configParam.pidAngle.yaw, anglePidDt);
  pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);

  pidInit(&pidRateRoll, 0, configParam.pidRate.roll, ratePidDt);
  pidInit(&pidRatePitch, 0, configParam.pidRate.pitch, ratePidDt);
  pidInit(&pidRateYaw, 0, configParam.pidRate.yaw, ratePidDt);
  pidSetIntegralLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);

  /* 初始化 PX4 速率控制器参数（沿用现有 Rate PID 的 P/I/D 配置） */
  rcpx4_init(&g_rate_ctrl_px4);
  float P[3] = {configParam.pidRate.roll.kp, configParam.pidRate.pitch.kp, configParam.pidRate.yaw.kp};
  float I[3] = {configParam.pidRate.roll.ki, configParam.pidRate.pitch.ki, configParam.pidRate.yaw.ki};
  float D[3] = {configParam.pidRate.roll.kd, configParam.pidRate.pitch.kd, configParam.pidRate.yaw.kd};
  float LIM[3] = {PID_RATE_ROLL_INTEGRATION_LIMIT, PID_RATE_PITCH_INTEGRATION_LIMIT, PID_RATE_YAW_INTEGRATION_LIMIT};
  float FF[3] = {0.f, 0.f, 0.f};
  rcpx4_set_pid(&g_rate_ctrl_px4, P, I, D);
  rcpx4_set_integrator_limit(&g_rate_ctrl_px4, LIM);
  rcpx4_set_ff(&g_rate_ctrl_px4, FF);
}

bool attitudeControlTest() { return true; }

void attitudeRatePID(const Axis3f *actualRate, const attitude_t *desiredRate, control_t *output) {
  float rate[3] = {actualRate->x, actualRate->y, actualRate->z};
  float rate_sp[3] = {desiredRate->roll, desiredRate->pitch, desiredRate->yaw};
  float angular_accel[3] = {0.f, 0.f, 0.f}; /* 当前无角加速度，置零 */
  float torque[3] = {0.f, 0.f, 0.f};

  /* 以油门近似是否离地，未解锁或极低油门时可视为落地，避免积分 */
  bool landed = (actualThrust_ < 5.0f);

  rcpx4_update(&g_rate_ctrl_px4, rate, rate_sp, angular_accel, RATE_PID_DT, landed, torque);

  output->roll = pidOutLimit(torque[0]);
  output->pitch = pidOutLimit(torque[1]);
  output->yaw = pidOutLimit(torque[2]);
}

void attitudeAnglePID(const attitude_t *actualAngle, const attitude_t *desiredAngle, attitude_t *outDesiredRate) {
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

void stateControlInit(void) { attitudeControlInit(RATE_PID_DT, ANGLE_PID_DT); /*初始化姿态PID*/ }

static bool resetControl(const state_t *state, const setpoint_t *setpoint, control_t *control) {
  static uint16_t cnt = 0;

  if (setpoint->thrust < 5.f || !setpoint->armed) {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    control->thrust = 0;
    attitudeResetAllPID();
    attitudeDesired_.yaw = state->attitude.yaw;
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
    actualThrust_ = setpoint->thrust;
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
  }

  if (RATE_DO_EXECUTE(RATE_250_HZ, tick)) {
    generateAttitudeDesired(setpoint, tick);
    if (setpoint->fly_mode == FLYER_MODE_STABLIZE) {
      /* 角度模式：由角度外环生成角速率期望 */
      attitudeAnglePID(&state->attitude, &attitudeDesired_, &rateDesired_);
    } else {
      /* 手动速率模式：由 RC 直接生成角速率期望 */
      manualRateFromRc(setpoint, &rateDesired_);
    }
  }

  if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
    attitudeRatePID(&state->gyro_filter, &rateDesired_, control);
  }

  control->thrust = actualThrust_;
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