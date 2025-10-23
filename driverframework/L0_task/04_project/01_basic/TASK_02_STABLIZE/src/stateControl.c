#include "rtthread.h"
#include <rtdevice.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "stateControl.h"
#include "pidMinifly.h"
#include "taskParam.h"

#define PID_ANGLE_ROLL_INTEGRATION_LIMIT 30.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT 30.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT 180.0

#define PID_RATE_ROLL_INTEGRATION_LIMIT 500.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT 500.0
#define PID_RATE_YAW_INTEGRATION_LIMIT 50.0

static float actualThrust_;
static attitude_t attitudeDesired_;
static attitude_t rateDesired_;

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
}

bool attitudeControlTest() { return true; }

void attitudeRatePID(const Axis3f *actualRate, const attitude_t *desiredRate, control_t *output) {
  output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desiredRate->roll - actualRate->x));
  output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->y));
  output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->z));
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

static void generateAttituedeDesierd(const setpoint_t* setpoint, const uint32_t tick) {
  if (setpoint->mode.z == modeDisable) {
    actualThrust_ = setpoint->thrust;
  }
  if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
    attitudeDesired_.roll = setpoint->attitude.roll;
    attitudeDesired_.pitch = setpoint->attitude.pitch;
  }

  attitudeDesired_.yaw += setpoint->attitude.yaw / ANGLE_PID_RATE;
  if (attitudeDesired_.yaw > 180.0f) attitudeDesired_.yaw -= 360.0f;
  if (attitudeDesired_.yaw < -180.0f) attitudeDesired_.yaw += 360.0f;
}

void stateControl(const state_t* state, const setpoint_t* setpoint, control_t* control, const uint32_t tick) {
  if (resetControl(state, setpoint, control)) {
    return;
  }

  if (RATE_DO_EXECUTE(ANGLE_PID_RATE, tick)) {
    generateAttituedeDesierd(setpoint, tick);
    attitudeAnglePID(&state->attitude, &attitudeDesired_, &rateDesired_);
  }

  if (RATE_DO_EXECUTE(RATE_PID_RATE, tick)) {
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