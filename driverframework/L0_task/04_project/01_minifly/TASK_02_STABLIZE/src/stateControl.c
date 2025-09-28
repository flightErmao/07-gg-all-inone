#include "rtthread.h"
#include <rtdevice.h>
#include <math.h>
#include "stateControl.h"
#include "attitudePid.h"
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_DEBUGPIN_EN
#include "debugPin.h"
#endif
#include "taskParam.h"

static float actualThrust_;
static attitude_t attitudeDesired_;
static attitude_t rateDesired_;

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

static void generateAttituedeDesierd(const setpoint_t *setpoint) {
  if (setpoint->mode.z == modeDisable) {
    actualThrust_ = setpoint->thrust;
  }
  if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
    attitudeDesired_.roll = setpoint->attitude.roll;
    attitudeDesired_.pitch = setpoint->attitude.pitch;
  }
}

static void updateYawAngle(const setpoint_t *setpoint) {
  attitudeDesired_.yaw += setpoint->attitude.yaw / ANGLE_PID_RATE;
  if (attitudeDesired_.yaw > 180.0f) attitudeDesired_.yaw -= 360.0f;
  if (attitudeDesired_.yaw < -180.0f) attitudeDesired_.yaw += 360.0f;
}

void stateControl(const state_t *state, const setpoint_t *setpoint, control_t *control, const uint32_t tick) {
  generateAttituedeDesierd(setpoint);

  if (resetControl(state, setpoint, control)) {
    return;
  }

  if (RATE_DO_EXECUTE(ANGLE_PID_RATE, tick)) {
    updateYawAngle(setpoint);
    attitudeAnglePID(&state->attitude, &attitudeDesired_, &rateDesired_);
  }

  if (RATE_DO_EXECUTE(RATE_PID_RATE, tick)) {
    attitudeRatePID(&state->gyro_filter, &rateDesired_, control);
  }

  control->thrust = actualThrust_;
}

void getAttitudeDesired(attitude_t *get) {
  get->pitch = attitudeDesired_.pitch;
  get->roll = attitudeDesired_.roll;
  get->yaw = attitudeDesired_.yaw;
}

void getRateDesired(attitude_t *get) {
  get->pitch = rateDesired_.pitch;
  get->roll = rateDesired_.roll;
  get->yaw = rateDesired_.yaw;
}