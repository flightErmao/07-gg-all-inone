#include "rtthread.h"
#include <rtdevice.h>
#include <math.h>
#include "stateControl.h"
#include "attitudePid.h"
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_DEBUGPIN_EN
#include "debugPin.h"
#endif
#include "taskParam.h"

/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 四轴姿态控制代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 ********************************************************************************/

static float actualThrust_;
static attitude_t attitudeDesired_;
static attitude_t rateDesired_;

void stateControlInit(void) { attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT); /*初始化姿态PID*/ }

static bool resetControl(const state_t *state, const setpoint_t *setpoint, control_t *control) {
  static uint16_t cnt = 0;

  if (setpoint->thrust < 5.f || !setpoint->armed) {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    control->thrust = 0;
    attitudeResetAllPID();                      /*复位姿态PID*/
    attitudeDesired_.yaw = state->attitude.yaw; /*复位计算的期望yaw值*/
    if (cnt++ > 1500) {
      cnt = 0;
      configParamGiveSemaphore();
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
  attitudeDesired_.yaw += setpoint->attitude.yaw / ANGEL_PID_RATE;
  if (attitudeDesired_.yaw > 180.0f) attitudeDesired_.yaw -= 360.0f;
  if (attitudeDesired_.yaw < -180.0f) attitudeDesired_.yaw += 360.0f;
}

void stateControl(const state_t *state, const setpoint_t *setpoint, control_t *control, const uint32_t tick) {
  generateAttituedeDesierd(setpoint);

  if (resetControl(state, setpoint, control)) {
    return;
  }

  if (RATE_DO_EXECUTE(ANGEL_PID_RATE, tick)) {
    attitudeAnglePID(&state->attitude, &attitudeDesired_, &rateDesired_);
    updateYawAngle(setpoint);
  }

  if (RATE_DO_EXECUTE(RATE_PID_RATE, tick)) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_DEBUGPIN_EN
    DEBUG_PIN_DEBUG0_TOGGLE();
#endif
    attitudeRatePID(&state->gyro_filter, &rateDesired_, control);
  }

  control->thrust = actualThrust_;
}

void getAttitudeDesired(attitude_t *get) {
  get->pitch = attitudeDesired_.pitch;
  get->roll = attitudeDesired_.roll;
  get->yaw = attitudeDesired_.yaw;
}
