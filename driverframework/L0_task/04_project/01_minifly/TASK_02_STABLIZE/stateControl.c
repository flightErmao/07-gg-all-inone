#include <math.h>
#include "stateControl.h"
#include "attitudePid.h"

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

static float actualThrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;

void stateControlInit(void) { attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT); /*初始化姿态PID*/ }

bool stateControlTest(void) {
  bool pass = true;
  pass &= attitudeControlTest();
  return pass;
}

void stateControl(control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint,
                  const uint32_t tick) {
  static uint16_t cnt = 0;

  if (!state->armed) {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    control->thrust = 0;

    attitudeResetAllPID();                     /*复位姿态PID*/
    attitudeDesired.yaw = state->attitude.yaw; /*复位计算的期望yaw值*/

    if (cnt++ > 1500) {
      cnt = 0;
      // configParamGiveSemaphore();
    }
    return;
  }

  if (RATE_DO_EXECUTE(ANGEL_PID_RATE, tick)) {
    if (setpoint->mode.z == modeDisable) {
      actualThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    if (control->flipDir == CENTER) {
      attitudeDesired.yaw += setpoint->attitude.yaw / ANGEL_PID_RATE; /*期望YAW 速率模式*/
      if (attitudeDesired.yaw > 180.0f) attitudeDesired.yaw -= 360.0f;
      if (attitudeDesired.yaw < -180.0f) attitudeDesired.yaw += 360.0f;
    }

    // attitudeDesired.roll += configParam.trimR;  // 叠加微调值
    // attitudeDesired.pitch += configParam.trimP;

    attitudeAnglePID(&state->attitude, &attitudeDesired, &rateDesired);
  }

  if (RATE_DO_EXECUTE(RATE_PID_RATE, tick)) {
    attitudeRatePID(&sensors->gyro_filter, &rateDesired, control);
  }

  control->thrust = actualThrust;

  if (control->thrust < 5.f) {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    attitudeResetAllPID(); /*复位姿态PID*/

    attitudeDesired.yaw = state->attitude.yaw; /*复位计算的期望yaw值*/

    if (cnt++ > 1500) {
      cnt = 0;
      // configParamGiveSemaphore();
    }
  } else {
    cnt = 0;
  }
}

void getAttitudeDesired(attitude_t *get) {
  get->pitch = -attitudeDesired.pitch;
  get->roll = attitudeDesired.roll;
  get->yaw = -attitudeDesired.yaw;
}
