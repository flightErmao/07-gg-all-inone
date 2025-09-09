#include "mixerControl.h"
#ifdef L2_DEVICE_03_MOTOR_01_PWM_EN
#include "motorsPwm.h"
#endif

static bool motorSetEnable = false;
static motorPWM_t motorPWM;
static motorPWM_t motorPWMSet = {0, 0, 0, 0};

uint16_t limitThrust(int value) {
  if (value > UINT16_MAX) {
    value = UINT16_MAX;
  } else if (value < 0) {
    value = 0;
  }

  return (uint16_t)value;
}

void mixerControl(control_t *control) /*功率输出控制*/
{
  int32_t r = control->roll / 2.0f;
  int32_t p = control->pitch / 2.0f;

  motorPWM.m1 = limitThrust(control->thrust - r - p + control->yaw);
  motorPWM.m2 = limitThrust(control->thrust - r + p - control->yaw);
  motorPWM.m3 = limitThrust(control->thrust + r + p + control->yaw);
  motorPWM.m4 = limitThrust(control->thrust + r - p - control->yaw);

  if (motorSetEnable) {
    motorPWM = motorPWMSet;
  }
#ifdef L2_DEVICE_03_MOTOR_01_PWM_EN
  motorsSetRatio(MOTOR_M1, motorPWM.m1); /*控制电机输出百分比*/
  motorsSetRatio(MOTOR_M2, motorPWM.m2);
  motorsSetRatio(MOTOR_M3, motorPWM.m3);
  motorsSetRatio(MOTOR_M4, motorPWM.m4);
#endif
}

void getMotorPWM(motorPWM_t *get) { *get = motorPWM; }

void setMotorPWM(bool enable, uint32_t m1_set, uint32_t m2_set, uint32_t m3_set, uint32_t m4_set) {
  motorSetEnable = enable;
  motorPWMSet.m1 = m1_set;
  motorPWMSet.m2 = m2_set;
  motorPWMSet.m3 = m3_set;
  motorPWMSet.m4 = m4_set;
}
