#include "commandMinifly.h"
// #if defined(PROJECT_MINIFLY_TASK06_RC_EN) || defined(PROJECT_FMT_TASK01_RC_EN)
void commanderGetSetpoint(const pilot_cmd_bus_t* rc_data, setpoint_t* setpoint) {
  if (!setpoint) return;

  setpoint->attitude.timestamp = rc_data->timestamp;
  setpoint->attitude.roll = rc_data->stick_roll;
  setpoint->attitude.pitch = rc_data->stick_pitch;
  setpoint->attitude.yaw = -rc_data->stick_yaw;
  setpoint->thrust = rc_data->stick_throttle;
  setpoint->mode.x = modeDisable;      // x方向不受控
  setpoint->mode.y = modeDisable;      // y方向不受控
  setpoint->mode.z = modeDisable;      // z方向速度环
  setpoint->mode.roll = modeDisable;   // 横滚角度环
  setpoint->mode.pitch = modeDisable;  // 俯仰角度环
  setpoint->mode.yaw = modeDisable;    // 偏航角度环
  setpoint->armed = rc_data->ram_status;
  return;
}
// #endif