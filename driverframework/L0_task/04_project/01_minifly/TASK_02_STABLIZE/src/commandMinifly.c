#include "commandMinifly.h"
#ifdef TASK_TOOL_02_SD_MLOG
#include "taskMlog.h"
#endif

static void commanderHandleArmLogging(const setpoint_t* setpoint) {
  static int prev_armed = -1;

  if (!setpoint) return;

  if (prev_armed < 0) {
    prev_armed = setpoint->armed;
    return;
  }

  if (prev_armed == ARM_STATUS_DISARM && setpoint->armed == ARM_STATUS_ARM) {
#ifdef TASK_TOOL_02_SD_MLOG
    task_mlog_start_logging(NULL);
#endif
  }

  if (prev_armed == ARM_STATUS_ARM && setpoint->armed == ARM_STATUS_DISARM) {
#ifdef TASK_TOOL_02_SD_MLOG
    task_mlog_stop_logging();
#endif
  }

  prev_armed = setpoint->armed;
}

// #if defined(PROJECT_MINIFLY_TASK06_RC_EN) || defined(PROJECT_FMT_TASK01_RC_EN)
void commanderGetSetpoint(const pilot_cmd_bus_t* rc_data, setpoint_t* setpoint) {
  if (!setpoint) return;

  setpoint->attitude.timestamp = rc_data->timestamp;
  setpoint->attitude.roll = rc_data->stick_roll;
  setpoint->attitude.pitch = rc_data->stick_pitch;
  setpoint->attitude.yaw = -rc_data->stick_yaw;
  setpoint->thrust = rc_data->stick_throttle;
  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;
  setpoint->mode.z = modeDisable;
  setpoint->mode.roll = modeDisable;
  setpoint->mode.pitch = modeDisable;
  setpoint->mode.yaw = modeDisable;
  setpoint->armed = rc_data->ram_status;
  commanderHandleArmLogging(setpoint);
  return;
}
// #endif