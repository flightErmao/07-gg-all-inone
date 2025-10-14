#include "command.h"
#ifdef TASK_TOOL_02_SD_MLOG
#include "taskMlog.h"
#endif

/* RC input smoothing filter configuration */
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_RC_FILTER_EN
  /* Convert Kconfig integer (x100) to float coefficient */
  #define RC_ATTITUDE_FILTER_ALPHA (PROJECT_MINIFLY_TASK_STABLIZE_RC_FILTER_ALPHA / 100.0f)
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

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_RC_FILTER_EN
/**
 * @brief Apply low-pass filter for smoothing RC input
 * @param input Current input value
 * @param prev_output Previous filtered output value
 * @return Filtered output value
 */
static inline float rcInputFilter(float input, float prev_output) {
  return RC_ATTITUDE_FILTER_ALPHA * input + (1.0f - RC_ATTITUDE_FILTER_ALPHA) * prev_output;
}
#endif

// #if defined(PROJECT_MINIFLY_TASK06_RC_EN) || defined(PROJECT_FMT_TASK01_RC_EN)
void commanderGetSetpoint(const pilot_cmd_bus_t* rc_data, setpoint_t* setpoint) {
  if (!setpoint) return;

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_RC_FILTER_EN
  /* Static variables to store previous filtered values */
  static float filtered_roll = 0.0f;
  static float filtered_pitch = 0.0f;
  static float filtered_yaw = 0.0f;
  static uint8_t filter_initialized = 0;
  static uint8_t prev_armed_status = 0;

  /* Reset filter when disarmed to avoid stale values on next arming */
  if (rc_data->ram_status != prev_armed_status) {
    if (rc_data->ram_status == ARM_STATUS_DISARM) {
      filter_initialized = 0;
    }
    prev_armed_status = rc_data->ram_status;
  }

  /* Initialize filter on first run or after disarm */
  if (!filter_initialized) {
    filtered_roll = rc_data->stick_roll;
    filtered_pitch = rc_data->stick_pitch;
    filtered_yaw = -rc_data->stick_yaw;
    filter_initialized = 1;
  }

  /* Apply low-pass filter to smooth RC input */
  filtered_roll = rcInputFilter(rc_data->stick_roll, filtered_roll);
  filtered_pitch = rcInputFilter(rc_data->stick_pitch, filtered_pitch);
  filtered_yaw = rcInputFilter(-rc_data->stick_yaw, filtered_yaw);

  setpoint->attitude.timestamp = rc_data->timestamp;
  setpoint->attitude.roll = filtered_roll;
  setpoint->attitude.pitch = filtered_pitch;
  setpoint->attitude.yaw = filtered_yaw;
#else
  /* Direct assignment without filtering */
  setpoint->attitude.timestamp = rc_data->timestamp;
  setpoint->attitude.roll = rc_data->stick_roll;
  setpoint->attitude.pitch = rc_data->stick_pitch;
  setpoint->attitude.yaw = -rc_data->stick_yaw;
#endif

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