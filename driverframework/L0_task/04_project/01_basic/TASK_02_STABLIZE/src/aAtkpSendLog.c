
#include <rtthread.h>
#include "floatConvert.h"
#include "stabilizerTypes.h"
#include "stateControl.h"
#include "aMcnStabilize.h"
#include "stateControl.h"

#ifdef TASK_TOOL_01_ANOTC_TELEM_EN

#include "protocolAtkpInterface.h"
#include "taskAnotcTelem.h"

// Extracted static helpers

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_FLYER_ANGLE
static void logFlyerAngle(uint16_t count_ms) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_FLYER_ANGLE_PERIOD_MS
#define ATKP_LOG_FLYER_ANGLE_PERIOD PROJECT_MINIFLY_TASK_STABLIZE_LOG_FLYER_ANGLE_PERIOD_MS
#else
#define ATKP_LOG_FLYER_ANGLE_PERIOD 20
#endif
  if (!(count_ms % ATKP_LOG_FLYER_ANGLE_PERIOD)) {
    state_t state_flyer = {0};
    mcnStateAcquire(&state_flyer);
    packStatus(state_flyer.attitude.roll, -state_flyer.attitude.pitch, -state_flyer.attitude.yaw,
               (int32_t)(state_flyer.position.z * 1000), state_flyer.fly_mode, state_flyer.armed);
  }
}
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_ANGLE_DEBUG
static void logAngleNowSpDate(uint16_t count_ms) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_ANGLE_PERIOD_MS
#define ATKP_LOG_ANGLE_PERIOD PROJECT_MINIFLY_TASK_STABLIZE_LOG_ANGLE_PERIOD_MS
#else
#define ATKP_LOG_ANGLE_PERIOD 20
#endif
  if (!(count_ms % ATKP_LOG_ANGLE_PERIOD)) {
    state_t state_flyer = {0};
    attitude_t attitude_current;
    attitude_t attitude_desired;

    mcnStateAcquire(&state_flyer);
    getAngleDesired(&attitude_desired);

    attitude_current.roll = state_flyer.attitude.roll;
    attitude_current.pitch = state_flyer.attitude.pitch;
    attitude_current.yaw = state_flyer.attitude.yaw;

    sendUserDatafloat6(PROJECT_MINIFLY_TASK_STABLIZE_LOG_GROUP_ANGLE, attitude_current.roll, attitude_current.pitch,
                       attitude_current.yaw, attitude_desired.roll, attitude_desired.pitch, attitude_desired.yaw);
  }
}
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_RATE_DEBUG
static void logRateNowSpDate(uint16_t count_ms) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_RATE_PERIOD_MS
#define ATKP_LOG_RATE_PERIOD PROJECT_MINIFLY_TASK_STABLIZE_LOG_RATE_PERIOD_MS
#else
#define ATKP_LOG_RATE_PERIOD 20
#endif
  if (!(count_ms % ATKP_LOG_RATE_PERIOD)) {
    state_t state_flyer = {0};
    attitude_t rate_current;
    attitude_t rate_desired;

    mcnStateAcquire(&state_flyer);
    getRateDesired(&rate_desired);

    rate_current.roll = state_flyer.gyro_filter.x;
    rate_current.pitch = state_flyer.gyro_filter.y;
    rate_current.yaw = state_flyer.gyro_filter.z;

    sendUserDatafloat6(PROJECT_MINIFLY_TASK_STABLIZE_LOG_GROUP_RATE, rate_current.roll, rate_current.pitch,
                       rate_current.yaw, rate_desired.roll, rate_desired.pitch, rate_desired.yaw);
  }
}
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_ANGLE_PID_DEBUG
static void logAnglePidDebug(uint16_t count_ms) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_ANGLE_PID_PERIOD_MS
#define ATKP_LOG_ANGLE_PID_PERIOD PROJECT_MINIFLY_TASK_STABLIZE_LOG_ANGLE_PID_PERIOD_MS
#else
#define ATKP_LOG_ANGLE_PID_PERIOD 20
#endif
  if (!(count_ms % ATKP_LOG_ANGLE_PID_PERIOD)) {
    float angle_roll_p, angle_roll_i, angle_roll_d;
    float angle_pitch_p, angle_pitch_i, angle_pitch_d;
    float angle_yaw_p, angle_yaw_i, angle_yaw_d;

    getAnglePidRollDebug(&angle_roll_p, &angle_roll_i, &angle_roll_d);
    getAnglePidPitchDebug(&angle_pitch_p, &angle_pitch_i, &angle_pitch_d);
    getAnglePidYawDebug(&angle_yaw_p, &angle_yaw_i, &angle_yaw_d);

    sendUserDatafloat9(PROJECT_MINIFLY_TASK_STABLIZE_LOG_GROUP_ANGLE_PID, 
                       angle_roll_p, angle_roll_i, angle_roll_d,
                       angle_pitch_p, angle_pitch_i, angle_pitch_d,
                       angle_yaw_p, angle_yaw_i, angle_yaw_d);
  }
}
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_RATE_PID_DEBUG
static void logRatePidDebug(uint16_t count_ms) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_RATE_PID_PERIOD_MS
#define ATKP_LOG_RATE_PID_PERIOD PROJECT_MINIFLY_TASK_STABLIZE_LOG_RATE_PID_PERIOD_MS
#else
#define ATKP_LOG_RATE_PID_PERIOD 20
#endif
  if (!(count_ms % ATKP_LOG_RATE_PID_PERIOD)) {
    float rate_roll_p, rate_pitch_p, rate_yaw_p;
    float rate_roll_i, rate_pitch_i, rate_yaw_i;
    float rate_roll_d, rate_pitch_d, rate_yaw_d;

    getRatePidRollDebug(&rate_roll_p, &rate_roll_i, &rate_roll_d);
    getRatePidPitchDebug(&rate_pitch_p, &rate_pitch_i, &rate_pitch_d);
    getRatePidYawDebug(&rate_yaw_p, &rate_yaw_i, &rate_yaw_d);

    sendUserDatafloat9(PROJECT_MINIFLY_TASK_STABLIZE_LOG_GROUP_RATE_PID,
                       rate_roll_p, rate_roll_i, rate_roll_d,
                       rate_pitch_p, rate_pitch_i, rate_pitch_d,
                       rate_yaw_p, rate_yaw_i, rate_yaw_d);
  }
}
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_CONTROL_OUTPUT
static void logControlOutput(uint16_t count_ms) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_CONTROL_OUTPUT_PERIOD_MS
#define ATKP_LOG_CONTROL_OUTPUT_PERIOD PROJECT_MINIFLY_TASK_STABLIZE_LOG_CONTROL_OUTPUT_PERIOD_MS
#else
#define ATKP_LOG_CONTROL_OUTPUT_PERIOD 20
#endif
  if (!(count_ms % ATKP_LOG_CONTROL_OUTPUT_PERIOD)) {
    control_t control_output = {0};
    getControlOutput(&control_output);
    sendUserDatafloat6(PROJECT_MINIFLY_TASK_STABLIZE_LOG_GROUP_CONTROL_OUTPUT, control_output.roll,
                       control_output.pitch, control_output.yaw, control_output.thrust, 0.f, 0.f);
  }
}
#endif

#ifdef PROJECT_MINIFLY_TASK_RATEPX4_LOG_EN
static void logRatePx4IntDebug(uint16_t count_ms) {
#ifdef PROJECT_MINIFLY_TASK_RATEPX4_LOG_PERIOD_MS
#define ATKP_LOG_RATEPX4_PERIOD PROJECT_MINIFLY_TASK_RATEPX4_LOG_PERIOD_MS
#else
#define ATKP_LOG_RATEPX4_PERIOD 20
#endif
  if (!(count_ms % ATKP_LOG_RATEPX4_PERIOD)) {
    float rc_pid_int_roll, rc_pid_int_pitch, rc_pid_int_yaw;
    getPx4RatePidInt(&rc_pid_int_roll, &rc_pid_int_pitch, &rc_pid_int_yaw);
    sendUserDatafloat3(PROJECT_MINIFLY_TASK_STABLIZE_LOG_GROUP_RATEPX4, rc_pid_int_roll, rc_pid_int_pitch,
                       rc_pid_int_yaw);
  }
}
#endif

void sendFlyerStates(uint16_t count_ms) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_FLYER_ANGLE
  logFlyerAngle(count_ms);
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_ANGLE_DEBUG
  logAngleNowSpDate(count_ms);
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_RATE_DEBUG
  logRateNowSpDate(count_ms);
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_ANGLE_PID_DEBUG
  logAnglePidDebug(count_ms);
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_RATE_PID_DEBUG
  logRatePidDebug(count_ms);
#endif

#ifdef PROJECT_MINIFLY_TASK_RATEPX4_LOG_EN
  logRatePx4IntDebug(count_ms);
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_CONTROL_OUTPUT
  logControlOutput(count_ms);
#endif
}

int addPeriodFunStablize(void) {
  anotcTelemAddSensorFunc(sendFlyerStates);
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_ATKP_LOG_EN
INIT_APP_EXPORT(addPeriodFunStablize);
#endif

#endif
