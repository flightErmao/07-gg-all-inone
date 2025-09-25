
#include <rtthread.h>
#include "taskMiniflyStabilizer.h"
#include "floatConvert.h"
#include "stabilizerTypes.h"
#include "stateControl.h"

#ifdef TASK_TOOL_01_ANOTC_TELEM_EN

#include "protocolAtkpInterface.h"
#include "taskAnotcTelem.h"

void sendFlyerStates(uint16_t count_ms) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_FLYER_ANGLE
  if (!(count_ms % PERIOD_20ms)) {
    state_t state_flyer = {0};
    stabilizerGetState(&state_flyer);
    packStatus(state_flyer.attitude.roll, -state_flyer.attitude.pitch, -state_flyer.attitude.yaw,
               (int32_t)(state_flyer.position.z * 1000), state_flyer.fly_mode, state_flyer.armed);
  }
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_LOG_ANGLE_DEBUG
  if (!(count_ms % PERIOD_20ms)) {
    state_t state_flyer = {0};
    attitude_t attitude_current;
    attitude_t attitude_desired;
    attitude_t rate_desired;

    stabilizerGetState(&state_flyer);
    getAttitudeDesired(&attitude_desired);
    getRateDesired(&rate_desired);

    attitude_current.roll = state_flyer.attitude.roll;
    attitude_current.pitch = state_flyer.attitude.pitch;
    attitude_current.yaw = state_flyer.attitude.yaw;

    sendUserDatafloat6(1, attitude_current.roll, attitude_current.pitch, state_flyer.gyro_filter.z,
                       attitude_desired.roll, attitude_desired.pitch, rate_desired.yaw);
  }
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
