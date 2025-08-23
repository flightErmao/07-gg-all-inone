
#include <rtthread.h>
#include "taskMiniflyStabilizer.h"
#include "floatConvert.h"
#include "stabilizerTypes.h"

#ifdef TOOL_TASK_ANOTC_TELEM_EN

#include "protocolAtkpInterface.h"
#include "taskAnotcTelem.h"

void sendFlyerStates(uint16_t count_ms) {
  if (!(count_ms % PERIOD_10ms)) {
    state_t state_flyer = {0};
    stabilizerGetState(&state_flyer);
    packStatus(state_flyer.attitude.roll, state_flyer.attitude.pitch, state_flyer.attitude.yaw,
               (int32_t)(state_flyer.position.z * 1000), state_flyer.fly_mode, state_flyer.armed);
  }
}

int addPeriodFunListProjectMiniFlySensor(void) {
  anotcTelemAddSensorFunc(sendFlyerStates);
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_ATKP_LOG_EN
INIT_APP_EXPORT(addPeriodFunListProjectMiniFlySensor);
#endif

#endif
