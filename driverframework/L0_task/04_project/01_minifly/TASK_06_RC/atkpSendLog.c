#include <rtthread.h>
#include "taskRc.h"
#include "floatConvert.h"

#ifdef TASK_TOOL_01_ANOTC_TELEM_EN

#include "protocolAtkpInterface.h"
#include "taskAnotcTelem.h"

void sendRcData(uint16_t count_ms) {
  if (!(count_ms % PERIOD_20ms)) {
    Pilot_Cmd_Bus rc_data = {0};
    rcPilotCmdAcquire(&rc_data);
    sendUserDatafloat6_u32(1, rc_data.stick_yaw, rc_data.stick_throttle, rc_data.stick_roll, rc_data.stick_pitch,
                           (float)rc_data.ram_status, (float)rc_data.ctrl_mode, rc_data.timestamp);
  }
}

int addPeriodFunRc(void) {
  anotcTelemAddSensorFunc(sendRcData);
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK06_RC_ATKP_LOG_EN
INIT_APP_EXPORT(addPeriodFunRc);
#endif

#endif