#include <rtthread.h>
#include "taskRc.h"
#include "floatConvert.h"

#ifdef TASK_TOOL_01_ANOTC_TELEM_EN

#include "protocolAtkpInterface.h"
#include "taskAnotcTelem.h"

void sendRcData(uint16_t count_ms) {
  if (!(count_ms % PERIOD_20ms)) {
    rt_uint16_t* rc_channels = getRcChannels();
    
    if (rc_channels != RT_NULL) {
      sendUserDatauint16_8(2, 
                          rc_channels[0],   // CH1
                          rc_channels[1],   // CH2
                          rc_channels[2],   // CH3
                          rc_channels[3],   // CH4
                          rc_channels[4],   // CH5
                          rc_channels[5],   // CH6
                          rc_channels[6],   // CH7
                          rc_channels[7]);  // CH8
    }
  }

  if (!(count_ms % PERIOD_20ms)) {
    pilot_cmd_bus_t rc_data = {0};
    rcPilotCmdAcquire(&rc_data);
    sendUserDatafloat6_u32(1, rc_data.stick_yaw, rc_data.stick_throttle, rc_data.stick_roll, rc_data.stick_pitch,
                           (float)rc_data.ram_status, (float)rc_data.ctrl_mode, rc_data.timestamp);
  }
}

int addPeriodFunRc(void) {
  anotcTelemAddSensorFunc(sendRcData);
  return 0;
}

#ifdef PROJECT_FMT_TASK01_RC_ATKP_LOG_EN
INIT_APP_EXPORT(addPeriodFunRc);
#endif

#endif