#include <rtthread.h>
#include "mag.h"
#include "mcnMagShow.h"

#ifdef TASK_TOOL_01_ANOTC_TELEM_EN

#include "protocolAtkpInterface.h"
#include "taskAnotcTelem.h"

void sendMagData(uint16_t count_ms) {
  mag_report_t mag_data = {0};
  if (!(count_ms % PERIOD_20ms)) {
    mcnMagReportAcquire(&mag_data);
    sendUserDatafloat3(TASK_MAG_ATKP_CHANNEL, mag_data.value_x, mag_data.value_y, mag_data.value_z);
  }
}

int addPeriodFunMag(void) {
  anotcTelemAddSensorFunc(sendMagData);
  return 0;
}

#ifdef TASK_MAG_ATKP_LOG_EN
INIT_APP_EXPORT(addPeriodFunMag);
#endif

#endif

