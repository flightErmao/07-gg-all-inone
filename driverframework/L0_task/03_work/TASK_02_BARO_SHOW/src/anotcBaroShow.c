#include <rtthread.h>
#include "barometer.h"
#include "mcnBaroShow.h"
#include "floatConvert.h"

#ifdef TASK_TOOL_01_ANOTC_TELEM_EN

#include "protocolAtkpInterface.h"
#include "taskAnotcTelem.h"

void sendBaroData(uint16_t count_ms) {
  baro_report_t baro_data = {0};
  if (!(count_ms % PERIOD_20ms)) {
    mcnBaroReportAcquire(&baro_data);
    sendUserDatafloat3(TASK_BARO_ATKP_CHANNEL, baro_data.pressure_Pa, baro_data.temperature_deg, 0);
  }
}

int addPeriodFunBaro(void) {
  anotcTelemAddSensorFunc(sendBaroData);
  return 0;
}

#ifdef TASK_BARO_ATKP_LOG_EN
INIT_APP_EXPORT(addPeriodFunBaro);
#endif

#endif