#include "barometer.h"

#ifdef TASK_TOOL_01_ANOTC_TELEM_EN

#include "protocolAtkpInterface.h"
#include "taskAnotcTelem.h"

// 外部变量声明，这些变量在task_baro_report.c中定义
extern rt_device_t dev_sensor_baro_spl16_001;
extern rt_device_t dev_sensor_baro_dps368;
extern rt_device_t dev_sensor_baro_spa06_003;
extern baro_report_t baro_report_spl16_001;
extern baro_report_t baro_report_dps368;
extern baro_report_t baro_report_spa06_003;

void sendBaroData(uint16_t count_ms) {
  if (!(count_ms % PERIOD_20ms)) {
#ifdef BARO_REPORT_USING_SPL16_001
    if (dev_sensor_baro_spl16_001) {
      sendUserDatafloat2(1, baro_report_spl16_001.pressure_Pa, baro_report_spl16_001.temperature_deg);
    }
#endif

#ifdef BARO_REPORT_USING_DPS368
    if (dev_sensor_baro_dps368) {
      sendUserDatafloat3(1, baro_report_dps368.pressure_Pa, baro_report_dps368.temperature_deg, 0.0f);
    }
#endif

#ifdef BARO_REPORT_USING_SPA06_003
    if (dev_sensor_baro_spa06_003) {
      sendUserDatafloat2(1, baro_report_spa06_003.pressure_Pa, baro_report_spa06_003.altitude_m);
    }
#endif
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