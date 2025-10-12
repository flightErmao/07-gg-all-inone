
#include <rtthread.h>
#include "sensorsTypes.h"
#include "aMcnSensorImu.h"
#include "floatConvert.h"

#ifdef TASK_TOOL_01_ANOTC_TELEM_EN

#include "protocolAtkpInterface.h"
#include "taskAnotcTelem.h"

void sendSensorImuData(uint16_t count_ms) {
  sensorData_t sensors = {0};
  if (!(count_ms % PERIOD_10ms)) {
    mcnSensorImuAcquire(&sensors);
    sendUserDatafloat6_u32(1, sensors.acc_raw.x, sensors.acc_raw.y, sensors.acc_raw.z, sensors.acc_filter.x,
                           sensors.acc_filter.y, sensors.acc_filter.z, sensors.timestamp);
    // sendUserDatafloat6_u32(2, sensors.acc_raw.x, sensors.acc_raw.y, sensors.acc_raw.z, sensors.gyro_raw.x,
    //                        sensors.gyro_raw.y, sensors.gyro_raw.z, sensors.timestamp);

    // sendUserDatafloat12_u32(1, sensors.acc_raw.x, sensors.acc_raw.y, sensors.acc_raw.z, sensors.gyro_raw.x,
    //                         sensors.gyro_raw.y, sensors.gyro_raw.z, sensors.acc_filter.x, sensors.acc_filter.y,
    //                         sensors.acc_filter.z, sensors.gyro_filter.x, sensors.gyro_filter.y,
    //                         sensors.gyro_filter.z, sensors.timestamp);
  }
}

int addPeriodFunListSensor(void) {
  anotcTelemAddSensorFunc(sendSensorImuData);
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK_SENSOR_ATKP_LOG_EN
INIT_APP_EXPORT(addPeriodFunListSensor);
#endif

#endif
