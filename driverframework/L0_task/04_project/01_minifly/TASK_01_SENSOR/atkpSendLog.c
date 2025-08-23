
#include <rtthread.h>
#include "sensorsTypes.h"
#include "taskMiniflySensor.h"
#include "floatConvert.h"

#ifdef TOOL_TASK_ANOTC_TELEM_EN

#include "packData.h"
#include "taskAnotcTelem.h"

void sendSensorImuData(uint16_t count_ms) {
  sensorData_t sensors = {0};
  if (!(count_ms % PERIOD_10ms)) {
    sensorsAcquire(&sensors);
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

int addPeriodFunListProjectMiniFlySensor(void) {
  anotcTelemAddSensorFunc(sendSensorImuData);
  return 0;
}

INIT_APP_EXPORT(addPeriodFunListProjectMiniFlySensor);

#endif
