
#include <rtthread.h>
#include "sensorsTypes.h"
#include "taskSensorMinifly.h"

#ifdef TOOL_TASK_ANOTC_TELEM_EN

#include "packData.h"
#include "taskAnotcTelem.h"

void sendSensorImuData(uint16_t count_ms) {
  sensorData_t sensors = {0};
  if (!(count_ms % PERIOD_10ms)) {
    sensorsAcquire(&sensors);
    sendUserDatafloat6(1, sensors.acc_filter.x, sensors.acc_filter.y, sensors.acc_filter.z, sensors.gyro_filter.x,
                       sensors.gyro_filter.y, sensors.gyro_filter.z);
  }
}

int addPeriodFunListProjectMiniFlySensor(void) {
  anotcTelemAddSensorFunc(sendSensorImuData);
  return 0;
}

INIT_APP_EXPORT(addPeriodFunListProjectMiniFlySensor);

#endif
