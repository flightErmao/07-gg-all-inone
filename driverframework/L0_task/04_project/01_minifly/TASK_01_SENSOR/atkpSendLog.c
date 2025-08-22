
#include <rtthread.h>
#include "sensorsTypes.h"
#include "taskSensorMinifly.h"
#include "floatConvert.h"

#ifdef TOOL_TASK_ANOTC_TELEM_EN

#include "packData.h"
#include "taskAnotcTelem.h"

void sendSensorImuData(uint16_t count_ms) {
  sensorData_t sensors = {0};
  if (!(count_ms % PERIOD_10ms)) {
    sensorsAcquire(&sensors);
    sendUserDatafloat6(1, sensors.acc_filter.x, sensors.acc_filter.y, sensors.acc_filter.z, sensors.gyro_filter.x,
                       sensors.gyro_filter.y, sensors.gyro_filter.z);

    static int cnt = 0;
    if (++cnt % 300 == 0) {
      char ax[16], ay[16], az[16], gx[16], gy[16], gz[16];
      float_to_string(sensors.acc_filter.x, ax, sizeof(ax));
      float_to_string(sensors.acc_filter.y, ay, sizeof(ay));
      float_to_string(sensors.acc_filter.z, az, sizeof(ax));
      float_to_string(sensors.gyro_filter.x, gx, sizeof(gx));
      float_to_string(sensors.gyro_filter.y, gy, sizeof(gy));
      float_to_string(sensors.gyro_filter.z, gz, sizeof(gz));
      rt_kprintf("atkpSendLog: acc: %s, %s, %s, gyro: %s, %s, %s\n", ax, ay, az, gx, gy, gz);
    }
  }
}

int addPeriodFunListProjectMiniFlySensor(void) {
  anotcTelemAddSensorFunc(sendSensorImuData);
  return 0;
}

INIT_APP_EXPORT(addPeriodFunListProjectMiniFlySensor);

#endif
