#include "attitude_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "anotc_attitude"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#ifdef PROJECT_BF_ANOTC_EN

extern "C" {
#include "protocolAtkpInterface.h"
#include "anotc_bf.h"
}

#ifdef PROJECT_BF_ATTITUDE_ANOTC_LOG_EN
// Static function to send attitude data (roll, pitch, yaw angles in degrees)
// Same as Betaflight: logs attitude angles for telemetry
static void sendAttitudeData(uint16_t count_ms) {
  if (!(count_ms % PROJECT_BF_ATTITUDE_ANOTC_LOG_PERIOD_MS)) {
    // Get data directly from AttitudeBf singleton
    AttitudeBf& attitude = AttitudeBf::instance();
    
    float attitude_data[3];
    attitude.getAttitude(attitude_data);
    
    // Send attitude data (roll, pitch, yaw in degrees)
    // Format: [roll, pitch, yaw]
    sendUserDatafloatN(PROJECT_BF_ATTITUDE_ANOTC_LOG_GROUP, attitude_data, 3);
  }
}
#endif  // PROJECT_BF_ATTITUDE_ANOTC_LOG_EN

int addPeriodFunListAttitude(void) {
#ifdef PROJECT_BF_ATTITUDE_ANOTC_LOG_EN
  anotcTelemAddSensorFunc(sendAttitudeData);
  LOG_I("anotcAttitude: Added attitude logging function");
#endif

  return 0;
}

#ifdef PROJECT_BF_ATTITUDE_ANOTC_LOG_EN
INIT_APP_EXPORT(addPeriodFunListAttitude);
#endif

#endif  // PROJECT_BF_ANOTC_EN

