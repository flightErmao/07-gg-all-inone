#include "pid_bf.hpp"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "anotc_pid"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#ifdef PROJECT_BF_ANOTC_EN

extern "C" {
#include "protocolAtkpInterface.h"
#include "anotc_bf.h"
}

#ifdef PROJECT_BF_PID_ANOTC_LOG_EN

// Static function to send rate setpoint and actual rate
static void sendRateSetpointActual(uint16_t count_ms) {
  if (!(count_ms % PROJECT_BF_PID_ANOTC_LOG_ANGULAR_RATE_PERIOD_MS)) {
    // Get data directly from singleton instances
    PidBf& pid = PidBf::instance();
    
    // Get data directly from PidBf singleton
    const gyro_filtered_msg_t& gyro_data = pid.getGyroFilteredData();
    const pid_setpoint_msg_t& setpoint_data = pid.getSetpointData();
    
    // Send rate setpoint (desired) and actual rate (from gyro)
    // Format: [roll_setpoint, pitch_setpoint, yaw_setpoint, roll_actual, pitch_actual, yaw_actual]
    sendUserDatafloat6(
        PROJECT_BF_PID_ANOTC_LOG_ANGULAR_RATE_GROUP,
        setpoint_data.rate[0],  // roll setpoint
        setpoint_data.rate[1],  // pitch setpoint
        setpoint_data.rate[2],  // yaw setpoint
        gyro_data.gyro_filtered[0],  // roll actual (gyro x)
        gyro_data.gyro_filtered[1],  // pitch actual (gyro y)
        gyro_data.gyro_filtered[2]   // yaw actual (gyro z)
    );
  }
}

int addPeriodFunListPid(void) {
#ifdef PROJECT_BF_PID_ANOTC_LOG_ANGULAR_RATE_EN
  anotcTelemAddSensorFunc(sendRateSetpointActual);
  LOG_I("anotcPid: Added rate setpoint/actual logging function");
#endif
  return 0;
}

#ifdef PROJECT_BF_PID_ANOTC_LOG_EN
INIT_APP_EXPORT(addPeriodFunListPid);
#endif

#endif  // PROJECT_BF_PID_ANOTC_LOG_EN

#endif  // PROJECT_BF_ANOTC_EN

