#include "pid_class.h"

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
#endif  // PROJECT_BF_PID_ANOTC_LOG_ANGULAR_RATE_EN

#ifdef PROJECT_BF_PID_ANOTC_LOG_PITCH_PIDF_EN
// Static function to send pitch axis PIDF and sum data
static void sendPitchPidfData(uint16_t count_ms) {
  if (!(count_ms % PROJECT_BF_PID_ANOTC_LOG_PITCH_PIDF_PERIOD_MS)) {
    // Get data directly from PidBf singleton (same as sendRateSetpointActual)
    PidBf& pid = PidBf::instance();
    const pidAxisData_t* pid_data = pid.getPidData();

    if (pid_data != nullptr) {
      // Pitch axis is index 1 (FD_PITCH = 1)
      const pidAxisData_t& pitch_data = pid_data[1];

      // Send pitch axis PIDF and sum data
      // Format: [P, I, D, F, Sum]
      float pitch_pidf_data[5] = {
          pitch_data.P,   // Pitch P term
          pitch_data.I,   // Pitch I term
          pitch_data.D,   // Pitch D term
          pitch_data.F,   // Pitch F term (feedforward)
          pitch_data.Sum  // Pitch Sum
      };
      sendUserDatafloatN(PROJECT_BF_PID_ANOTC_LOG_PITCH_PIDF_GROUP, pitch_pidf_data, 5);
    }
  }
}
#endif  // PROJECT_BF_PID_ANOTC_LOG_PITCH_PIDF_EN

#ifdef PROJECT_BF_PID_ANOTC_LOG_ANGLE_EN
// Static function to send angle setpoint and actual angle data
static void sendAngleSetpointActual(uint16_t count_ms) {
  if (!(count_ms % PROJECT_BF_PID_ANOTC_LOG_ANGLE_PERIOD_MS)) {
    // Get data directly from PidBf singleton
    PidBf& pid = PidBf::instance();
    
#ifdef PROJECT_BF_ATTITUDE_EN
    // Get actual angle from attitude data
    const attitude_msg_t& attitude_data = pid.getAttitudeData();
    
    // Send angle setpoint (desired) and actual angle
    // Format: [roll_setpoint, pitch_setpoint, roll_actual, pitch_actual, yaw_setpoint, yaw_actual]
    // Note: Angle mode only uses roll and pitch, yaw values are set to 0
    sendUserDatafloat6(
        PROJECT_BF_PID_ANOTC_LOG_ANGLE_GROUP,
        pid.getAngleTarget(0),      // roll setpoint (degrees)
        pid.getAngleTarget(1),      // pitch setpoint (degrees)
        attitude_data.values[0],    // roll actual (degrees)
        attitude_data.values[1],    // pitch actual (degrees)
        0.0f,                       // yaw setpoint (not used in angle mode, set to 0)
        attitude_data.values[2]     // yaw actual (degrees, for reference)
    );
#endif  // PROJECT_BF_ATTITUDE_EN
  }
}
#endif  // PROJECT_BF_PID_ANOTC_LOG_ANGLE_EN

int addPeriodFunListPid(void) {
#ifdef PROJECT_BF_PID_ANOTC_LOG_ANGULAR_RATE_EN
  anotcTelemAddSensorFunc(sendRateSetpointActual);
  LOG_I("anotcPid: Added rate setpoint/actual logging function");
#endif

#ifdef PROJECT_BF_PID_ANOTC_LOG_PITCH_PIDF_EN
  anotcTelemAddSensorFunc(sendPitchPidfData);
  LOG_I("anotcPid: Added pitch PIDF logging function");
#endif

#ifdef PROJECT_BF_PID_ANOTC_LOG_ANGLE_EN
  anotcTelemAddSensorFunc(sendAngleSetpointActual);
  LOG_I("anotcPid: Added angle setpoint/actual logging function");
#endif

  return 0;
}

#ifdef PROJECT_BF_PID_ANOTC_LOG_EN
INIT_APP_EXPORT(addPeriodFunListPid);
#endif

#endif  // PROJECT_BF_PID_ANOTC_LOG_EN
