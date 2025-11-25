#include "motor_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "anotc_motor"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#ifdef PROJECT_BF_ANOTC_EN

extern "C" {
#include "protocolAtkpInterface.h"
#include "anotc_bf.h"
}

#ifdef PROJECT_BF_MOTOR_ANOTC_LOG_EN
// Static function to send motor output data (normalized values 0.0-1.0)
// Same as Betaflight: logs normalized motor output before scaling to 48-2047
static void sendMotorOutputData(uint16_t count_ms) {
  if (!(count_ms % PROJECT_BF_MOTOR_ANOTC_LOG_PERIOD_MS)) {
    // Get data directly from MotorBf singleton
    MotorBf& motor = MotorBf::instance();
    
    const float* motor_output = motor.getMotorOutput();
    float throttle = motor.getThrottle();
    uint8_t motor_count = motor.getMotorCount();
    
    // Send motor output data (normalized 0.0-1.0) and throttle
    // Format: [motor0, motor1, motor2, motor3, throttle]
    // Note: motor output is normalized (0.0-1.0) before scaling to 48-2047 for DShot
    //       throttle is normalized (0.0-1.0) after constrainThrottleForMix
    if (motor_count >= 4) {
      // Send 4 motors + throttle (5 values total)
      float motor_data[5] = {
          motor_output[0],  // Motor 0 (normalized)
          motor_output[1],  // Motor 1 (normalized)
          motor_output[2],  // Motor 2 (normalized)
          motor_output[3],  // Motor 3 (normalized)
          throttle           // Throttle (normalized, after constrainThrottleForMix)
      };
      sendUserDatafloatN(PROJECT_BF_MOTOR_ANOTC_LOG_GROUP, motor_data, 5);
    } else {
      // For less than 4 motors, send available motors + throttle
      float motor_data[5] = {0.0f, 0.0f, 0.0f, 0.0f, throttle};
      for (uint8_t i = 0; i < motor_count && i < 4; i++) {
        motor_data[i] = motor_output[i];
      }
      sendUserDatafloatN(PROJECT_BF_MOTOR_ANOTC_LOG_GROUP, motor_data, 5);
    }
  }
}
#endif  // PROJECT_BF_MOTOR_ANOTC_LOG_EN

int addPeriodFunListMotor(void) {
#ifdef PROJECT_BF_MOTOR_ANOTC_LOG_EN
  anotcTelemAddSensorFunc(sendMotorOutputData);
  LOG_I("anotcMotor: Added motor output logging function");
#endif

  return 0;
}

#ifdef PROJECT_BF_MOTOR_ANOTC_LOG_EN
INIT_APP_EXPORT(addPeriodFunListMotor);
#endif

#endif  // PROJECT_BF_ANOTC_EN

