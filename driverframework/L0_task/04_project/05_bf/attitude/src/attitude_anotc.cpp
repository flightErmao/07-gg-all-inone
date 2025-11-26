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
#include "rc_mcn.h"
#include "uMCN.h"
}

#ifdef PROJECT_BF_ATTITUDE_ANOTC_LOG_EN
// Static MCN subscription node for RC aux data (armed and flight_mode)
static McnNode_t rc_aux_node_ = RT_NULL;
static rc_aux_msg_t rc_aux_data_ = {0};
static bool rc_aux_data_valid_ = false;

// Static function to send attitude data using packStatus interface
// This allows the ground station to display aircraft attitude information
static void sendAttitudeData(uint16_t count_ms) {
  if (!(count_ms % PROJECT_BF_ATTITUDE_ANOTC_LOG_PERIOD_MS)) {
    // Get attitude data directly from AttitudeBf singleton
    AttitudeBf& attitude = AttitudeBf::instance();

    float attitude_data[3];  // [roll, pitch, yaw] in degrees
    attitude.getAttitude(attitude_data);

    // Update RC aux data from MCN (non-blocking)
    // This provides armed status and flight mode
    if (rc_aux_node_ != RT_NULL) {
      if (mcn_poll(rc_aux_node_) == RT_TRUE) {
        if (mcn_copy(MCN_HUB(aux), rc_aux_node_, &rc_aux_data_) == RT_EOK) {
          rc_aux_data_valid_ = true;
        }
      }
    }

    // Get armed status and flight mode (use default values if not available)
    uint8_t armed = rc_aux_data_valid_ ? rc_aux_data_.armed : 0;
    uint8_t flight_mode = rc_aux_data_valid_ ? rc_aux_data_.flight_mode : 0;

    // Height is currently set to 0 (can be updated later if altitude data is available)
    int32_t alt = 0;

    // Send attitude data using packStatus interface
    // packStatus(float roll, float pitch, float yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
    packStatus(attitude_data[0], attitude_data[1], attitude_data[2], alt, flight_mode, armed);
  }
}
#endif  // PROJECT_BF_ATTITUDE_ANOTC_LOG_EN

int addPeriodFunListAttitude(void) {
#ifdef PROJECT_BF_ATTITUDE_ANOTC_LOG_EN
  // Subscribe to RC aux MCN topic for armed status and flight mode
  rc_aux_node_ = mcn_subscribe(MCN_HUB(aux), RT_NULL, RT_NULL);
  if (rc_aux_node_ == RT_NULL) {
    LOG_W("anotcAttitude: Failed to subscribe to aux MCN topic, using default values");
    rc_aux_data_valid_ = false;
  } else {
    LOG_I("anotcAttitude: Subscribed to aux MCN topic for armed/flight_mode data");
    rc_aux_data_valid_ = false;
  }

  anotcTelemAddSensorFunc(sendAttitudeData);
  LOG_I("anotcAttitude: Added attitude logging function using packStatus interface");
#endif

  return 0;
}

#ifdef PROJECT_BF_ATTITUDE_ANOTC_LOG_EN
INIT_APP_EXPORT(addPeriodFunListAttitude);
#endif

#endif  // PROJECT_BF_ANOTC_EN

