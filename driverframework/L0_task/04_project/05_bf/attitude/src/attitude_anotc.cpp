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
#include "attitude_mcn.h"
}

#ifdef PROJECT_BF_ATTITUDE_ANOTC_LOG_EN
// Static MCN subscription node for RC aux data (armed and flight_mode)
static McnNode_t rc_aux_node_ = RT_NULL;
static rc_aux_msg_t rc_aux_data_ = {0};
static bool rc_aux_data_valid_ = false;

// Static MCN subscription node for attitude data (to get latest attitude values)
static McnNode_t attitude_node_ = RT_NULL;
static attitude_msg_t attitude_data_cache_ = {0};
static bool attitude_data_valid_ = false;

// Static function to send attitude data using packStatus interface
// This allows the ground station to display aircraft attitude information
// Similar to Betaflight: subscribe to attitude MCN topic to get latest values
static void sendAttitudeData(uint16_t count_ms) {
  if (!(count_ms % PROJECT_BF_ATTITUDE_ANOTC_LOG_PERIOD_MS)) {
    // Update attitude data from MCN (non-blocking)
    // This ensures we get the latest attitude values from the attitude estimation thread
    if (attitude_node_ != RT_NULL) {
      if (mcn_poll(attitude_node_) == RT_TRUE) {
        if (mcn_copy(MCN_HUB(att), attitude_node_, &attitude_data_cache_) == RT_EOK) {
          attitude_data_valid_ = true;
        }
      }
    }

    // Get attitude data from MCN (latest values from attitude thread)
    // Similar to Betaflight: always use latest data from attitude estimation thread
    float attitude_data[3] = {0.0f, 0.0f, 0.0f};  // [roll, pitch, yaw] in degrees

    if (attitude_data_valid_ && attitude_data_cache_.seq > 0) {
      // Use data from MCN (latest values from attitude thread)
      // Only use if seq > 0, which indicates attitude has been updated at least once
      attitude_data[0] = attitude_data_cache_.values[0];  // roll in degrees
      attitude_data[1] = -attitude_data_cache_.values[1];  // pitch in degrees
      attitude_data[2] = -attitude_data_cache_.values[2];  // yaw in degrees
    } else {
      // Attitude not initialized yet or MCN subscription failed
      // Don't send uninitialized data (will send 0,0,0 which is better than stale data)
      // In Betaflight, attitude is initialized from accelerometer on first update
    }

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
  // Subscribe to attitude MCN topic to get latest attitude values
  // This is similar to Betaflight: get attitude data from MCN subscription
  attitude_node_ = mcn_subscribe(MCN_HUB(att), RT_NULL, RT_NULL);
  if (attitude_node_ == RT_NULL) {
    LOG_W("anotcAttitude: Failed to subscribe to attitude MCN topic, will use singleton fallback");
    attitude_data_valid_ = false;
  } else {
    LOG_I("anotcAttitude: Subscribed to attitude MCN topic for latest attitude data");
    attitude_data_valid_ = false;
  }

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

