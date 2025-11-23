#include "motor_bf.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "motor_bf"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "rc_mcn.h"  // For rc_aux_msg_t, rc_armed_status_t
#include "pid_mcn.h"
#include "uMCN.h"
}

// Initialize MCN (no hub publishing needed for now)
rt_err_t MotorBf::initMcn() {
  LOG_I("Motor MCN initialized (subscriptions will be created in thread)");
  return RT_EOK;
}

// Subscribe to PID output MCN topic (with event semaphore for blocking poll)
rt_err_t MotorBf::subscribePidOutput() {
  // Create event semaphore for pid (required for mcn_poll_sync)
  if (pid_output_event_ == RT_NULL) {
    pid_output_event_ = rt_sem_create("motor_pid_evt", 0, RT_IPC_FLAG_FIFO);
    if (pid_output_event_ == RT_NULL) {
      LOG_E("create pid event semaphore failed");
      return -RT_ERROR;
    }
  }

  // Subscribe to PID output MCN topic with event semaphore
  pid_output_node_ = mcn_subscribe(MCN_HUB(pid), pid_output_event_, RT_NULL);
  if (pid_output_node_ == RT_NULL) {
    LOG_E("Failed to subscribe to pid");
    if (pid_output_event_ != RT_NULL) {
      rt_sem_delete(pid_output_event_);
      pid_output_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }

  LOG_I("Subscribed to pid MCN topic");
  return RT_EOK;
}

// Subscribe to RC aux MCN topic (for arm status and flight mode)
rt_err_t MotorBf::subscribeRcAux() {
  rc_aux_node_ = mcn_subscribe(MCN_HUB(aux), RT_NULL, RT_NULL);
  if (rc_aux_node_ == RT_NULL) {
    LOG_W("Failed to subscribe to aux, continuing without aux channel support");
    // Not critical, continue running
    return -RT_ERROR;
  }

  LOG_I("Subscribed to aux MCN topic");
  return RT_EOK;
}

// Unsubscribe from all MCN topics
void MotorBf::unsubscribeMcnTopics() {
  if (pid_output_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(pid), pid_output_node_);
    pid_output_node_ = RT_NULL;
  }
  if (rc_aux_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(aux), rc_aux_node_);
    rc_aux_node_ = RT_NULL;
  }
}

// Update aux data from MCN (non-blocking)
bool MotorBf::updateAuxData(rc_aux_msg_t* aux_data, bool* aux_data_valid) {
  if (rc_aux_node_ == RT_NULL || aux_data == nullptr || aux_data_valid == nullptr) {
    return false;
  }

  // Check arm status from aux channels (non-blocking)
  // Update cached data if new data is available
  if (mcn_poll(rc_aux_node_) == RT_TRUE) {
    if (mcn_copy(MCN_HUB(aux), rc_aux_node_, aux_data) == RT_EOK) {
      *aux_data_valid = true;
      return true;  // New data available
    }
  }
  return false;  // No new data, but cached data may still be valid
}
