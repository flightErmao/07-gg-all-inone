#include "motor_bf.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "motor_bf"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "rc_aux_msg.h"
#include "rc_setpoint_msg.h"
#include "pid_output_msg.h"
#include "uMCN.h"
}

rt_err_t MotorBf::initMcn() {
  // Motor MCN subscriptions are created in the motor thread
  // No hub publishing needed for now
  LOG_I("Motor MCN initialized (subscriptions will be created in thread)");
  return RT_EOK;
}

rt_err_t MotorBf::subscribeMcnTopics(McnNode_t* pid_output_node, McnNode_t* rc_setpoint_node, McnNode_t* rc_aux_node) {
  if (pid_output_node == nullptr || rc_setpoint_node == nullptr || rc_aux_node == nullptr) {
    return -RT_ERROR;
  }

  // Create event semaphore for pid (required for mcn_poll_sync)
  if (pid_output_event_ == RT_NULL) {
    pid_output_event_ = rt_sem_create("motor_pid_evt", 0, RT_IPC_FLAG_FIFO);
    if (pid_output_event_ == RT_NULL) {
      LOG_E("create pid event semaphore failed");
      return -RT_ERROR;
    }
  }

  // Subscribe to PID output MCN topic with event semaphore
  *pid_output_node = mcn_subscribe(MCN_HUB(pid), pid_output_event_, RT_NULL);
  if (*pid_output_node == RT_NULL) {
    LOG_E("Failed to subscribe to pid");
    if (pid_output_event_ != RT_NULL) {
      rt_sem_delete(pid_output_event_);
      pid_output_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }

  // Subscribe to RC setpoint MCN topic (for throttle)
  *rc_setpoint_node = mcn_subscribe(MCN_HUB(rc), RT_NULL, RT_NULL);
  if (*rc_setpoint_node == RT_NULL) {
    LOG_E("Failed to subscribe to rc");
    mcn_unsubscribe(MCN_HUB(pid), *pid_output_node);
    return -RT_ERROR;
  }

  // Subscribe to RC aux MCN topic (for arm status and flight mode)
  *rc_aux_node = mcn_subscribe(MCN_HUB(aux), RT_NULL, RT_NULL);
  if (*rc_aux_node == RT_NULL) {
    LOG_E("Failed to subscribe to aux");
    mcn_unsubscribe(MCN_HUB(pid), *pid_output_node);
    mcn_unsubscribe(MCN_HUB(rc), *rc_setpoint_node);
    return -RT_ERROR;
  }

  LOG_I("Motor MCN subscriptions ready");
  return RT_EOK;
}

void MotorBf::unsubscribeMcnTopics() {
  if (pid_output_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(pid), pid_output_node_);
    pid_output_node_ = RT_NULL;
  }
  if (rc_setpoint_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(rc), rc_setpoint_node_);
    rc_setpoint_node_ = RT_NULL;
  }
  if (rc_aux_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(aux), rc_aux_node_);
    rc_aux_node_ = RT_NULL;
  }
}

bool MotorBf::updateAuxData(McnNode_t rc_aux_node, rc_aux_msg_t* aux_data, bool* aux_data_valid) {
  if (rc_aux_node == RT_NULL || aux_data == nullptr || aux_data_valid == nullptr) {
    return false;
  }

  // Check arm status from aux channels (non-blocking)
  // Update cached data if new data is available
  if (mcn_poll(rc_aux_node) == RT_TRUE) {
    if (mcn_copy(MCN_HUB(aux), rc_aux_node, aux_data) == RT_EOK) {
      *aux_data_valid = true;
      return true;  // New data available
    }
  }
  return false;  // No new data, but cached data may still be valid
}
