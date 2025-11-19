/**
 * @file pid_setpoint_msg.h
 *
 * PID setpoint message type definition (C interface, for C++ use)
 */

#ifndef PID_SETPOINT_MSG_H__
#define PID_SETPOINT_MSG_H__

#include <rtdef.h>
#include "uMCN.h"

/* PID setpoint message type */
typedef struct {
  float rate[3];        // [roll, pitch, yaw] rate setpoint in deg/s
  float feedforward[3]; // Feedforward values for each axis
  uint32_t timestamp;
  uint32_t seq;
} pid_setpoint_msg_t;

MCN_DECLARE(pid_setpoint);

#endif /* PID_SETPOINT_MSG_H__ */

