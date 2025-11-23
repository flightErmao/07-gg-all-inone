/**
 * @file rc_setpoint_msg.h
 *
 * RC setpoint message type definition (C interface, for C++ use)
 * This message contains raw setpoint data from RC processing, used for smoothing filter in PID thread
 */

#ifndef RC_SETPOINT_MSG_H__
#define RC_SETPOINT_MSG_H__

#include <rtdef.h>
#include "uMCN.h"

#define XYZ_AXIS_COUNT 3
#define PRIMARY_CHANNEL_COUNT 4  // Roll, Pitch, Yaw, Throttle

/* RC setpoint message type */
/* This message contains only the data needed by smoothing filter */
typedef struct {
  float rawSetpoint[XYZ_AXIS_COUNT];  // Raw setpoint rates [roll, pitch, yaw] (deg/s)
  float rcCommandThrottle;            // RC command throttle (1000-2000)
  float feedforward[XYZ_AXIS_COUNT];  // Feedforward values [roll, pitch, yaw]
  rt_uint32_t seq;                    // Sequence number
  rt_uint32_t timestamp;              // Timestamp in microseconds
} rc_setpoint_msg_t;

MCN_DECLARE(rc);

#endif /* RC_SETPOINT_MSG_H__ */

