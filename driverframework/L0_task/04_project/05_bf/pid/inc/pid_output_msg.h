/**
 * @file pid_output_msg.h
 *
 * PID output message type definition (C interface, for C++ use)
 */

#ifndef PID_OUTPUT_MSG_H__
#define PID_OUTPUT_MSG_H__

#include <rtdef.h>
#include "uMCN.h"

/* PID output message type */
typedef struct {
  float pid_sum[3];  // [roll, pitch, yaw] PID sum output
  float pid_p[3];    // P term
  float pid_i[3];    // I term
  float pid_d[3];    // D term
  float pid_f[3];    // F term (feedforward)
  uint32_t timestamp;
  uint32_t seq;
} pid_output_msg_t;

MCN_DECLARE(pid_output);

#endif /* PID_OUTPUT_MSG_H__ */

