#ifndef PID_MCN_H__
#define PID_MCN_H__

#include <rtthread.h>
#include <rtdef.h>
#include "uMCN.h"

MCN_DECLARE(pid);

/* PID output message type */
typedef struct {
  float pid_sum[3];         // [roll, pitch, yaw] PID sum output
  float pid_p[3];           // P term
  float pid_i[3];           // I term
  float pid_d[3];           // D term
  float pid_f[3];           // F term (feedforward)
  float smoothed_throttle;  // Smoothed throttle value (from RC smoothing filter, in PWM range 1000-2000)
  uint32_t timestamp;
  uint32_t seq;
} pid_output_msg_t;

#endif /* PID_MCN_HPP__ */