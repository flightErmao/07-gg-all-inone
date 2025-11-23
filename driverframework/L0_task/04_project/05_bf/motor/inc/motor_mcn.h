/**
 * @file motor_mcn.h
 *
 * Motor output message type definition (C interface, for C++ use)
 * This message contains mixed motor values after PID and mixer calculation
 */

#ifndef MOTOR_OUTPUT_MSG_H__
#define MOTOR_OUTPUT_MSG_H__

#include <rtdef.h>
#include "uMCN.h"

#define MAX_SUPPORTED_MOTORS 8

/* Motor output message type */
typedef struct {
  float motor[MAX_SUPPORTED_MOTORS];  // Mixed motor output values [0-1.0 or 1000-2000 range]
  uint8_t motor_count;                // Number of motors
  uint32_t timestamp;                 // Timestamp in microseconds
  uint32_t seq;                       // Sequence number
} motor_output_msg_t;

MCN_DECLARE(motor_output);

#endif /* MOTOR_OUTPUT_MSG_H__ */

