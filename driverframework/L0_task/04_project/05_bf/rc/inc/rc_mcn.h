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

// PWM range constants (same as Betaflight)
#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE (PWM_RANGE_MAX - PWM_RANGE_MIN)
#define PWM_RANGE_MIDDLE (PWM_RANGE_MIN + (PWM_RANGE / 2))
#define PWM_PULSE_MIN 750
#define PWM_PULSE_MAX 2250

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
} rc_command_msg_t;

MCN_DECLARE(rc);

#define MAX_AUX_CHANNEL_COUNT 14  // AUX1-AUX14 (channels 5-18)

/* Arming status enumeration */
typedef enum {
  RC_ARMED_STATUS_DISARMED = 0,  // Disarmed state
  RC_ARMED_STATUS_ARMED = 1       // Armed state
} rc_armed_status_t;

/* RC auxiliary channels message type */
/* This message contains auxiliary channel data, arm status, and flight mode */
typedef struct {
  float aux_channels[MAX_AUX_CHANNEL_COUNT];  // AUX channel values [AUX1-AUX14] (1000-2000)
  uint8_t aux_channel_count;                  // Number of available AUX channels (0-14)
  rt_uint8_t rx_receiving_signal;             // RX receiving signal status (0=false, 1=true)
  rt_uint8_t rx_flight_channels_valid;        // Flight channels valid status (0=false, 1=true)
  rt_uint8_t armed;                           // Arming status (RC_ARMED_STATUS_DISARMED=0, RC_ARMED_STATUS_ARMED=1)
  uint8_t flight_mode;                        // Flight mode (0=角速度模式/Rate, 1=角度模式/Angle, 2=高度模式/Altitude)
  rt_uint32_t seq;                            // Sequence number
  rt_uint32_t timestamp;                      // Timestamp in microseconds
} rc_aux_msg_t;

MCN_DECLARE(aux);


#endif /* RC_SETPOINT_MSG_H__ */

