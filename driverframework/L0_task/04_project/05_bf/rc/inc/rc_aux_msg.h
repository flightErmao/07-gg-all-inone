/**
 * @file rc_aux_msg.h
 *
 * RC auxiliary channels message type definition (C interface, for C++ use)
 * This message contains auxiliary channel data from RC processing, including failsafe handling
 */

#ifndef RC_AUX_MSG_H__
#define RC_AUX_MSG_H__

#include <rtdef.h>
#include "uMCN.h"

#define MAX_AUX_CHANNEL_COUNT 14  // AUX1-AUX14 (channels 5-18)

/* RC auxiliary channels message type */
typedef struct {
  float aux_channels[MAX_AUX_CHANNEL_COUNT];  // AUX channel values [AUX1-AUX14] (1000-2000)
  uint8_t aux_channel_count;                  // Number of available AUX channels (0-14)
  rt_uint8_t rx_receiving_signal;             // RX receiving signal status (0=false, 1=true)
  rt_uint8_t rx_flight_channels_valid;        // Flight channels valid status (0=false, 1=true)
  rt_uint32_t seq;                            // Sequence number
  rt_uint32_t timestamp;                      // Timestamp in microseconds
} rc_aux_msg_t;

MCN_DECLARE(aux);

#endif /* RC_AUX_MSG_H__ */

