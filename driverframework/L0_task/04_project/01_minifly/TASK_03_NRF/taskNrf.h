#ifndef __TASK_NRF_H__
#define __TASK_NRF_H__

#ifdef TASK_TOOL_01_ANOTC_TELEM_EN
#include "protocolAtkpInterface.h"
#else
typedef struct {
  uint8_t msgID;
  uint8_t dataLen;
  uint8_t data[128];  // ATKP_MAX_DATA_SIZE
} atkp_t;
#endif

#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF
#define ATKP_MAX_DATA_SIZE 128

typedef enum {
    waitForStartByte1,
    waitForStartByte2,
    waitForMsgID,
    waitForDataLength,
    waitForData,
    waitForChksum1
} rx_state_t;

void radiolinkTask(void *param);

#endif 