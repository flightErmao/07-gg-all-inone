#ifndef __TASK_NRF_H__
#define __TASK_NRF_H__

#include "protocolAtkpInterface.h"

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