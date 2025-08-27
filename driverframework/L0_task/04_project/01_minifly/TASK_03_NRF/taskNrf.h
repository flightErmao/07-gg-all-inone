#ifndef __TASK_NRF_H__
#define __TASK_NRF_H__
#include <rtthread.h>
#include <rtdevice.h>
#include "protocolAtkpInterface.h"

#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF
#define ATKP_MAX_DATA_SIZE 128

rt_device_t getNrfDevice(void);

#endif