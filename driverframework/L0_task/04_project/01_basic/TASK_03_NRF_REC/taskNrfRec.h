#ifndef __TASK_NRF_H__
#define __TASK_NRF_H__
#include <rtthread.h>
#include <rtdevice.h>
#include "protocolAtkpInterface.h"
#include "rtconfig.h"
#include "debugPin.h"

rt_device_t getNrfDevice(void);
struct rt_messagequeue* getNrfRecvMq(void);

#endif