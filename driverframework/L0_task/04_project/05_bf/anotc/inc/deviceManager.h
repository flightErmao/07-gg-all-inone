#ifndef DEVICE_MANAGER_H__
#define DEVICE_MANAGER_H__

#include <rtthread.h>
#include "protocolAtkpInterface.h"

extern "C" {
extern rt_device_t dev_anotc_telem_;
}

/* Public interface functions */
extern "C" {
rt_err_t uartDevAnotcInit(char* device_name);
void anotcDeviceSendDirect(atkp_t* p);
struct rt_messagequeue* getAnotcRecMq(void);
}

#endif /* DEVICE_MANAGER_H__ */

