#ifndef __DEVICE_MANAGER_H__
#define __DEVICE_MANAGER_H__

#include <rtthread.h>
#include "protocolAtkpInterface.h"

/* 外部变量声明 */
extern rt_device_t dev_anotc_telem_;

/* 设备管理接口 */
rt_err_t task_dev_init(char* device_name);
void anotcDeviceSendDirect(atkp_t* p);

#endif /* __DEVICE_MANAGER_H__ */