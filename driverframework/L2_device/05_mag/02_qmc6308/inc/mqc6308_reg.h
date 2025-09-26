#ifndef __MQC6308_REG_H__
#define __MQC6308_REG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t drv_mqc6308_init(const char* i2c_device_name, const char* device_name);

#ifdef __cplusplus
}
#endif

#endif
