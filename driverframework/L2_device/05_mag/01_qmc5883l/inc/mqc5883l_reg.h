#ifndef __MQC5883L_REG_H__
#define __MQC5883L_REG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C"
{
#endif

    rt_err_t drv_mqc5883l_init(const char* i2c_device_name, const char* device_name);

#ifdef __cplusplus
}
#endif

#endif
