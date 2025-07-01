#ifndef SPL06_H__
#define SPL06_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t drv_dps368_init(const char* i2c_device_name, const char* baro_device_name);

#ifdef __cplusplus
}
#endif

#endif
