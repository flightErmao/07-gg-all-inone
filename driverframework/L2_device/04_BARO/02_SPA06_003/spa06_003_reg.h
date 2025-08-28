#ifndef SPL06_H__
#define SPL06_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t drv_spl16_001_init(const char* spi_device_name, const char* baro_device_name);

#ifdef __cplusplus
}
#endif

#endif
