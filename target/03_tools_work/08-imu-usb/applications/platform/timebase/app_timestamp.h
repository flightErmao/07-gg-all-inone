#ifndef APPLICATIONS_PLATFORM_TIMEBASE_APP_TIMESTAMP_H
#define APPLICATIONS_PLATFORM_TIMEBASE_APP_TIMESTAMP_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_uint64_t app_timestamp_now_us(void);
rt_uint32_t app_timestamp_now_ms(void);

#ifdef __cplusplus
}
#endif

#endif
