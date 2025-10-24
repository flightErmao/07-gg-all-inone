#ifndef __TIMESTAMP_H__
#define __TIMESTAMP_H__

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Include platform-specific timestamp headers */
#ifdef SOC_FAMILY_AT32
#include "at32TimeStamp.h"

/* Platform-specific function mappings */
#define timestamp_init()           at32_timestamp_init()
#define timestamp_micros()         at32_timestamp_micros()
#define timestamp_millis()         at32_timestamp_millis()
#define timestamp_cycles()         at32_timestamp_cycles()
#define timestamp_delay_us(us)     at32_timestamp_delay_us(us)
#define timestamp_delay_ms(ms)     at32_timestamp_delay_ms(ms)
#define timestamp_get_sys_freq()   at32_timestamp_get_sys_freq()

#elif defined(SOC_FAMILY_STM32)
#include "stm32TimeStamp.h"

/* Platform-specific function mappings */
#define timestamp_init()           stm32_timestamp_init()
#define timestamp_micros()         stm32_timestamp_micros()
#define timestamp_millis()         stm32_timestamp_millis()
#define timestamp_cycles()         stm32_timestamp_cycles()
#define timestamp_delay_us(us)     stm32_timestamp_delay_us(us)
#define timestamp_delay_ms(ms)     stm32_timestamp_delay_ms(ms)
#define timestamp_get_sys_freq()   stm32_timestamp_get_sys_freq()

#else
#error "Unsupported platform for timestamp functionality"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __TIMESTAMP_H__ */
