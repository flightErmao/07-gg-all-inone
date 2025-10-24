#ifndef __AT32_TIMESTAMP_H__
#define __AT32_TIMESTAMP_H__

#include <rtthread.h>
#include <stdint.h>

/* System clock frequency macros */
#define SYS_CLOCK_FREQ_HZ system_core_clock

/* Time conversion macros */
#define TICKS_PER_US (SYS_CLOCK_FREQ_HZ / 1000000)

#define US_TO_CYCLES(us) ((us) * TICKS_PER_US)
#define CYCLES_TO_US(cycles) ((cycles) / TICKS_PER_US)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get current time in microseconds
 * @return Current time in microseconds
 * @note This function can be called from both interrupt and normal context
 */
uint32_t at32_timestamp_micros(void);

#ifdef __cplusplus
}
#endif

#endif /* __AT32_TIMESTAMP_H__ */
