#ifndef __STM32_TIMESTAMP_H__
#define __STM32_TIMESTAMP_H__

#include <rtthread.h>
#include <stdint.h>

/* System clock frequency macros */
#define SYS_CLOCK_FREQ_HZ SystemCoreClock

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
uint32_t stm32_timestamp_micros(void);

/**
 * @brief Get current cycle count
 * @return Current DWT cycle count
 */
uint32_t stm32_timestamp_cycles(void);

/**
 * @brief Get system clock frequency
 * @return System clock frequency in Hz
 */
uint32_t stm32_timestamp_get_sys_freq(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32_TIMESTAMP_H__ */
