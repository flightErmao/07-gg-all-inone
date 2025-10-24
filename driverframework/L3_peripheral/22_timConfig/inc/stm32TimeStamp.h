#ifndef __STM32_TIMESTAMP_H__
#define __STM32_TIMESTAMP_H__

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize timestamp system
 * @note This function initializes the system tick and DWT for high precision timing
 */
void stm32_timestamp_init(void);

/**
 * @brief Get current time in microseconds
 * @return Current time in microseconds
 * @note This function can be called from both interrupt and normal context
 */
uint32_t stm32_timestamp_micros(void);

/**
 * @brief Get current time in milliseconds
 * @return Current time in milliseconds
 */
uint32_t stm32_timestamp_millis(void);

/**
 * @brief Get current cycle count
 * @return Current DWT cycle count
 */
uint32_t stm32_timestamp_cycles(void);

/**
 * @brief Delay for specified microseconds
 * @param us Microseconds to delay
 * @note Uses DWT cycle counter for precise delay
 */
void stm32_timestamp_delay_us(uint32_t us);

/**
 * @brief Delay for specified milliseconds
 * @param ms Milliseconds to delay
 */
void stm32_timestamp_delay_ms(uint32_t ms);

/**
 * @brief Get system clock frequency
 * @return System clock frequency in Hz
 */
uint32_t stm32_timestamp_get_sys_freq(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32_TIMESTAMP_H__ */
