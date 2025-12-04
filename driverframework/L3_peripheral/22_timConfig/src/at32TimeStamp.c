#include "at32TimeStamp.h"
#include "board.h"
#include <rtthread.h>

/* Static variables for timestamp system */
static uint32_t ticks_per_us = 0;

/**
 * @brief Initialize DWT for cycle counting
 */
static void at32_dwt_init(void) {
    /* Enable DWT counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* Enable trace and debug block */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* Reset cycle counter */
    DWT->CYCCNT = 0;
    /* Enable cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Initialize timestamp system
 */
static int at32_timestamp_init(void)
{
    /* Calculate ticks per microsecond */
    ticks_per_us = system_core_clock / 1000000;
    
    /* Initialize DWT for cycle counting */
    at32_dwt_init();
    
    rt_kprintf("AT32 Timestamp initialized, ticks_per_us: %d\n", ticks_per_us);
    
    return 0;
}

/**
 * @brief Get current time in microseconds
 * @return Current time in microseconds
 * @note This function handles SysTick overflow to ensure monotonic timestamps
 * 
 * The issue: Reading ms_timestamp and SysTick->VAL separately can cause race condition.
 * If SysTick overflows between the two reads, we get incorrect timestamp.
 * 
 * Solution: Use a loop to ensure atomic reading. Read ms_timestamp, then SysTick->VAL,
 * then ms_timestamp again. If ms_timestamp changed, it means SysTick overflowed, so retry.
 */
uint32_t at32_timestamp_micros(void)
{
    uint32_t ms_timestamp1, ms_timestamp2;
    uint32_t systick_val;
    uint32_t us_timestamp;
    uint32_t systick_load;
    
    // SysTick->LOAD is the reload value, actual period is LOAD+1
    systick_load = SysTick->LOAD + 1;
    
    // Loop until we get consistent readings (handle SysTick overflow race condition)
    // This ensures atomic reading of both ms_timestamp and SysTick->VAL
    do {
        // Read ms_timestamp first
        ms_timestamp1 = rt_tick_get_millisecond();
        
        // Read SysTick->VAL (this is the critical section)
        systick_val = SysTick->VAL;
        
        // Read ms_timestamp again to check if SysTick overflowed
        ms_timestamp2 = rt_tick_get_millisecond();
        
        // If ms_timestamp changed, SysTick overflowed between the two reads
        // In this case, we need to retry to get consistent readings
        if (ms_timestamp1 != ms_timestamp2) {
            // Overflow occurred, retry
            continue;
        }
        
        // Validate systick_val (should be <= LOAD)
        if (systick_val > SysTick->LOAD) {
            // Invalid value, retry
            continue;
        }
        
        // Calculate microseconds within current millisecond
        // SysTick counts down from LOAD+1 to 0, so elapsed = (LOAD+1) - VAL
        us_timestamp = (systick_load - systick_val) / ticks_per_us;
        
        // Ensure us_timestamp is within [0, 1000) range
        if (us_timestamp >= 1000) {
            // This shouldn't happen, but handle it to be safe
            us_timestamp = 999;
        }
        
        // Success: we have consistent readings
        break;
    } while (1);
    
    return ms_timestamp2 * 1000 + us_timestamp;
}

/**
 * @brief Get current cycle count
 * @return Current DWT cycle count
 */
uint32_t at32_timestamp_cycles(void)
{
    return DWT->CYCCNT;
}

/**
 * @brief Get system clock frequency
 * @return System clock frequency in Hz
 */
uint32_t at32_timestamp_get_sys_freq(void)
{
    return system_core_clock;
}

/* Auto-initialize timestamp system */
INIT_DEVICE_EXPORT(at32_timestamp_init);