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
 */
uint32_t at32_timestamp_micros(void)
{
    uint32_t ms_timestamp = rt_tick_get_millisecond();
    uint32_t us_timestamp = (SysTick->LOAD - SysTick->VAL) / ticks_per_us;
    
    return ms_timestamp * 1000 + us_timestamp;
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