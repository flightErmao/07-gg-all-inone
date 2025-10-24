#include "stm32TimeStamp.h"
#include "board.h"
#include "stm32f4xx.h"
#include <rtthread.h>

/* Auto-initialization flag */
static rt_bool_t timestamp_initialized = RT_FALSE;

/* Global variables for timestamp system */
volatile uint32_t systick_count = 0;
static volatile uint32_t systick_val = 0;
static volatile uint32_t systick_pending = 0;
static uint32_t sys_freq = 0;

/* Calculate ticks per microsecond */
#define TICKS_PER_US (sys_freq / 1000000UL)

/**
 * @brief Initialize DWT for cycle counting
 */
static void stm32_dwt_init(void) {
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
 * @brief Auto-initialization check
 */
static void stm32_timestamp_auto_init(void) {
    if (!timestamp_initialized) {
        stm32_timestamp_init();
        timestamp_initialized = RT_TRUE;
    }
}

/**
 * @brief Initialize timestamp system
 */
void stm32_timestamp_init(void) {
    /* Get system clock frequency */
    sys_freq = SystemCoreClock;
    
    /* Configure SysTick for 1ms interrupts */
    SysTick_Config(sys_freq / 1000);
    
    /* Initialize DWT for cycle counting */
    stm32_dwt_init();
    
    rt_kprintf("STM32 Timestamp initialized, sys_freq: %d Hz\n", sys_freq);
}

/**
 * @brief SysTick interrupt handler
 */
void SysTick_Handler(void) {
    systick_count++;
    systick_val = SysTick->VAL;
    systick_pending = 0;
    (void)(SysTick->CTRL);
}

/**
 * @brief Get microseconds from interrupt context
 * @return Current time in microseconds
 */
static uint32_t stm32_timestamp_micros_isr(void) {
    register uint32_t ticks = SysTick->VAL;
    
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
        systick_pending = 1;
        ticks = SysTick->VAL;
    }
    
    register uint32_t count = systick_count;
    register uint32_t pending = systick_pending;
    
    return ((count + pending) * 1000) + (TICKS_PER_US * 1000 - ticks) / TICKS_PER_US;
}

/**
 * @brief Get current time in microseconds
 * @return Current time in microseconds
 */
uint32_t stm32_timestamp_micros(void) {
    /* Auto-initialization check */
    stm32_timestamp_auto_init();
    
    /* Check if we're in interrupt context */
    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || (__get_BASEPRI())) {
        return stm32_timestamp_micros_isr();
    }
    
    register uint32_t count = 0;
    register uint32_t ticks = 0;
    
    do {
        /* Number of 1ms systicks */
        count = systick_count;
        
        /* Number of ticks between 1ms systicks */
        ticks = SysTick->VAL;
    } while (count != systick_count || ticks > systick_val);
    
    return (count * 1000) + (TICKS_PER_US * 1000 - ticks) / TICKS_PER_US;
}

/**
 * @brief Get current time in milliseconds
 * @return Current time in milliseconds
 */
uint32_t stm32_timestamp_millis(void) {
    /* Auto-initialization check */
    stm32_timestamp_auto_init();
    return systick_count;
}

/**
 * @brief Get current cycle count
 * @return Current DWT cycle count
 */
uint32_t stm32_timestamp_cycles(void) {
    /* Auto-initialization check */
    stm32_timestamp_auto_init();
    return DWT->CYCCNT;
}

/**
 * @brief Delay for specified microseconds
 * @param us Microseconds to delay
 */
void stm32_timestamp_delay_us(uint32_t us) {
    /* Auto-initialization check */
    stm32_timestamp_auto_init();
    
    volatile uint32_t delay = us * (sys_freq / 1000000UL);
    volatile uint32_t start = DWT->CYCCNT;
    while (DWT->CYCCNT - start < delay) {
        __asm("NOP");
    }
}

/**
 * @brief Delay for specified milliseconds
 * @param ms Milliseconds to delay
 */
void stm32_timestamp_delay_ms(uint32_t ms) {
    /* Auto-initialization check */
    stm32_timestamp_auto_init();
    
    while (ms--) {
        stm32_timestamp_delay_us(1000);
    }
}

/**
 * @brief Get system clock frequency
 * @return System clock frequency in Hz
 */
uint32_t stm32_timestamp_get_sys_freq(void) {
    /* Auto-initialization check */
    stm32_timestamp_auto_init();
    return sys_freq;
}
