/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <rtthread.h>
#include "hardTimestamp.h"
#include "rtconfig.h"

#ifdef L1_MIDDLEWARE_01_MODULE_06_HARDTIMESTAMP_EN

/* Include CMSIS core header for SysTick register access */
#include "core_cm4.h"

/* Declare extern system clock variables */
extern uint32_t SystemCoreClock;
extern unsigned int system_core_clock;

typedef struct {
    volatile uint32_t ticks_per_us;  /* ticks per microsecond */
} systime_t;

static systime_t __systime;

/* Tick fraction calculation: microseconds per tick */
#define USEC_PER_TICK (1000000 / RT_TICK_PER_SECOND)

/**
 * @brief Check if the period of time has elapsed
 *
 * @param timetag Time tag which stores the period and time information
 * @return uint8_t 1 indicates true
 */
uint8_t check_timetag(TimeTag* timetag)
{
    uint32_t now = systime_now_ms();

    if (timetag->period > 0 && now - timetag->tag >= timetag->period) {
        timetag->tag = now;
        return 1;
    }
    return 0;
}

/**
 * @brief Check if the period of time has elapsed with specified time now
 *
 * @param timetag Time tag which stores the period and time information
 * @param now Time now in ms
 * @return uint8_t uint8_t 1 indicates true
 */
uint8_t check_timetag2(TimeTag* timetag, uint32_t now)
{
    if (timetag->period && now - timetag->tag >= timetag->period) {
        timetag->tag = now;
        return 1;
    }
    return 0;
}

/**
 * @brief Check if the period of time has elapsed with specified time now and period
 *
 * @param timetag Time tag which stores the period and time information
 * @param now Time now in ms
 * @param period Period in ms
 * @return uint8_t uint8_t 1 indicates true
 */
uint8_t check_timetag3(TimeTag* timetag, uint32_t now, uint32_t period)
{
    if (period > 0 && now - timetag->tag >= period) {
        timetag->tag = now;
        return 1;
    }
    return 0;
}

/**
 * @brief Get current systime in us using hardware SysTick
 *
 * @return uint64_t systime in us
 */
uint64_t systime_now_us(void)
{
    rt_tick_t current_tick;
    uint32_t level;
    uint64_t current_us;
    uint32_t systick_val;
    uint32_t systick_load;

    level = rt_hw_interrupt_disable();
    
    /* Read SysTick hardware registers for precise timing */
    current_tick = rt_tick_get();
    systick_val = SysTick->VAL;
    systick_load = SysTick->LOAD;
    
    /* Calculate microseconds from current tick count */
    current_us = (uint64_t)current_tick * USEC_PER_TICK;
    
    /* Add microsecond fraction from current SysTick cycle */
    /* Formula: fraction_us = (load - val) / ticks_per_us */
    if (__systime.ticks_per_us > 0) {
        current_us += (systick_load - systick_val) / __systime.ticks_per_us;
    }
    
    rt_hw_interrupt_enable(level);

    return current_us;
}

/**
 * @brief Get current systime in ms
 *
 * @return uint32_t systime in ms
 */
uint32_t systime_now_ms(void)
{
    uint32_t time_now_ms = systime_now_us() / 1000;

    return time_now_ms;
}

/**
 * @brief Delay for us
 *
 * @param time_us Delay time in us
 */
void systime_udelay(uint32_t time_us)
{
    uint64_t target = systime_now_us() + time_us;

    while (systime_now_us() < target)
        ;
}

/**
 * @brief Delay for ms
 *
 * @param time_ms Delay time in ms
 */
inline void systime_mdelay(uint32_t time_ms)
{
    systime_udelay(time_ms * 1000);
}

/**
 * @brief Sleep for ms
 * @note In thread context it will suspend the thread for specific milliseconds,
 *       otherwise it will just do normal delay.
 *
 * @param time_ms Sleep time in ms
 */
void systime_msleep(uint32_t time_ms)
{
    if (rt_thread_self()) {
        rt_thread_delay(TICKS_FROM_MS(time_ms));
    } else {
        systime_mdelay(time_ms);
    }
}

/**
 * @brief Initialize systime module with hardware SysTick support
 *
 * @return rt_err_t RT_EOK indicates success
 */
rt_err_t systime_init(void)
{
    uint32_t level;
    uint32_t core_clock;
    
    level = rt_hw_interrupt_disable();
    
    /* Get system core clock frequency */
    #ifdef AT32F4XX
        core_clock = system_core_clock;
    #else
        core_clock = SystemCoreClock;
    #endif
    
    /* Calculate ticks per microsecond */
    if (core_clock > 0) {
        __systime.ticks_per_us = core_clock / 1000000;
    } else {
        /* Fallback: assume 168MHz for STM32F4 */
        __systime.ticks_per_us = 168;
    }
    
    rt_hw_interrupt_enable(level);
    
    rt_kprintf("Hardware timestamp initialized successfully\n");
    rt_kprintf("  Core clock: %d Hz\n", core_clock);
    rt_kprintf("  Ticks per us: %d\n", __systime.ticks_per_us);
    rt_kprintf("  Tick rate: %d ticks/sec\n", RT_TICK_PER_SECOND);

    return RT_EOK;
}

/**
 * @brief Tick update function (empty, not needed anymore)
 * @note Kept for compatibility, but no longer needed
 */
void systime_tick_update(void)
{
    /* Do nothing - calculation is done directly in systime_now_us */
}

/* Auto-initialize hardware timestamp module */
INIT_DEVICE_EXPORT(systime_init);

#else /* L1_MIDDLEWARE_01_MODULE_06_HARDTIMESTAMP_EN not defined */

/* Empty implementations when hardware timestamp is not enabled */
rt_err_t systime_init(void) {
    return RT_EOK;
}

uint64_t systime_now_us(void) {
    return rt_tick_get() * 1000000 / RT_TICK_PER_SECOND;
}

uint32_t systime_now_ms(void) {
    return rt_tick_get() * 1000 / RT_TICK_PER_SECOND;
}

void systime_udelay(uint32_t delay) {
    RT_UNUSED(delay);
    /* Do nothing when disabled */
}

void systime_mdelay(uint32_t time_ms) {
    RT_UNUSED(time_ms);
    /* Do nothing when disabled */
}

void systime_msleep(uint32_t time_ms) {
    RT_UNUSED(time_ms);
    /* Do nothing when disabled */
}

void systime_tick_update(void) {
    /* Do nothing when disabled */
}

uint8_t check_timetag(TimeTag* timetag) {
    RT_UNUSED(timetag);
    return 0;
}

uint8_t check_timetag2(TimeTag* timetag, uint32_t now) {
    RT_UNUSED(timetag);
    RT_UNUSED(now);
    return 0;
}

uint8_t check_timetag3(TimeTag* timetag, uint32_t now, uint32_t period) {
    RT_UNUSED(timetag);
    RT_UNUSED(now);
    RT_UNUSED(period);
    return 0;
}

#endif /* L1_MIDDLEWARE_01_MODULE_06_HARDTIMESTAMP_EN */
