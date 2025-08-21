/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 * 2023-12-03     Meco Man     support nano version
 */

#include <board.h>
#include <rtthread.h>
#include "drv_gpio.h"


#define LED0_PIN    GET_PIN(A, 0)

int main(void) {

#ifndef BSP_USING_TASK_1_LED_BLINK
  rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
  rt_pin_write(LED0_PIN, PIN_LOW);
#endif

  // 获取系统时钟主频
  extern uint32_t SystemCoreClock;
  
  // 每秒打印一次系统时钟频率
  rt_uint32_t last_tick = rt_tick_get();
  rt_uint32_t current_tick;
  
  rt_kprintf("=== STM32F411CE 时钟配置信息 ===\n");
  rt_kprintf("外部晶振频率: %d Hz (%.1f MHz)\n", HSE_VALUE, HSE_VALUE/1000000.0);
  rt_kprintf("系统时钟频率: %d Hz (%.1f MHz)\n", SystemCoreClock, SystemCoreClock/1000000.0);
  rt_kprintf("RT_TICK_PER_SECOND: %d\n", RT_TICK_PER_SECOND);
  rt_kprintf("预期系统时钟: 100 MHz\n");
  rt_kprintf("==============================\n");

  while (1) {
#ifndef BSP_USING_TASK_1_LED_BLINK
    rt_pin_write(LED0_PIN, PIN_HIGH);
    rt_thread_mdelay(500);  // 延时500ms
    
    rt_pin_write(LED0_PIN, PIN_LOW);
    rt_thread_mdelay(500);  // 延时500ms
#else
    rt_thread_mdelay(1000);  // 延时500ms
#endif
  }
}
