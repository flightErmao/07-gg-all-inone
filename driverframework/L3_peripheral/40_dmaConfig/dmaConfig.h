/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-09     shelton      first version (AT32)
 * 2025-01-XX     gg           Add STM32 support
 */

#ifndef __DMA_CONFIG_H__
#define __DMA_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Platform-specific DMA configuration */
#if defined(SOC_FAMILY_STM32) || defined(SOC_SERIES_STM32H7)
    /* STM32 platform */
    #include "stm32h7xx_hal.h"
    
    /* STM32H7 DShot DMA Configuration for TIM4 */
    /* TIM4 Overflow Event (Update Event) -> DMA1 Stream X */
    /* Update Event 在 STM32 中就是定时器溢出时产生的事件（计数器达到 ARR 值并重载时） */
    
    /* DMA Stream (根据实际硬件连接选择，例如 DMA1 Stream 0) */
    #define DSHOT_DMA_STREAM              DMA1_Stream0
    
    /* DMA Clock (STM32H7 使用 HAL 库的 __HAL_RCC_DMA1_CLK_ENABLE，这里定义为 0) */
    #define DSHOT_DMA_CLOCK               0
    
    /* DMA IRQ */
    #define DSHOT_DMA_IRQ                 DMA1_Stream0_IRQn
    
    /* DMA Request (TIM4 Update Event = 定时器溢出事件) */
    /* 当定时器计数器达到 Period 值并溢出时，产生 Update Event，触发 DMA 请求 */
    #define DSHOT_DMA_REQUEST             DMA_REQUEST_TIM4_UP
    
    /* DMA Interrupt Enable Flag (用于使能中断) - STM32H7 使用 DMA_IT_TC */
    /* DMA_IT_TC = Transfer Complete interrupt enable (传输完成中断使能) */
    #define DSHOT_DMA_INT_FLAG             DMA_IT_TC
    
    /* DMA Interrupt Handler - STM32H7 使用 HAL 库的回调机制 */
    /* HAL 库会自动调用 HAL_DMA_IRQHandler，然后调用注册的回调函数 dshot_dma_xfer_cplt_callback */
    /* 中断服务函数在 dshotHwOpt.c 中定义，仅调用 HAL_DMA_IRQHandler */
    #define DSHOT_DMA_IRQHandler          DMA1_Stream0_IRQHandler
    
#elif defined(SOC_FAMILY_AT32)
    /* AT32 platform */
    /* TMR4 Overflow Event -> DMA2 Channel 7 */
    /* 当定时器计数器达到 Period 值并溢出时，产生 Overflow Event，触发 DMA 请求 */
    #define DSHOT_DMA_IRQHandler         DMA2_Channel7_IRQHandler
    #define DSHOT_DMA_CLOCK              CRM_DMA2_PERIPH_CLOCK
    #define DSHOT_DMA_CHANNEL            DMA2_CHANNEL7
    #define DSHOT_DMA_IRQ                DMA2_Channel7_IRQn
    #define DSHOT_DMA_MUX_CHANNEL        DMA2MUX_CHANNEL7
    #define DSHOT_DMA_REQ_ID             DMAMUX_DMAREQ_ID_TMR4_OVERFLOW
    /* DMA Interrupt Status Flag (用于检查中断状态) - AT32 使用 DMA2_FDT7_FLAG */
    /* DMA2_FDT7_FLAG = Full Data Transfer flag (全数据传输完成标志) */
    /* FDT = Full Data Transfer，表示 DMA2 Channel 7 传输完成 */
    #define DSHOT_DMA_INT_FLAG           DMA2_FDT7_FLAG
#else
    #error "Unsupported platform for DMA configuration"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DMA_CONFIG_H__ */

