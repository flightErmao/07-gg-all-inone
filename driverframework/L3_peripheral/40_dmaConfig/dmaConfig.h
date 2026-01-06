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
    /* TIM4 Update Event -> DMA1 Stream X */
    
    /* DMA Stream (根据实际硬件连接选择，例如 DMA1 Stream 0) */
    #define DSHOT_DMA_STREAM              DMA1_Stream0
    
    /* DMA Clock (STM32H7 使用 HAL 库的 __HAL_RCC_DMA1_CLK_ENABLE，这里定义为 0) */
    #define DSHOT_DMA_CLOCK               0
    
    /* DMA IRQ */
    #define DSHOT_DMA_IRQ                 DMA1_Stream0_IRQn
    
    /* DMA Request (TIM4 Update Event) */
    #define DSHOT_DMA_REQUEST             DMA_REQUEST_TIM4_UP
    
    /* DMA Interrupt Flag (用于中断处理) - STM32H7 使用 DMA_IT_TC */
    #define DSHOT_DMA_INT_FLAG             DMA_IT_TC
    
    /* DMA Interrupt Handler */
    #define DSHOT_DMA_IRQHandler          DMA1_Stream0_IRQHandler
    
#elif defined(SOC_FAMILY_AT32)
    /* AT32 platform */
    #define DSHOT_DMA_IRQHandler         DMA2_Channel7_IRQHandler
    #define DSHOT_DMA_CLOCK              CRM_DMA2_PERIPH_CLOCK
    #define DSHOT_DMA_CHANNEL            DMA2_CHANNEL7
    #define DSHOT_DMA_IRQ                DMA2_Channel7_IRQn
    #define DSHOT_DMA_MUX_CHANNEL        DMA2MUX_CHANNEL7
    #define DSHOT_DMA_REQ_ID             DMAMUX_DMAREQ_ID_TMR4_OVERFLOW
    #define DSHOT_DMA_INT_FLAG           DMA2_FDT7_FLAG
#else
    #error "Unsupported platform for DMA configuration"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DMA_CONFIG_H__ */

