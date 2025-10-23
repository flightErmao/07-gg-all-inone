/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-09     shelton      first version
 */

#ifndef __DMA_CONFIG_H__
#define __DMA_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DSHOT_DMA_IRQHandler         DMA2_Channel7_IRQHandler
#define DSHOT_DMA_CLOCK              CRM_DMA2_PERIPH_CLOCK
#define DSHOT_DMA_CHANNEL            DMA2_CHANNEL7
#define DSHOT_DMA_IRQ                DMA2_Channel7_IRQn
#define DSHOT_DMA_MUX_CHANNEL        DMA2MUX_CHANNEL7
#define DSHOT_DMA_REQ_ID             DMAMUX_DMAREQ_ID_TMR4_OVERFLOW

#ifdef __cplusplus
}
#endif

#endif /* __DMA_CONFIG_H__ */
