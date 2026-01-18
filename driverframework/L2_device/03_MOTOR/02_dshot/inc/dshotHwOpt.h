/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-25     gg           V1.0 first version
 * 2024-10-01     gg           V2.0 work normal
 * 2025-09-24     gg           V3.0 Refactored for modular design
 */
#ifndef __DSHOT_HW_CONFIG_H__
#define __DSHOT_HW_CONFIG_H__

#include <rtthread.h>
#include "dshotConfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef DSHOT_PLATFORM_STM32
void bbSaveDMARegs(void *dmaResource, dmaRegCache_t *dmaRegCache);
#elif defined(DSHOT_PLATFORM_AT32)
void bbSaveDMARegs(dma_channel_type *dmaResource, dmaRegCache_t *dmaRegCache);
#endif
rt_err_t setDshotValue(void);

/* Timer interrupt handler for debugging */
#ifdef DSHOT_DEBUGPIN_EN
void DSHOT_TIMER_IRQHandler(void);
#endif

#ifdef __cplusplus
}
#endif

#endif  // __DSHOT_HW_CONFIG_H__