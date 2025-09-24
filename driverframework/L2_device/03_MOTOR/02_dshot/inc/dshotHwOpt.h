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

void bbSaveDMARegs(dma_channel_type *dmaResource, dmaRegCache_t *dmaRegCache);
rt_err_t setDshotValue(void);

#ifdef __cplusplus
}
#endif

#endif  // __DSHOT_HW_CONFIG_H__