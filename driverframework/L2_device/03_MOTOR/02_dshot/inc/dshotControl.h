/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-25     gg           V1.0 first version
 * 2024-10-01     gg           V2.0 work normal
 * 2025-09-24     gg           V3.0 Refactored for modular design
 */
#ifndef __DSHOT_CONTROL_H__
#define __DSHOT_CONTROL_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

void motor_stop(void);
rt_err_t motor_reverse(uint8_t *arg);
void dshot_handle_dir_change(uint8_t motor_index);
void writeDshotValueSingleChannle(uint8_t chan_id, rt_uint16_t dc);

#ifdef __cplusplus
}
#endif

#endif  // __DSHOT_CONTROL_H__