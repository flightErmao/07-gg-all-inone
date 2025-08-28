/*
 * Copyright (c) 2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-19     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifdef L3_PERIPHERAL_02_NVIC_CONFIG_EN

static int stm32_nvic_priority_group_config(void) {
  uint32_t priority_group = NVIC_PRIORITYGROUP_4; /* Default to group 4 */

  /* Get configured priority group from Kconfig */
  switch (L3_PERIPHERAL_02_NVIC_STM32_PRIORITY_GROUP) {
    case 0:
      priority_group = NVIC_PRIORITYGROUP_0;
      break;
    case 1:
      priority_group = NVIC_PRIORITYGROUP_1;
      break;
    case 2:
      priority_group = NVIC_PRIORITYGROUP_2;
      break;
    case 3:
      priority_group = NVIC_PRIORITYGROUP_3;
      break;
    case 4:
    default:
      priority_group = NVIC_PRIORITYGROUP_4;
      break;
  }

  /* Set the NVIC Priority Group */
  HAL_NVIC_SetPriorityGrouping(priority_group);

  rt_kprintf("[NVIC] STM32 Priority Group set to %d\n", L3_PERIPHERAL_02_NVIC_STM32_PRIORITY_GROUP);
  return 0;
}

/* Initialize STM32 NVIC configuration using INIT_BOARD_EXPORT */
INIT_BOARD_EXPORT(stm32_nvic_priority_group_config);

#endif /* L3_PERIPHERAL_02_NVIC_CONFIG_EN */