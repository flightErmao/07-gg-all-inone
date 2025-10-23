/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-17     armink       the first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <board.h>
#include <rtconfig.h>

#define NOR_FLASH_DEV_NAME "at32_flash"

/* ===================== Flash device Configuration ========================= */
extern const struct fal_flash_dev at32f4_onchip_flash;

/* flash device table */
#define FAL_FLASH_DEV_TABLE   \
    {                         \
        &at32f4_onchip_flash, \
    }
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
/* partition table */
#define FAL_PART_TABLE                                                                      \
    {                                                                                       \
        {FAL_PART_MAGIC_WORD, "par", NOR_FLASH_DEV_NAME, 3968 * 1024, 64 * 1024, 0}, \
    }
#endif /* FAL_PART_HAS_TABLE_CFG */

#endif /* _FAL_CFG_H_ */
