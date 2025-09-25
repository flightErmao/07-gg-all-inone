/*
 * @Author: Tiger
 * @Date: 2024-11-12 14:12:39
 * @LastEditors: Tiger
 * @LastEditTime: 2024-11-13 15:36:51
 * @FilePath: \at32f403a-start\board\ports\fal_flash_at32f4_port.c
 * @Description:
 */
/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-01-26     armink       the first version
 */

#include <fal.h>

#include "at32f435_437.h"
#include "drv_flash.h"

#define SECTOR_SIZE 4096 /* this parameter depends on the specific model of the chip */

uint8_t g_flash_buffer[SECTOR_SIZE];

static error_status write_flash_data_no_check(uint32_t write_address, const uint8_t *p_buffer, uint16_t size) {
    flash_status_type status = FLASH_OPERATE_DONE;
    for (int i = 0; i < size; i++) {
        status = flash_byte_program(write_address, p_buffer[i]);
        if (status != FLASH_OPERATE_DONE) return ERROR;
        write_address++;
    }
    return SUCCESS;
}

error_status write_flash_data(uint32_t write_address, const uint8_t *p_buffer, size_t size) {
    uint32_t offset_addr;
    uint32_t sector_position;
    uint16_t sector_offset;
    uint16_t sector_remain;
    int i;
    flash_status_type status = FLASH_OPERATE_DONE;

    flash_unlock();

    offset_addr = write_address - FLASH_BASE;
    sector_position = offset_addr / SECTOR_SIZE;
    sector_offset = offset_addr % SECTOR_SIZE;
    sector_remain = SECTOR_SIZE - sector_offset;

    if (size <= sector_remain) sector_remain = size;

    while (1) {
        at32_flash_read(sector_position * SECTOR_SIZE + FLASH_BASE, g_flash_buffer, SECTOR_SIZE);
        for (i = 0; i < sector_remain; i++) {
            if (g_flash_buffer[sector_offset + i] != 0xFF) break;
        }

        if (i < sector_remain) {
            /* wait for operation to be completed */
            status = flash_operation_wait_for(ERASE_TIMEOUT);

            if ((status == FLASH_PROGRAM_ERROR) || (status == FLASH_EPP_ERROR))
                flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
            else if (status == FLASH_OPERATE_TIMEOUT)
                return ERROR;

            status = flash_sector_erase(sector_position * SECTOR_SIZE + FLASH_BASE);

            if (status != FLASH_OPERATE_DONE) return ERROR;

            for (i = 0; i < sector_remain; i++) {
                g_flash_buffer[i + sector_offset] = p_buffer[i];
            }

            if (write_flash_data_no_check(sector_position * SECTOR_SIZE + FLASH_BASE, g_flash_buffer, SECTOR_SIZE) !=
                SUCCESS)
                return ERROR;
        } else {
            if (write_flash_data_no_check(write_address, p_buffer, sector_remain) != SUCCESS) return ERROR;
        }

        if (size == sector_remain)
            break;
        else {
            sector_position++;
            sector_offset = 0;
            p_buffer += sector_remain;
            write_address += sector_remain;
            size -= sector_remain;
            if (size > (SECTOR_SIZE))
                sector_remain = SECTOR_SIZE;
            else
                sector_remain = size;
        }
    }
    flash_lock();
    return SUCCESS;
}

static int init(void) {
    /* do nothing now */
    return 0;
}

static int read(long offset, uint8_t *buf, size_t size) {
    // size_t i;
    uint32_t addr = at32f4_onchip_flash.addr + offset;
    // for (i = 0; i < size; i++, addr++, buf++)
    // {
    //     *buf = *(uint8_t *) addr;
    // }
    at32_flash_read(addr, buf, size);
    return size;
}

static int write(long offset, const uint8_t *buf, size_t size) {
    // size_t i;
    // uint32_t read_data;
    uint32_t addr = at32f4_onchip_flash.addr + offset;
    error_status ret;

    // flash_unlock();
    // FLASH_ClearFlag(
    //         FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR
    //                 | FLASH_FLAG_PGSERR);
    // for (i = 0; i < size; i++, buf++, addr++)
    // {
    //     /* write data */
    //     FLASH_ProgramByte(addr, *buf);
    //     read_data = *(uint8_t *) addr;
    //     /* check data */
    //     if (read_data != *buf)
    //     {
    //         return -1;
    //     }
    // }
    // flash_lock();
    ret = write_flash_data(addr, buf, size);
    if (ret == SUCCESS)
        return size;
    else
        return -1;
    // size = at32_flash_write(addr, buf, size);
}

static int erase(long offset, size_t size) {
    // FLASH_Status flash_status;
    // size_t erased_size = 0;
    // uint32_t cur_erase_sector;
    uint32_t addr = at32f4_onchip_flash.addr + offset;

    // /* start erase */
    // FLASH_Unlock();
    // FLASH_ClearFlag(
    //         FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR
    //                 | FLASH_FLAG_PGSERR);
    // /* it will stop when erased size is greater than setting size */
    // while (erased_size < size)
    // {
    //     cur_erase_sector = at32_get_sector(addr + erased_size);
    //     flash_status = FLASH_EraseSector(cur_erase_sector, VoltageRange_3);
    //     if (flash_status != FLASH_COMPLETE)
    //     {
    //         return -1;
    //     }
    //     erased_size += at32_get_sector_size(cur_erase_sector);
    // }
    // FLASH_Lock();
    at32_flash_erase(addr, size);
    return size;
}

const struct fal_flash_dev at32f4_onchip_flash = {.name = NOR_FLASH_DEV_NAME,
                                                  .addr = 0x08000000,
                                                  .len = 1024 * 1024 * 4,  // 4MB flash size
                                                  .blk_size = 4 * 1024,
                                                  .ops = {init, read, write, erase},
                                                  .write_gran = 8};

#ifdef L3_PERIPHERAL_USING_ON_CHIP_FLASH_AUTOSTART_FALINIT
INIT_DEVICE_EXPORT(fal_init);
#endif
