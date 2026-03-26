/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-10-25     zylx         first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifdef RT_USING_CHERRYUSB
#include "stm32h7xx_hal.h"

extern void cdc_acm_chardev_init(uint8_t busid, uintptr_t reg_base);

static void usb_fs_force_reenumerate(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    /* Keep D+ low long enough for the host to detect a disconnect after reflashing. */
    rt_thread_mdelay(80);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
    rt_thread_mdelay(20);
}

static int usb_init(void)
{
    usb_fs_force_reenumerate();
    cdc_acm_chardev_init(0, USB_OTG_FS_PERIPH_BASE);
    rt_kprintf("USB CDC init done, waiting for host enumeration...\n");
    return 0;
}
INIT_APP_EXPORT(usb_init);
#endif

/* defined the LED0 pin: PE3 */
#define LED0_PIN    GET_PIN(E, 3)

int main(void)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
    return RT_EOK;
}
