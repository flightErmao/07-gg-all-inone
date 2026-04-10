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

#include "app_init.h"
#include "led_status.h"

int main(void)
{
    app_init_run();
    led_status_set_mode(LED_STATUS_IDLE);

    while (1)
    {
        rt_thread_mdelay(1000);
    }
    return RT_EOK;
}
