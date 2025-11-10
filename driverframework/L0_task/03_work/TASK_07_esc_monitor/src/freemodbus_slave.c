/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-06-21     flybreak     first version
 */

#include <rtthread.h>

#include "mb.h"
#include "user_mb_app.h"
#include "esc_monitor_i2c.h"


#define SLAVE_ADDR      0x01   // 从机地址
#define PORT_NUM        1      // 串口号
#define PORT_BAUDRATE   115200 // 波特率

#define PORT_PARITY     MB_PAR_EVEN

#define MB_POLL_THREAD_PRIORITY  10
#define MB_SEND_THREAD_PRIORITY  11
#define MB_POLL_CYCLE_MS 200

extern USHORT usSRegHoldBuf[S_REG_HOLDING_NREGS];

static void send_thread_entry(void *parameter) {
  USHORT *usRegHoldingBuf;
  usRegHoldingBuf = usSRegHoldBuf;

  while (1) {
    rt_bool_t monitor_result = RT_FALSE;
    if (esc_monitor_get_detection_result(&monitor_result)) {
      rt_base_t level = rt_hw_interrupt_disable();
      if (WORK_TASK_ESC_MONITOR_MODBUS_REG_INDEX < S_REG_HOLDING_NREGS) {
        usRegHoldingBuf[WORK_TASK_ESC_MONITOR_MODBUS_REG_INDEX] = monitor_result ? 1 : 0;
      }
      rt_hw_interrupt_enable(level);
    }
    rt_thread_mdelay(1000);
  }
}

static void mb_slave_poll(void *parameter)
{
    if (rt_strstr(parameter, "RTU"))
    {
#ifdef PKG_MODBUS_SLAVE_RTU
        eMBInit(MB_RTU, SLAVE_ADDR, PORT_NUM, PORT_BAUDRATE, PORT_PARITY);
#else
        rt_kprintf("Error: Please open RTU mode first");
#endif
    }
    else if (rt_strstr(parameter, "ASCII"))
    {
#ifdef PKG_MODBUS_SLAVE_ASCII
        eMBInit(MB_ASCII, SLAVE_ADDR, PORT_NUM, PORT_BAUDRATE, PORT_PARITY);
#else
        rt_kprintf("Error: Please open ASCII mode first");
#endif
    }
    else if (rt_strstr(parameter, "TCP"))
    {
#ifdef PKG_MODBUS_SLAVE_TCP
        eMBTCPInit(0);
#else
        rt_kprintf("Error: Please open TCP mode first");
#endif
    }
    else
    {
        rt_kprintf("Error: unknown parameter");
    }
    eMBEnable();
    while (1)
    {
        eMBPoll();
        rt_thread_mdelay(MB_POLL_CYCLE_MS);
    }
}

int mb_slave_sample(void)
{
    rt_thread_t tid1 = RT_NULL, tid2 = RT_NULL;

    tid1 = rt_thread_create("md_s_poll", mb_slave_poll, "RTU", 1024, MB_POLL_THREAD_PRIORITY, 10);
    if (tid1 != RT_NULL)
    {
        rt_thread_startup(tid1);
    }
    else
    {
        goto __exit;
    }

    tid2 = rt_thread_create("md_s_send", send_thread_entry, RT_NULL, 1024, MB_SEND_THREAD_PRIORITY, 10);
    if (tid2 != RT_NULL)
    {
        rt_thread_startup(tid2);
    }
    else
    {
        goto __exit;
    }

    return RT_EOK;

__exit:
    if (tid1)
        rt_thread_delete(tid1);
    if (tid2)
        rt_thread_delete(tid2);

    return -RT_ERROR;
}

#ifdef WORK_TASK_ESC_MONITOR_EN
INIT_APP_EXPORT(mb_slave_sample);
#endif


