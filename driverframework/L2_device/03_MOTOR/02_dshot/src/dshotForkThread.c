/*
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-01-XX     gg           V1.0 first version - DShot fork thread for testing
 */

#include <rtconfig.h>
#include <rtthread.h>
#include "actuator.h"
#include "dshotConfig.h"

#ifdef DSHOT_FORK_THREAD_EN

#define DSHOT_FORK_THREAD_STACK_SIZE 512
#define DSHOT_FORK_THREAD_PRIORITY   25
#define DSHOT_FORK_THREAD_TIMESLICE  5

static rt_thread_t dshot_fork_thread = RT_NULL;
static rt_bool_t dshot_fork_thread_running = RT_FALSE;

extern dshot_config_t dshot_config_;

/* DShot fork thread entry function */
static void dshot_fork_thread_entry(void *parameter)
{
    rt_device_t dev = rt_device_find(DSHOT_DEVICE_NAME);
    if (dev == RT_NULL)
    {
        rt_kprintf("[dshotForkThread] DShot device not found\n");
        return;
    }

    rt_uint16_t values[DSHOT_MOTOR_NUMS];
    
    rt_kprintf("[dshotForkThread] Thread started, maintaining dshot_write every 1ms\n");
    
    while (dshot_fork_thread_running)
    {
        /* Read current motor values from dshot_config_ */
        for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++)
        {
            values[i] = dshot_config_.dshot_output_value[i];
        }
        
        /* Write motor values via device interface */
        /* Cast to actuator_dev_t to access ops directly */
        actuator_dev_t act_dev = (actuator_dev_t)dev;
        if (act_dev != RT_NULL && act_dev->ops != RT_NULL && act_dev->ops->act_write != RT_NULL)
        {
            act_dev->ops->act_write(act_dev, 0, values, DSHOT_MOTOR_NUMS);
        }
        
        /* Sleep for 1ms */
        rt_thread_mdelay(1);
    }
    
    rt_kprintf("[dshotForkThread] Thread stopped\n");
}

/* Initialize DShot fork thread */
rt_err_t dshot_fork_thread_init(void)
{
    if (dshot_fork_thread != RT_NULL)
    {
        rt_kprintf("[dshotForkThread] Thread already exists\n");
        return RT_EOK;
    }

    dshot_fork_thread_running = RT_TRUE;
    
    dshot_fork_thread = rt_thread_create(
        "dshot_fork",
        dshot_fork_thread_entry,
        RT_NULL,
        DSHOT_FORK_THREAD_STACK_SIZE,
        DSHOT_FORK_THREAD_PRIORITY,
        DSHOT_FORK_THREAD_TIMESLICE);

    if (dshot_fork_thread == RT_NULL)
    {
        rt_kprintf("[dshotForkThread] Failed to create thread\n");
        dshot_fork_thread_running = RT_FALSE;
        return -RT_ERROR;
    }

    rt_thread_startup(dshot_fork_thread);
    return RT_EOK;
}

/* Deinitialize DShot fork thread */
rt_err_t dshot_fork_thread_deinit(void)
{
    if (dshot_fork_thread == RT_NULL)
    {
        return RT_EOK;
    }

    dshot_fork_thread_running = RT_FALSE;
    
    /* Wait for thread to exit */
    rt_thread_mdelay(10);
    
    if (rt_thread_delete(dshot_fork_thread) == RT_EOK)
    {
        dshot_fork_thread = RT_NULL;
        rt_kprintf("[dshotForkThread] Thread deleted\n");
        return RT_EOK;
    }

    return -RT_ERROR;
}

/* Auto initialize fork thread after DShot device is registered */
static int dshot_fork_thread_auto_init(void)
{
    /* Wait a bit for DShot device to be registered */
    rt_thread_mdelay(100);
    
    rt_err_t ret = dshot_fork_thread_init();
    if (ret != RT_EOK)
    {
        rt_kprintf("[dshotForkThread] Auto init failed: %d\n", ret);
        return -1;
    }
    
    return 0;
}

#ifdef L2_DEVICE_03_MOTOR_02_DSHOT_EN
INIT_APP_EXPORT(dshot_fork_thread_auto_init);
#endif

#endif /* DSHOT_FORK_THREAD_EN */

