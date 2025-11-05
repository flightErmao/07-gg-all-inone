/****************************************************************************
 *
 * Rate Control Task
 * This task periodically calls MulticopterRateControl::Run()
 *
 ****************************************************************************/

#include "rtthread.h"
#include <rtdevice.h>
#include <stdbool.h>
#include <stdint.h>
#include "taskRateControl.h"
#include "multicopter_rate_control_wrapper.h"
#include "rtconfig.h"

#ifdef PROJECT_PX4_TASK_RATE_CONTROL_DEBUGPIN_EN
#include "debugPin.h"
#endif

static void taskRateControlInit(void) {
    // Initialize MulticopterRateControl instance
    // Set vtol = 0 for regular multicopter mode
    int ret = multicopter_rate_control_init(0);
    if (ret != 0) {
        rt_kprintf("Failed to initialize MulticopterRateControl\n");
        return;
    }
    rt_kprintf("MulticopterRateControl initialized successfully\n");
}

static void rateControlThreadEntry(void* parameter) {
    uint32_t tick = 0;
    
    taskRateControlInit();
    
    // Wait a bit for initialization to complete
    rt_thread_mdelay(100);
    
    while (1) {
#ifdef PROJECT_PX4_TASK_RATE_CONTROL_DEBUGPIN_EN
        DEBUG_PIN_DEBUG1_HIGH();
#endif
        
        // Call the rate control run function
        multicopter_rate_control_run();
        
#ifdef PROJECT_PX4_TASK_RATE_CONTROL_DEBUGPIN_EN
        DEBUG_PIN_DEBUG1_LOW();
#endif
        
        // Sleep for the task period
        // Default period is 4ms (250Hz), configurable via Kconfig
        rt_thread_mdelay(PROJECT_PX4_TASK_RATE_CONTROL_PERIOD_MS);
        
        tick++;
    }
}

static int taskRateControlThreadAutoStart(void) {
#define THREAD_PRIORITY PROJECT_PX4_TASK_RATE_CONTROL_PRIORITY
#define THREAD_STACK_SIZE PROJECT_PX4_TASK_RATE_CONTROL_STACK_SIZE
#define THREAD_TIMESLICE 5

    static struct rt_thread task_tid_rate_control;
    static rt_uint8_t task_stack_rate_control[THREAD_STACK_SIZE];

    rt_thread_init(&task_tid_rate_control, "rate_control", rateControlThreadEntry, RT_NULL, 
                   task_stack_rate_control, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    rt_thread_startup(&task_tid_rate_control);
    
    rt_kprintf("Rate Control Task started\n");
    return RT_EOK;
}

#ifdef PROJECT_PX4_TASK_RATE_CONTROL_EN
INIT_APP_EXPORT(taskRateControlThreadAutoStart);
#endif

