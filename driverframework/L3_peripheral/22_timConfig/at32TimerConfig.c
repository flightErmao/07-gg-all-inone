#include "at32TimerConfig.h"
#include "board.h"
#include "at32f435_437.h"
#include <drivers/hwtimer.h>

#define MAX_TIMER_DEVICES 8

/* Static storage for timer device pointers to avoid calling rt_device_find in interrupt */
static struct {
    const char *name;
    rt_device_t dev;
} timer_devices[MAX_TIMER_DEVICES] = {0};

rt_err_t hwtimerDeviceInit(const char *timer_name) {
    if (timer_name == RT_NULL) {
        rt_kprintf("Timer name is NULL!\n");
        return -RT_ERROR;
    }
    
    /* Check if device is already initialized */
    for (int i = 0; i < MAX_TIMER_DEVICES; i++) {
        if (timer_devices[i].name != RT_NULL && 
            rt_strcmp(timer_devices[i].name, timer_name) == 0) {
            return RT_EOK; /* Already initialized */
        }
    }
    
    /* Find empty slot */
    for (int i = 0; i < MAX_TIMER_DEVICES; i++) {
        if (timer_devices[i].name == RT_NULL) {
            timer_devices[i].dev = rt_device_find(timer_name);
            if (timer_devices[i].dev == RT_NULL) {
                rt_kprintf("Timer device init failed! Can't find %s device!\n", timer_name);
                return -RT_ERROR;
            }
            timer_devices[i].name = timer_name;
            rt_kprintf("Timer device %s initialized successfully!\n", timer_name);
            return RT_EOK;
        }
    }
    
    rt_kprintf("Timer device storage full! Cannot initialize %s\n", timer_name);
    return -RT_ERROR;
}

#ifdef BSP_USING_HWTMR10
static rt_device_t timer10_dev = RT_NULL;

void TMR1_OVF_TMR10_IRQHandler(void) {
  /* enter interrupt */
  rt_interrupt_enter();

  if (tmr_flag_get(TMR10, TMR_OVF_FLAG) == SET) {
    /* Get timer10 device from static storage */
    if (timer10_dev == RT_NULL) {
        for (int i = 0; i < MAX_TIMER_DEVICES; i++) {
            if (timer_devices[i].name != RT_NULL && 
                rt_strcmp(timer_devices[i].name, "timer10") == 0) {
                timer10_dev = timer_devices[i].dev;
                break;
            }
        }
    }
    
    if (timer10_dev != RT_NULL) {
        rt_device_hwtimer_isr((rt_hwtimer_t *)timer10_dev);
    }
    tmr_flag_clear(TMR10, TMR_OVF_FLAG);
  }
  /* leave interrupt */
  rt_interrupt_leave();
}

#endif /* BSP_USING_HWTMR10 */

