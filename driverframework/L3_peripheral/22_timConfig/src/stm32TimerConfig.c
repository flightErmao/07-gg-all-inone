#include "stm32TimerConfig.h"
#include "board.h"
#include <drivers/hwtimer.h>

#define MAX_TIMER_DEVICES 8

/* Static storage for timer device pointers to avoid calling rt_device_find in interrupt */
static struct {
    const char *name;
    rt_device_t dev;
} timer_devices[MAX_TIMER_DEVICES] = {0};

/**
 * @brief Initialize hardware timer device for interrupt handling
 * @param timer_name Timer device name (e.g., "timer1", "timer2")
 * @return RT_EOK on success, error code on failure
 * @note This function must be called before starting the timer to pre-initialize device memory
 */
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

