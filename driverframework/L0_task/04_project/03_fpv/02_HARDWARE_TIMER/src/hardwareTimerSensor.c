#include "hardwareTimerSensor.h"
#include <rtdevice.h>
#include <rtthread.h>
#include "rtconfig.h"

#ifdef SOC_FAMILY_AT32
#include "at32TimerConfig.h"
#endif

#ifdef PROJECT_HARDWARE_TIMER_SENSOR_EN

/* Hardware timer device name - configured via Kconfig */
#define HWTIMER_DEV_NAME PROJECT_HARDWARE_TIMER_SENSOR_DEV_NAME

/* Event group for hardware timer */
static struct rt_event sensor_hwtimer_event;

/* Flag to track if event is initialized */
static volatile rt_bool_t event_initialized = RT_FALSE;

/**
 * @brief Hardware timer timeout callback
 * @param dev timer device
 * @param size parameter size
 * @return RT_EOK
 */
static rt_err_t hwtimer_timeout_cb(rt_device_t dev, rt_size_t size) {
    RT_UNUSED(dev);
    RT_UNUSED(size);
    
    /* Send event to trigger sensor reading */
    rt_event_send(&sensor_hwtimer_event, SENSOR_HWTIMER_EVENT_FLAG);
    
    return RT_EOK;
}

/**
 * @brief Initialize hardware timer for sensor triggering
 * @return RT_EOK on success, error code on failure
 */
int hardwareTimerSensorInit(void) {
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t timeout_val;
    rt_device_t hw_dev = RT_NULL;
    rt_hwtimer_mode_t mode;
    rt_uint32_t freq = 1000000;  // 10MHz timer frequency
    
    /* Initialize event group */
    ret = rt_event_init(&sensor_hwtimer_event, "sns_hwtmr", RT_IPC_FLAG_PRIO);
    if (ret != RT_EOK) {
        rt_kprintf("Init sensor hwtimer event failed!\n");
        event_initialized = RT_FALSE;
        return ret;
    }
    
    /* Mark event as initialized */
    event_initialized = RT_TRUE;

    /* Initialize timer device for interrupt handler */
#if defined(SOC_FAMILY_AT32) && defined(BSP_USING_HWTMR10)
    ret = hwtimerDeviceInit(HWTIMER_DEV_NAME);
    if (ret != RT_EOK) {
      rt_kprintf("Timer device init failed!\n");
      event_initialized = RT_FALSE;
      return ret;
    }
#endif

    /* Find hardware timer device */
    hw_dev = rt_device_find(HWTIMER_DEV_NAME);
    if (hw_dev == RT_NULL) {
        rt_kprintf("Hardware timer init failed! Can't find %s device!\n", HWTIMER_DEV_NAME);
        event_initialized = RT_FALSE;
        return RT_ERROR;
    }
    
    /* Open hardware timer device */
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) {
        rt_kprintf("Open %s device failed!\n", HWTIMER_DEV_NAME);
        return ret;
    }
    
    /* Set timeout callback */
    rt_device_set_rx_indicate(hw_dev, hwtimer_timeout_cb);
    
    /* Set timer frequency */
    ret = rt_device_control(hw_dev, HWTIMER_CTRL_FREQ_SET, &freq);
    if (ret != RT_EOK) {
        rt_kprintf("Set timer frequency failed!\n");
        return ret;
    }
    
    /* Set periodic mode */
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK) {
        rt_kprintf("Set timer mode failed!\n");
        return ret;
    }
    
    /* Calculate timeout value based on configured frequency
     * Period (in seconds) = 1 / frequency
     * Period in microseconds = 1,000,000 / frequency
     * Example: 3200 Hz -> 1,000,000 / 3200 = 312.5 us
     */
    rt_uint32_t period_usec = 1000000 / PROJECT_HARDWARE_TIMER_SENSOR_FREQ;
    
    timeout_val.sec = 0;
    timeout_val.usec = period_usec;
    
    ret = rt_device_write(hw_dev, 0, &timeout_val, sizeof(timeout_val));
    if (ret != sizeof(timeout_val)) {
        rt_kprintf("Set timer timeout value failed!\n");
        return RT_ERROR;
    }
    
    rt_kprintf("Hardware timer sensor initialized: %s, freq=%dHz (period=%d.%03dms)\n", 
               HWTIMER_DEV_NAME, PROJECT_HARDWARE_TIMER_SENSOR_FREQ, 
               period_usec / 1000, period_usec % 1000);
    
    return RT_EOK;
}

/**
 * @brief Receive hardware timer event
 * @param timeout timeout in ticks (RT_WAITING_FOREVER for blocking)
 * @return RT_EOK on success, error code on failure
 */
rt_err_t hardwareTimerSensorRecvEvent(rt_int32_t timeout) {
    /* Protect: if event not initialized, delay 1 second instead of waiting for event */
    if (!event_initialized) {
        rt_kprintf("Warning: Hardware timer event not initialized, delaying 1s\n");
        rt_thread_mdelay(1000);
        return RT_ETIMEOUT;
    }
    
    return rt_event_recv(&sensor_hwtimer_event, 
                         SENSOR_HWTIMER_EVENT_FLAG, 
                         RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, 
                         timeout, 
                         RT_NULL);
}

INIT_APP_EXPORT(hardwareTimerSensorInit);

#else /* PROJECT_HARDWARE_TIMER_SENSOR_EN not defined */

/* Empty implementations when hardware timer is not enabled */
int hardwareTimerSensorInit(void) {
    return RT_EOK;
}

rt_err_t hardwareTimerSensorRecvEvent(rt_int32_t timeout) {
    RT_UNUSED(timeout);
    return RT_EOK;
}

#endif /* PROJECT_HARDWARE_TIMER_SENSOR_EN */

