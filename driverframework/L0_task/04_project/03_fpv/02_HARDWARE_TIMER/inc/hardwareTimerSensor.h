#ifndef __HARDWARE_TIMER_SENSOR_H__
#define __HARDWARE_TIMER_SENSOR_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Hardware timer event for sensor triggering */
#define SENSOR_HWTIMER_EVENT_FLAG (1u << 0)

/**
 * @brief Receive hardware timer event
 * @param timeout timeout in ticks (RT_WAITING_FOREVER for blocking)
 * @return RT_EOK on success, error code on failure
 */
rt_err_t hardwareTimerSensorRecvEvent(rt_int32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* __HARDWARE_TIMER_SENSOR_H__ */

