#ifndef __STM32_TIMER_CONFIG_H__
#define __STM32_TIMER_CONFIG_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/hwtimer.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize hardware timer device for interrupt handling
 * @param timer_name Timer device name (e.g., "timer1", "timer2")
 * @return RT_EOK on success, error code on failure
 * @note This function must be called before starting the timer to pre-initialize device memory
 */
rt_err_t hwtimerDeviceInit(const char *timer_name);

#ifdef __cplusplus
}
#endif

#endif /* __STM32_TIMER_CONFIG_H__ */

