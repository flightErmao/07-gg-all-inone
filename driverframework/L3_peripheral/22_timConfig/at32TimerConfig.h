#ifndef __AT32_TIMER_CONFIG_H__
#define __AT32_TIMER_CONFIG_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/hwtimer.h>

#ifdef __cplusplus
extern "C" {
#endif

struct at32_hwtimer;

#ifdef BSP_USING_HWTMR10
#ifndef TMR10_CONFIG
#define TMR10_CONFIG                  \
    {                                 \
       .tmr_x         = TMR10,        \
       .tmr_irqn      = TMR1_OVF_TMR10_IRQn, \
       .name          = "timer10",    \
    }
#endif /* TMR10_CONFIG */
#endif /* BSP_USING_HWTMR10 */

/**
 * @brief Initialize hardware timer device for interrupt handling
 * @param timer_name Timer device name (e.g., "timer10", "timer11")
 * @return RT_EOK on success, error code on failure
 * @note This function must be called before starting the timer to pre-initialize device memory
 */
rt_err_t hwtimerDeviceInit(const char *timer_name);

#ifdef __cplusplus
}
#endif

#endif /* __AT32_TIMER_CONFIG_H__ */