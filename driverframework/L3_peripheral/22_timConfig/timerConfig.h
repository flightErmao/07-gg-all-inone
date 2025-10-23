#ifndef __TIMER_CONFIG_H__
#define __TIMER_CONFIG_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/hwtimer.h>

#ifdef __cplusplus
extern "C" {
#endif

// 前向声明结构体，在 drv_hwtimer.c 中定义
struct at32_hwtimer;

// TMR10 配置定义
#ifdef BSP_USING_HWTMR10
#ifndef TMR10_CONFIG
#define TMR10_CONFIG                  \
    {                                 \
       .tmr_x         = TMR10,        \
       .tmr_irqn      = TMR1_OVF_TMR10_IRQn, \
       .name          = "timer10",    \
    }
#endif /* TMR10_CONFIG */

/**
 * @brief 初始化 TMR10 定时器设备
 * @return RT_EOK 成功，其他值表示失败
 * @note 此函数必须在启动定时器之前调用，用于预初始化设备句柄
 */
rt_err_t timer10_device_init(void);

#endif /* BSP_USING_HWTMR10 */

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_CONFIG_H__ */