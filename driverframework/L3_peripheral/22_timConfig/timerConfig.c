#include "timerConfig.h"
#include "board.h"
#include "at32f435_437.h"
#include <drivers/hwtimer.h>

#ifdef BSP_USING_HWTMR10
// 静态存储定时器设备指针，避免在中断中调用 rt_device_find
static rt_device_t timer10_dev = RT_NULL;

/**
 * @brief 初始化 TMR10 定时器设备
 * @return RT_EOK 成功，其他值表示失败
 * @note 此函数必须在启动定时器之前调用，用于预初始化设备句柄
 */
rt_err_t timer10_device_init(void) {
    if (timer10_dev == RT_NULL) {
        timer10_dev = rt_device_find("timer10");
        if (timer10_dev == RT_NULL) {
            rt_kprintf("Timer10 device init failed! Can't find timer10 device!\n");
            return -RT_ERROR;
        }
        rt_kprintf("Timer10 device initialized successfully!\n");
    }
    return RT_EOK;
}

void TMR1_OVF_TMR10_IRQHandler(void) {
  /* enter interrupt */
  rt_interrupt_enter();

  if (tmr_flag_get(TMR10, TMR_OVF_FLAG) == SET) {
    // 使用预初始化的设备指针，避免在中断中查找设备
    if (timer10_dev != RT_NULL) {
        rt_device_hwtimer_isr(timer10_dev);
    }
    tmr_flag_clear(TMR10, TMR_OVF_FLAG);
  }
  /* leave interrupt */
  rt_interrupt_leave();
}

#endif /* BSP_USING_HWTMR10 */

