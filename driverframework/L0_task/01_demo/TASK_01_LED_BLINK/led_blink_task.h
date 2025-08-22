#ifndef LED_BLINK_TASK_H
#define LED_BLINK_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化LED闪烁任务
 * @return 0: 成功, -1: 失败
 */
int led_blink_task_init(void);

/**
 * @brief 停止LED闪烁任务
 */
void led_blink_task_stop(void);

/**
 * @brief 获取任务运行状态
 * @return 1: 运行中, 0: 已停止
 */
int led_blink_task_is_running(void);

#ifdef __cplusplus
}
#endif

#endif  // LED_BLINK_TASK_H