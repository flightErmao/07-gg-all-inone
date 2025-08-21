#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <board.h>
#include <rtthread.h>
#include <drv_gpio.h>
#include <ctype.h>
#include <rtconfig.h>

// 包含LED闪烁接口头文件
#include "led_blink_interface.h"

// 任务相关定义
#define LED_BLINK_TASK_NAME "led_blink_task"

// LED引脚定义
// 通过 Kconfig 字符串配置解析 LED 引脚，形如 "PC13"
// 若解析失败，将回退到 PC13
static rt_base_t parse_led_pin_from_config(const char* pin_name) {
  if (pin_name == RT_NULL) {
    return (rt_base_t)(2 * 16 + 13);  // PC13
  }

  const char* s = pin_name;
  // 允许可选的前缀 'P'
  if (s[0] == 'P' || s[0] == 'p') {
    s++;
  }
  // 读取端口字母
  if (s[0] == '\0' || !isalpha((unsigned char)s[0])) {
    return (rt_base_t)(2 * 16 + 13);
  }
  char port_char = (char)toupper((unsigned char)s[0]);
  s++;

  // 读取数字部分
  int pin_num = 0;
  int has_digit = 0;
  while (*s) {
    if (!isdigit((unsigned char)*s)) break;
    has_digit = 1;
    pin_num = pin_num * 10 + (*s - '0');
    s++;
  }
  if (!has_digit) {
    return (rt_base_t)(2 * 16 + 13);
  }

  // 映射端口字母到索引（A->0, B->1, C->2, ...）
  if (port_char < 'A' || port_char > 'Z') {
    return (rt_base_t)(2 * 16 + 13);
  }
  int port_index = port_char - 'A';

  // STM32 驱动版编码：port*16 + pin
  return (rt_base_t)(port_index * 16 + pin_num);
}

// 事件定义
#define LED_BLINK_EVENT_STEP (1 << 0)  // LED闪烁步进事件
#define LED_BLINK_EVENT_STOP (1 << 1)  // LED闪烁停止事件

// 全局变量
static rt_thread_t led_blink_thread = RT_NULL;
static rt_timer_t led_blink_timer = RT_NULL;
static rt_event_t led_blink_event = RT_NULL;
static int led_blink_running = 0;
static rt_base_t led_pin = RT_NULL;  // LED引脚句柄

// 初始化函数声明
static rt_err_t led_blink_task_init_resources(void);

// 定时器回调函数
static void led_blink_timer_callback(void* parameter) {
  if (led_blink_running && led_blink_event != RT_NULL) {
    // 发送LED闪烁步进事件
    rt_event_send(led_blink_event, LED_BLINK_EVENT_STEP);
  }
}

// LED任务资源初始化函数
static rt_err_t led_blink_task_init_resources(void) {
  rt_err_t result = RT_EOK;

  printf("[%s] 开始初始化LED任务资源\n", LED_BLINK_TASK_NAME);

  // 初始化LED引脚（从 Kconfig 字符串解析）
#ifdef TASK_1_LED_BLINK_PIN
  led_pin = parse_led_pin_from_config(TASK_1_LED_BLINK_PIN);
#else
  led_pin = parse_led_pin_from_config("PC13");
#endif
  rt_pin_mode(led_pin, PIN_MODE_OUTPUT);
  rt_pin_write(led_pin, PIN_LOW);  // 初始状态设为低电平
  printf("[%s] LED引脚初始化完成，引脚: %d\n", LED_BLINK_TASK_NAME, led_pin);

  // 初始化LED闪烁接口
  led_blink_interface_init();
  
  // 设置LED闪烁周期（使用Kconfig配置的定时器间隔）
#ifdef TASK_1_LED_BLINK_TIMER_INTERVAL
  led_blink_set_period_ms(TASK_1_LED_BLINK_TIMER_INTERVAL);
  printf("[%s] LED闪烁周期设置为: %d ms\n", LED_BLINK_TASK_NAME, TASK_1_LED_BLINK_TIMER_INTERVAL);
#else
  led_blink_set_period_ms(500);  // 默认500ms周期
  printf("[%s] LED闪烁周期设置为默认值: 500 ms\n", LED_BLINK_TASK_NAME);
#endif

  // 创建事件对象
  led_blink_event = rt_event_create("led_blink_event", RT_IPC_FLAG_FIFO);
  if (led_blink_event == RT_NULL) {
    printf("[%s] 创建事件对象失败\n", LED_BLINK_TASK_NAME);
    result = -RT_ERROR;
    goto _exit;
  }

  // 创建定时器
  led_blink_timer = rt_timer_create("led_blink_timer", led_blink_timer_callback, RT_NULL,
                                    rt_tick_from_millisecond(TASK_1_LED_BLINK_TIMER_INTERVAL), RT_TIMER_FLAG_PERIODIC);
  if (led_blink_timer == RT_NULL) {
    printf("[%s] 创建定时器失败\n", LED_BLINK_TASK_NAME);
    rt_event_delete(led_blink_event);
    led_blink_event = RT_NULL;
    result = -RT_ERROR;
    goto _exit;
  }

  printf("[%s] LED任务资源初始化成功\n", LED_BLINK_TASK_NAME);

_exit:
  if (result != RT_EOK) {
    printf("[%s] LED任务资源初始化失败\n", LED_BLINK_TASK_NAME);
  }
  return result;
}

// LED闪烁任务线程函数
static void led_blink_task_thread(void* parameter) {
  rt_uint32_t recv_event = 0;
  rt_err_t result;

  printf("[%s] LED闪烁任务线程启动\n", LED_BLINK_TASK_NAME);

  // 启动定时器
  rt_timer_start(led_blink_timer);
  led_blink_running = 1;
  printf("[%s] 定时器已启动，间隔: %d ms\n", LED_BLINK_TASK_NAME, TASK_1_LED_BLINK_TIMER_INTERVAL);

  // 线程主循环 - 等待事件
  while (led_blink_running) {
    // 等待事件，超时时间设为定时器间隔
    result = rt_event_recv(led_blink_event, LED_BLINK_EVENT_STEP | LED_BLINK_EVENT_STOP,
                           RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recv_event);

    if (result == RT_EOK) {
      if (recv_event & LED_BLINK_EVENT_STEP) {
        // 执行LED闪烁步进
        led_blink_interface_step(rt_tick_get());

        // 根据LED输出状态控制引脚
        if (led_blink_get_output()) {
          rt_pin_write(led_pin, PIN_HIGH);
#ifdef DEBUG
          printf("[%s] LED ON\n", LED_BLINK_TASK_NAME);
#endif
        } else {
          rt_pin_write(led_pin, PIN_LOW);
#ifdef DEBUG
          printf("[%s] LED OFF\n", LED_BLINK_TASK_NAME);
#endif
        }
      }
      if (recv_event & LED_BLINK_EVENT_STOP) {
        // 收到停止事件
        break;
      }
    }
  }

  // 清理资源
  if (led_blink_timer != RT_NULL) {
    rt_timer_stop(led_blink_timer);
    rt_timer_delete(led_blink_timer);
    led_blink_timer = RT_NULL;
  }

  if (led_blink_event != RT_NULL) {
    rt_event_delete(led_blink_event);
    led_blink_event = RT_NULL;
  }

  // 关闭LED
  if (led_pin != RT_NULL) {
    rt_pin_write(led_pin, PIN_LOW);
  }

  printf("[%s] LED闪烁任务线程退出\n", LED_BLINK_TASK_NAME);
}

// 初始化LED闪烁任务
int led_blink_task_init(void) {
  rt_err_t result;

  printf("[%s] 初始化LED闪烁任务\n", LED_BLINK_TASK_NAME);

  // 初始化LED任务资源
  result = led_blink_task_init_resources();
  if (result != RT_EOK) {
    printf("[%s] LED任务资源初始化失败\n", LED_BLINK_TASK_NAME);
    return -1;
  }

  // 创建线程
  led_blink_thread = rt_thread_create(LED_BLINK_TASK_NAME, led_blink_task_thread, RT_NULL,
                                      2048,                       // 栈大小
                                      TASK_1_LED_BLINK_PRIORITY,  // 优先级
                                      20);                        // 时间片
  if (led_blink_thread == RT_NULL) {
    printf("[%s] 创建线程失败\n", LED_BLINK_TASK_NAME);
    return -1;
  }

  // 启动线程
  rt_thread_startup(led_blink_thread);

  printf("[%s] LED闪烁任务初始化成功\n", LED_BLINK_TASK_NAME);
  return 0;
}

// 停止LED闪烁任务
void led_blink_task_stop(void) {
  printf("[%s] 停止LED闪烁任务\n", LED_BLINK_TASK_NAME);

  led_blink_running = 0;

  // 发送停止事件
  if (led_blink_event != RT_NULL) {
    rt_event_send(led_blink_event, LED_BLINK_EVENT_STOP);
  }

  // 等待线程结束
  if (led_blink_thread != RT_NULL) {
    rt_thread_delete(led_blink_thread);
    led_blink_thread = RT_NULL;
  }

  printf("[%s] LED闪烁任务已停止\n", LED_BLINK_TASK_NAME);
}

// 获取任务状态
int led_blink_task_is_running(void) { return led_blink_running; }

INIT_APP_EXPORT(led_blink_task_init);