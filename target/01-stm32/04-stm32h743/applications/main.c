
#include <board.h>
#include <rtthread.h>
#include <drv_gpio.h>
#include <rtdevice.h>

// weikong led E4/E5/E6
#define LED0_PIN GET_PIN(E, 4)
#define LED1_PIN GET_PIN(E, 5)
#define LED2_PIN GET_PIN(E, 6)

int main(void) {
  extern uint32_t SystemCoreClock;

  rt_kprintf("=== STM32H743VI 时钟配置信息 ===\n");
  rt_kprintf("外部晶振频率: %d Hz (%.1f MHz)\n", HSE_VALUE, HSE_VALUE / 1000000.0);
  rt_kprintf("系统时钟频率: %d Hz (%.1f MHz)\n", SystemCoreClock, SystemCoreClock / 1000000.0);
  rt_kprintf("RT_TICK_PER_SECOND: %d\n", RT_TICK_PER_SECOND);
  rt_kprintf("预期系统时钟: 240 MHz\n");
  rt_kprintf("==============================\n");

  /* set LED0 pin mode to output */
  rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
  rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
  rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);

  while (1) {
    rt_pin_write(LED0_PIN, PIN_HIGH);
    rt_thread_mdelay(1000);
    rt_pin_write(LED0_PIN, PIN_LOW);
    rt_thread_mdelay(1000);
    rt_pin_write(LED1_PIN, PIN_HIGH);
    rt_thread_mdelay(1000);
    rt_pin_write(LED1_PIN, PIN_LOW);
    rt_thread_mdelay(1000);
    rt_pin_write(LED2_PIN, PIN_HIGH);
    rt_thread_mdelay(1000);
    rt_pin_write(LED2_PIN, PIN_LOW);
    rt_thread_mdelay(1000);
  }
}
