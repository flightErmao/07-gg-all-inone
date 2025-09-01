
#include <board.h>
#include <rtthread.h>
#include <drv_gpio.h>
#include <rtdevice.h>

#define LED0_PIN GET_PIN(E, 12)

int main(void)
{
  extern uint32_t SystemCoreClock;

  rt_kprintf("=== STM32F427VI 时钟配置信息 ===\n");
  rt_kprintf("外部晶振频率: %d Hz (%.1f MHz)\n", HSE_VALUE, HSE_VALUE / 1000000.0);
  rt_kprintf("系统时钟频率: %d Hz (%.1f MHz)\n", SystemCoreClock, SystemCoreClock / 1000000.0);
  rt_kprintf("RT_TICK_PER_SECOND: %d\n", RT_TICK_PER_SECOND);
  rt_kprintf("预期系统时钟: 168 MHz\n");
  rt_kprintf("==============================\n");

  /* set LED0 pin mode to output */
  rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

  while (1) {
    rt_pin_write(LED0_PIN, PIN_HIGH);
    rt_thread_mdelay(500);
    rt_pin_write(LED0_PIN, PIN_LOW);
    rt_thread_mdelay(500);
  }
}
