
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
  rt_kprintf("预期系统时钟: 400 MHz\n");

  /* set LED0 pin mode to output */
  rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
  rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
  rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);

  while (1) {
    // LED闪烁
    rt_pin_write(LED0_PIN, PIN_HIGH);
    rt_thread_mdelay(100);
    rt_pin_write(LED0_PIN, PIN_LOW);
    rt_thread_mdelay(100);
    rt_pin_write(LED1_PIN, PIN_HIGH);
    rt_thread_mdelay(100);
    rt_pin_write(LED1_PIN, PIN_LOW);
    rt_thread_mdelay(100);
    rt_pin_write(LED2_PIN, PIN_HIGH);
    rt_thread_mdelay(100);
    rt_pin_write(LED2_PIN, PIN_LOW);
    rt_thread_mdelay(100);

    // 每3秒打印一次时钟信息
    rt_kprintf("\n=== 时钟频率监控 (每3秒更新) ===\n");
    rt_kprintf("系统时钟频率: %d Hz (%.1f MHz)\n", SystemCoreClock, SystemCoreClock / 1000000.0);
    rt_kprintf("HCLK (AHB总线): %d Hz (%.1f MHz)\n", HAL_RCC_GetHCLKFreq(), HAL_RCC_GetHCLKFreq() / 1000000.0);
    rt_kprintf("PCLK1 (APB1总线): %d Hz (%.1f MHz)\n", HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetPCLK1Freq() / 1000000.0);
    rt_kprintf("PCLK2 (APB2总线): %d Hz (%.1f MHz)\n", HAL_RCC_GetPCLK2Freq(), HAL_RCC_GetPCLK2Freq() / 1000000.0);

    // USB FS时钟
    uint32_t usb_clock = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_USB);
    rt_kprintf("USB FS时钟: %d Hz (%.1f MHz)\n", usb_clock, usb_clock / 1000000.0);

    // 检查USB时钟是否正常 (USB FS需要48MHz，允许±0.25%误差)
    if (usb_clock >= 47988000 && usb_clock <= 48012000) {
      rt_kprintf("✓ USB FS时钟配置正确 (48MHz ±0.25%%)\n");
    } else if (usb_clock >= 49975000 && usb_clock <= 50025000) {
      rt_kprintf("⚠ USB FS时钟为50MHz (可接受范围)\n");
    } else {
      rt_kprintf("✗ USB FS时钟配置错误! 期望48MHz, 实际%.1fMHz\n", usb_clock / 1000000.0);
    }

    rt_kprintf("================================\n");

    // 延迟3秒
    rt_thread_mdelay(3000);
  }
}
