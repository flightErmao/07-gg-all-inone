#include <board.h>
#include <rtthread.h>
#include <drv_gpio.h>
#include <stm32f4xx_hal.h>
/* 函数指针类型定义 */
typedef void (*pFunction)(void);

/* 函数声明 */
static void JumpToBootloader(void);
static void system_deinit(void);
static void usb_deinit(void);

/**
 * @brief USB反初始化 - 彻底释放USB资源
 */
static void usb_deinit(void) {
  /* 禁用USB OTG FS时钟 */
  __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

  /* 反初始化USB GPIO引脚 */
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

  /* 延时确保USB断开 */
  rt_thread_mdelay(100);

  /* 重新配置PA12为输出低电平，确保USB D+为低 */
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /* 再次延时确保USB完全断开 */
  rt_thread_mdelay(200);
}

/**
 * @brief 系统反初始化 - 关闭所有系统功能
 */
static void system_deinit(void) {
  uint32_t i;

  /* 关闭全局中断 */
  __disable_irq();

  /* 关闭滴答定时器,复位到默认值 */
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  /* 关闭所有外设时钟 */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  __HAL_RCC_GPIOF_CLK_DISABLE();
  __HAL_RCC_GPIOG_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();
  __HAL_RCC_GPIOI_CLK_DISABLE();

  /* 关闭其他外设时钟 */
  __HAL_RCC_USART1_CLK_DISABLE();
  __HAL_RCC_USART2_CLK_DISABLE();
  __HAL_RCC_USART3_CLK_DISABLE();
  __HAL_RCC_UART4_CLK_DISABLE();
  __HAL_RCC_UART5_CLK_DISABLE();
  __HAL_RCC_USART6_CLK_DISABLE();
  __HAL_RCC_SPI1_CLK_DISABLE();
  __HAL_RCC_SPI2_CLK_DISABLE();
  __HAL_RCC_SPI3_CLK_DISABLE();
  __HAL_RCC_I2C1_CLK_DISABLE();
  __HAL_RCC_I2C2_CLK_DISABLE();
  __HAL_RCC_I2C3_CLK_DISABLE();
  __HAL_RCC_TIM1_CLK_DISABLE();
  __HAL_RCC_TIM2_CLK_DISABLE();
  __HAL_RCC_TIM3_CLK_DISABLE();
  __HAL_RCC_TIM4_CLK_DISABLE();
  __HAL_RCC_TIM5_CLK_DISABLE();
  __HAL_RCC_TIM6_CLK_DISABLE();
  __HAL_RCC_TIM7_CLK_DISABLE();
  __HAL_RCC_TIM8_CLK_DISABLE();
  __HAL_RCC_TIM9_CLK_DISABLE();
  __HAL_RCC_TIM10_CLK_DISABLE();
  __HAL_RCC_TIM11_CLK_DISABLE();
  __HAL_RCC_TIM12_CLK_DISABLE();
  __HAL_RCC_TIM13_CLK_DISABLE();
  __HAL_RCC_TIM14_CLK_DISABLE();
  __HAL_RCC_ADC1_CLK_DISABLE();
  __HAL_RCC_ADC2_CLK_DISABLE();
  __HAL_RCC_ADC3_CLK_DISABLE();
  __HAL_RCC_DAC_CLK_DISABLE();
  __HAL_RCC_CAN1_CLK_DISABLE();
  __HAL_RCC_CAN2_CLK_DISABLE();
  /* 注意：ETH、OTGFS、OTGHS宏可能在某些HAL版本中不存在，使用条件编译 */
#ifdef __HAL_RCC_ETH_CLK_DISABLE
  __HAL_RCC_ETH_CLK_DISABLE();
#endif
#ifdef __HAL_RCC_OTGFS_CLK_DISABLE
  __HAL_RCC_OTGFS_CLK_DISABLE();
#endif
#ifdef __HAL_RCC_OTGHS_CLK_DISABLE
  __HAL_RCC_OTGHS_CLK_DISABLE();
#endif

  /* 设置所有时钟到默认状态,使用HSI时钟 */
  HAL_RCC_DeInit();

  /* 关闭所有中断,清除所有中断挂起标志 */
  for (i = 0; i < 8; i++) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  /* 清除所有外设复位 */
  __HAL_RCC_GPIOA_FORCE_RESET();
  __HAL_RCC_GPIOB_FORCE_RESET();
  __HAL_RCC_GPIOC_FORCE_RESET();
  __HAL_RCC_GPIOD_FORCE_RESET();
  __HAL_RCC_GPIOE_FORCE_RESET();
  __HAL_RCC_GPIOF_FORCE_RESET();
  __HAL_RCC_GPIOG_FORCE_RESET();
  __HAL_RCC_GPIOH_FORCE_RESET();
  __HAL_RCC_GPIOI_FORCE_RESET();

  /* 延时确保复位完成 */
  for (i = 0; i < 1000; i++) {
    __NOP();
  }

  /* 释放所有外设复位 */
  __HAL_RCC_GPIOA_RELEASE_RESET();
  __HAL_RCC_GPIOB_RELEASE_RESET();
  __HAL_RCC_GPIOC_RELEASE_RESET();
  __HAL_RCC_GPIOD_RELEASE_RESET();
  __HAL_RCC_GPIOE_RELEASE_RESET();
  __HAL_RCC_GPIOF_RELEASE_RESET();
  __HAL_RCC_GPIOG_RELEASE_RESET();
  __HAL_RCC_GPIOH_RELEASE_RESET();
  __HAL_RCC_GPIOI_RELEASE_RESET();

  /* 使能全局中断 */
  __enable_irq();
}

/**
 * @brief 跳转到DFU bootloader
 */
static void JumpToBootloader(void) {
  pFunction SysMemBootJump;

  /* 启用SYSCFG时钟 */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* 重新映射系统内存到地址0x00000000 */
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

  /* 设置向量表偏移为0 */
  SCB->VTOR = 0;

  /* 获取跳转函数指针 */
  SysMemBootJump = (pFunction)(*((uint32_t *)(4)));

  /* 设置主堆栈指针 */
  __set_MSP(*(uint32_t *)(0));

  /* 跳转到bootloader */
  SysMemBootJump();
}

/**
 * @brief DFU跳转命令
 * @param argc 参数个数
 * @param argv 参数数组
 * @return 0
 */
static int cmd_dfu(int argc, char **argv) {
  rt_kprintf("=== DFU跳转命令 ===\n");
  rt_kprintf("准备进入DFU模式...\n");

  /* USB反初始化 */
  rt_kprintf("释放USB资源...\n");
  usb_deinit();

  /* 系统反初始化 */
  rt_kprintf("关闭系统功能...\n");
  system_deinit();

  /* 跳转到bootloader */
  rt_kprintf("跳转到DFU bootloader...\n");
  JumpToBootloader();

  /* 正常情况下不会执行到这里 */
  return 0;
}

#ifdef L3_PERIPHERAL_03_DFU_CONFIG_EN
/* 导出MSH命令 */
MSH_CMD_EXPORT_ALIAS(cmd_dfu, dfu, jump to DFU bootloader mode);
#endif