#include <board.h>
#include <rtthread.h>
#include <drv_gpio.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_rtc.h>

/* 函数指针类型定义 */
typedef void (*pFunction)(void);

/* DFU魔术数字定义 - 修复十六进制常量 */
#define DFU_MAGIC_NUMBER 0xDF12345

/* 函数声明 */
static void system_deinit(void);
static void usb_deinit(void);
static void gpio_deinit_all(void);
static void set_dfu_flag(void);

/**
 * @brief 设置DFU标志到RTC备份寄存器
 */
static void set_dfu_flag(void) {
  /* 启用PWR时钟 */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* 启用RTC备份寄存器访问 */
  HAL_PWR_EnableBkUpAccess();

  /* 启用RTC时钟 */
  __HAL_RCC_RTC_ENABLE();

  /* 设置DFU魔术数字到RTC备份寄存器0 */
  RTC->BKP0R = DFU_MAGIC_NUMBER;

  /* 确保数据写入完成 */
  __DSB();
  __ISB();
}

/**
 * @brief 检查DFU标志
 * @return 1表示需要进入DFU，0表示正常启动
 */
int check_dfu_flag(void) {
  uint32_t magic_value;

  /* 启用PWR时钟 */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* 启用RTC备份寄存器访问 */
  HAL_PWR_EnableBkUpAccess();

  /* 启用RTC时钟 */
  __HAL_RCC_RTC_ENABLE();

  /* 读取RTC备份寄存器0 */
  magic_value = RTC->BKP0R;

  if (magic_value == DFU_MAGIC_NUMBER) {
    /* 清除标志，避免重复进入DFU */
    RTC->BKP0R = 0;
    return 1; /* 需要进入DFU */
  }

  return 0; /* 正常启动 */
}

/**
 * @brief 彻底释放所有GPIO资源
 */
static void gpio_deinit_all(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* 启用所有GPIO时钟以便配置 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  /* 配置所有GPIO为模拟输入模式，断开外部连接 */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  /* 配置GPIOA所有引脚 */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 配置GPIOB所有引脚 */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 配置GPIOC所有引脚 */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* 配置GPIOD所有引脚 */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* 配置GPIOE所有引脚 */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* 配置GPIOF所有引脚 */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* 配置GPIOG所有引脚 */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* 配置GPIOH所有引脚 */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* 配置GPIOI所有引脚 */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /* 延时确保GPIO配置生效 */
  rt_thread_mdelay(50);

  /* 反初始化所有GPIO */
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_All);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_All);
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_All);
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_All);
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_All);
  HAL_GPIO_DeInit(GPIOF, GPIO_PIN_All);
  HAL_GPIO_DeInit(GPIOG, GPIO_PIN_All);
  HAL_GPIO_DeInit(GPIOH, GPIO_PIN_All);
  HAL_GPIO_DeInit(GPIOI, GPIO_PIN_All);

  /* 延时确保反初始化完成 */
  rt_thread_mdelay(50);
}

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

  /* 更彻底地关闭所有中断和清除中断标志位 */
  /* 关闭所有NVIC中断 */
  for (i = 0; i < 8; i++) {
    NVIC->ICER[i] = 0xFFFFFFFF; /* 禁用所有中断 */
    NVIC->ICPR[i] = 0xFFFFFFFF; /* 清除所有中断挂起标志 */
  }

  /* 关闭所有外设中断 */
  /* 关闭定时器中断 */
  TIM1->DIER = 0;
  TIM2->DIER = 0;
  TIM3->DIER = 0;
  TIM4->DIER = 0;
  TIM5->DIER = 0;
  TIM6->DIER = 0;
  TIM7->DIER = 0;
  TIM8->DIER = 0;
  TIM9->DIER = 0;
  TIM10->DIER = 0;
  TIM11->DIER = 0;
  TIM12->DIER = 0;
  TIM13->DIER = 0;
  TIM14->DIER = 0;

  /* 关闭串口中断 */
  USART1->CR1 &= ~USART_CR1_RXNEIE;
  USART1->CR1 &= ~USART_CR1_TXEIE;
  USART2->CR1 &= ~USART_CR1_RXNEIE;
  USART2->CR1 &= ~USART_CR1_TXEIE;
  USART3->CR1 &= ~USART_CR1_RXNEIE;
  USART3->CR1 &= ~USART_CR1_TXEIE;
  UART4->CR1 &= ~USART_CR1_RXNEIE;
  UART4->CR1 &= ~USART_CR1_TXEIE;
  UART5->CR1 &= ~USART_CR1_RXNEIE;
  UART5->CR1 &= ~USART_CR1_TXEIE;
  USART6->CR1 &= ~USART_CR1_RXNEIE;
  USART6->CR1 &= ~USART_CR1_TXEIE;

  /* 关闭SPI中断 */
  SPI1->CR2 = 0;
  SPI2->CR2 = 0;
  SPI3->CR2 = 0;

  /* 关闭I2C中断 - 直接清零CR2寄存器来关闭所有中断 */
  I2C1->CR2 = 0;
  I2C2->CR2 = 0;
  I2C3->CR2 = 0;

  /* 关闭CAN中断 */
  CAN1->IER = 0;
  CAN2->IER = 0;

  /* 关闭ADC中断 */
  ADC1->CR1 = 0;
  ADC2->CR1 = 0;
  ADC3->CR1 = 0;

  /* 关闭DAC中断 */
  DAC->CR = 0;

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
  for (i = 0; i < 3000; i++) {
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

  /* 清除所有中断挂起标志 */
  for (i = 0; i < 8; i++) {
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  /* 最后再次确保全局中断关闭 */
  __disable_irq();
}

/**
 * @brief DFU跳转命令
 * @param argc 参数个数
 * @param argv 参数数组
 * @return 0
 */
static int cmdDfu(int argc, char **argv) {
  rt_kprintf("=== DFU跳转命令 ===\n");
  rt_kprintf("准备进入DFU模式...\n");

  /* 设置DFU标志到RTC备份寄存器 */
  rt_kprintf("设置DFU标志...\n");
  set_dfu_flag();

  /* 彻底释放所有GPIO资源 */
  rt_kprintf("释放GPIO资源...\n");
  gpio_deinit_all();

  /* USB反初始化 */
  rt_kprintf("释放USB资源...\n");
  usb_deinit();

  /* 系统反初始化 */
  rt_kprintf("关闭系统功能...\n");
  system_deinit();

  /* 延时确保所有操作完成 */
  rt_thread_mdelay(100);

  /* 重启系统 */
  rt_kprintf("重启系统进入DFU模式...\n");
  rt_hw_cpu_reset();

  /* 正常情况下不会执行到这里 */
  return 0;
}

#ifdef L3_PERIPHERAL_03_DFU_CONFIG_EN
/* 导出MSH命令 */
MSH_CMD_EXPORT_ALIAS(cmdDfu, cmdDfu, jump to DFU bootloader mode);
#endif