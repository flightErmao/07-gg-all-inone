/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 * 2023-12-03     Meco Man     support nano version
 */

#include <board.h>
#include <rtthread.h>
#include <drv_gpio.h>
#ifndef RT_USING_NANO
#include <rtdevice.h>
#endif /* RT_USING_NANO */

/* 包含必要的HAL库头文件 */
#include <stm32f4xx_hal.h>

/* defined the LED0 pin: PC4 */
/* LEDs: PC4, PA8, PC5, PA10 */
#define LED0_PIN GET_PIN(A, 0)
// #define LED0_PIN GET_PIN(A, 15)

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

/**
 * @brief 显示DFU使用说明
 * @param argc 参数个数
 * @param argv 参数数组
 * @return 0
 */
static int cmd_dfu_help(int argc, char **argv) {
  rt_kprintf("=== DFU使用说明 ===\n");
  rt_kprintf("命令列表:\n");
  rt_kprintf("1. dfu - 直接跳转到DFU bootloader模式\n");
  rt_kprintf("2. reboot - 重启系统(系统内置命令)\n");
  rt_kprintf("3. dfu_help - 显示此帮助信息\n");
  rt_kprintf("\n");
  rt_kprintf("DFU跳转说明:\n");
  rt_kprintf("- 执行dfu命令后，系统将直接跳转到DFU bootloader\n");
  rt_kprintf("- 进入DFU模式后，可以使用STM32CubeProgrammer等工具更新固件\n");
  rt_kprintf("\n");
  rt_kprintf("注意事项:\n");
  rt_kprintf("- 确保USB连接正常\n");
  rt_kprintf("- 跳转后系统将停止运行，直到固件更新完成\n");
  rt_kprintf("- 系统重启可使用内置的reboot命令\n");

  return 0;
}

int main(void)
{
  extern uint32_t SystemCoreClock;

  rt_kprintf("=== STM32F405RG 时钟配置信息 ===\n");
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

/* 导出MSH命令 */
MSH_CMD_EXPORT_ALIAS(cmd_dfu, dfu, 跳转到DFU bootloader模式);
MSH_CMD_EXPORT_ALIAS(cmd_dfu_help, dfu_help, 显示DFU使用说明);
