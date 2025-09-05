/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <board.h>
#include <drv_common.h>

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 1. 使能 PWR 时钟，设置电压调节器为 Scale1 */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* 2. 配置 HSE 和 PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON; /* 外部 24MHz 晶振开启 */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;            /* 24 MHz / 12 = 2 MHz (VCO 输入) */
  RCC_OscInitStruct.PLL.PLLN = 168;           /* VCO = 2 MHz * 168 = 336 MHz */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; /* SYSCLK = 336 / 2 = 168 MHz */
  RCC_OscInitStruct.PLL.PLLQ = 7;             /* USB 时钟 = 336 / 7 = 48 MHz */

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    goto error;
  }

  /* 3. 启用 Over-Drive 模式（支持 168MHz） */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    goto error;
  }

  /* 4. 配置系统时钟源和总线分频 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; /* HCLK = 168 MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  /* PCLK1 = 42 MHz */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  /* PCLK2 = 84 MHz */

  /* 设置 Flash 5 个等待周期（168MHz 需要） */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    goto error;
  }

  return;

error:
  /* 错误处理：死循环或点灯 */
  while (1) {
    // 可选：翻转 LED 指示错误
  }
}