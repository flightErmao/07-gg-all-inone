/**
 ******************************************************************************
 * @file    py32f0xx_hal_msp.c
 * @author  MCU SYSTEM Team
 * @Version V1.0.0
 * @Date    
 * @brief   This file provides code for the MSP Initialization
 *          and de-Initialization codes.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* External functions --------------------------------------------------------*/

/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void)
{
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	/* 使能LSI时钟 */
	__HAL_RCC_LSI_ENABLE();
	__HAL_RCC_HSI_ENABLE();
	/* 等待直到LSI READY置位 */
	while (READ_BIT(RCC->CSR, RCC_CSR_LSIRDY) == 0U);
}
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  /* 使能时钟 */
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /**USART1引脚配置
    PA2     ------> USART1_TX
    PA3    ------> USART1_RX
    */
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


/************************ (C) COPYRIGHT Puya *****END OF FILE****/
