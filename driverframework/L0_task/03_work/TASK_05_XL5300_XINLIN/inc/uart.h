#ifndef __uart_H
#define __uart_H
#include "main.h"

/* Exported functions prototypes ---------------------------------------------*/
#define USARTx                              USART2
#define USARTx_TX_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_FORCE_RESET()                __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()              __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                       GPIO_PIN_4
#define USARTx_TX_GPIO_PORT                 GPIOA
#define USARTx_TX_AF                        GPIO_AF9_USART2

void Uart_Init(uint32_t bound);

#endif
