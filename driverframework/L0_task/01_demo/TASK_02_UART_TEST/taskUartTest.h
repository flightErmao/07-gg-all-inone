#ifndef __TASK_UART_TEST_H__
#define __TASK_UART_TEST_H__

#include <rtthread.h>

/**
 * @brief UART test task function declaration
 * 
 * This task demonstrates optimized UART configuration with:
 * - DMA RX mode enabled for efficient data reception
 * - DMA interrupts (HAL_UART_RxCpltCallback, HAL_UART_RxHalfCpltCallback) disabled
 * - Only UART IDLE interrupt enabled for data indication
 * - Message queue based data processing to avoid interrupt conflicts
 * - Efficient ring buffer data handling with minimal overhead
 * 
 * Key benefits:
 * 1. Avoids HAL DMA callback conflicts
 * 2. Reduces interrupt processing overhead
 * 3. Maintains DMA efficiency for data transfer
 * 4. Provides predictable interrupt behavior
 * 5. Enables custom data processing logic
 */
void uartTestTask(void *param);

#endif 