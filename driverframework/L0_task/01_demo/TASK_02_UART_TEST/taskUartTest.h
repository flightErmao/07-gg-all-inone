#ifndef __TASK_UART_TEST_H__
#define __TASK_UART_TEST_H__

#include <rtthread.h>

/**
 * @brief UART test task function declaration
 * 
 * This task demonstrates optimized UART configuration with:
 * - Only IDLE interrupt enabled (for DMA RX mode)
 * - All other interrupts (DMA TX, UART RX/TX) disabled
 * - Efficient ring buffer data handling
 * - Minimal interrupt overhead
 */
void uartTestTask(void *param);

#endif 