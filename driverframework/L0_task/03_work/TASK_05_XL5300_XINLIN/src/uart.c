/********************************************************************************
File:			
Author:			
Date:		
Version:		V001
Description:				
*********************************************************************************/
#include "uart.h"

UART_HandleTypeDef UartHandle;

/********************************************************************************
Description:
Input:
Output:
Return:
*********************************************************************************/

/********************************************************************************
Description:
Input:
Output:
Return:
*********************************************************************************/

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
	/*##-1- Reset peripherals ##################################################*/
	USARTx_FORCE_RESET();
	USARTx_RELEASE_RESET();

	/*##-2- Disable peripherals and GPIO Clocks #################################*/
	/* Configure UART Tx as alternate function  */
	HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
	/* Configure UART Rx as alternate function  */
}

/********************************************************************************
Description:
Input:
Output:
Return:
*********************************************************************************/
#if (defined(__CC_ARM)) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);
	return (ch);
}
int fgetc(FILE *f)
{
	int ch;
	HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 1000);
	return (ch);
}
#elif defined(__ICCARM__)
int putchar(int ch)
{
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);
	return (ch);
}
#endif

/********************************************************************************
Description:
Input:
Output:
Return:
*********************************************************************************/
void Uart_Init(uint32_t bound)
{
	UartHandle.Instance = USARTx;
	UartHandle.Init.BaudRate = bound;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode = UART_MODE_TX;
	if (HAL_UART_DeInit(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}
}
/********************************************************************************
Description:
Input:
Output:
Return:
*********************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	/* Set transmission flag: transfer complete */
}
/********************************************************************************
Description:	Rx Transfer completed callback
Input:			UartHandle: UART handle
Output:
Return:
*********************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	/* Set transmission flag: transfer complete */
}
