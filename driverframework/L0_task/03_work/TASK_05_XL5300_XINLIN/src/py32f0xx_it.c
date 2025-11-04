/**
  ******************************************************************************
  * @file    py32f0xx_it.c
  * @author  MCU SYSTEM Team
  * @Version V1.0.0
  * @Date    
  * @brief   Interrupt Service Routines.
  ******************************************************************************

  */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "py32f0xx_it.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	while (1)
	{
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	HAL_IncTick();
}

/******************************************************************************/
/* PY32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_py32f003xx.s).                    */
/******************************************************************************/

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
