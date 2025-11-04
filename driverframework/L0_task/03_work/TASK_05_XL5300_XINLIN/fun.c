#include "fun.h"
#include "flash.h"
void XL5300_Io_Init(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct;
	 __HAL_RCC_GPIOB_CLK_ENABLE();
	 __HAL_RCC_GPIOA_CLK_ENABLE();
	 GPIO_InitStruct.Pin = GPIO_PIN_0; //  PA0-> XSHUT
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_PULLUP; //上拉
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	 GPIO_InitStruct.Pin = GPIO_PIN_1; // PA1 -> GPIOI
	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 GPIO_InitStruct.Pull = GPIO_PULLUP; //上拉
	 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

     GPIO_InitStruct.Pin = GPIO_PIN_6; // PB6 -> SCK
     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_PULLUP; //上拉
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	 GPIO_InitStruct.Pin = GPIO_PIN_7;		//PB7 -> SDA 
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_PULLUP; //上拉
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//key
	 GPIO_InitStruct.Pin = GPIO_PIN_12; // PA12
	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 GPIO_InitStruct.Pull = GPIO_PULLUP; //上拉
	 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/********************************************************************************
Description:
Input:
Output:
Return:
*********************************************************************************/
void XL5300_All_Init(void)
{
	XL5300_Io_Init();
	//////////////////////5300 api start//////////////////////	
		XL5300_Chip_PowerON(XSHUT_Pin1); // powerON

		if (XL5300_Device_Check() != 0xD8) // check IIC ok
		{
			printf(("Check device ID fail!\r\n"));
		}	
}

/********************************************************************************
Description:
Input:
Output:
Return:
*********************************************************************************/
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	//开启HSI,HSE,LSE,LSI,PLL所有时钟
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz; //配置HSI输出时钟为24MHz
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;						  // HSI分频后作为HSISYS输出给系统时钟
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;    
	RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);  
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	//初始化CPU,AHB,APB总线时钟 1分频
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	 HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);   
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
#if IWDT_EN
IWDG_HandleTypeDef IwdgHandle;
void wdg_Init(void)
{
	IwdgHandle.Instance = IWDG;					   //选择IWDG
	IwdgHandle.Init.Prescaler = IWDG_PRESCALER_32; //配置32分频
	IwdgHandle.Init.Reload = (1000);			   // IWDG计数器重装载值为1000，1s

	if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK) //初始化IWDG
	{
	
		printf("IWDT init erro");
		Error_Handler();
	}
}
#endif
/********************************************************************************
Description:    清看门狗
Input:
Output:
Return:
*********************************************************************************/
#if IWDT_EN
void clear_Wdg(void)
{
	if (HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK)
	{
		/* Refresh Error */

		printf("Refresh Error");
		Error_Handler();
	}
}
#endif

/********************************************************************************
Description:
Input:
Output:
Return:
*********************************************************************************/
