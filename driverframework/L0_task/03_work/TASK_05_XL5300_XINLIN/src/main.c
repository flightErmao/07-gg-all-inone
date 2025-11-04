/**
 ******************************************************************************
 * @file    main.c
 * @author  
 * @Version V1.0.0
 * @Date    
 * @brief   main function
 ******************************************************************************
 */
#include "main.h"
#include "xl_sw_i2c.h"
#include "fun.h"
#include "function.h"
#include "XL5300_API.h"
extern XL5300_MEASURE_TypeDef Measure_Data;
extern XL5300_Calibration_TypeDef XL5300_Cali_Data;
extern uint8_t ref_tof_flag;
int main(void)
{
	uint8_t ret = 0;
	HAL_Init();		//0、初始化所有外设，flash接口，systick
	SystemClock_Config(); 														//配置系统时钟
	Uart_Init(115200); 															//调试打印
	#if IWDT_EN
	wdg_Init();																	//看门狗初始化
	#endif	
	XL5300_Cali_Data.XL5300_Interrupt_Mode_Status = 0x00;//软件中断 
	XL5300_Cali_Data.XL5300_Power_Manage_Status = 0x01;//开启电源管理
	XL5300_All_Init();//IO初始化
	//
	ret |= VI530x_Chip_Init(); //寄存器初始化 
	ret |= VI530x_Download_Firmware((uint8_t *)Firmware_Ranging, LoadFirmware()); //固件写入
	#if 0
	/************************* 标定后正常测距流程 *****************************/
	/***********从主控端NVM读取标定值，并赋值到标定全局变量**********************/
	/* Start user code for adding. */
	//1 从主控端NVM（Flash/EEROM..）读取标定值
     F003_Flash_Read();
	/* End user code.  */
	//2 配置标定值
	//	VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio += 10;	//增加Xtalk处理上限，正常不需要启用
	ret |= VI530x_Set_Californiation_Data(XL5300_Cali_Data.XL5300_Calibration_Offset);
#else
		/************************* 生产标定流程 *****************************/
	//CK标定
	ret |= XL5300_CK_Calibration();  //成功
	if(ret == 0)
	{
		printf("XL5300_Calibration_CK = %#x\r\n",XL5300_Cali_Data.XL5300_Calibration_CK);
	}
	else
	{
		printf("CK Calibration Fail!\r\n");
	}
	
	//MP标定
	ret |= VI530x_MP_Calibration();
	if(ret == 0)
	{
		printf("XL5300_Calibration_MP = %4d\r\n",XL5300_Cali_Data.XL5300_Calibration_MP);
	}
	else
	{
		printf("MP Calibration Fail!\r\n");
	}
	//ref_tof标定，环境要求室温；
	ret |= VI530x_Reftof_Calibration();
	if(ret == 0)
	{
		printf("XL5300_Calibration_Reftof = %4d\r\n",XL5300_Cali_Data.XL5300_Calibration_Reftof);
	}
	else
	{
		printf("RefTof Calibration Fail!\r\n");
	}
	
	//Xtalk标定，环境要求60cm内无目标物
	ret |= VI530x_Xtalk_Calibration();
	if(ret == 0)
	{
		printf("XL5300_Calibration_CG_Pos = %d\r\n",XL5300_Cali_Data.XL5300_Calibration_CG_Pos);
		printf("XL5300_Calibration_CG_Maxratio = %d\r\n",XL5300_Cali_Data.XL5300_Calibration_CG_Maxratio);
		/***！Xtalk卡控，建议在标定工装上卡控，便于根据结构调整  ***/
		if(XL5300_Cali_Data.XL5300_Calibration_CG_Maxratio > 15)	
		{
		  //检测结构Xtalk大小，盖板为主要影响，可以根据实际情况微调
			printf("Xtalk is too large, Fail!\r\n");
		}
	}
	else
	{
		printf("Xtalk Calibration Fail!\r\n");
	}
	//Offset标定
	//在固定距离做offset标定,如10cm，则把参数VI530x_OFFSET_DISTANCE改成100（mm）
	ret |= VI530x_Offset_Calibration(VI530x_OFFSET_DISTANCE);
	if(ret == 0)
	{
		printf("VI530x_Calibration_Offset = %f\r\n",XL5300_Cali_Data.XL5300_Calibration_Offset);
	}
	else
	{
		printf("Offset Calibration Fail!\r\n");
	}
	F003_Flash_Write();
#endif

	//7、其它配置
	//开启温度校准:0x00-关，0x01-开，请做了RefTof标定后再开户温度校准
			if(ref_tof_flag==0)
	{
	ret |= XL5300_Temp_Enable(0x01);
	}
	else
	{
	 ret |= XL5300_Temp_Enable(0x00);
	}
	
	ret |= XL5300_Set_Integralcounts_Frame(50,120000); //帧率，积分次数
	//8、开启测距
	ret |= XL5300_Start_Continue_Ranging_Cmd();	//连续模式
//	ret = VI530x_Start_Single_Ranging_Cmd();	//单次模式,功耗低
		if(ret)
	{
		printf("XL5300 Configer Error\r\n");
	}
	else
	{
		printf("XL5300 Configer Ok\r\n");
	}
			while(1)
{
		xtalk();///按键触发进行标定。标定时在2米处放置灰卡，请将tof对准2米处灰卡进行标定(注意TOF角度)；标定完成后将标定值写入FLASH。
		ret = XL5300_Get_Measure1_Data(&Measure_Data);
   if(!ret)
		{			
	    	printf("tof = %4d,confidece = %3d\r\n",Measure_Data.correction_tof,Measure_Data.confidence);
		}
/*			
			Measure_Data.correction_tof：距离值，毫米为单位；
			Measure_Data.confidence：表示当前 TOF 值的可信度；
			Measure_Data.peak：表征接收到光信号强度；
			Measure_Data.intecounts； 测距输出的积分次数    
*/	
 }
}

/********************************************************************************
Description:
Input:
Output:
Return:
*********************************************************************************/
void Error_Handler(void)
{
	while (1)
	{
  		HAL_FLASH_OB_Launch();/* 产生一个复位*/
	}
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
