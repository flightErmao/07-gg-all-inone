#include "VI530x_API.h"
#include "VI530x_Algorithm.h"
#include "VI530x_Firmware.h"
#include "VI530x_System_Data.h"



// VI530x GPIO中断信号
// 0-清零/没有中断信号
// 1-有中断信号
uint8_t VI530x_GPIO_Interrupt_status = 0;

//VI530x IIC的设备地址上电默认8位地址为0xD8
uint8_t VI530x_IIC_Dev_Addr_Now = VI530x_IIC_DEV_ADDR;

//全局配置参数
VI530x_Calibration_TypeDef VI530x_Cali_Data;
uint8_t VI530x_Chip_ID = 0x31;


/**************************************************************************************************
******************************************  Init API *********************************************
**************************************************************************************************/

/**
 * @brief 	通过VI530X的XSHUT引脚拉低再拉高进行复位，保持常拉高---使能
 * @param 	[none]  
 * @return 	[none]
 */
void VI530x_Chip_PowerON(void)
{
	// Xshut pin
	VI530x_XSHUT_Enable(0); // set 0
	VI530x_Delay_Ms(10);											 // delay 10ms
	VI530x_XSHUT_Enable(1);	 // set 1
	VI530x_Delay_Ms(10);											 // delay 10ms,至少5ms,延时不足容易出现Init失败
}

/**
 * @brief 	通过VI530X的XSHUT引脚拉低，保持常拉低---失能
 * @param 	[none]  
 * @return 	[none]
 */
void VI530x_Chip_PowerOFF(void)
{
	VI530x_XSHUT_Enable(0);
}

/**
 * @brief 	VI530X芯片软复位
 * @param 	[none]
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Chip_SWReset(void)
{
	VI530x_Status ret = VI530x_OK;
	ret |= VI530x_IIC_Write_One_Byte(VAN_REG_PW_CTRL, 0x0E);
	ret |= VI530x_IIC_Write_One_Byte(VAN_REG_PW_CTRL, 0x0F);
	VI530x_Delay_Ms(5);
	VI530x_IIC_Write_One_Byte(VAN_REG_AO_DOMAIN, 0x90); //0x90写入后IIC会reset,无ACK返回
	VI530x_Delay_Ms(5);
	return ret;
}

/**
 * @brief 	初始化VI530X寄存器
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Chip_Register_Init(uint8_t *chip_id)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t version_31_cnt = 5;
	uint8_t version_30_cnt = 5;

	//读取芯片版本
	do
	{
		ret |= VI530x_IIC_Read_One_Byte(0x38, chip_id);
		if (*chip_id == 0x30)
		{
			version_31_cnt--;
		}
		else
		{
			version_30_cnt--;
		}
	} while (version_31_cnt && version_30_cnt); // 0x30 v3.1, 0x00 v3.0

	if (version_31_cnt == 0)
	{
		*chip_id = 0x31;
	}
	else
	{
		*chip_id = 0x30;
	}
#ifdef Debug_Mode
	printf("VI530x chip ID is %#x\r\n",*chip_id);
#endif

	//寄存器初始化
	/*****************************************************************/
	/***********************for v3.1 test 20210304********************/
	//#define VI530x_POWER_MANAGE       1

	if(VI530x_Cali_Data.VI530x_Power_Manage_Status)
	{
		//开启电源管理模式
		ret |= VI530x_IIC_Write_One_Byte(VAN_REG_SYS_CFG, 0x0C);
	}
	else
	{
		//关闭启电源管理模式
		ret |= VI530x_IIC_Write_One_Byte(VAN_REG_SYS_CFG, 0x08);
	}

	ret |= VI530x_IIC_Write_One_Byte(0x07, 0x00); //閿熸枻鎷蜂綅PD閿熸枻鎷锋媷閿熸枻鎷烽敓锟?
	ret |= VI530x_IIC_Write_One_Byte(0x07, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x07, 0x00); //閿熻?闈╂嫹A0閿熸枻鎷锋媷閿熸枻鎷烽敓琛楋拷
	ret |= VI530x_IIC_Write_One_Byte(0x04, 0x21);
	ret |= VI530x_IIC_Write_One_Byte(0x05, 0x0e);

	ret |= VI530x_IIC_Write_One_Byte(0x08, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x37, 0x80);
	ret |= VI530x_IIC_Write_One_Byte(0x38, 0x30); // v3.0 0x00
	ret |= VI530x_IIC_Write_One_Byte(0x39, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x3a, 0x30); // v3.0 0x00
	ret |= VI530x_IIC_Write_One_Byte(0x3b, 0x80);
	ret |= VI530x_IIC_Write_One_Byte(0x3c, 0x80);
	ret |= VI530x_IIC_Write_One_Byte(0x3d, 0x80);
	ret |= VI530x_IIC_Write_One_Byte(0x3e, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x3f, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x07, 0x0e);
	ret |= VI530x_IIC_Write_One_Byte(0x07, 0x0f);

	return ret;
}

/**
 * @brief 	查询VI530X芯片busy等状态
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Wait_For_CPU_Ready(void)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t stat = 0;
	int retry = 0;

	do
	{
		VI530x_Delay_Ms(1); // delay 1ms
		ret = VI530x_IIC_Read_One_Byte(0x02, &stat);
	}
	while ((retry++ < 20) && (stat & 0x01));
	if (retry >= 20)
	{
	#ifdef Debug_Mode
		printf("CPU Busy stat = %d\n", stat);
	#endif
		return VI530x_BUSY;
	}

	return ret;
}

/**
 * @brief 	VI530X中断清除
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Clear_Interrupt(void)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t temp_status = 0;
	//硬件中断
	VI530x_GPIO_Interrupt_status = 0;
	//寄存器中断（软件中断）
	ret |= VI530x_IIC_Read_One_Byte(VAN_RET_INT_STATUS, &temp_status);
	return ret;
}

/**
 * @brief 	VI530X中断 读取 & 清除
 * @param 	[uint8_t] *interrupt_status：获取中断状态
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Get_And_Clear_Interrupt(uint8_t *interrupt_status)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t temp_status = 0;

	//使用寄存器中断（软件中断）
	if(!VI530x_Cali_Data.VI530x_Interrupt_Mode_Status)
	{
		ret |= VI530x_IIC_Read_One_Byte(VAN_RET_INT_STATUS, &temp_status);
	}
	if(VI530x_GPIO_Interrupt_status || (temp_status & 0x01))
	{
		*interrupt_status = 0x01;
		VI530x_GPIO_Interrupt_status = 0;
	}
	else
	{
		*interrupt_status = 0x00;
	}
	return ret;
}

/**
 * @brief 改IIC设备地址
 * @param [uint8_t] addr_val	8位地址，注意不是7位地址，最低位为0
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Set_ModelChangeAddr(uint8_t addr_val)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t IIC_ID = 0;
	//0x0c进入standby,0x08退出standby	,0x88允许更改VI530x的IIC 地址
	ret = VI530x_IIC_Write_One_Byte(VAN_REG_SYS_CFG,0x88);
	//更改设备地址，写入IIC寄存器
	ret |= VI530x_IIC_Write_One_Byte(VAN_REG_IIC_DEV_ADDR,addr_val);
	if(ret == VI530x_OK)
	{
		//新地址赋值IIC地址全局变量
		VI530x_IIC_Dev_Addr_Now = addr_val;
	}

	ret |= VI530x_IIC_Write_One_Byte(VAN_REG_SYS_CFG,0x08);
	VI530x_Delay_Ms(5);
	ret |= VI530x_IIC_Read_One_Byte(VAN_REG_IIC_DEV_ADDR, &IIC_ID);
	if(IIC_ID != addr_val)
	{
		VI530x_IIC_Dev_Addr_Now = VI530x_IIC_DEV_ADDR;
		ret |= VI530x_ERROR_CONFIG;
	}
	return ret;
}

/**
 * @brief 	VI530X写命令
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530xWriteCommand(uint8_t cmd)
{
	return VI530x_IIC_Write_One_Byte(VAN_REG_CMD, cmd);
}


/**
 * @brief 	初始化VI530X
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Chip_Init(void)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t IIC_ID = 0;

	//配置电源管理模式：0----关闭电源管理模式，其他值----开启电源管理
	//开启后VI530x待机进入低功耗
	VI530x_Cali_Data.VI530x_Power_Manage_Status = 0x01; //开启电源管理

	ret |= VI530x_IIC_Read_One_Byte(VAN_REG_IIC_DEV_ADDR, &IIC_ID);
	if(IIC_ID != VI530x_IIC_DEV_ADDR)
	{
		ret |= VI530x_ERROR_IIC_ID;
		//Debug
	#ifdef Debug_Mode
		/* VI530x Device ID 为 0xD8, ID不对时不可通讯，请检测	*/
		printf("Check device ID is 0x%2x!\r\n",IIC_ID);
	#endif
	}

#ifdef Change_IIC_Dev_Addr	
	//更改IIC地址为新地址，调用前需要保证IIC总线上地址为VI530x_IIC_DEV_ADDR的器件释放IIC
	//多颗VI530x共IIC总线时需要分别使能对应Xshut引脚配置
	ret |= VI530x_Set_ModelChangeAddr(VI530x_IIC_DEV_ADDR2);			
#endif

	ret |= VI530x_Chip_Register_Init(&VI530x_Chip_ID);

	ret |= VI530x_Wait_For_CPU_Ready();

	return ret;
}



/**************************************************************************************************
******************************************  Range API *********************************************
**************************************************************************************************/

/**
 * @brief 	VI530X开始单次测距命令
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Start_Single_Ranging_Cmd(void)
{
	VI530x_Status ret = VI530x_OK;
	ret |= VI530x_Clear_Interrupt();
	ret |= Get_VI530x_Download_Firmware_Status();
	ret |= VI530x_Set_Digital_Clock_Dutycycle();

	ret |= VI530xWriteCommand(VAN_START_RANG_CMD);

	return ret;
}

/**
 * @brief 	VI530X开始连续测距命令
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Start_Continue_Ranging_Cmd(void)
{
	VI530x_Status ret = VI530x_OK;

	ret |= VI530x_Clear_Interrupt();
	ret |= Get_VI530x_Download_Firmware_Status();
	ret |= VI530x_Set_Digital_Clock_Dutycycle();

	ret |= VI530xWriteCommand(0x0F);

	return ret;
}

/**
 * @brief 	VI530X停止连续测距命令
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Stop_Continue_Ranging_Cmd(void)
{
	VI530x_Status ret = VI530x_OK;

	ret |= VI530x_Clear_Interrupt();
	ret |= VI530x_Set_Digital_Clock_Dutycycle();
	//VI530x_POWER_MANAGE == 1 => VI530x_STOP_MEASURE_SET == 1
	if(VI530x_Cali_Data.VI530x_Power_Manage_Status)
	{
		//开启电源管理模式
		ret |= VI530xWriteCommand(0x1F);
	}
	else
	{
		//关闭启电源管理模式
		ret |= VI530xWriteCommand(0X00);
	}	

	VI530x_Delay_Ms(10);
	return ret;
}

/**
 * @brief 	VI530X获取测距测距数据
 * @param 	[uint8_t] *data_buff:获取测距数据（包括校准的TOF、confidence）
 * @param 	[uint8_t] wait_mode:1-在一定时间内等待中断信号，0-没有中断信号则直接退出
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Get_Measure_Data(VI530x_MEASURE_TypeDef *measure_data, uint8_t wait_mode)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t Interrupt_status = 0;
	uint8_t data_buff[32] = {0};
	uint16_t time_out_cnt = 3000;
	//raw 参数
	int16_t raw_tof = 0;

	uint32_t intecounts = 0;
	uint32_t peak = 0;
	uint16_t noise = 0;	
	uint16_t xtalk_count = 0;
	int16_t ref_tof = 0;
	uint32_t ref_peak = 0;
	//计算参数
	float bias = 0;
	int16_t correction_tof = 0;	
	uint8_t confidence = 0;	

	while (time_out_cnt--)
	{
		ret |= VI530x_Get_And_Clear_Interrupt(&Interrupt_status);
		if (Interrupt_status)
		{
			ret |= VI530x_IIC_Read_X_Bytes(0x0C, data_buff, 32);
		#if 1
			/*** 主控为小端模式 ***/
			memcpy(&ref_peak,&data_buff[8],4);
			memcpy(&ref_tof,&data_buff[20],2);
			memcpy(&raw_tof,&data_buff[12],2);
			memcpy(&intecounts,&data_buff[22],4);
				intecounts = intecounts & 0x00FFFFFF;
			memcpy(&peak,&data_buff[28],4);
			memcpy(&noise,&data_buff[26],2);
			memcpy(&xtalk_count,&data_buff[14],2);
		#else
			/*** 主控为大端/小端模式 ***/
			ref_peak = (uint32_t)((((uint32_t)data_buff[11])<<24) |( ((uint32_t)data_buff[10])<<16)|(((uint32_t)data_buff[9])<<8)|( (uint32_t)data_buff[8]));
			ref_tof = (int16_t)((((uint16_t)data_buff[21])<<8) |(((uint16_t) data_buff[20])));
			raw_tof = (int16_t)((((uint16_t)data_buff[13])<<8) |(((uint16_t) data_buff[12])));
			intecounts = (uint32_t)((((uint32_t)data_buff[25])<<24) |( ((uint32_t)data_buff[24])<<16)|( ((uint32_t)data_buff[23])<<8)|( (uint32_t)data_buff[22]));
				intecounts = intecounts & 0x00FFFFFF;
			peak = (uint32_t)((((uint32_t)data_buff[31])<<24) |((uint32_t)data_buff[30]<<16)|((uint32_t)data_buff[29]<<8)|((uint32_t)data_buff[28]));
			noise = (uint16_t)((((uint16_t)data_buff[27])<<8) |(((uint16_t) data_buff[26])));
			xtalk_count = (uint16_t)((((uint16_t)data_buff[15])<<8) |(((uint16_t) data_buff[14])));
		#endif
			
			//校正
			bias = VI530x_V10_Calculate_Pileup_Bias(ref_peak, noise, intecounts);
			
		#ifdef VI530x_GRADIENTK_CALIBRATION
			//修正斜率和减去offset
			correction_tof  = VI530x_Cali_Data.VI530x_Calibration_GradientK*((float)raw_tof+bias) - VI530x_Cali_Data.VI530x_Calibration_Offset;
		#else
			correction_tof  = raw_tof  + bias - VI530x_Cali_Data.VI530x_Calibration_Offset;//减去offset	
		#endif
			if(correction_tof  < 0)
			{
				correction_tof  = 0;
			}
			//置信度
			confidence = VI530x_Calculate_Confidence(noise, peak, intecounts);
			if( (correction_tof  < 50)&& (peak < 800000)&& (confidence > 50) )
			{
				confidence = 50;
			}

			measure_data->intecounts = intecounts;
			measure_data->correction_tof = correction_tof;
			measure_data->confidence = confidence;
			measure_data->peak = peak;	
			measure_data->noise = noise;	
			measure_data->xtalk_count = xtalk_count;
			break;
		}
		else
		{
			if(wait_mode == 0)
			{
				ret |= VI530x_INTERRUPT_NONE;
				break;
			}
		}
		if (time_out_cnt == 0)
		{
			ret |= VI530x_ERROR_TIME_OUT;
		}
		VI530x_Delay_Ms(1);
	}

	//Debug
#ifdef Debug_Mode
	printf("tof = %2d, peak = %4d, noise = %4d, intecounts = %4d, confidence = %2d, \
	raw_tof = %2d, ref_tof = %2d, bias = %2f, cali_offset = %4f, ma_sum = %2d, xtalk_count = %2d, time_out_cnt = %2d, status = %d, ret = %d\r\n",
				correction_tof,peak,noise,intecounts,confidence,raw_tof,ref_tof,bias,VI530x_Cali_Data.VI530x_Calibration_Offset,
				VI530x_Cali_Data.VI530x_MA_Sum,xtalk_count,time_out_cnt,Interrupt_status,ret);
#endif

	return ret;
}



/**************************************************************************************************
******************************************  Calibration *******************************************
**************************************************************************************************/

/**
 * @brief xTalk 标定
 * @param *pbuff xtalk_pos(1 Byte) + xtalk_peak(2 Bytes) + xtalk_maxratio(1 Bytes)
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
*/
VI530x_Status VI530x_Xtalk_Calibration(void)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t status = 0;
	uint16_t time_out_cnt = 1000;
	uint8_t xtalk_buff[10] = {0};

	ret |= VI530x_Stop_Continue_Ranging_Cmd();
	//关闭温度校准:0x00-关，0x01-开
	ret |= VI530x_Set_Sys_Temperature_Enable(0x00);
	//Xtalk命令
	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x0D);
	VI530x_Delay_Ms(600);
	while (time_out_cnt--)
	{
		VI530x_Delay_Ms(10);
		ret |= VI530x_Get_And_Clear_Interrupt(&status);
		if (status)
		{
			VI530x_Delay_Ms(10);
			ret |= VI530x_IIC_Read_One_Byte(0x08, &status);
			if (status == 0xAA)
			{
				ret |= VI530x_IIC_Read_X_Bytes(0x0C, xtalk_buff, 5);
				//将xtalk_pos/xtalk_maxratio参数写到固件
				VI530x_Cali_Data.VI530x_Calibration_CG_Pos = xtalk_buff[0];
				VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio = xtalk_buff[3];
				VI530x_Cali_Data.VI530x_Calibration_CG_peak = (uint16_t)((((uint16_t)xtalk_buff[2])<<8) |(( (uint16_t)xtalk_buff[1])));
				ret |= VI530x_Set_Sys_Xtalk_Position(VI530x_Cali_Data.VI530x_Calibration_CG_Pos);
				ret |= VI530x_Set_Sys_CG_Maxratio(VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio);
				break;
			}
			else
			{
				ret |= VI530x_ERROR_XTALK_CALIB;
				break;
			}
		}
		if (time_out_cnt == 0)
		{
			ret |= VI530x_ERROR_TIME_OUT;
			return ret;
		}		
	}
	return ret;
}


/**
 * @brief RefToF 标定
 * @param 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
*/
VI530x_Status VI530x_Reftof_Calibration(void)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t interrupt_status = 0;
	// 采集数据计数
	uint8_t get_data_cnt = 0;
	uint8_t data_buff[2] = {0};
	uint16_t time_out_cnt = 0;
	//采样上下限
	uint8_t get_data_total_times = 20;
	uint8_t start_get_data_times = 10;

	//参数
	uint16_t ref_tof = 0;
	int32_t sum_reftof = 0;

	ret |= VI530x_Stop_Continue_Ranging_Cmd();
	//关闭温度校准:0x00-关
	ret |= VI530x_Set_Sys_Temperature_Enable(0x00);
	ret |= VI530x_Start_Continue_Ranging_Cmd();

	while(1)
	{
		VI530x_Delay_Ms(5);
		ret |= VI530x_Get_And_Clear_Interrupt(&interrupt_status);
		if (interrupt_status)
		{
			// 获取Reftof
			ret |= VI530x_IIC_Read_X_Bytes(0x20, data_buff, 2);
			time_out_cnt = 0;
			//memcpy(&ref_tof,&data_buff[0],2);	
			ref_tof = (uint16_t)((((uint16_t)data_buff[1])<<8) |(((uint16_t) data_buff[0])));			
			
			if(get_data_cnt > start_get_data_times)
			{
				sum_reftof += ref_tof;
			}			
			get_data_cnt++;
			
		//Debug_Mode
		#ifdef Debug_Mode
			printf("RefTof:ref_tof = %2d, cnt = %d\r\n",
					ref_tof,get_data_cnt);
		#endif
		}
		if(get_data_cnt > get_data_total_times)
		{
			VI530x_Cali_Data.VI530x_Calibration_Reftof = sum_reftof / (get_data_total_times-start_get_data_times);
			//设置RefToF标定值启用
			ret |= VI530x_Set_Sys_Reftof(VI530x_Cali_Data.VI530x_Calibration_Reftof);
			break;
		}
		time_out_cnt++;
		if(time_out_cnt > 200)
		{
			//超时异常
			ret |= VI530x_ERROR_TIME_OUT;
			break;
		}
	}
	ret |= VI530x_Stop_Continue_Ranging_Cmd();
	return ret;
}


/**
 * @brief Offset标定
 * @param mili	标定的位置
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Offset_Calibration(uint16_t mili)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t interrupt_status = 0;
	// 采集数据计数
	uint8_t get_data_cnt = 0;
	uint8_t data_buff[32] = {0};
	uint16_t time_out_cnt = 0;
	//采样上下限
	uint8_t get_data_total_times = 40;
	uint8_t start_get_data_times = 10;

	//参数
	uint16_t noise = 0;
	int16_t raw_tof = 0;
	uint32_t ref_peak = 0, peak1 = 0;
	uint32_t intecounts = 0;

	uint8_t confidence = 0;
	float bias = 0;
	float offset_mili = 0.0;
	int32_t sum_tof = 0, sum_bias = 0;

	ret |= VI530x_Stop_Continue_Ranging_Cmd();
	//关闭温度校准:0x00-关
	ret |= VI530x_Set_Sys_Temperature_Enable(0x00);
	//ret |= VI530x_Clear_Interrupt();
	ret |= VI530x_Start_Continue_Ranging_Cmd();

	while(1)
	{
		VI530x_Delay_Ms(5);
		ret |= VI530x_Get_And_Clear_Interrupt(&interrupt_status);
		if (interrupt_status)
		{
			// 获取tof
			ret |= VI530x_IIC_Read_X_Bytes(0x0C, data_buff, 32);
			time_out_cnt = 0;
		#if 1
			/*** 主控为小端模式 ***/
			memcpy(&ref_peak,&data_buff[8],4);
			memcpy(&raw_tof,&data_buff[12],2);
			memcpy(&intecounts,&data_buff[22],4);
				intecounts = intecounts & 0x00FFFFFF;
			memcpy(&peak1,&data_buff[28],4);
			memcpy(&noise,&data_buff[26],2);
			
		#else
			/*** 主控为大端/小端模式 ***/
			ref_peak = (uint32_t)((((uint32_t)data_buff[11])<<24) |( ((uint32_t)data_buff[10])<<16)|(((uint32_t)data_buff[9])<<8)|( (uint32_t)data_buff[8]));
			raw_tof = (int16_t)((((uint16_t)data_buff[13])<<8) |(((uint16_t) data_buff[12])));
			intecounts = (uint32_t)((((uint32_t)data_buff[25])<<24) |( ((uint32_t)data_buff[24])<<16)|( ((uint32_t)data_buff[23])<<8)|( (uint32_t)data_buff[22]));
				intecounts = intecounts & 0x00FFFFFF;
			peak1 = (uint32_t)((((uint32_t)data_buff[31])<<24) |((uint32_t)data_buff[30]<<16)|((uint32_t)data_buff[29]<<8)|((uint32_t)data_buff[28]));
			noise = (uint16_t)((((uint16_t)data_buff[27])<<8) |(((uint16_t) data_buff[26])));
		#endif
			
			bias = VI530x_V10_Calculate_Pileup_Bias(ref_peak,noise,intecounts);
			confidence = VI530x_Calculate_Confidence(noise, peak1, intecounts);
			if( confidence != 100)
			{	
				//输出异常
				ret |= VI530x_ERROR_OFFSET_CALIB;
				break;
			}		
			
			if(get_data_cnt > start_get_data_times)
			{
				sum_tof += raw_tof;
				sum_bias += bias;
			}			
			get_data_cnt++;
			
		//Debug_Mode
		#ifdef Debug_Mode
			printf("Offset:raw_tof = %2d, ref_peak = %4d, peak = %4d, noise = %4d, bias = %2f, intecounts = %4d, cnt = %d\r\n",
					raw_tof,ref_peak,peak1,noise,bias,intecounts,get_data_cnt);
		#endif
		}
		if(get_data_cnt > get_data_total_times)
		{
			offset_mili = (float)(sum_tof+sum_bias ) / (get_data_total_times-start_get_data_times) - mili;
			break;
		}
		time_out_cnt++;
		if(time_out_cnt > 200)
		{
			//超时异常
			ret |= VI530x_ERROR_TIME_OUT;
			break;
		}
	}
	ret |= VI530x_Stop_Continue_Ranging_Cmd();
	//Offset标定值赋值
	VI530x_Cali_Data.VI530x_Calibration_Offset = offset_mili;
	return ret;
}


/**
 * @brief Gradient标定
 * @param mili_offset	Offset标定的位置
 * @param mili_k	gradient标定的位置
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_GradientK_Calibration(uint16_t mili_offset, uint16_t mili_k)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t interrupt_status = 0;
	// 采集数据计数
	uint8_t get_data_cnt = 0;
	uint8_t data_buff[32] = {0};
	uint16_t time_out_cnt = 0;
	//采样上下限
	uint8_t get_data_total_times = 40;
	uint8_t start_get_data_times = 10;

	//参数
	uint16_t noise = 0;
	int16_t raw_tof = 0;
	uint32_t ref_peak = 0, peak1 = 0;
	uint32_t intecounts = 0;

	uint8_t confidence = 0;
	float bias = 0;
	float offset_mili = 0.0;
	float k_mili = 0.0;
	float slope_k = 1;
	int32_t sum_tof = 0, sum_bias = 0;
	
	if(mili_offset==mili_k)
	{
		return VI530x_ERROR;
	}

	ret |= VI530x_Stop_Continue_Ranging_Cmd();
	//关闭温度校准:0x00-关
	ret |= VI530x_Set_Sys_Temperature_Enable(0x00);
	//ret |= VI530x_Clear_Interrupt();
	ret |= VI530x_Start_Continue_Ranging_Cmd();

	while(1)
	{
		VI530x_Delay_Ms(5);
		ret |= VI530x_Get_And_Clear_Interrupt(&interrupt_status);
		if (interrupt_status)
		{
			// 获取tof
			ret |= VI530x_IIC_Read_X_Bytes(0x0C, data_buff, 32);
			time_out_cnt = 0;
		#if 1
			/*** 主控为小端模式 ***/
			memcpy(&ref_peak,&data_buff[8],4);
			memcpy(&raw_tof,&data_buff[12],2);
			memcpy(&intecounts,&data_buff[22],4);
				intecounts = intecounts & 0x00FFFFFF;
			memcpy(&peak1,&data_buff[28],4);
			memcpy(&noise,&data_buff[26],2);
			
		#else
			/*** 主控为大端/小端模式 ***/
			ref_peak = (uint32_t)((((uint32_t)data_buff[11])<<24) |( ((uint32_t)data_buff[10])<<16)|(((uint32_t)data_buff[9])<<8)|( (uint32_t)data_buff[8]));
			raw_tof = (int16_t)((((uint16_t)data_buff[13])<<8) |(((uint16_t) data_buff[12])));
			intecounts = (uint32_t)((((uint32_t)data_buff[25])<<24) |( ((uint32_t)data_buff[24])<<16)|( ((uint32_t)data_buff[23])<<8)|( (uint32_t)data_buff[22]));
				intecounts = intecounts & 0x00FFFFFF;
			peak1 = (uint32_t)((((uint32_t)data_buff[31])<<24) |((uint32_t)data_buff[30]<<16)|((uint32_t)data_buff[29]<<8)|((uint32_t)data_buff[28]));
			noise = (uint16_t)((((uint16_t)data_buff[27])<<8) |(((uint16_t) data_buff[26])));
		#endif
		
			bias = VI530x_V10_Calculate_Pileup_Bias(ref_peak,noise,intecounts);
			confidence = VI530x_Calculate_Confidence(noise, peak1, intecounts);
			if( confidence != 100)
			{	
				//输出异常
				ret |= VI530x_ERROR_OFFSET_CALIB;
				break;
			}		
			
			if(get_data_cnt > start_get_data_times)
			{
				sum_tof += raw_tof;
				sum_bias += bias;
			}			
			get_data_cnt++;
			
		//Debug_Mode
		#ifdef Debug_Mode
			printf("Offset:raw_tof = %2d, ref_peak = %4d, peak = %4d, noise = %4d, bias = %2f, intecounts = %4d, cnt = %d\r\n",
					raw_tof,ref_peak,peak1,noise,bias,intecounts,get_data_cnt);
		#endif
		}
		if(get_data_cnt > get_data_total_times)
		{
			
			k_mili = (float)(sum_tof+sum_bias ) / (get_data_total_times-start_get_data_times);
			if(mili_k > mili_offset)
			{
				//slope_k = (float)( k_mili-(mili_offset+VI530x_Cali_Data.VI530x_Calibration_Offset) ) / (mili_k-mili_offset);
				slope_k = (float)(mili_k-mili_offset)/( k_mili-(mili_offset+VI530x_Cali_Data.VI530x_Calibration_Offset) );
			}
			else
			{
				slope_k = (float)(mili_offset-mili_k)/( (mili_offset+VI530x_Cali_Data.VI530x_Calibration_Offset)-k_mili );
			}
			offset_mili = slope_k * k_mili - mili_k;
			break;
		}
		time_out_cnt++;
		if(time_out_cnt > 200)
		{
			//超时异常
			ret |= VI530x_ERROR_TIME_OUT;
			break;
		}
	}
	ret |= VI530x_Stop_Continue_Ranging_Cmd();
	//Offset标定值赋值
	VI530x_Cali_Data.VI530x_Calibration_Offset = offset_mili;
	VI530x_Cali_Data.VI530x_Calibration_GradientK = slope_k;
	return ret;
}

/**
 * @brief 	VI530X 下载完固件后设置标定数据
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Set_Californiation_Data(VI530x_Calibration_TypeDef calidata)
{
	VI530x_Status ret = VI530x_OK;
	//标定的pos写入固件
	ret |= VI530x_Set_Sys_Xtalk_Position(calidata.VI530x_Calibration_CG_Pos);
	//标定的maxratio写入固件
	ret |= VI530x_Set_Sys_CG_Maxratio(calidata.VI530x_Calibration_CG_Maxratio);
	//标定的Reftof写入固件
	if(calidata.VI530x_Calibration_Reftof != 0)
	{
		ret |= VI530x_Set_Sys_Reftof(calidata.VI530x_Calibration_Reftof);
	}
	else
	{
		ret |= VI530x_Set_Sys_Temperature_Enable(0x00);
	}
	//Offset标定值用于驱动层，不需要写入芯片
	//VI530x_Cali_Data.VI530x_Calibration_Offset = calidata.VI530x_Calibration_Offset;
	return ret;
}


/**************************************************************************************************
******************************************  Other *******************************************
**************************************************************************************************/
/**
 * @brief 	VI530X 唤醒
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Set_Digital_Clock_Dutycycle(void)
{
	VI530x_Status ret = VI530x_OK;
	ret |= VI530x_IIC_Write_One_Byte(VAN_REG_PW_CTRL, 0x0F);
	ret |= VI530x_IIC_Write_One_Byte(VAN_REG_PW_CTRL, 0x0E);
	VI530x_Delay_Ms(5);
	return ret;
}

/**
 * @brief 	VI530X 休眠
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Close_Digital_Clock_Dutycycle(void)
{
	VI530x_Status ret = VI530x_OK;
	ret |= VI530x_IIC_Write_One_Byte(VAN_REG_PW_CTRL, 0x0E);
	ret |= VI530x_IIC_Write_One_Byte(VAN_REG_PW_CTRL, 0x0F);
	VI530x_Delay_Ms(5);
	return ret;
}

/**
 * @brief 	VI530X Debug打印
 * @param 	[none] 
 * @return 	[uint8_t]	ret:0-操作成 ; other-异常
 */
VI530x_Status  VI530x_Print_All_Reg(void)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t value = 0;
	uint16_t i = 0;

	for (i = 0; i <= 0xFF; i++) 
	{
		ret |= VI530x_IIC_Read_One_Byte(i, &value);
		printf("Reg[%#x]=%#x\n", i, value);
	}
	return ret;
}


