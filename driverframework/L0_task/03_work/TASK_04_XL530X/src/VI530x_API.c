#include "VI530x_API.h"
#include "VI530x_Algorithm.h"
#include "VI530x_Firmware.h"



// VI530x GPIO中断信号
// 0-清零/没有中断信号
// 1-有中断信号
uint8_t VI530x_GPIO_Interrupt_status = 0;

//VI530x IIC的设备地址上电默认8位地址为0xD8
uint8_t VI530x_IIC_Dev_Addr_Now = VI530x_IIC_DEV_ADDR;

//VI530x全局参数
VI530x_Params_T VI530x_Cali_Data;


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
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Chip_SWReset(void)
{
	VI530x_Status ret = VI530x_OK;
	ret |= VI530x_Close_Digital_Clock_Dutycycle();
	VI530x_IIC_Write_One_Byte(VI530x_REG_AO_DOMAIN, 0x90); //0x90写入后IIC会reset,无ACK返回
	VI530x_Delay_Ms(5);
	return ret;
}

/**
 * @brief 	获取VI530X芯片版本
 * @param 	[uint8_t] *Chip_Version获取的版本
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Read_ChipID(void)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t chipid[3] = {0};
	volatile uint32_t ChipID = 0;

	ret |= VI530x_IIC_Read_X_Bytes(VI530x_REG_CHIPID_BASE, chipid, 3);
	ChipID = (chipid[1] << 16) + (chipid[0] << 8) + chipid[2];

#ifdef Debug_Mode
	printf("VI530x chip ID is %#x\r\n",ChipID);
#endif
	return ret;
}

/**
 * @brief 	查询VI530X芯片busy等状态
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
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
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Clear_Interrupt(void)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t temp_status = 0;
	//硬件中断
	VI530x_GPIO_Interrupt_status = 0;
	//寄存器中断（软件中断）
	ret |= VI530x_IIC_Read_One_Byte(VI530x_RET_INT_STATUS, &temp_status);
	return ret;
}

/**
 * @brief 	VI530X中断 读取 & 清除
 * @param 	[uint8_t] *interrupt_status：获取中断状态
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Get_And_Clear_Interrupt(uint8_t *interrupt_status)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t temp_status = 0;

	//使用寄存器中断（软件中断）
	if(!VI530x_Cali_Data.VI530x_Interrupt_Mode_Status)
	{
		ret |= VI530x_IIC_Read_One_Byte(VI530x_RET_INT_STATUS, &temp_status);
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
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Set_ModelChangeAddr(uint8_t addr_val)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t IIC_ID = 0;
	//0x0c进入standby,0x08退出standby	,0x88允许更改VI530x的IIC 地址
	ret |= VI530x_IIC_Write_One_Byte(VI530x_REG_SYS_CFG,0x88);
	//更改设备地址，写入IIC寄存器
	ret |= VI530x_IIC_Write_One_Byte(VI530x_REG_IIC_DEV_ADDR,addr_val);
	if(ret == VI530x_OK)
	{
		//新地址赋值IIC地址全局变量
		VI530x_IIC_Dev_Addr_Now = addr_val;
	}

	ret |= VI530x_IIC_Write_One_Byte(VI530x_REG_SYS_CFG,0x08);
	VI530x_Delay_Ms(5);
	ret |= VI530x_IIC_Read_One_Byte(VI530x_REG_IIC_DEV_ADDR, &IIC_ID);
	if(IIC_ID != addr_val)
	{
		VI530x_IIC_Dev_Addr_Now = VI530x_IIC_DEV_ADDR;
		ret |= VI530x_ERROR_CONFIG;
	}
	return ret;
}

/**
 * @brief 	初始化VI530X
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Chip_Init(void)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t IIC_ID = 0;

	//配置电源管理模式：0----关闭电源管理模式，其他值----开启电源管理
	//开启后VI530x待机进入低功耗
	VI530x_Cali_Data.VI530x_Power_Manage_Status = 0x01; //开启电源管理

	ret |= VI530x_IIC_Read_One_Byte(VI530x_REG_IIC_DEV_ADDR, &IIC_ID);
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

	ret |= VI530x_Read_ChipID();

	// VI530x状态检测
	ret |= VI530x_Wait_For_CPU_Ready();

	return ret;
}



/**************************************************************************************************
******************************************  Range API *********************************************
**************************************************************************************************/

/**
 * @brief 	VI530X开始单次测距命令
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Start_Single_Ranging_Cmd(void)
{
	VI530x_Status ret = VI530x_OK;
	ret |= Get_VI530x_Download_Firmware_Status();
	ret |= VI530x_Clear_Interrupt();
	ret |= VI530x_Wait_For_CPU_Ready();

	ret |= VI530x_IIC_Write_One_Byte(VI530x_REG_CMD, VI530x_SINGLE_RANGE_CMD);

	return ret;
}

/**
 * @brief 	VI530X开始连续测距命令
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Start_Continue_Ranging_Cmd(void)
{
	VI530x_Status ret = VI530x_OK;
	ret |= VI530x_Wait_For_CPU_Ready();

	ret |= VI530x_Clear_Interrupt();
	ret |= VI530x_IIC_Write_One_Byte(VI530x_REG_CMD, VI530x_CONTINOUS_RANGE_CMD);
	return ret;
}

/**
 * @brief 	VI530X停止连续测距命令
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Stop_Continue_Ranging_Cmd(void)
{
	VI530x_Status ret = VI530x_OK;
	ret |= VI530x_Clear_Interrupt();
	ret |= VI530x_IIC_Write_One_Byte(VI530x_REG_CMD, VI530x_STOP_RANGE_CMD);
	VI530x_Delay_Ms(5);
	return ret;
}

/**
 * @brief 	VI530X获取测距测距数据
 * @param 	[VI530x_MEASURE_TypeDef] *result:获取测距数据（包括校准的TOF、confidence）
 * @param 	[uint8_t] wait_mode:1-在一定时间内等待中断信号，0-没有中断信号则直接退出
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Get_Measure_Data(VI530x_MEASURE_TypeDef *result, uint8_t wait_mode)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t Interrupt_status = 0;
	uint8_t i = 0;
	uint8_t data_buff[32] = {0};
	uint16_t time_out_cnt = 2000;
	int16_t xtalk_tof[3] = {11,23,33};
	uint32_t xtalk_threshold[3] = {500,140,230};
	float offset1 = 100;
	float re_th = 3;
	float peak_th = 0;
	float upper_co = 0.13;
	float lower_co = 0.07;
	//raw 参数
	uint32_t noise = 0;
	uint32_t intecounts = 0;
//	uint32_t peak1 = 0, peak2 = 0, peak = 0;
//	int16_t tof1 = 0, tof2 = 0, ori_tof = 0;
	int16_t ref_tof = 0;
	uint16_t xtalk_count = 0;
	uint8_t tof1_bin = 0, tof2_bin = 0;
	int16_t tof[3] = {0,0,0};
	int16_t reftof = 0;
	uint32_t peak[3] = {0,0,0};
	uint32_t ac[3] = {0,0,0};
	int8_t ts = 0;
	//计算参数
	int16_t raw_tof = 0;
	uint32_t raw_peak = 0;
	uint32_t raw_ac = 0;
	float bias = 0;
	float reflectivity[3] = {0,0,0};
	int16_t correction_tof = 0;
	float confidence = 0;
	uint32_t peak_r1 = 0;
	

	while(time_out_cnt--)
	{
		ret |= VI530x_Get_And_Clear_Interrupt(&Interrupt_status);
		if (Interrupt_status)
		{
			ret |= VI530x_IIC_Read_X_Bytes(0x0C, data_buff, 32);
			memcpy(&tof[0], &data_buff[1], 2);
			memcpy(&tof[1], &data_buff[3], 2);
			memcpy(&tof[2], &data_buff[5], 2);
			memcpy(&peak[0], &data_buff[7], 4);
			memcpy(&peak[1], &data_buff[10], 4);
			memcpy(&peak[2], &data_buff[13], 4);
			memcpy(&ac[0], &data_buff[16], 4);
			memcpy(&ac[1], &data_buff[19], 4);
			memcpy(&ac[2], &data_buff[22], 4);
			memcpy(&intecounts, &data_buff[26], 4);
			memcpy(&noise, &data_buff[29], 4);
			intecounts = intecounts & 0x00FFFFFF;
			noise = noise & 0x00FFFFFF;
			//reftof = (data_buff[25] << 8) + data_buff[0];
			ts = data_buff[25];

			peak[0] = (peak[0] & 0x00FFFFFF) << 8;
			peak[1] = (peak[1] & 0x00FFFFFF) << 8;
			peak[2] = (peak[2] & 0x00FFFFFF) << 8;
			ac[0] = (ac[0] & 0x00FFFFFF) << 8;
			ac[1] = (ac[1] & 0x00FFFFFF) << 8;
			ac[2] = (ac[2] & 0x00FFFFFF) << 8;

			for (i = 0; i < 3; ++i)
			{
						float a = (peak[i] - peak_th) / intecounts;
						float b = (tof[i] + offset1) * (tof[i] + offset1);
						reflectivity[i] = (a * b) / 100000;
			}
			if (reflectivity[0] >= re_th)
			{
						raw_tof = tof[0];
						raw_peak = peak[0];
						raw_ac = ac[0];
			}
			else if (reflectivity[0] < re_th && reflectivity[1] >= re_th)
			{
						raw_tof = tof[1];
						raw_peak = peak[1];
						raw_ac = ac[1];
			}
			else if (reflectivity[0] < re_th && reflectivity[1] < re_th && reflectivity[2] >= re_th)
			{
						raw_tof = tof[2];
						raw_peak = peak[2];
						raw_ac = ac[2];
			}
			else if (reflectivity[0] < re_th && reflectivity[1] < re_th && reflectivity[2] < re_th)
			{
						raw_tof = 0;
						raw_peak = 0;
						raw_ac = 0;
			}
			confidence = (321000 * (float)raw_peak / intecounts - lower_co * raw_ac) * 100 / (upper_co * raw_ac + 1 - lower_co * raw_ac);
			if (confidence > 100)
				confidence = 100;
			if (confidence < 0)
				confidence = 0;
			bias = VI530x_Calculate_Pileup_Bias_V40_LR(VI530x_Cali_Data.VI530X_MA_Sum, raw_peak, noise, intecounts);
			correction_tof = raw_tof + (int16_t)bias - VI530x_Cali_Data.VI530x_Cali_Offset;
					//输出
					result->intecounts = intecounts;
					result->correction_tof = correction_tof;
					result->confidence = (uint8_t)confidence;
					result->peak = raw_peak;
					result->noise = noise;	
					result->xtalk_count = xtalk_count;
					result->ts = ts;
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
	printf("correction_tof = %2d, peak = %4d, noise = %4d, intecounts = %4d, confidence = %2f, ret = %d, \
ref_tof = %2d, bias = %2f, \
cali_offset = %4f, ma_sum = %2d, xtalk_count = %2d, time_out_cnt = %2d, Interstatus = %d, \r\n",
				correction_tof,raw_peak,noise,intecounts,confidence,ret,ref_tof,bias,VI530x_Cali_Data.VI530x_Cali_Offset,
				VI530x_Cali_Data.VI530X_MA_Sum,xtalk_count,time_out_cnt,Interrupt_status);
#endif

	return ret;
}



/**************************************************************************************************
******************************************  Calibration *******************************************
**************************************************************************************************/

/**
 * @brief xTalk 标定
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Xtalk_Calibration(void)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t status = 0;
	uint32_t time_out_cnt = 100;
	uint8_t xtalk_buff[10] = {0};

	ret |= VI530x_Stop_Continue_Ranging_Cmd();
	//关闭温度校准:0x00-关，0x01-开
	ret |= VI530x_Set_Sys_Temperature_Enable(0x00);
	//Xtalk命令
	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x0D);
	VI530x_Delay_Ms(600);			//Xtalk标定时间

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
				//将xtalk参数写到固件
				VI530x_Cali_Data.VI530x_Cali_CG_Pos = xtalk_buff[0];
				VI530x_Cali_Data.VI530x_Cali_CG_Maxratio = xtalk_buff[3];
				VI530x_Cali_Data.VI530x_Cali_CG_Peak = (uint16_t)((((uint16_t)xtalk_buff[2])<<8) |(( (uint16_t)xtalk_buff[1])));
				ret |= VI530x_IIC_Read_One_Byte(0x1d, &VI530x_Cali_Data.VI530x_Cali_CG_Bin);
				ret |= VI530x_Set_Sys_Xtalk_Position(VI530x_Cali_Data.VI530x_Cali_CG_Pos);
				ret |= VI530x_Set_Sys_Xtalk_Maxratio(VI530x_Cali_Data.VI530x_Cali_CG_Maxratio);
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
 * @brief 	Offset标定
 * @param 	mili	标定的位置
 * @return	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Offset_Calibration(float mili)
{
	VI530x_Status ret = VI530x_OK;
	// 采集数据计数
	uint8_t get_data_cnt = 0;
	uint8_t get_data_num = 20;			//算均值数量,越在越平稳
	uint8_t i = 0;
	uint8_t data_buff[32] = {0};
	uint8_t interrupt_status = 0;
	uint16_t time_out_cnt = 0;
	float offset1 = 100;
	float re_th = 3;
	float peak_th = 0;
	float upper_co = 0.13;
	float lower_co = 0.07;
	//raw 参数
	uint32_t noise = 0;
	uint32_t intecounts = 0;
//	uint32_t peak1 = 0, peak2 = 0, peak = 0;
//	int16_t tof1 = 0, tof2 = 0, ori_tof = 0;
	uint8_t tof1_bin = 0, tof2_bin = 0;
	int16_t tof[3] = {0,0,0};
	int16_t reftof = 0;
	uint32_t peak[3] = {0,0,0};
	uint32_t ac[3] = {0,0,0};
	
	//计算参数
	int16_t raw_tof = 0;
	uint32_t raw_peak = 0;
	uint32_t raw_ac = 0;
	float reflectivity[3] = {0,0,0};
	float sum_tof = 0, sum_bias = 0;
	float bias = 0;
	float confidence = 0;
	float offset_mili = 0.0;
	
	ret |= VI530x_Stop_Continue_Ranging_Cmd();
	//开启温度校准:0x00-关，0x01-开
	//ret |= VI530x_Set_Sys_Temperature_Enable(0x00);
	ret |= VI530x_Set_Sys_Temperature_Enable(0x01);

	ret |= VI530x_Start_Continue_Ranging_Cmd();
	while(1)
	{
		VI530x_Delay_Ms(1);
		ret |= VI530x_Get_And_Clear_Interrupt(&interrupt_status);
		if (interrupt_status)
		{
			time_out_cnt = 0;
			// 获取tof
			ret |= VI530x_IIC_Read_X_Bytes(0x0C, data_buff, 32);

      memcpy(&tof[0], &data_buff[1], 2);
      memcpy(&tof[1], &data_buff[3], 2);
      memcpy(&tof[2], &data_buff[5], 2);
      memcpy(&peak[0], &data_buff[7], 4);
      memcpy(&peak[1], &data_buff[10], 4);
      memcpy(&peak[2], &data_buff[13], 4);
      memcpy(&ac[0], &data_buff[16], 4);
      memcpy(&ac[1], &data_buff[19], 4);
      memcpy(&ac[2], &data_buff[22], 4);
      memcpy(&intecounts, &data_buff[26], 4);
      memcpy(&noise, &data_buff[29], 4);
      intecounts = intecounts & 0x00FFFFFF;
      noise = noise & 0x00FFFFFF;

      peak[0] = (peak[0] & 0x00FFFFFF) << 8;
      peak[1] = (peak[1] & 0x00FFFFFF) << 8;
      peak[2] = (peak[2] & 0x00FFFFFF) << 8;
      ac[0] = (ac[0] & 0x00FFFFFF) << 8;
      ac[1] = (ac[1] & 0x00FFFFFF) << 8;
      ac[2] = (ac[2] & 0x00FFFFFF) << 8;

      for (i = 0; i < 3; ++i)
      {
          float a = (peak[i] - peak_th) / intecounts;
          float b = (tof[i] + offset1) * (tof[i] + offset1);
          reflectivity[i] = (a * b) / 100000;
      }
      if (reflectivity[0] >= re_th)
      {
          raw_tof = tof[0];
          raw_peak = peak[0];
          raw_ac = ac[0];
      }
      else if (reflectivity[0] < re_th && reflectivity[1] >= re_th)
      {
          raw_tof = tof[1];
          raw_peak = peak[1];
          raw_ac = ac[1];
      }
      else if (reflectivity[0] < re_th && reflectivity[1] < re_th && reflectivity[2] >= re_th)
      {
          raw_tof = tof[2];
          raw_peak = peak[2];
          raw_ac = ac[2];
      }
      else if (reflectivity[0] < re_th && reflectivity[1] < re_th && reflectivity[2] < re_th)
      {
          raw_tof = 0;
          raw_peak = 0;
          raw_ac = 0;
      }
      confidence = (321000 * (float)raw_peak / intecounts - lower_co * raw_ac) * 100 / (upper_co * raw_ac + 1 - lower_co * raw_ac);
      if (confidence > 100)
          confidence = 100;
      if (confidence < 0)
          confidence = 0;
      bias = VI530x_Calculate_Pileup_Bias_V40_LR(VI530x_Cali_Data.VI530X_MA_Sum, raw_peak, noise, intecounts);

				
			if(get_data_cnt > 9)
			{
        sum_tof += raw_tof;
				sum_bias += bias;
			}
			
			get_data_cnt++;
			//Debug_Mode
			#ifdef Debug_Mode
				printf("Offset:raw_tof = %2d, peak = %4d, noise = %4d, bias = %2f, intecounts = %4d, confidence = %2f, cnt = %d\r\n",
						raw_tof,raw_peak,noise,bias,intecounts,confidence,get_data_cnt);
			#endif
			if( confidence != 100)
			{	
				//输出异常
				ret |= VI530x_ERROR_OFFSET_CALIB;
				break;
			}		
		}
		if(get_data_cnt > (9+get_data_num) )
		{
			offset_mili = (float)sum_tof / get_data_num + (float)sum_bias / get_data_num - mili;
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
	VI530x_Cali_Data.VI530x_Cali_Offset = offset_mili;
	return ret;
}

/**
 * @brief Gradient标定
 * @param mili_offset	Offset标定的位置
 * @param mili_k	gradient标定的位置
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_GradientK_Calibration(float mili_offset, float mili_k)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t interrupt_status = 0;
	// 采集数据计数
	uint8_t get_data_cnt = 0;
	uint8_t get_data_num = 20;			//算均值数量,越在越平稳
	uint8_t i = 0;
	uint8_t data_buff[32] = {0};
	uint16_t time_out_cnt = 0;
	float offset1 = 100;
	float re_th = 3;
	float peak_th = 0;
	float upper_co = 0.13;
	float lower_co = 0.07;

	//raw 参数
	uint32_t noise = 0;
	uint32_t intecounts = 0;
//	uint32_t peak1 = 0, peak2 = 0, peak = 0;
//	int16_t tof1 = 0, tof2 = 0, ori_tof = 0;
	uint8_t tof1_bin = 0, tof2_bin = 0;
	int16_t tof[3] = {0,0,0};
	int16_t reftof = 0;
	uint32_t peak[3] = {0,0,0};
	uint32_t ac[3] = {0,0,0};
	
	//计算参数
	int16_t raw_tof = 0;
	uint32_t raw_peak = 0;
	uint32_t raw_ac = 0;
	float reflectivity[3] = {0,0,0};
	float sum_tof = 0, sum_bias = 0;
	float bias = 0;
	float confidence = 0;
	float offset_mili = 0.0;
	float k_mili = 0.0;
	float slope_k = 1;
	
	if(mili_offset==mili_k)
	{
		ret |= VI530x_Stop_Continue_Ranging_Cmd();
		//Offset标定值赋值
		VI530x_Cali_Data.VI530x_Cali_GradientK = slope_k;
		return ret;
	}

	ret |= VI530x_Stop_Continue_Ranging_Cmd();
	//开启温度校准:0x00-关，0x01-开
	//ret |= VI530x_Set_Sys_Temperature_Enable(0x00);
	ret |= VI530x_Set_Sys_Temperature_Enable(0x01);

	ret |= VI530x_Start_Continue_Ranging_Cmd();
	while(1)
	{
		VI530x_Delay_Ms(1);
		ret |= VI530x_Get_And_Clear_Interrupt(&interrupt_status);
		if (interrupt_status)
		{
			time_out_cnt = 0;
			// 获取tof
			ret |= VI530x_IIC_Read_X_Bytes(0x0C, data_buff, 32);

      memcpy(&tof[0], &data_buff[1], 2);
      memcpy(&tof[1], &data_buff[3], 2);
      memcpy(&tof[2], &data_buff[5], 2);
      memcpy(&peak[0], &data_buff[7], 4);
      memcpy(&peak[1], &data_buff[10], 4);
      memcpy(&peak[2], &data_buff[13], 4);
      memcpy(&ac[0], &data_buff[16], 4);
      memcpy(&ac[1], &data_buff[19], 4);
      memcpy(&ac[2], &data_buff[22], 4);
      memcpy(&intecounts, &data_buff[26], 4);
      memcpy(&noise, &data_buff[29], 4);
      intecounts = intecounts & 0x00FFFFFF;
      noise = noise & 0x00FFFFFF;

      peak[0] = (peak[0] & 0x00FFFFFF) << 8;
      peak[1] = (peak[1] & 0x00FFFFFF) << 8;
      peak[2] = (peak[2] & 0x00FFFFFF) << 8;
      ac[0] = (ac[0] & 0x00FFFFFF) << 8;
      ac[1] = (ac[1] & 0x00FFFFFF) << 8;
      ac[2] = (ac[2] & 0x00FFFFFF) << 8;

      for (i = 0; i < 3; ++i)
      {
          float a = (peak[i] - peak_th) / intecounts;
          float b = (tof[i] + offset1) * (tof[i] + offset1);
          reflectivity[i] = (a * b) / 100000;
      }
      if (reflectivity[0] >= re_th)
      {
          raw_tof = tof[0];
          raw_peak = peak[0];
          raw_ac = ac[0];
      }
      else if (reflectivity[0] < re_th && reflectivity[1] >= re_th)
      {
          raw_tof = tof[1];
          raw_peak = peak[1];
          raw_ac = ac[1];
      }
      else if (reflectivity[0] < re_th && reflectivity[1] < re_th && reflectivity[2] >= re_th)
      {
          raw_tof = tof[2];
          raw_peak = peak[2];
          raw_ac = ac[2];
      }
      else if (reflectivity[0] < re_th && reflectivity[1] < re_th && reflectivity[2] < re_th)
      {
          raw_tof = 0;
          raw_peak = 0;
          raw_ac = 0;
      }
      confidence = (321000 * (float)raw_peak / intecounts - lower_co * raw_ac) * 100 / (upper_co * raw_ac + 1 - lower_co * raw_ac);
      if (confidence > 100)
          confidence = 100;
      if (confidence < 0)
          confidence = 0;
      bias = VI530x_Calculate_Pileup_Bias_V40_LR(VI530x_Cali_Data.VI530X_MA_Sum, raw_peak, noise, intecounts);

				
			if(get_data_cnt > 9)
			{
        sum_tof += raw_tof;
				sum_bias += bias;
			}
			
			get_data_cnt++;
			//Debug_Mode
			#ifdef Debug_Mode
				printf("Offset:raw_tof = %2d, peak = %4d, noise = %4d, bias = %2f, intecounts = %4d, confidence = %2f, cnt = %d\r\n",
						raw_tof,raw_peak,noise,bias,intecounts,confidence,get_data_cnt);
			#endif
			if( confidence != 100)
			{	
				//输出异常
				ret |= VI530x_ERROR_OFFSET_CALIB;
				break;
			}		
		}
		if(get_data_cnt > (9+get_data_num) )
		{
			
			k_mili = (float)sum_tof / get_data_num + (float)sum_bias / get_data_num;
			if(mili_k > mili_offset)
			{
				//slope_k = (float)( k_mili-(mili_offset+VI530x_Cali_Data.VI530x_Calibration_Offset) ) / (mili_k-mili_offset);
				slope_k = (float)(mili_k-mili_offset)/( k_mili-(mili_offset+VI530x_Cali_Data.VI530x_Cali_Offset) );
			}
			else
			{
				slope_k = (float)(mili_offset-mili_k)/( (mili_offset+VI530x_Cali_Data.VI530x_Cali_Offset)-k_mili );
			}
			offset_mili = slope_k * k_mili - mili_k;
			break;
		}
		time_out_cnt++;
		if(time_out_cnt > 1000)
		{
			//超时异常
			ret |= VI530x_ERROR_TIME_OUT;
			break;
		}
	}
	ret |= VI530x_Stop_Continue_Ranging_Cmd();
	//Offset标定值赋值
	VI530x_Cali_Data.VI530x_Cali_Offset = offset_mili;
	VI530x_Cali_Data.VI530x_Cali_GradientK = slope_k;
	return ret;
}

/**
 * @brief 	VI530X设置 xtalk_position
 * @param 	[uint8_t] xtalk_position:设置cg标定的position
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Set_Sys_Xtalk_Position(uint8_t xtalk_position)
{
	VI530x_Status ret = VI530x_OK;

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x26);
	ret |= VI530x_IIC_Write_One_Byte(0x0F, xtalk_position);
	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	VI530x_Delay_Ms(5);
	return ret;
}

/**
 * @brief 	VI530X读取 xtalk_position
 * @param 	[uint8_t] *xtalk_position:读取cg标定的position
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Sys_Xtalk_Position(uint8_t *xtalk_position)
{
	VI530x_Status ret = VI530x_OK;

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x26);
	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	VI530x_Delay_Ms(5);
	ret |= VI530x_IIC_Read_One_Byte(0x0C, xtalk_position);

	return ret;
}

/**
 * @brief 	VI530X设置 Maxratio
 * @param 	[uint8_t] Maxratio:设置cg标定的Maxratio
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Sys_Xtalk_Maxratio(uint8_t maxratio)
{
	VI530x_Status ret = VI530x_OK;

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x25);
	ret |= VI530x_IIC_Write_One_Byte(0x0F, maxratio);
	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);

	VI530x_Delay_Ms(5);

	return ret;
}

/**
 * @brief 	VI530X读取 Maxratio
 * @param 	[uint8_t] Maxratio:读取cg标定的Maxratio
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Sys_Xtalk_Maxratio(uint8_t *maxratio)
{
	VI530x_Status ret = VI530x_OK;

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x25);

	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);

	VI530x_Delay_Ms(5);
	ret |= VI530x_IIC_Read_One_Byte(0x0C, maxratio);

	return ret;
}

/**
 * @brief 	VI530X 下载完固件后设置标定数据
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Californiation_Data(float cali_offset)
{
	VI530x_Status ret = VI530x_OK;
	//标定的pos写入固件
	ret |= VI530x_Set_Sys_Xtalk_Position(VI530x_Cali_Data.VI530x_Cali_CG_Pos);
	//标定的maxratio写入固件
	ret |= VI530x_Set_Sys_Xtalk_Maxratio(VI530x_Cali_Data.VI530x_Cali_CG_Maxratio);
	//Offset标定值用于驱动层，不需要写入芯片
	VI530x_Cali_Data.VI530x_Cali_Offset = cali_offset;
	//以下CG值需要从主控NVM存储并赋值，用于驱动层，不需要写入芯片
//	VI530x_Cali_Data.VI530x_Cali_CG_Bin = 0;
//	VI530x_Cali_Data.VI530x_Cali_CG_Peak = 1000;
	return ret;
}





/**************************************************************************************************
********************************************  OTP  Read  *******************************************
**************************************************************************************************/
/**
* @brief 	VI530X 读取otp数据（连续读取的长度不超29byte）
 * @param 	[uint8_t] base：读取的起始地址
 * @param 	[uint8_t] *read_buff：标定得到的reftof值
 * @param 	[uint8_t] len：读取的长度，不超29byte
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_OTP_Read_Less_Than_29Bytes(uint8_t base, uint8_t *read_buff, uint8_t len)
{
	VI530x_Status ret = VI530x_OK;
	if (len > 29 || len == 0)
	{
		return VI530x_ERROR;
	}

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x03);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, len);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, base);
	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);

	VI530x_Delay_Ms(100);

	ret |= VI530x_IIC_Read_X_Bytes(0x0F, read_buff, len);

	return ret;
}

/**
 * @brief 	VI530X 读取otp数据（连续读取的长度不限制）
 * @param 	[uint8_t] base：读取的起始地址
 * @param 	[uint8_t] *read_buff：标定得到的reftof值
 * @param 	[uint8_t] len：读取的长度
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_OTP_Read_More_Than_29Bytes(uint8_t base, uint8_t *read_buff, uint8_t len)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t i = 0;

	while (len > 29)
	{
		ret |= VI530x_OTP_Read_Less_Than_29Bytes(base + i * 29, &read_buff[i * 29], 29);

		len -= 29;
		i++;
	}
	if (len > 0)
	{
		ret |= VI530x_OTP_Read_Less_Than_29Bytes(base + i * 29, &read_buff[i * 29], len);
	}
	return ret;
}

/**************************************************************************************************
******************************************  Other *******************************************
**************************************************************************************************/

/**
 * @brief 	VI530X 唤醒（VI530x使用，vincent一般不使用）
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Digital_Clock_Dutycycle(void)
{
	VI530x_Status ret = VI530x_OK;

	ret |= VI530x_IIC_Write_One_Byte(VI530x_REG_PW_CTRL, 0x0F);
	ret |= VI530x_IIC_Write_One_Byte(VI530x_REG_PW_CTRL, 0x0E);
	VI530x_Delay_Ms(4); // 默认4ms ，延时不准可以增加

	return ret;
}

/**
 * @brief 	VI530X 休眠（VI530x使用，vincent一般不使用）
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Close_Digital_Clock_Dutycycle(void)
{
	VI530x_Status ret = VI530x_OK;
	ret |= VI530x_IIC_Write_One_Byte(VI530x_REG_PW_CTRL, 0x0E);
	ret |= VI530x_IIC_Write_One_Byte(VI530x_REG_PW_CTRL, 0x0F);
	VI530x_Delay_Ms(5);
	return ret;
}

/**
 * @brief 	VI530X 设置系统参数
 * @param 	[uint8_t] offset_addr：起始地址
 * @param 	[uint8_t] *buff：缓存空间
 * @param 	[uint8_t] len：设置的长度
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Write_System_Data(uint8_t offset_addr, uint8_t *buff, uint8_t len)
{
  VI530x_Status ret = VI530x_OK;

	//寄存器0x0C写(0x00-读，0x01写)
  ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);
  ret |= VI530x_IIC_Write_One_Byte(0x0D, len);
  ret |= VI530x_IIC_Write_One_Byte(0x0E, offset_addr);
  ret |= VI530x_IIC_Write_X_Bytes(0x0F, buff, len);
  ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);

  VI530x_Delay_Ms(5);
  return ret;
}


/**
 * @brief 	VI530X 读取系统参数
 * @param 	[uint8_t] offset_addr：起始地址
 * @param 	[uint8_t] *buff：缓存空间
 * @param 	[uint8_t] len：读取的长度
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Read_System_Data(uint8_t offset_addr, uint8_t *buff, uint8_t len)
{
  VI530x_Status ret = VI530x_OK;

  //寄存器0x0C写(0x00-读，0x01写)
  ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
  ret |= VI530x_IIC_Write_One_Byte(0x0D, len);
  ret |= VI530x_IIC_Write_One_Byte(0x0E, offset_addr);
  ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);

  VI530x_Delay_Ms(5);
  ret |= VI530x_IIC_Read_X_Bytes(0x0C, buff, len);

  return ret;
}


/**
 * @brief 	VI530X设置积分次数
 * @param 	[uint32_t] inte_counts:设置的积分次数
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Sys_Integral_Time(uint32_t inte_counts)
{
	VI530x_Status ret = VI530x_OK;

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x03);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x14);

	// 小端模式，从小到大保存
	ret |= VI530x_IIC_Write_One_Byte(0x0F, (inte_counts)&0xFF);
	ret |= VI530x_IIC_Write_One_Byte(0x10, (inte_counts >> 8) & 0xFF);
	ret |= VI530x_IIC_Write_One_Byte(0x11, (inte_counts >> 16) & 0xFF);

	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	VI530x_Delay_Ms(5);
	return ret;
}

/**
 * @brief 	VI530X获取积分次数
 * @param 	[uint32_t] *inte_counts:读取到的积分次数
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Sys_Integral_Time(uint32_t *inte_counts)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t buff[3] = {0};

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x03);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x14);
	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	VI530x_Delay_Ms(5);
	ret |= VI530x_IIC_Read_X_Bytes(0x0C, buff, 3); // 小端模式
	*inte_counts = (buff[2] << 16) + (buff[1] << 8) + (buff[0] << 0);

	return ret;
}

/**
 * @brief 	VI530X设置delay_count(影响测距帧率)
 * @param 	[uint16_t] delay_count:设置的delay_count
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Delay_Count_Write(uint16_t delay_count)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t buf[2] = {0};

	// 大端模式
	buf[0] = (delay_count >> 8) & 0xFF;
	buf[1] = (delay_count)&0xFF;

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x02);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x17);
	ret |= VI530x_IIC_Write_One_Byte(0x0F, buf[1]); // 小端模式
	ret |= VI530x_IIC_Write_One_Byte(0x10, buf[0]);

	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	VI530x_Delay_Ms(5);
	return ret;
}

/**
 * @brief 	VI530X读取delay_count(影响测距帧率)
 * @param 	[uint16_t] *delay_count:读取到的delay_count
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Delay_Count_Read(uint16_t *delay_count)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t buff[2] = {0};

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x02);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x17);
	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	VI530x_Delay_Ms(5);
	ret |= VI530x_IIC_Read_X_Bytes(0x0C, buff, 2); // 小端模式
	*delay_count = (buff[1] << 8) + buff[0];

	return ret;
}

/**
 * @brief 	VI530X设置 积分次数 和 帧率
 * @param 	[uint8_t] fps:测距的帧率
 * @param 	[uint32_t] intecoutns:设置的积分次数
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Integralcounts_Frame(uint8_t fps, uint32_t intecoutns)
{
	VI530x_Status ret = VI530x_OK;
	//uint32_t integral_time = 0;
	uint32_t inte_time = 0;
	uint32_t fps_time = 0;
	int32_t delay_time = 0;
	uint16_t delay_counts = 0;

	inte_time = intecoutns * 1463 / 10;
	fps_time = 1000000000 / fps;
	delay_time = fps_time - inte_time - 4920000;
	//delay_time = fps_time - inte_time - 2040000;
	if( delay_time <= 0 )
	{
		delay_counts = 1;
	}
	else
	{
		delay_counts = (uint16_t)(delay_time / 3400);
	}

	ret = VI530x_Set_Sys_Integral_Time(intecoutns);
	ret |= VI530x_Delay_Count_Write(delay_counts);

	return ret;
}

/**
 * @brief 	VI530X设置 帧率
 * @param 	[uint32_t] period:测距的帧率
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Sys_FPS(uint32_t period)
{
	VI530x_Status ret = VI530x_OK;
	uint32_t integral_time = 0;
	uint32_t inte_time = 0;
	uint32_t fps_time = 0;
	uint32_t delay_time = 0;
	uint16_t delay_counts = 0;

	if(period == 0)
	{
		return VI530x_ERROR;
	}

	ret |= VI530x_Get_Sys_Integral_Time(&integral_time);
	inte_time = integral_time * 1463 / 10;
	fps_time = 1000000000 / period;
	delay_time = fps_time - inte_time - 4920000;
	delay_counts = (uint16_t)(delay_time / 3400);

	ret |= VI530x_Delay_Count_Write(delay_counts);

	return ret;
}

/**
 * @brief 	VI530X 设置温补
 * @param 	[uint8_t] status: 0-关温补，1-开温补
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Sys_Temperature_Enable(uint8_t status)
{
	VI530x_Status ret = VI530x_OK;

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x24);
	ret |= VI530x_IIC_Write_One_Byte(0x0F, status);
	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	VI530x_Delay_Ms(5);
	return ret;
}

/**
 * @brief 	VI530X 读取温补开关状态
 * @param 	[uint8_t] *status：读取的状态: 0-关温补，1-开温补
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Sys_Temperature_Enable(uint8_t *status)
{
	VI530x_Status ret = VI530x_OK;
	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x24);
	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	VI530x_Delay_Ms(5);
	ret |= VI530x_IIC_Read_One_Byte(0x0C, status);

	return ret;
}

/**
 * @brief 	VI530X读取 MA系数、MA系数之和
 * @param 	[uint8_t] *getting_buff:读取MA系数
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Sys_Histogram_MA_Window_Data(uint8_t *getting_buff)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t i = 0;
	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x08);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x1A);

	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	VI530x_Delay_Ms(5);
	ret |= VI530x_IIC_Read_X_Bytes(0x0C, getting_buff, 8);
	getting_buff[8] = 0;
	for (i = 0; i < 8; i++)
	{
		// MA系数之和
		getting_buff[8] += ((getting_buff[i] & 0x0F) + ((getting_buff[i] >> 4) & 0x0F));
	}

	return ret;
}

/**
 * @brief 	VI530X 设置SPAD通道
 * @param 	[uint8_t] channel: 1~9
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Sys_SAPD_Channel(uint8_t channel)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t spad_channel = 1;
	if( (channel > 9) || (channel == 0) )
		return VI530x_ERROR;
	spad_channel = channel;
	ret |= VI530x_Write_System_Data(0x39,&spad_channel,1);
	ret |= VI530x_Read_System_Data(0x39,&spad_channel,1);
	if(spad_channel != channel)
	{
		ret |= VI530x_ERROR_CONFIG;
	#ifdef Debug_Mode
		printf("VI530x SPAD Channel setup fail| %#x\r\n",spad_channel);
	#endif	
	}
	else
	{
		VI530x_Cali_Data.VI530x_SPAD_CHANNEL = channel;
	}
	return ret;
}

/**
 * @brief 	VI530X 读取SPAD通道
 * @param 	[uint8_t] channel: 1~9
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Sys_SAPD_Channel(uint8_t *channel)
{
	VI530x_Status ret = VI530x_OK;
	ret = VI530x_Read_System_Data(0x39,channel,1);
	return ret;
}

VI530x_Status VI530x_Get_FW_Version(uint8_t *rVersion)
{
	VI530x_Status ret = VI530x_OK;

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x06);
	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x04);
	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	VI530x_Delay_Ms(5);
	ret |= VI530x_IIC_Read_X_Bytes(0x0C, rVersion, 4);

	return ret;
}

/**
 * @brief 	VI530X SN打印
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_SN_Number(void)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t SN_buffer[7] = {0};
	uint8_t i = 0;

	ret |= VI530x_OTP_Read_Less_Than_29Bytes(0x23, SN_buffer, 7);
	printf("SN: 0x");
	for( i=0; i<7; i++ )
	{
		printf("%02x", SN_buffer[i]);
	}
	printf("\n");
	return ret;
}

/**
 * @brief 	VI530X Debug打印
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
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


