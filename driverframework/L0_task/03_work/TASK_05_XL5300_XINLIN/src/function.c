#include "function.h"
#include "XL5300_UserPlatform.h"
#include "XL5300_API.h"
#include <string.h>
#include "das.h"
extern XL5300_Calibration_TypeDef XL5300_Cali_Data;
XL5300_MEASURE_TypeDef Measure_Data;
uint8_t data_buff[10];
uint8_t ret = 0;
void XL5300_Delay_Ms(uint16_t nMs)
{
		HAL_Delay(nMs);
}
uint8_t XL5300_Get_And_Clear_Interrupt(uint8_t *interrupt_status)
{
	uint8_t ret = 0, temp_status = 0;
	//使用寄存器中断（软件中断）
	if(!XL5300_Cali_Data.XL5300_Interrupt_Mode_Status)
	{
		ret |=ReadOneReg(XL5300_RET_INT_STATUS, &temp_status);
	}
	if(XL5300_GPIO_Interrupt_status || (temp_status & 0x01))
	{
		*interrupt_status = 0x01;
		XL5300_GPIO_Interrupt_status = 0;
	}
	else
	{
		*interrupt_status = 0x00;
	}
	return ret;
}
uint8_t XL5300_Set_Sys_CK(uint8_t ck)
{
    uint8_t ret = 0;

    ret |= XL5300_Set_Digital_Clock_Dutycycle();
	ret |= WriteOneReg(0x0C, 0x01);
	ret |= WriteOneReg(0x0D, 0x01);
	ret |= WriteOneReg(0x0E, 0x1B);
	ret |= WriteOneReg(0x0F, ck);
	ret |= WriteOneReg(0x0A, 0x09);
	HAL_Delay(5);
    return ret;
}

XL5300_Status XL5300_CK_Calibration(void)
{
	XL5300_Status ret = XL5300_OK;
	uint8_t phase_buff[2] = {0x00};
	uint8_t interrupt_status = 0;
	uint16_t timeout = 0;
	timeout = 300;

	ret |= XL5300_Temp_Enable(0x00);
	ret |= XL5300_Set_Digital_Clock_Dutycycle();
	ret |= XL5300_Clear_Interrupt();

	ret |= WriteOneReg(0x0A, 0x0B);
	while (timeout--)
	{
		XL5300_Delay_Ms(10);
		ret |= XL5300_Get_And_Clear_Interrupt(&interrupt_status);
		if (interrupt_status)
		{
			XL5300_Delay_Ms(10);
			ret |= ReadOneReg(0x0D, phase_buff + 1);
			if (phase_buff[1] == 0xBB)
			{
					ret |= ReadOneReg(0x0C, phase_buff);
					XL5300_Cali_Data.XL5300_Calibration_CK = phase_buff[0];
					XL5300_Set_Sys_CK(XL5300_Cali_Data.XL5300_Calibration_CK);
			}
			else
			{
				ret |= XL5300_ERROR_XTALK_CALIB;
			}
			break;
		}
	}
	if (timeout == 0)
	{
		ret |= XL5300_ERROR_XTALK_CALIB;
	}		
	return ret;
}
uint8_t XL5300_Set_Sys_Xtalk_Position(uint8_t xtalk_position)
{
    uint8_t ret = 0;

    ret |= XL5300_Set_Digital_Clock_Dutycycle();	
	ret |= WriteOneReg(0x0C, 0x01);
	ret |= WriteOneReg(0x0D, 0x01);
	ret |= WriteOneReg(0x0E, 0x00);
	ret |= WriteOneReg(0x0F,  xtalk_position);
	ret |= WriteOneReg(0x0A, 0x09);
	XL5300_Delay_Ms(5);
    return ret;
}
uint8_t XL5300_Set_Sys_CG_Maxratio(uint8_t maxratio)
{
    uint8_t ret = 0;

	ret |= XL5300_Set_Digital_Clock_Dutycycle();
	ret |= WriteOneReg(0x0C, 0x01);
	ret |= WriteOneReg(0x0D, 0x01);
	ret |= WriteOneReg(0x0E, 0x1A);
	ret |= WriteOneReg(0x0F,  maxratio);
	ret |= WriteOneReg(0x0A, 0x09);
    XL5300_Delay_Ms(5);
    return ret;
}
uint8_t XL5300WriteCommand(uint8_t cmd)
{
	return WriteOneReg(VAN_REG_CMD, cmd);
}

XL5300_Status VI530x_Start_Single_Ranging_Cmd(void)
{
	XL5300_Status ret = XL5300_OK;
	ret |= XL5300_Clear_Interrupt();
	ret |= Get_XL5300_Download_Firmware_Status();
	ret |= XL5300_Set_Digital_Clock_Dutycycle();
	ret |= XL5300WriteCommand(0x0e);

	return ret;
}

uint8_t XL5300_Start_Continue_Ranging_Cmd(void)
{
	uint8_t ret = 0;
	ret |= XL5300_Clear_Interrupt();
	ret |= Get_XL5300_Download_Firmware_Status();
	ret |= XL5300_Set_Digital_Clock_Dutycycle();

	ret |= XL5300WriteCommand(0x0F);

	return ret;
}
/**
 * @brief 	VI530X停止连续测距命令
 * @param 	[none] 
 * @return 	[uint8_t]	ret:0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */
uint8_t XL5300_Stop_Continue_Ranging_Cmd(void)
{
	uint8_t ret = 0;

	ret |= XL5300_Clear_Interrupt();
	ret |= XL5300_Set_Digital_Clock_Dutycycle();
	//XL5300_POWER_MANAGE == 1 => XL5300_STOP_MEASURE_SET == 1
	if(XL5300_Cali_Data.XL5300_Power_Manage_Status)
	{
		//开启电源管理模式
		ret |= XL5300WriteCommand(0x1F);
	}
	else
	{
		//关闭启电源管理模式
		ret |= XL5300WriteCommand(0X00);
	}	
	
	XL5300_Delay_Ms(10);
	return ret;
}
uint8_t XL5300_Set_Sys_Reftof(uint16_t reftof)
{
    uint8_t ret = 0;
	
        ret |= XL5300_Set_Digital_Clock_Dutycycle();
		ret |= WriteOneReg(0x0C, 0x01);
		ret |= WriteOneReg(0x0D, 2);
	    ret |= WriteOneReg(0x0E, 0x17);
		ret |= WriteOneReg(0x0F,  ((reftof >> 8)& 0xFF));
		ret |= WriteOneReg(0x10,(reftof & 0xFF));
		ret |= WriteOneReg(0x0A, 0x09);
		XL5300_Delay_Ms(5);
    return ret;
}
/**
* @brief xTalk 标定
* @param *pbuff xtalk_pos(1 Byte) + xtalk_peak(2 Bytes) + xtalk_maxratio(1 Bytes)
* @return
*/
XL5300_Status VI530x_Xtalk_Calibration(void)
{
	XL5300_Status ret = XL5300_OK;
	uint8_t status = 0;
	uint16_t time_out_cnt = 1000;
	uint8_t xtalk_buff[10] = {0};

	//关闭温度校准:0x00-关
	ret |= XL5300_Temp_Enable(0x00);
	ret |= XL5300_Clear_Interrupt();
	ret |= XL5300_Set_Digital_Clock_Dutycycle();
	//Xtalk命令
	ret |= WriteOneReg(0x0A, 0x0D);

	while (time_out_cnt--)
	{
		XL5300_Delay_Ms(10);
		ret |= XL5300_Get_And_Clear_Interrupt(&status);
		if (status)
		{
			XL5300_Delay_Ms(10);
			ret |= ReadOneReg(0x08, &status);
			if (status == 0xAA)
			{
				ret |= I2C_ReadXBytes(0x0C, xtalk_buff, 5);
				//将xtalk_pos/xtalk_maxratio参数写到固件
				XL5300_Cali_Data.XL5300_Calibration_CG_Pos = xtalk_buff[0];
				XL5300_Cali_Data.XL5300_Calibration_CG_Maxratio = xtalk_buff[3];
				XL5300_Cali_Data.XL5300_Calibration_CG_peak = (uint16_t)((((uint16_t)xtalk_buff[2])<<8) |(( (uint16_t)xtalk_buff[1])));
				ret |= XL5300_Set_Sys_Xtalk_Position(XL5300_Cali_Data.XL5300_Calibration_CG_Pos);
				ret |= XL5300_Set_Sys_CG_Maxratio(XL5300_Cali_Data.XL5300_Calibration_CG_Maxratio);
				break;
			}
			else
			{
				ret |= XL5300_ERROR_XTALK_CALIB;
				return ret;
			}
		}
		if (time_out_cnt == 0)
		{
			ret |= XL5300_ERROR_XTALK_CALIB;
			return ret;
		}		
	}
	return ret;
}
/**
* @brief RefToF 标定
* @param 
* @return
*/
uint8_t VI530x_Reftof_Calibration(void)
{
	XL5300_Status ret = XL5300_OK;
	uint8_t interrupt_status = 0;
	// 采集数据计数
	uint8_t get_data_cnt = 0;
	uint8_t databuff[2] = {0};
	uint16_t time_out_cnt = 0;
	//采样上下限
	uint8_t get_data_total_times = 20;
	uint8_t start_get_data_times = 10;

	//参数
	uint16_t ref_tof = 0;
	int32_t sum_reftof = 0;

	//关闭温度校准:0x00-关
	ret |= XL5300_Temp_Enable(0x00);
	ret |= XL5300_Clear_Interrupt();
	ret |= XL5300_Start_Continue_Ranging_Cmd();

	while(1)
	{
		XL5300_Delay_Ms(5);
		ret |= XL5300_Get_And_Clear_Interrupt(&interrupt_status);
		if (interrupt_status)
		{
			// 获取Reftof
			ret |= I2C_ReadXBytes(0x20, databuff, 2);
			time_out_cnt = 0;
			memcpy(&ref_tof,&databuff[0],2);		
			
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
			XL5300_Cali_Data.XL5300_Calibration_Reftof = sum_reftof / (get_data_total_times-start_get_data_times);
			//设置RefToF标定值启用
			ret |= XL5300_Set_Sys_Reftof(XL5300_Cali_Data.XL5300_Calibration_Reftof);
			break;
		}
		time_out_cnt++;
		if(time_out_cnt > 200)
		{
			//超时异常
			ret |= XL5300_ERROR;
			break;
		}
	}
	ret |= XL5300_Stop_Continue_Ranging_Cmd();
	return ret;
}
uint8_t XL5300_Write_System_Data(uint8_t offset_addr, uint8_t *buff, uint8_t len)
{
    uint8_t ret = 0;
    ret |= XL5300_Set_Digital_Clock_Dutycycle();
    ret |= WriteOneReg(0x0C, 0x01);
	ret |= WriteOneReg(0x0D,  len);
	ret |= WriteOneReg(0x0E,  offset_addr);
	ret |= I2C_WriteXBytes(0x0F, buff, len);
	ret |= WriteOneReg(0x0A, 0x09);
	XL5300_Delay_Ms(5);
    return ret;
}
uint8_t XL5300_Get_Measure1_Data(XL5300_MEASURE_TypeDef *measure_data)
{
	XL5300_Status ret = XL5300_OK;
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
		ret |= XL5300_Get_And_Clear_Interrupt(&Interrupt_status);
		if (Interrupt_status)
		{
			ret |= I2C_ReadXBytes(0x0C, data_buff, 32);
			memcpy(&ref_peak,&data_buff[8],4);
			memcpy(&ref_tof,&data_buff[20],2);
			memcpy(&raw_tof,&data_buff[12],2);
			memcpy(&intecounts,&data_buff[22],4);
			intecounts = intecounts & 0x00FFFFFF;
			memcpy(&peak,&data_buff[28],4);
			memcpy(&noise,&data_buff[26],2);

			memcpy(&xtalk_count,&data_buff[14],2);
			//校正
			bias = XL5300_Long_Calculate_Pileup_Bias(peak, noise, intecounts); // 长距方案pileup校正
			correction_tof= raw_tof  + bias - XL5300_Cali_Data.XL5300_Calibration_Offset;
			if(correction_tof  < 0)
			{
				correction_tof  = 0;
			}
			confidence = VI530x_Long_Calculate_Confidence(noise, peak);		 
			measure_data->intecounts = intecounts;
			measure_data->correction_tof = correction_tof;
			measure_data->confidence = confidence;
			measure_data->peak = peak;	
			measure_data->noise = noise;	
			measure_data->xtalk_count = xtalk_count;
			break;
		}
		if (time_out_cnt == 0)
		{
			ret |= XL5300_ERROR;
		}
		XL5300_Delay_Ms(1);
	}
	//Debug
#ifdef Debug_Mode
	printf("raw_tof = %2d, peak = %4d, ref_tof = %2d, ref_peak = %4d, noise = %4d, bias = %2f, cali_offset = %4f, intecounts = %4d, xtalk_count = %2d\r\n",
				raw_tof,peak,ref_tof,ref_peak,noise,bias,
				 XL5300_Cali_Data.XL5300_Calibration_Offset,intecounts,xtalk_count);
#endif
	return ret;
}
uint8_t XL5300_Set_Sys_MP(uint8_t mp_area)
{
    uint8_t ret = 0;
    ret |= XL5300_Set_Digital_Clock_Dutycycle();
	ret |= WriteOneReg(0x0C, 0x01);
	ret |= WriteOneReg(0x0D, 0x01);
	ret |= WriteOneReg(0x0E, 0x14);
	ret |= WriteOneReg(0x0F,  mp_area);
	ret |= WriteOneReg(0x0A, 0x09);
    XL5300_Delay_Ms(5);

    return ret;
}

XL5300_Status VI530x_MP_Calibration(void)
{
//MP标定
	XL5300_Status ret = XL5300_OK;
	uint8_t spad_cnt = 0,spad_mode = 0;
	uint8_t j = 0;
	//uint8_t xtalk_buff[10] = {0};
	uint32_t peak_sum_3x3_buff[4] = {0};


	XL5300_MEASURE_TypeDef Measure_Data_l;
	uint8_t cali_MP = 0;
	volatile uint32_t total_peak = 0;
	volatile int32_t total_tof = 0;

	//关闭温度校准:0x00-关
	ret |= XL5300_Temp_Enable(0x00);
	ret |= XL5300_Clear_Interrupt();
	memset(peak_sum_3x3_buff, 0, 4 * 4);
//	memset(get_data_buff, 0, 129);

	for (spad_cnt = 0; spad_cnt < 16; spad_cnt++)
	{
		XL5300_Set_Digital_Clock_Dutycycle();
		spad_mode = 0x80 + spad_cnt;
		ret |= XL5300_Write_System_Data(0x14, &spad_mode, 1);
		ret |= XL5300_Start_Continue_Ranging_Cmd();

		for (j = 0; j < 7; j++)
		{
			ret |= XL5300_Get_Measure1_Data(&Measure_Data_l);
			if(ret)
			{
				ret |= XL5300_ERROR_XTALK_CALIB;
				ret |= XL5300_Stop_Continue_Ranging_Cmd();
				return ret;
			}
			if (j >= 3)
			{
					total_peak += Measure_Data_l.peak;
					total_tof += Measure_Data_l.correction_tof;
			}
		}
		ret |= XL5300_Stop_Continue_Ranging_Cmd();
		// tof peak取平均
		total_tof = (total_tof >> 2);
		total_peak = (total_peak >> 2);
		if (spad_cnt == 0x00 || spad_cnt == 0x01 || spad_cnt == 0x02 ||
				spad_cnt == 0x04 || spad_cnt == 0x05 || spad_cnt == 0x06 ||
				spad_cnt == 0x08 || spad_cnt == 0x09 || spad_cnt == 0x0A)
		{
			peak_sum_3x3_buff[0] += total_peak;
		}
		if (spad_cnt == 0x01 || spad_cnt == 0x02 || spad_cnt == 0x03 ||
				spad_cnt == 0x05 || spad_cnt == 0x06 || spad_cnt == 0x07 ||
				spad_cnt == 0x09 || spad_cnt == 0x0A || spad_cnt == 0x0B)
		{
			peak_sum_3x3_buff[1] += total_peak;
		}
		if (spad_cnt == 0x04 || spad_cnt == 0x05 || spad_cnt == 0x06 ||
				spad_cnt == 0x08 || spad_cnt == 0x09 || spad_cnt == 0x0A ||
				spad_cnt == 0x0C || spad_cnt == 0x0D || spad_cnt == 0x0E)
		{
			peak_sum_3x3_buff[2] += total_peak;
		}
		if (spad_cnt == 0x07 || spad_cnt == 0x05 || spad_cnt == 0x06 ||
				spad_cnt == 0x0B || spad_cnt == 0x09 || spad_cnt == 0x0A ||
				spad_cnt == 0x0F || spad_cnt == 0x0D || spad_cnt == 0x0E)
		{
			peak_sum_3x3_buff[3] += total_peak;
		}
		total_tof = 0;
		total_peak = 0;
	}

	//计算MP peak最大的区域
	for (j = 0; j < 4; j++)
	{
		if (peak_sum_3x3_buff[j] >= total_peak)
		{
			total_peak = peak_sum_3x3_buff[j];
			cali_MP = j;
		}
	}

	XL5300_Cali_Data.XL5300_Calibration_MP = cali_MP;
	ret |= XL5300_Set_Sys_MP(XL5300_Cali_Data.XL5300_Calibration_MP);
	return ret;
}

/**
 * @brief Offset标定
 * @param mili	标定的位置
 * @param *pbuff	标定结果：(int32) 单位0.1mm
 * @return
 */
XL5300_Status VI530x_Offset_Calibration(uint16_t mili)
{
	XL5300_Status ret = XL5300_OK;
	uint8_t interrupt_status = 0;
	// 采集数据计数
	uint8_t get_data_cnt = 0;
	uint8_t databuff[32] = {0};
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

	//关闭温度校准:0x00-关
	ret |= XL5300_Temp_Enable(0x00);
	ret |= XL5300_Clear_Interrupt();
	ret |= XL5300_Start_Continue_Ranging_Cmd();

	while(1)
	{
		XL5300_Delay_Ms(5);
		ret |= XL5300_Get_And_Clear_Interrupt(&interrupt_status);
		if (interrupt_status)
		{
			// 获取tof 
			ret |= I2C_ReadXBytes(0x0C, databuff, 32);
			time_out_cnt = 0;
			memcpy(&intecounts,&databuff[22],4);
			intecounts = intecounts & 0x00FFFFFF;

			memcpy(&noise,&databuff[26],2);
			memcpy(&peak1,&databuff[28],4);
			memcpy(&ref_peak,&databuff[8],4);
			memcpy(&raw_tof,&databuff[12],2);		
			bias = VI530x_V10_Calculate_Pileup_Bias(ref_peak,noise,intecounts);
		    confidence = VI530x_Calculate_Confidence(noise, peak1, intecounts);
			if( confidence != 100)
			{	
				//输出异常
				ret |= XL5300_ERROR_OFFSET_CALIB;
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
			ret |= XL5300_ERROR_OFFSET_CALIB;
			break;
		}
	}
	ret |= XL5300_Stop_Continue_Ranging_Cmd();
	//Offset标定值赋值
	XL5300_Cali_Data.XL5300_Calibration_Offset = offset_mili;
	return ret;
}
XL5300_Status XL5300_Set_Sys_TDC_Phase(uint8_t phase)
{
	XL5300_Status ret = XL5300_OK;
	ret |= XL5300_Set_Digital_Clock_Dutycycle();

	ret |= WriteOneReg(0x0C, 0x01);
	ret |= WriteOneReg(0x0D, 0x01);
	ret |= WriteOneReg(0x0E, 0x1B);
	ret |= WriteOneReg(0x0F, phase);

	ret |= WriteOneReg(0x0A, 0x09);
	XL5300_Delay_Ms(5);
	return ret;
}
/**
 * @brief 	VI530X ????????????
 * @param 	[none] 
 * @return 	[uint8_t]	ret:0-????(I2C?????);other-??(I2C?????)
 */
XL5300_Status VI530x_Set_Californiation_Data(float cali_offset)
{
	XL5300_Status ret = XL5300_OK;
	ret |= XL5300_Set_Sys_TDC_Phase(XL5300_Cali_Data.XL5300_Calibration_CK);
	ret |= XL5300_Set_Sys_MP(XL5300_Cali_Data.XL5300_Calibration_MP);
	//???pos????
	ret |= XL5300_Set_Sys_Xtalk_Position(XL5300_Cali_Data.XL5300_Calibration_CG_Pos);
	//???maxratio????
	ret |= XL5300_Set_Sys_CG_Maxratio(XL5300_Cali_Data.XL5300_Calibration_CG_Maxratio);
	//???Reftof????
	ret |= XL5300_Set_Sys_Reftof(XL5300_Cali_Data.XL5300_Calibration_Reftof);
	//Offset????????,???????
	XL5300_Cali_Data.XL5300_Calibration_Offset = cali_offset;
	return ret;
}
uint8_t ref_tof_flag=1;
void xtalk(void)
{
	/*
	注意：模组只需做一次标定即可，之后将标定的值保存
	VI530x_Cali_Data.VI530x_Calibration_CG_Pos			//一般值在区域-10~+10
	VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio 	//一般值在区域 0~+10
	VI530x_Cali_Data.VI530x_Calibration_Reftof			//一般值在区域 300~720
	VI530x_Cali_Data.VI530x_Calibration_Offset 			//一般值在区域 -50~+50
	*/
	uint8_t ret=0;
		/************************* 生产标定流程 *****************************/
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)==0)
	{
		HAL_Delay(5);
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)==0)
		{
	//CK标定
	ret |= XL5300_CK_Calibration();  
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
		ret |= XL5300_Temp_Enable(0x01);
		ref_tof_flag=0;
		printf("XL5300_Calibration_Reftof = %4d\r\n",XL5300_Cali_Data.XL5300_Calibration_Reftof);
	}
	else
	{
		ref_tof_flag=1;
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
	ret |= XL5300_Temp_Enable(0x01);
	ret |= XL5300_Start_Continue_Ranging_Cmd();	//连续模式
    }
  }
}
