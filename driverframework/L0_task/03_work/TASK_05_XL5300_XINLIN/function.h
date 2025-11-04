#ifndef _function_h_
#define _function_h_
#include "main.h"
typedef struct
{
	//校正的tof
	int16_t correction_tof;		
	//置信度
	uint8_t confidence;		
	//积分次数
	uint32_t intecounts;
	//peak
	uint32_t peak;
	//Noise
	uint16_t noise;	
	//xtalk_count
	uint16_t xtalk_count;	
}XL5300_MEASURE_TypeDef;
//XL5300状态
typedef enum Result_Status
{
	Result_OK       = 0x00,
	Result_ERROR    = 0x90
}Result_Status;
uint8_t VI530x_Reftof_Calibration(void);
XL5300_Status XL5300_CK_Calibration(void);
XL5300_Status VI530x_MP_Calibration(void);
XL5300_Status VI530x_Xtalk_Calibration(void);
XL5300_Status VI530x_Offset_Calibration(uint16_t mili);
uint8_t XL5300_Get_Measure1_Data(XL5300_MEASURE_TypeDef *measure_data);
XL5300_Status VI530x_Set_Californiation_Data(float cali_offset);
extern uint8_t XL5300_GPIO_Interrupt_status;
extern XL5300_Calibration_TypeDef XL5300_Cali_Data;
void xtalk(void);
XL5300_Status VI530x_Start_Single_Ranging_Cmd(void);
uint8_t XL5300_Start_Continue_Ranging_Cmd(void);
void XL5300_Delay_Ms(uint16_t nMs);
#endif
