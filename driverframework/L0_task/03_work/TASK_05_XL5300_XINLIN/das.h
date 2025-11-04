#ifndef __das_H_
#define __das_H_
#define XL5300_LONG_PARAM_LEN    70
typedef struct
{
	//5300ID值
	unsigned short int XL5300_Calibration_ID;	
	//XL5300电源管理设置
	//0x88：不开启电源管理，其他：开启电源管理
	unsigned  char XL5300_Power_Manage_Status;
	//XL5300中断状态设置
	//0x88：使用硬件中断,不使用寄存器中断，其他：不使用硬件中断,使用寄存器中断
	unsigned  char XL5300_Interrupt_Mode_Status;	
	//0x00: 芯片内部晶振，other-芯片外部晶振
	unsigned  char XL5300_Crystal_Mode_Status;
	//offset标定值
	float XL5300_Calibration_Offset;
	//XL5300 CG_Pos
	signed  char XL5300_Calibration_CG_Pos;		
		//XL5300 CG_Pos
	unsigned short int XL5300_Calibration_CG_peak;		
	//XL5300 CG_Maxratio
	 unsigned   char XL5300_Calibration_CG_Maxratio;		
	//XL5300 CK
	 unsigned   char XL5300_Calibration_CK;		
	//XL5300 MP
	 unsigned   char XL5300_Calibration_MP;		
	//XL5300 reftof标定值
	unsigned short int XL5300_Calibration_Reftof;		
	
	//保存Long_Peak1_Buff、Long_Peak2_Buff、Long_MeanTof_Buff、Long_MinPeak1_Buff、Long_MaxPeak1_Buff有效数据的个数 
	 unsigned   char Long_Param_Len;
	//斜率
	signed   int Long_K;
	//pileup校正参数
	signed   int Long_Pileup_A;
	signed   int Long_Pileup_B;
	signed   int Long_Pileup_C;
	
	//多段offset校正参数
	signed  int Long_Peak1_Buff[XL5300_LONG_PARAM_LEN];
	signed  int Long_Peak2_Buff[XL5300_LONG_PARAM_LEN];
	signed  int Long_MeanTof_Buff[XL5300_LONG_PARAM_LEN];
	signed  int Long_MinPeak1_Buff[XL5300_LONG_PARAM_LEN];
	signed  int Long_MaxPeak1_Buff[XL5300_LONG_PARAM_LEN];
	//Long_MaxPeak1_Buff中最大值
	signed  int MaxPeak;
	//MA系数之和
	 unsigned  char MA_Sum;
}XL5300_Calibration_TypeDef;

float VI530x_V10_Calculate_Pileup_Bias(unsigned int peak2, unsigned int noise, unsigned int integral_times);
unsigned  char VI530x_Calculate_Confidence( unsigned short noise, unsigned int peak1, unsigned  int integral_times);
unsigned  short  XL5300_Long_Calculate_Pileup_Bias(unsigned int peak, unsigned short int noise, unsigned int integral_times);
unsigned  char VI530x_Long_Calculate_Confidence(unsigned short  int noise, unsigned  int peak1);

#endif

