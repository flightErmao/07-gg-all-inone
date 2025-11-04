
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VI530x_ALGORITHM_H
#define __VI530x_ALGORITHM_H

/* Includes ------------------------------------------------------------------*/
#include "VI530x_User_Handle.h"


#define PILEUP_A (9231000)
#define PILEUP_B (4896)
#define PILEUP_C (1922)
#define PILEUP_D (10)



/**
 * @brief 	VI5300通用模式pileup校正
 * @param 	[uint32_t] peak2 ：测距输出的ref_peak
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] integral_times ：测距输出的积分次数
 * @return 	[uint32_t] bias:校正值,校正TOF = 原始TOF + bias - offset
 */
float VI530x_V10_Calculate_Pileup_Bias(uint32_t peak2, uint32_t noise, uint32_t integral_times);

/**
 * @brief 	VI5300通用模式confidence计算
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint32_t] integral_times ：测距输出的积分次数
 * @return 	[uint8_t] confidence
 */
uint8_t VI530x_Calculate_Confidence(uint16_t noise,uint32_t peak1,uint32_t integral_times);

uint8_t VI530x_Calculate_Xtalk_Ratio(int16_t tof, uint16_t count, uint32_t noise, uint8_t confidence, uint32_t peak);
VI530x_Status VI530x_Calibration_Xtalk_Ratio(uint8_t xtalk_ratio);


#endif 



