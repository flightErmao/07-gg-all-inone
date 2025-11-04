
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VI530x_ALGORITHM_H
#define __VI530x_ALGORITHM_H			  	 

/* Includes ------------------------------------------------------------------*/
#include "VI530x_User_Handle.h"


/**
 * @brief 	Short Range 模式 pileup校正
 * @param 	[uint16_t] vi530x_ma_sum ：MA系数之和
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] integral_times ：测距输出的积分次数
 * @return 	[float]	bias:校正值,校正TOF = 原始TOF + bias - offset
 */
float VI530x_Calculate_Pileup_Bias_V40_YQ(uint16_t VI530x_ma_sum,uint32_t peak1, uint32_t noise,uint32_t integral_times);

/**
 * @brief 	Short Range 模式 confidence计算
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint16_t] vi530x_ma_sum ：MA系数之和
 * @return 	[uint8_t] confidence
 */
uint8_t VI530x_Calculate_Confidence_V40_YQ(uint32_t noise, uint32_t peak1, uint16_t VI530x_ma_sum);

/**
 * @brief 	Long Range 模式 pileup校正
 * @param 	[uint16_t] vi530x_ma_sum ：MA系数之和
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] integral_times ：测距输出的积分次数
 * @return 	[float] bias:校正值,校正TOF = 原始TOF + bias - offset
 */
float VI530x_Calculate_Pileup_Bias_V40_LR(uint16_t vi530x_ma_sum,uint32_t peak1, uint32_t noise,uint32_t integral_times);

/**
 * @brief 	Long Range 模式 confidence1计算
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint32_t] integral_times ：测距输出的积分次数
 * @return 	[uint8_t] confidence
 */
uint8_t VI530x_Calculate_Confidence1_V40_LR(uint32_t noise, uint32_t peak, uint32_t integral_times);

/**
 * @brief 	Long Range 模式 confidence2计算
 * @param 	[int16_t] tof ：测距输出的tof
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint32_t] integral_times ：测距输出的积分次数
 * @param 	[uint16_t] vi530x_ma_sum ：MA系数之和
 * @return 	[uint8_t] confidence
 */
uint8_t VI530x_Calculate_Confidence2_V40_LR(int16_t tof, uint32_t noise, uint32_t peak, uint16_t vi530x_ma_sum);

/**
 * @brief 	校验和计算
 * @param 	[uint8_t] *buff ：数据缓存区
 * @param 	[uint8_t] len ：数据长度
 * @return 	[uint8_t] checksum
 */
uint16_t VI530x_Calculate_CheckSum(uint8_t *buff, uint8_t len);

/**
 * @brief 	Peak归一化
 * @param 	[uint16_t] vi530x_ma_sum ：MA系数之和
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] integral_times ：测距输出的积分次数
 * @return 	[uint32_t] peak_r：归一化值
 */
uint32_t VI530x_Calculate_Normalization_Peak(uint16_t VI530x_ma_sum, uint32_t peak, uint32_t noise, uint32_t integral_times);
 
uint8_t VI530x_Calculate_Xtalk_Ratio(int16_t tof, uint16_t count, uint32_t noise, uint8_t confidence, uint32_t peak);
VI530x_Status VI530x_Calibration_Xtalk_Ratio(uint8_t xtalk_ratio);


#endif  
