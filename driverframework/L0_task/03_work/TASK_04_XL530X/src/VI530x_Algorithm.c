#include "VI530x_Algorithm.h"
#include "VI530x_API.h"


/**
 * @brief 	Short Range 模式 pileup校正
 * @param 	[uint16_t] vi530x_ma_sum ：MA系数之和
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] integral_times ：测距输出的积分次数
 * @return 	[float] bias:校正值,校正TOF = 原始TOF + bias - offset
 */
float VI530x_Calculate_Pileup_Bias_V40_YQ(uint16_t VI530x_ma_sum,uint32_t peak1, uint32_t noise, uint32_t integral_times)
{
	//********pileup
	float peak_tmp = 0;
	const uint16_t xth[] = {0,122,415,757,1155,1444,1749,2000};
	const uint16_t pth[] = {0,0,4,9,18,29,43,60};

	uint8_t i = 0;
	float bias = 0;
	uint8_t buff_len = 0;
	
	buff_len = sizeof(xth) / sizeof(xth[0]);
	noise = noise / 8;
	if(integral_times == 0)
	{
		return bias;
	}
	if(peak1 > (noise * VI530x_ma_sum))
		peak_tmp = (peak1 - noise * VI530x_ma_sum) * 16 / integral_times;
	else
		peak_tmp = 0;

	for(i = 0;i < buff_len  - 1;i++)
	{
		if(peak_tmp < xth[i+1])
		{
			bias = (pth[i+1] - pth[i])*( peak_tmp -xth[i])/(xth[i+1]-xth[i])+ pth[i];
			return bias;
		}    
	}
	
	if(peak_tmp >= xth[i])
	{
		bias = (pth[i]-pth[i-1])*( peak_tmp - xth[i-1])/(xth[i]-xth[i-1])+pth[i-1];
	}
	
	return bias;
}


/**
 * @brief 	Short Range 模式 confidence计算
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint16_t] vi530x_ma_sum ：MA系数之和
 * @return 	[uint8_t] confidence
 */
uint8_t VI530x_Calculate_Confidence_V40_YQ(uint32_t noise, uint32_t peak1, uint16_t VI530x_ma_sum)
{
	int32_t Confidence_A,Confidence_B,Confidence_C,Confidence_D;
	int32_t cal_noise;
	uint32_t Upper,Lower;
	uint8_t confidence = 0;
	
	double temp = 0;
	
	if(noise < 200)
	{
		Confidence_A = -475;
		Confidence_B = 57;
		Confidence_C = 1959;
		Confidence_D = 10;
	}
	else if(noise < 5000)
	{
		Confidence_A = -60410;
		Confidence_B = 1432;
		Confidence_C = 390;
		Confidence_D = 50;
	}
	else
	{
		Confidence_A = -1125000;
		Confidence_B = 10640;
		Confidence_C = 155;
		Confidence_D = 131;
	}
	
	temp = (Confidence_A / (int32_t)(noise + Confidence_B));
	cal_noise = ((6592 * (temp + ((Confidence_C * noise) >> 16) + Confidence_D)) / 181)  + VI530x_ma_sum;

	Upper = ((VI530x_ma_sum * noise) >> 3) + 6 * cal_noise;
	Lower = ((VI530x_ma_sum * noise) >> 3) + 4 * cal_noise ;

	if(peak1 > Upper)
	{
		confidence = 100;
	}
	else if(peak1 < Lower)
	{
		confidence = 0;
	}
	else
	{
		confidence = 100 * (peak1 - Lower) / (Upper - Lower);
	}

	return confidence;
}

/**
 * @brief 	Long Range 模式 pileup校正
 * @param 	[uint16_t] vi530x_ma_sum ：MA系数之和
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] integral_times ：测距输出的积分次数
 * @return 	[float] bias:校正值,校正TOF = 原始TOF + bias - offset
 */
float VI530x_Calculate_Pileup_Bias_V40_LR(uint16_t vi530x_ma_sum,uint32_t peak, uint32_t noise,uint32_t integral_times)
{
	//********pileup
	float peak_tmp = 0;
	const int32_t xth[] = {0,596,1027,1500,1835,3064,3500,3922,4120,4500};
	const int32_t pth[] = {0,-10,-3,5,10,44,60,74,91,100};
	uint8_t i = 0;
	uint32_t bias = 0;
	uint8_t buff_len = 0;
	
	buff_len = sizeof(xth) / sizeof(xth[0]);
	noise = noise / 8;
	if(integral_times == 0)
	{
		return bias;
	}
	if (peak > noise * vi530x_ma_sum)
	{
		peak_tmp = (peak - noise * vi530x_ma_sum) * 16 / integral_times;
	}

	for(i = 0;i < buff_len  - 1;i++)
	{
		if(peak_tmp < xth[i+1])
		{
			bias = (pth[ i + 1 ] - pth[i])*( peak_tmp -xth[i])/(xth[i+1]-xth[i])+ pth[i];
			return bias;
		}    
	}
	
	if(peak_tmp >= xth[i])
	{
		bias = (pth[i]-pth[i-1])*( peak_tmp - xth[i-1])/(xth[i]-xth[i-1])+pth[i-1];
	}
	
	return bias;
}

/**
 * @brief 	Long Range 模式 confidence1计算
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint32_t] integral_times ：测距输出的积分次数
 * @return 	[uint8_t] confidence
 */
uint8_t VI530x_Calculate_Confidence1_V40_LR(uint32_t noise, uint32_t peak, uint32_t integral_times)
{
	uint8_t i = 0;
	uint8_t len = 0;
	uint8_t confidence = 0;
	uint32_t Noise_regu = 0;
	uint32_t Peak_regu = 0;
	uint32_t Lower = 0;
	uint32_t Upper = 0;
	const uint32_t xth[] = {4, 32, 114, 175, 313, 482, 539, 657, 1472, 2421, 3223, 6777, 7217, 12326, 14946, 20906, 25976, 32287, 35000, 47000, 51439, 56032, 80216};
	const uint32_t ylower[] = {4, 7, 16, 22, 34, 49, 54, 66, 136, 211, 279, 566, 600, 1025, 1221, 1682, 2086, 2500, 3200, 3600, 3950, 4796, 6400};
	const uint32_t yupper[] = {7, 9, 28, 40, 43, 60, 66, 80, 162, 243, 321, 630, 666, 1138, 1350, 1825, 2400, 2800, 3300, 3720, 4100, 5050, 6500};

	if(integral_times == 0)
	{
		return 0;
	}
	if(noise > 32768)
	{
		Noise_regu = (noise * 256) / (uint32_t)(integral_times) * 256 * 2;
	}
	else
	{
		Noise_regu = (noise * 131702) / (uint32_t)(integral_times);
	}

	if (peak > 8000000)
	{
		Peak_regu = peak * 256 / integral_times * 4;
	}
	else if (peak > 4000000)
	{
		Peak_regu = peak * 512 / integral_times * 2;
	}
	else
	{
		Peak_regu = peak * 1024 / integral_times;
	}
	
	len = sizeof(xth) / sizeof(xth[0]);
	for (i = 0; i < (len - 1); i++)
	{
		if (Noise_regu < xth[i + 1])
		{
			Lower = (ylower[i + 1] - ylower[i]) * abs((int)(Noise_regu - xth[i])) / (xth[i + 1] - xth[i]) + ylower[i];
			Upper = (yupper[i + 1] - yupper[i]) * abs((int)(Noise_regu - xth[i])) / (xth[i + 1] - xth[i]) + yupper[i];
			break;
		}
		else if (Noise_regu >= xth[len - 1])
		{
			Lower = (ylower[len - 1] - ylower[len - 2]) * abs((int)(Noise_regu - xth[len - 2])) / (xth[len - 1] - xth[len - 2]) + ylower[len - 2];
			Upper = (yupper[len - 1] - yupper[len - 2]) * abs((int)(Noise_regu - xth[len - 2])) / (xth[len - 1] - xth[len - 2]) + yupper[len - 2];
			break;
		}
	}

	if (Peak_regu > Upper)
	{
		confidence = 100;
	}
	else if (Peak_regu < Lower)
	{
		confidence = 0;
	}
	else
	{
		confidence = 100 * (Peak_regu - Lower) / (Upper - Lower);
	}
	return confidence;
}

/**
 * @brief 	Long Range 模式 confidence2计算
 * @param 	[int16_t] tof ：测距输出的tof
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint32_t] integral_times ：测距输出的积分次数
 * @param 	[uint16_t] vi530x_ma_sum ：MA系数之和
 * @return 	[uint8_t] confidence
 */
uint8_t VI530x_Calculate_Confidence2_V40_LR(int16_t tof, uint32_t noise, uint32_t peak, uint16_t vi530x_ma_sum)
{
    uint8_t confidence = 0;
    float b;
    float peak_t;
    float r;
    float lower = 107.4;
    float upper = 108;

    b = (float)noise / 8;
    peak_t = (float)peak / vi530x_ma_sum;
    r = peak_t / b * 100;

    if (r > upper)
        confidence = 100;
    else if (r < lower)
        confidence = 0;
    else
        confidence = 100 * (r - lower) / (upper - lower);
    
    if (tof < 300 && r < 125)
        confidence = 0;
    return confidence;
}

/**
 * @brief 	校验和计算
 * @param 	[uint8_t] *buff ：数据缓存区
 * @param 	[uint8_t] len ：数据长度
 * @return 	[uint8_t] checksum
 */
uint16_t VI530x_Calculate_CheckSum(uint8_t *buff, uint8_t len)
{
	uint16_t checksum = 0;
	uint8_t i = 0;
	
	for (i = 0; i < len; ++i)
		checksum += (uint16_t)buff[i];
	
	return checksum;
}


/**
 * @brief 	Peak归一化
 * @param 	[uint16_t] vi530x_ma_sum ：MA系数之和
 * @param 	[uint32_t] peak1 ：测距输出的peak1
 * @param 	[uint32_t] noise ：测距输出的noise
 * @param 	[uint32_t] integral_times ：测距输出的积分次数
 * @return 	[uint32_t] peak_r：归一化值
 */
uint32_t VI530x_Calculate_Normalization_Peak(uint16_t VI530x_ma_sum, uint32_t peak, uint32_t noise, uint32_t integral_times)
{
	uint32_t peak_r = 0;
	
	noise = noise * VI530x_ma_sum;
	
	if(peak > noise)
	{
		if (peak > 2000000)
			peak_r = (peak-noise)*100/(integral_times/16);
		else
			peak_r = (peak-noise)*16*100/integral_times;
	}
	else
	{
		peak_r = 0;
	}
	
	return peak_r;
}


/**
 * @brief  Xtalk比值计算
 * @param  measure_data：测距输出
* @retval ratio：0为无效值，不可以使用
 */
//可用于参考评估cg的Xtalk强度，容易受环境影响
uint8_t VI530x_Calculate_Xtalk_Ratio(int16_t tof, uint16_t count, uint32_t noise, uint8_t confidence, uint32_t peak)
{
	int16_t ratio=0;
	
	if( (confidence==0) | ((tof<20)&&(peak<500000)&&(confidence==100)) |
		((tof>500)&&(confidence==100)) )
	{
		ratio = ( count + 81/2 - noise/8 )/81 + 3;
		if(ratio<0)
		{
			ratio = 0;
		}
	}
	return ratio;
}


/**
 * @brief  Xtalk比值校正
 * @param  xtalk_ratio：比值计算值
* @retval ret：0为OK
 */
//xtalk_ratio以应用环境评估，建议不超过15
VI530x_Status VI530x_Calibration_Xtalk_Ratio(uint8_t xtalk_ratio)
{
	VI530x_Status ret = VI530x_OK;
	uint8_t ratio = 0;
	if(xtalk_ratio<VI530x_Cali_Data.VI530x_Cali_CG_Maxratio)
	{
		//小于出厂标定值不建议使用
		xtalk_ratio = VI530x_Cali_Data.VI530x_Cali_CG_Maxratio;
	}
	else
	{
		VI530x_Cali_Data.VI530x_Xtalk_Ratio = xtalk_ratio;
		ret |= VI530x_Stop_Continue_Ranging_Cmd();	
		ret |= VI530x_Set_Sys_Xtalk_Maxratio( xtalk_ratio );
		ret |= VI530x_Get_Sys_Xtalk_Maxratio(&ratio);
		ret |= VI530x_Start_Continue_Ranging_Cmd();	
		if( ratio != xtalk_ratio )
		{
			return VI530x_ERROR_CONFIG;
		}
	}
	return ret;
}

