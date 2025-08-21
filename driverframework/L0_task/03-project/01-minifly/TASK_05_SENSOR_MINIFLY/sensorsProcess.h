#ifndef __SENSORS_H
#define __SENSORS_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 传感器控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

//#define SENSORS_ENABLE_MAG_AK8963
#define SENSORS_ENABLE_PRESSURE_BMP280	/*气压计使用bmp280*/

#define BARO_UPDATE_RATE		RATE_50_HZ
#define SENSOR9_UPDATE_RATE   	RATE_500_HZ
#define SENSOR9_UPDATE_DT     	(1.0f/SENSOR9_UPDATE_RATE)

typedef struct {
  float pressure;
  float temperature;
  float asl;
} baro_t;

typedef struct {
  Axis3f acc_raw;
  Axis3f acc_filter;
  Axis3f gyro_raw;
  Axis3f gyro_filter;
  Axis3f mag;
  baro_t baro;
  point_t position;
  zRange_t zrange;
} sensorData_t;

typedef struct
{
	Axis3i16	gyroRaw;
	Axis3i16	accRaw;
	u32 time_stamp;
} imuDataFrame_t;

void filterInitLpf2AccGyro(void);
void processAccGyroMeasurements(const uint8_t* buffer);

#endif //__SENSORS_H
