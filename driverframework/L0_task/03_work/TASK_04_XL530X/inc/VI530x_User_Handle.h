/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VI530x_USER_HANDLE_H
#define __VI530x_USER_HANDLE_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* Start user code for adding. */

/* End user code.  */

/***Debug时开启***/
//#define Debug_Mode

/***两点标定开启***/
//#define VI530x_GRADIENTK_CALIBRATION

//VI530x Status
typedef enum 
{
	VI530x_OK						= 0x0000,
	VI530x_RANGING					= 0x0001,
	VI530x_BUSY						= 0x0002,		//启动异常或工作在测距/标定中
	VI530x_SLEEP					= 0x0004,
	VI530x_IIC_ERROR				= 0x0008,		//IIC通讯异常		
	VI530x_ERROR					= 0x0010,		//操作异常
	VI530x_ERROR_XTALK_CALIB		= 0x0020,		//Xtalk标定失败
	VI530x_ERROR_OFFSET_CALIB		= 0x0040,		//Offset标定失败，标定环境异常
	VI530x_ERROR_FW_FAILURE			= 0x0080,		//固件异常/Xtalk标定后
	VI530x_ERROR_IIC_ID				= 0x0100,		//默认IIC ID不是0xD8,芯片无工作/IIC无正常通讯
	VI530x_ERROR_CONFIG				= 0x0200,		//配置失败，芯片无工作/IIC无正常通讯
	VI530x_ERROR_TIME_OUT			= 0x0400,		//操作超时
	VI530x_INTERRUPT_NONE			= 0x0800,		//无测距完成标志，芯片测距没完成/芯片无测距工作/主控中断检测异常
	VI530x_ERROR_CHECK				= 0x1000		//Check位异常，建议重读
} VI530x_Status;

#define VI530x_OFFSET_DISTANCE	300.0 //30cm Offset

#ifdef VI530x_GRADIENTK_CALIBRATION
	#define VI530x_GRADIENT_DISTANCE	200 //20cm Gradient K，不可以等于VI530x_OFFSET_DISTANCE
#endif

//i2c读写函数
/**
 * @brief 	VI530X I2C 1Byte 读
 * @param 	[uint8_t] addr：读地址
 * @param 	[uint8_t] *value：读到的值
 * @return 	[VI530x_Status]	ret:0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */
VI530x_Status VI530x_IIC_Read_One_Byte(uint8_t addr, uint8_t *value);

/**
 * @brief 	VI530X I2C x Byte 读
 * @param 	[uint8_t] addr：读地址
 * @param 	[uint8_t] *value：读到的值
 * @param 	[uint16_t] tlen：读取的长度
 * @return 	[VI530x_Status]	ret:0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */
VI530x_Status VI530x_IIC_Read_X_Bytes(uint8_t addr, uint8_t *value, uint16_t tlen);

/**
 * @brief 	VI530X I2C 1 Byte 写
 * @param 	[uint8_t] addr：写的地址
 * @param 	[uint8_t] value：写入的值
 * @return 	[VI530x_Status]	ret:0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */
VI530x_Status VI530x_IIC_Write_One_Byte(uint8_t addr, uint8_t value);

/**
 * @brief 	VI530X I2C 1 Byte 写
 * @param 	[uint8_t] addr：写的地址
 * @param 	[uint8_t] value：写入的值
 * @param 	[uint16_t] tlen：写入的长度
 * @return 	[VI530x_Status]	ret:0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */
VI530x_Status VI530x_IIC_Write_X_Bytes(uint8_t addr, uint8_t *pValue, uint16_t tlen);

/**
 * @brief 	VI530X 延时 单位：ms
 * @param 	[none]
 * @return 	[none]
 */
void VI530x_Delay_Ms(uint16_t nMs);

/**
 * @brief 	VI530X 延时 硬件中断处理
 * @param 	[none]
 * @return 	[none]
 */
void VI530x_GPIO_Interrupt_Handle(void);
/**
 * @brief 	VI530X XSHUS引脚控制
 * @param 	[uint8_t] state：0-拉低，1-拉高
 * @return 	[none]
 */
void VI530x_XSHUT_Enable(uint8_t state);


void VI530x_main(void);
#endif /* __VI530x_USER_HANDLE_H */

/*****************************END OF FILE*************************/

