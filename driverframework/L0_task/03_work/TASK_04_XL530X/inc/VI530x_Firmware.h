/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VI530x_FIRMWARE_H
#define __VI530x_FIRMWARE_H			 

/* Includes ------------------------------------------------------------------*/
#include "VI530x_User_Handle.h"

/**
 * @brief 	VI530X 固件存放的数据
 */
extern const uint8_t VI5301_M40_firmware_buff[8192];

/**
 * @brief 	VI530X 获取固件运行状态
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status Get_VI530x_Download_Firmware_Status(void);
/**
 * @brief 	VI530X 写固件
 * @param 	[uint8_t] *Firmware_buff：固件数据地址
 * @param 	[uint16_t] size：固件长度
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Download_Firmware(uint8_t *Firmware_buff, uint16_t size);

/**
 * @brief 	固件大小
 * @param 	
 * @return 	[uint16_t]	固件数据数组大小
 */
uint16_t FirmwareSize(void);

#endif  

