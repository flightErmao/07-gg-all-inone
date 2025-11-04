#ifndef __VI530x_SYSTEM_DATA_H
#define __VI530x_SYSTEM_DATA_H

/* Includes ------------------------------------------------------------------*/
#include "VI530x_User_Handle.h"


//设置积分次数
VI530x_Status VI530x_Set_Sys_Integral_Time(uint32_t integral_time);
//积分次数读取
VI530x_Status VI530x_Get_Sys_Integral_Time(uint32_t *integral_time);

//设置帧率
VI530x_Status VI530x_Set_FPS(uint8_t data_fps);
//读取帧率
VI530x_Status VI530x_Get_FPS(uint8_t *data_fps);

//设置delaytime
VI530x_Status VI530x_Set_Sys_DelayTime(uint16_t delay_time);
//读取delaytime
VI530x_Status VI530x_Get_Sys_DelayTime(uint16_t *delay_time);
//设置帧率和积分次数
VI530x_Status VI530x_Set_Integralcounts_Frame(uint8_t fps, uint32_t intecoutns);
VI530x_Status VI530x_Set_Sys_Reftof(uint16_t reftof);
VI530x_Status VI530x_Get_Sys_Reftof(uint16_t *reftof);
VI530x_Status VI530x_Set_Sys_Histogram_MA_Window_Data(uint8_t *setting_buff);
VI530x_Status VI530x_Set_Sys_Temperature_Enable(uint8_t status);
VI530x_Status VI530x_Get_Sys_Temperature_Enable(uint8_t *status);
VI530x_Status VI530x_Set_Sys_Limit_Distance(uint16_t distance);
VI530x_Status VI530x_Get_Sys_Limit_Distance(uint16_t *distance);
VI530x_Status VI530x_Set_Sys_CG_Maxratio(uint8_t maxratio);
VI530x_Status VI530x_Get_Sys_CG_Maxratio(uint8_t *maxratio);
VI530x_Status VI530x_Set_Sys_CK(uint8_t ck);
VI530x_Status VI530x_Set_Sys_Xtalk_Position(uint8_t xtalk_position);
VI530x_Status VI530x_Get_Sys_Xtalk_Position(uint8_t *xtalk_position);
VI530x_Status VI530x_Set_Sys_Histogram_MA_Window_Data(uint8_t *setting_buff);
VI530x_Status VI530x_Get_Sys_Histogram_MA_Window_Data(uint8_t *getting_buff);
VI530x_Status VI530x_Write_System_Data(uint8_t offset_addr, uint8_t *buff, uint8_t len);
VI530x_Status VI530x_Read_System_Data(uint8_t offset_addr, uint8_t *buff, uint8_t len);
VI530x_Status VI530x_Bvd_Add(uint8_t bvd_add);
VI530x_Status VI530x_Set_Sys_Pos_MaxMin(uint8_t pos_max, uint8_t pos_min);
VI530x_Status VI530x_Get_Sys_Pos_MaxMin(uint8_t *pos_max, uint8_t *pos_min);
#endif



