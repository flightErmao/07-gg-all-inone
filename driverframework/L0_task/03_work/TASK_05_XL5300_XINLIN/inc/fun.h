/********************************************************************************
File:
Author:			
Date:
Version:		V001
Description:
*********************************************************************************/

#ifndef __USER_FUN_H__
#define __USER_FUN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SYS_SLEEP_EN                (1)           //1:休眠使能 0:禁用休眠（调试使用）
#define IWDT_EN                     (0)           //1:开看门狗 0：不用看
     	     
extern void SystemClock_Config(void);
extern void wdg_Init(void);
extern void clear_Wdg(void);
extern void XL5300_All_Init(void);

#ifdef __cplusplus
}
#endif

#endif
