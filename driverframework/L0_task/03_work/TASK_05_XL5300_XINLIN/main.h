/**
  ******************************************************************************
  * @file    main.h
  * @author  Application Team
  * @Version V1.0.0
  * @Date
  * @brief   Header for main.c file.
  *          This file contains the common defines of the application.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx_hal.h"
#include "uart.h"
#include "fun.h"
#include "XL5300_Config.h"
#include "XL5300_API.h"
#include "flash.h"
#include "XL5300_Firmware_8.h"
#include "das.h"
/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

