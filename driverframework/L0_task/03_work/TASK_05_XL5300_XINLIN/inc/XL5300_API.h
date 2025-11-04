#ifndef __XL5300_API_H__
#define __XL5300_API_H__

#if defined(STM32H7xx)
#include "stm32h7xx_hal.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#elif defined(PY32F0xx)
#include "py32f0xx_hal.h"
#else
#include <rtthread.h>
#endif
#include "stdio.h"
#include "XL5300_Config.h"
#define API_VERSION  V10  

#define STATUS_TOF_CONFIDENT      0
#define STATUS_TOF_SEMI_CONFIDENT 1
#define STATUS_TOF_NOT_CONFIDENT  255

#define XL5300_DEVICE_ADDR          0xD8
#define XL5300_REG_MCU_CFG          0x00
#define XL5300_RET_INT_STATUS       0x03
#define XL5300_REG_SYS_CFG          0x01
#define XL5300_REG_PW_CTRL          0x07
#define XL5300_REG_CMD              0x0a
#define XL5300_REG_SIZE             0x0b
#define XL5300_REG_SCRATCH_PAD_BASE 0x0c 

#define XL5300_WRITEFW_CMD          0x03
#define XL5300_USER_CFG_CMD         0x09
#define XL5300_START_RANG_CMD       0x0E

#define XL5300_CFG_SUBCMD           0x01
#define XL5300_OTPW_SUBCMD          0x02
#define XL5300_OTPR_SUBCMD          0x03
#define VAN_REG_CMD                 0x0A  
#define VAN_REG_IIC_DEV_ADDR     0x06
#define VAN_REG_MCU_CFG          0x00
#define VAN_REG_AO_DOMAIN        0x3b

//#define Debug_Mode 

#define XSHUT_Pin1 GPIO_PIN_0
#define XSHUT_GPIO_Port GPIOA   
#define XL5300_CHECK_RET(a)   if(a != XL5300_OK) return a
	
extern uint8_t gSalve;
typedef enum 
{
    XL5300_OK       = 0x00,
    XL5300_RANGING  = 0x01,
    XL5300_BUSY     = 0x02,
    XL5300_BUS_BUSY = 0x03,
    XL5300_SLEEP    = 0x04,
    XL5300_BOOTING  = 0x05,
    XL5300_ERROR    = 0x06,
    XL5300_ERROR_FW_FAILURE		= 0x80,
    XL5300_ERROR_XTALK_CALIB	= 0x20,
	XL5300_ERROR_OFFSET_CALIB = 0x40,
	
} XL5300_Status;
#define VI530x_OFFSET_DISTANCE	2000 //2m Offset
extern XL5300_Status Get_XL5300_Download_Firmware_Status(void);
extern XL5300_Status VI530x_Chip_Init(void);
extern XL5300_Status VI530x_Download_Firmware(uint8_t *Firmware_buff, uint16_t size);
extern void XL5300_Chip_PowerON(uint16_t pin);
extern XL5300_Status XL5300_Set_Digital_Clock_Dutycycle(void);
extern XL5300_Status XL5300_Integral_Counts_Write(uint32_t inte_counts);
extern XL5300_Status XL5300_Set_Integralcounts_Frame(uint8_t fps, uint32_t intecoutns);
extern uint8_t XL5300_Stop_Continuous_Measure(void);
extern XL5300_Status XL5300_Get_Interrupt_State(uint8_t *status);
extern XL5300_Status XL5300_Clear_Interrupt(void);//�?己写�?
uint8_t XL5300_Device_Check(void);
extern XL5300_Status XL5300_Temp_Enable(uint8_t enable);//加的
#endif


