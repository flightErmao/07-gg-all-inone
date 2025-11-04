#ifndef __XL5300_CONFIG_H__
#define __XL5300_CONFIG_H__

#include "py32f0xx_hal.h"

// typedef unsigned long long uint64_t;

// typedef unsigned int uint32_t;

// typedef int int32_t;

// typedef unsigned short uint16_t;

// typedef short int16_t;

// typedef unsigned char uint8_t;

// typedef signed char int8_t;

typedef uint32_t FixPoint1616_t;


#define NULL 0


#define config_USE_CG_Correction     1
#define config_USE_PileUp_Correction 1
#define config_CAL_COFF              1
#define config_CAL_DMAX              1
#define config_SET_MODE              1


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

#define XL5300_CHECK_RET(a)   if(a != XL5300_OK) return a



#endif


