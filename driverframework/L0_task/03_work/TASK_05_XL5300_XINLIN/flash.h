#ifndef _FLASH_H
#define _FLASH_H

#include "py32f0xx_hal.h"
#include "main.h"
#include "XL5300_API.h"

typedef struct
{
  uint32_t arrA[64];
} NewDataType;                                                      /* 结构体定义 */
#define VarA (*(volatile NewDataType *)FLASH_USER_START_ADDR)       /* Flash存储地址定义 */
typedef union {
 float fa;
 char farray[4];
}utemp;
extern uint32_t DATA[64];
#define FLASH_USER_START_ADDR     0x08007F00
void F003_Flash_Write(void);
void F003_Flash_Read(void);

#endif
