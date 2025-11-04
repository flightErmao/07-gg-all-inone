#ifndef __XL5300_USERPLATFORM_H__
#define __XL5300_USERPLATFORM_H__
#include "XL5300_API.h"
#include "XL5300_Config.h"

XL5300_Status I2C_WriteXBytes(uint8_t startaddr, uint8_t *buf, uint8_t len);
XL5300_Status I2C_ReadXBytes(uint8_t startaddr, uint8_t *buf, uint8_t len);
XL5300_Status WriteCommand(uint8_t cmd);
XL5300_Status I2C_2V1_WriteOneReg(uint8_t addr, uint8_t value);
XL5300_Status I2C_2V1_ReadOneReg(uint8_t addr, uint8_t *value);
XL5300_Status WriteOneReg(uint8_t addr, uint8_t value);
XL5300_Status ReadOneReg(uint8_t addr, uint8_t *value);

#endif





