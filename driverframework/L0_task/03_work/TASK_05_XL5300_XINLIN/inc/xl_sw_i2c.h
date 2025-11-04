
#ifndef _BSP_SW_I2C_H
#define _BSP_SW_I2C_H

#include <inttypes.h>
#include "py32f0xx_hal.h"

#define USE_SOFTWARE_IIC

#ifdef USE_SOFTWARE_IIC

#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */

void i2c_sw_gpio_config(void);
void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReadByte(uint8_t ack);
uint8_t i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);
uint8_t i2c_CheckDevice(uint8_t _Address);//检查



extern uint8_t vi_sw_writereg(uint8_t slave, uint8_t reg_add,uint8_t reg_dat);
extern uint8_t vi_sw_readreg(uint8_t slave, uint8_t reg_add,uint8_t *buf,uint8_t num);
extern uint8_t vi_sw_readRegs(uint8_t slave,uint8_t Addr, uint8_t *data, uint16_t len);
extern uint8_t vi_sw_writeRegs(uint8_t slave,uint8_t Addr, uint8_t *data, uint8_t len);
#endif
#endif
