#ifndef __CONFIG_PARAM_H
#define __CONFIG_PARAM_H

#include <stdbool.h>
#include "pidMinifly.h"
#include "stdint.h"
/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 配置参数驱动代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 ********************************************************************************/

typedef struct {
  pidInit_t vx; /*X轴速度PID*/
  pidInit_t vy; /*Y轴速度PID*/
  pidInit_t vz; /*Z轴速度PID*/
  pidInit_t x;  /*X轴位置PID*/
  pidInit_t y;  /*Y轴位置PID*/
  pidInit_t z;  /*Z轴位置PID*/
} pidParamPos_t;

typedef struct {
  uint8_t version;      /*软件版本号*/
  pidParam_t pidAngle;  /*角度PID*/
  pidParam_t pidRate;   /*角速度PID*/
  pidParamPos_t pidPos; /*位置PID*/
  float trimP;          /*pitch微调*/
  float trimR;          /*roll微调*/
  uint16_t thrustBase;  /*油门基础值*/
  uint8_t cksum;        /*校验*/
} configParam_t;

void getConfigParam(configParam_t *configParam_temp);
void configParamGiveSemaphore(void);
void resetConfigParamPID(void);

#endif /*__CONFIG_PARAM_H */
