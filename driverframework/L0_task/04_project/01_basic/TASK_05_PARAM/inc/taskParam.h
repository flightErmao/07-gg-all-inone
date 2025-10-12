#ifndef __CONFIG_PARAM_H
#define __CONFIG_PARAM_H

#include <stdbool.h>
#include "pidMinifly.h"
#include "stdint.h"
#include "pidMinifly.h"

#pragma pack(push, 1)  // 设置1字节对齐

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

#pragma pack(pop)  // 恢复默认对齐

void getConfigParam(configParam_t *configParam_temp);
void updateConfigAnglePID(pidParam_t pidParam_temp);
void updateConfigRatePID(pidParam_t pidParam_temp);
void configParamGiveSemaphore(void);
void resetConfigParamPID(void);

#endif /*__CONFIG_PARAM_H */
