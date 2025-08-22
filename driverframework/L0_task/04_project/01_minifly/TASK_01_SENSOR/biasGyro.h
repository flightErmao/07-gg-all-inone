#ifndef __BIASGYRO_H__
#define __BIASGYRO_H__

#include "sensorsTypes.h"

void sensorsBiasObjInit(void);
bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);

#endif
