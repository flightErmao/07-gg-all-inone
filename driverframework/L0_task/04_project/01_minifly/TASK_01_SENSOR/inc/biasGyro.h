#ifndef __BIASGYRO_H__
#define __BIASGYRO_H__

#include "sensorsTypes.h"

void sensorsBiasObjInit(void);
bool getGyroBias(Axis3i16 gyroRaw, Axis3f *gyroBiasOut);
bool outputGyroBiasFound(void);

#endif
