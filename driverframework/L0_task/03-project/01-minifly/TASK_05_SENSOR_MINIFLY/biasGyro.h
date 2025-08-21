#ifndef FILTERS_H
#define FILTERS_H

#include <stdint.h>
#include <stdbool.h>
#include "axis.h"

void sensorsBiasObjInit(void);
bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);

#endif

