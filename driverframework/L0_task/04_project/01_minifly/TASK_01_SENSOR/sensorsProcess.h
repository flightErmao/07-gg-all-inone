#ifndef __SENSORSPROCESS_H
#define __SENSORSPROCESS_H

#include "sensorsTypes.h"

void filterInitLpf2AccGyro(void);
sensorData_t processAccGyroMeasurements(const uint8_t* buffer);

#endif  //__SENSORSPROCESS_H
