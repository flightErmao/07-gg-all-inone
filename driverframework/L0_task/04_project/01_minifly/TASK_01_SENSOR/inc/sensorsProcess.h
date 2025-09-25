#ifndef __SENSORSPROCESS_H
#define __SENSORSPROCESS_H

#include "sensorsTypes.h"

void filterInitLpf2AccGyro(void);
sensorData_t processAccGyroMeasurements(const uint8_t* buffer);
void sensorsProcess_set_lsb(float acc_g_per_lsb, float gyro_deg_per_lsb);
void initImuRotationDir(void);
#endif  //__SENSORSPROCESS_H
