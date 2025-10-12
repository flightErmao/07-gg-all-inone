#ifndef __A_MCN_SENSOR_IMU_H__
#define __A_MCN_SENSOR_IMU_H__

#include <rtthread.h>
#include "uMCN.h"
#include "sensorsTypes.h"

/* Sensor IMU MCN functions */
int mcnSensorImuPublish(const sensorData_t* sensor_data);
int mcnSensorImuAcquire(sensorData_t* sensor_data);

#endif /* __A_MCN_SENSOR_IMU_H__ */

