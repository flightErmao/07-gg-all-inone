#ifndef TASK_IMU_PUB_MCN_H_
#define TASK_IMU_PUB_MCN_H_

#include <rtdef.h>
#include "uMCN.h"
#include "sensorsTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

MCN_DECLARE(px4_imu);

int px4ImuMcnInit(void);
int px4ImuMcnPublish(const sensorData_t *sensor_data);
int px4ImuMcnAcquire(sensorData_t *sensor_data);
void px4ImuMcnWait(void);

#ifdef __cplusplus
}
#endif

#endif  // TASK_IMU_PUB_MCN_H_


