#ifndef RPM_FILTER_H__
#define RPM_FILTER_H__

#include <rtthread.h>
#include <stdint.h>

#include "dal/imu_data.h"

#ifdef __cplusplus
extern "C" {
#endif

void initRpmFilter(void);
void rpmFilter(void);
// void publish_rpm_filtered_gyro_buffer(void);
float getRpmFilteredGyroData(uint8_t axis);

float *getPostRpmFilterGyroData(void);
uint32_t getImuReadTimeStamp(void);

// below are for blackbox logging
float *bbGetMotorFrequency(void);
float *bbGetGyroScaleData(void);

#ifdef __cplusplus
}
#endif

#endif  // RPM_FILTER_H__