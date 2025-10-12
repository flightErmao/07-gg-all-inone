#ifndef RPM_FILTER_H__
#define RPM_FILTER_H__

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void rpmFilter(float* gyroData, float* rpmData, float* filteredGyroData);

#ifdef __cplusplus
}
#endif

#endif  // RPM_FILTER_H__