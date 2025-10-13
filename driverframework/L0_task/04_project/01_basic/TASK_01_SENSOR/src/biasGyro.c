#include <math.h>
#include <stdlib.h>
#include "biasGyro.h"
#include "rtconfig.h"
#include <rtthread.h>

#define SENSORS_NBR_OF_BIAS_SAMPLES 1024
#ifndef PROJECT_MINIFLY_TASK_SENSOR_GYRO_VARIANCE_BASE
#define PROJECT_MINIFLY_TASK_SENSOR_GYRO_VARIANCE_BASE 5000
#endif
#define GYRO_VARIANCE_BASE PROJECT_MINIFLY_TASK_SENSOR_GYRO_VARIANCE_BASE /* 陀螺仪零偏方差阈值 */

typedef struct {
  Axis3f bias;
  bool isBiasValueFound;
  bool isBufferFilled;
  Axis3i16 *bufHead;
  Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static BiasObj gyroBiasRunning;

void sensorsBiasObjInit(void) {
  gyroBiasRunning.isBufferFilled = false;
  gyroBiasRunning.bufHead = gyroBiasRunning.buffer;
}

static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z) {
  bias->bufHead->x = x;
  bias->bufHead->y = y;
  bias->bufHead->z = z;
  bias->bufHead++;
  if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES]) {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

static void sensorsCalculateVarianceAndMean(const BiasObj *bias, Axis3f *variance_out, Axis3f *mean_out) {
  int64_t sum[3] = {0};
  int64_t sumsq[3] = {0};
  for (uint32_t i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumsq[0] += (int64_t)bias->buffer[i].x * bias->buffer[i].x;
    sumsq[1] += (int64_t)bias->buffer[i].y * bias->buffer[i].y;
    sumsq[2] += (int64_t)bias->buffer[i].z * bias->buffer[i].z;
  }
  variance_out->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  variance_out->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  variance_out->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

  mean_out->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  mean_out->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  mean_out->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/*传感器查找偏置值*/
static void sensorsFindBiasValue(BiasObj *bias) {
  if (bias->isBufferFilled) {
    Axis3f mean;
    Axis3f variance;
    sensorsCalculateVarianceAndMean(bias, &variance, &mean);

    if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE) {
      bias->bias.x = mean.x;
      bias->bias.y = mean.y;
      bias->bias.z = mean.z;
      bias->isBiasValueFound = true;
    } else
      bias->isBufferFilled = false;
  }
}

bool getGyroBias(Axis3i16 gyroRaw, Axis3f *gyroBiasOut) {
  if (!gyroBiasRunning.isBiasValueFound) {
    static bool first_times = true;
    if (first_times) {
      rt_thread_mdelay(1000);
      rt_thread_mdelay(1000);
      rt_thread_mdelay(1000);
      first_times = false;
    }
  } else {
    return true;
  }

  sensorsAddBiasValue(&gyroBiasRunning, gyroRaw.x, gyroRaw.y, gyroRaw.z);

  if (!gyroBiasRunning.isBiasValueFound) {
    sensorsFindBiasValue(&gyroBiasRunning);
  }

  gyroBiasOut->x = gyroBiasRunning.bias.x;
  gyroBiasOut->y = gyroBiasRunning.bias.y;
  gyroBiasOut->z = gyroBiasRunning.bias.z;

  return gyroBiasRunning.isBiasValueFound;
}

bool outputGyroBiasFound(void) { return gyroBiasRunning.isBiasValueFound; }