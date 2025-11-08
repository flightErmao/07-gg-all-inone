#include <math.h>
#include <stdlib.h>
#include "filterLpf2p.h"
#include "rtconfig.h"

#define M_PI_F (float)3.14159265
#define IIR_SHIFT 8

#ifdef PROJECT_MINIFLY_TASK_SENSOR_GYRO_LPF_CUTOFF_FREQ
#define GYRO_LPF_CUTOFF_FREQ PROJECT_MINIFLY_TASK_SENSOR_GYRO_LPF_CUTOFF_FREQ
#else
#define GYRO_LPF_CUTOFF_FREQ 70
#endif

#ifdef PROJECT_MINIFLY_TASK_SENSOR_ACC_LPF_CUTOFF_FREQ
#define ACCEL_LPF_CUTOFF_FREQ PROJECT_MINIFLY_TASK_SENSOR_ACC_LPF_CUTOFF_FREQ
#else
#define ACCEL_LPF_CUTOFF_FREQ 30
#endif

typedef struct {
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
  float delay_element_1;
  float delay_element_2;
} lpf2pData;

#ifdef PROJECT_MINIFLY_TASK_SENSOR_LPF_EN

static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static lpf2pData angularAccelLpf[3];

/**
 * 设置二阶低通滤波截至频率
 */
static void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq) {
  float fr = sample_freq / cutoff_freq;
  float ohm = tanf(M_PI_F / fr);
  float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
  lpfData->b0 = ohm * ohm / c;
  lpfData->b1 = 2.0f * lpfData->b0;
  lpfData->b2 = lpfData->b0;
  lpfData->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
  lpfData->a2 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
  lpfData->delay_element_1 = 0.0f;
  lpfData->delay_element_2 = 0.0f;
}

/**
 * 二阶低通滤波
 */
static void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq) {
  if (lpfData == NULL || cutoff_freq <= 0.0f) {
    return;
  }

  lpf2pSetCutoffFreq(lpfData, sample_freq, cutoff_freq);
}

static float lpf2pApply(lpf2pData* lpfData, float sample) {
  float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
  if (!isfinite(delay_element_0)) {
    // don't allow bad values to propigate via the filter
    delay_element_0 = sample;
  }

  float output =
      delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

  lpfData->delay_element_2 = lpfData->delay_element_1;
  lpfData->delay_element_1 = delay_element_0;
  return output;
}

float lpf2pReset(lpf2pData* lpfData, float sample) {
  float dval = sample / (lpfData->b0 + lpfData->b1 + lpfData->b2);
  lpfData->delay_element_1 = dval;
  lpfData->delay_element_2 = dval;
  return lpf2pApply(lpfData, sample);
}

#endif // PROJECT_MINIFLY_TASK_SENSOR_LPF_EN

void filterInitLpf2AccGyro(void) {
#ifdef PROJECT_MINIFLY_TASK_SENSOR_LPF_EN
  for (uint8_t i = 0; i < 3; i++) {
    lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
  }
#endif
}

void filterInitLpfAngularAccel(float sample_freq, float cutoff_freq) {
#ifdef PROJECT_MINIFLY_TASK_SENSOR_LPF_EN
  if (cutoff_freq <= 0.0f) {
    return;
  }
  for (uint8_t i = 0; i < 3; i++) {
    lpf2pInit(&angularAccelLpf[i], sample_freq, cutoff_freq);
  }
#else
  (void)sample_freq;
  (void)cutoff_freq;
#endif
}

void applyAxis3fLpfGyro(Axis3f* in) {
#ifdef PROJECT_MINIFLY_TASK_SENSOR_LPF_EN
  for (uint8_t i = 0; i < 3; i++) {
    in->axis[i] = lpf2pApply(&gyroLpf[i], in->axis[i]);
  }
#endif
}

void applyAxis3fLpfAcc(Axis3f* in) {
#ifdef PROJECT_MINIFLY_TASK_SENSOR_LPF_EN
  for (uint8_t i = 0; i < 3; i++) {
    in->axis[i] = lpf2pApply(&accLpf[i], in->axis[i]);
  }
#endif
}

void applyAxis3fLpfAngularAccel(Axis3f* in) {
#ifdef PROJECT_MINIFLY_TASK_SENSOR_LPF_EN
  for (uint8_t i = 0; i < 3; i++) {
    in->axis[i] = lpf2pApply(&angularAccelLpf[i], in->axis[i]);
  }
#else
  (void)in;
#endif
}
