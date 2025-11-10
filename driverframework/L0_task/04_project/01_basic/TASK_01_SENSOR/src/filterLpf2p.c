#include <math.h>
#include <stdlib.h>
#include "filterLpf2p.h"
#include "rtconfig.h"
#include "param.h"

#define IMU_FILTER_LPF2P_DEBUG

#define LOG_TAG "imu_filter_lpf2p"
#ifdef IMU_FILTER_LPF2P_DEBUG
#define LOG_LVL LOG_LVL_DBG
#else
#define LOG_LVL LOG_LVL_WARNING
#endif
#include <ulog.h>

#define M_PI_F (float)3.14159265
#define IIR_SHIFT 8

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
  float gyro_lpf_cutoff_hz = 70.0f;
  float acc_lpf_cutoff_hz = 30.0f;
  float sample_rate_hz = 1000.0f;

  if (getParam("imu_filter_lpf_gyro_cutoff_hz", &gyro_lpf_cutoff_hz, sizeof(gyro_lpf_cutoff_hz)) != RT_EOK ||
      gyro_lpf_cutoff_hz <= 0.0f) {
    LOG_W("Failed to get gyro LPF cutoff frequency, using default: %f", gyro_lpf_cutoff_hz);
    gyro_lpf_cutoff_hz = 70.0f;
  }

  if (getParam("imu_filter_lpf_acc_cutoff_hz", &acc_lpf_cutoff_hz, sizeof(acc_lpf_cutoff_hz)) != RT_EOK ||
      acc_lpf_cutoff_hz <= 0.0f) {
    acc_lpf_cutoff_hz = 30.0f;
    LOG_W("Failed to get acc LPF cutoff frequency, using default: %f", acc_lpf_cutoff_hz);
  }

  if (getParam("imu_filter_sample_rate_hz", &sample_rate_hz, sizeof(sample_rate_hz)) != RT_EOK) {
    LOG_W("Failed to get IMU sample rate, using default: 1000Hz");
    sample_rate_hz = 1000.0f;
  }

  for (uint8_t i = 0; i < 3; i++) {
    lpf2pInit(&gyroLpf[i], sample_rate_hz, gyro_lpf_cutoff_hz);
    lpf2pInit(&accLpf[i], sample_rate_hz, acc_lpf_cutoff_hz);
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
