#include <stdint.h>
#include <string.h>
#include "rtthread.h"
#include "rtconfig.h"
#include "biquad.h"
#include "ptx.h"
#include "maths.h"

// TODO: this should be read from config
#define TICK_INTERVAL 0.001f

#define constrainG(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

static float motorLPFCutoffFrequency;
static float rpmFilterMinFrequency;
static float rpmFilterMaxFrequency;
static float rpmFilterFadeRangeFrequency;
static float rpmFilterNotchQ;

static bf_pt1Filter_t motorLPF[DSHOT_MOTOR_NUMS];
static biquadFilter_t rpmNotchFilter[3][DSHOT_MOTOR_NUMS][3];
static float filterStrength[3];

// strength for base, first harmonic, second harmonic. 1.0f = 100%, 0.0f = 0%
// 3 axis, 4 motors, 3 notches for base, first harmonic, second harmonic
// for tick frame, we update the base frequency and first harmonic frequency of motor frequency
// for tock frame, we update the base frequency and second harmonic frequency of motor frequency

static void updateRpmFilterCoeWeight(float frequency, uint8_t motorIndex, uint8_t filterIndex) {
  biquadFilter_t* template = &rpmNotchFilter[0][motorIndex][filterIndex];
  float weight = 1.0f;
  frequency = constrainG(frequency, rpmFilterMinFrequency, rpmFilterMaxFrequency);
  if (frequency < rpmFilterMinFrequency + rpmFilterFadeRangeFrequency) {
    weight = weight * (frequency - rpmFilterMinFrequency) / rpmFilterFadeRangeFrequency;
  }
  weight = weight * filterStrength[filterIndex];
  updateBiquadFilter(template, frequency, TICK_INTERVAL, rpmFilterNotchQ, FILTER_NOTCH, weight);
  for (uint8_t axis = 1; axis < 3; axis++) {
    biquadFilter_t* target = &rpmNotchFilter[axis][motorIndex][filterIndex];
    copyBiquadFilterCoeWeight(template, target);
  }
}

// TO DO: Load RPM filter strength parameters from flash
static void getFilterStrength(void) {
  filterStrength[0] = 1.0f;
  filterStrength[1] = 1.0f;
  filterStrength[2] = 1.0f;
}

int initRpmFilter(void) {
  getFilterStrength();

  motorLPFCutoffFrequency = 150.0f;
  rpmFilterMinFrequency = 50.0f;
  rpmFilterMaxFrequency = 0.48f * 1 / TICK_INTERVAL;  // 0.96 nyquist frequency
  rpmFilterFadeRangeFrequency = 50.0f;
  rpmFilterNotchQ = 10.0f;

  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    bf_pt1FilterInit(&motorLPF[i], motorLPFCutoffFrequency, TICK_INTERVAL);
  }
  for (uint8_t axis = 0; axis < 3; axis++) {
    for (uint8_t motorIndex = 0; motorIndex < DSHOT_MOTOR_NUMS; motorIndex++) {
      for (uint8_t filterIndex = 0; filterIndex < 3; filterIndex++) {
        initBiquadFilter(&rpmNotchFilter[axis][motorIndex][filterIndex], rpmFilterMinFrequency, TICK_INTERVAL,
                         rpmFilterNotchQ, FILTER_NOTCH, 1.0f);
      }
    }
  }
  return 0;
}

#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
INIT_COMPONENT_EXPORT(initRpmFilter);
#endif

void rpmFilter(float* gyroData, float* rpmData, float* filteredGyroData) {
  float postRpmGyroData[3];
  static uint8_t rpmFilterTickTock = 0;  // 0 for tick, 1 for tock

  if (gyroData == RT_NULL || rpmData == RT_NULL || filteredGyroData == RT_NULL) {
    return;
  }

  // step 3: filt rpm_hz
  float motorFrequency[DSHOT_MOTOR_NUMS];
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    motorFrequency[i] = bf_pt1FilterApply(&motorLPF[i], rpmData[i]);
  }

  // step 4: recalculate biquad filters
  if (rpmFilterTickTock == 0) {
    rpmFilterTickTock = 1;
    // tick frame, update base and first harmonic
    for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
      updateRpmFilterCoeWeight(motorFrequency[i], i, 0);      // base frequency
      updateRpmFilterCoeWeight(motorFrequency[i] * 2, i, 1);  // first harmonic
    }
  } else if (rpmFilterTickTock == 1) {
    rpmFilterTickTock = 0;
    // tock frame, update base and second harmonic
    for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
      updateRpmFilterCoeWeight(motorFrequency[i], i, 0);      // base frequency
      updateRpmFilterCoeWeight(motorFrequency[i] * 3, i, 2);  // second harmonic
    }
  }

  // step 5: apply_rpm_filter
  for (uint8_t axis = 0; axis < 3; axis++) {
    postRpmGyroData[axis] = gyroData[axis];
    for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
      for (uint8_t j = 0; j < 3; j++) {
        postRpmGyroData[axis] = applyDF1WeightedBiquadFilter(&rpmNotchFilter[axis][i][j], postRpmGyroData[axis]);
      }
    }
  }

  memcpy(filteredGyroData, postRpmGyroData, sizeof(postRpmGyroData));
}
