#include <firmament.h>
#include <stdint.h>

#include "drv_dshot.h"
#include "filter_data.h"
#include "module/filter/biquad.h"
#include "module/filter/ptx.h"
#include "module/math/common.h"
#include "module/param/param_fydelix.h"
#include "task/init/task_init.h"

#define TICK_INTERVAL 0.0003125f  // FIXME: this should be read from config

static float motorLPFCutoffFrequency;
static float rpmFilterMinFrequency;
static float rpmFilterMaxFrequency;
static float rpmFilterFadeRangeFrequency;
static float rpmFilterNotchQ;

float rawGyroData[3];
float motorFrequency[DSHOT_MOTOR_NUMS];
static bf_pt1Filter_t motorLPF[DSHOT_MOTOR_NUMS];
static biquadFilter_t rpmNotchFilter[3][DSHOT_MOTOR_NUMS][3];
// 3 axis, 4 motors, 3 notches for base, first harmonic, second harmonic

static float filterStrength[3];  // strength for base, first harmonic, second harmonic. 1.0f = 100%, 0.0f = 0%

static float postRpmGyroData[3];

static uint8_t rpmFilterTickTock;  // 0 for tick, 1 for tock
// for tick frame, we update the base frequency and first harmonic frequency of motor frequency
// for tock frame, we update the base frequency and second harmonic frequency of motor frequency

static void updateRpmFilterCoeWeight(float frequency, uint8_t motorIndex, uint8_t filterIndex) {
    biquadFilter_t *template = &rpmNotchFilter[0][motorIndex][filterIndex];
    float weight = 1.0f;
    frequency = constrain(frequency, rpmFilterMinFrequency, rpmFilterMaxFrequency);
    if (frequency < rpmFilterMinFrequency + rpmFilterFadeRangeFrequency) {
        weight = weight * (frequency - rpmFilterMinFrequency) / rpmFilterFadeRangeFrequency;
    }
    weight = weight * filterStrength[filterIndex];
    updateBiquadFilter(template, frequency, TICK_INTERVAL, rpmFilterNotchQ, FILTER_NOTCH, weight);
    for (uint8_t axis = 1; axis < 3; axis++) {
        biquadFilter_t *target = &rpmNotchFilter[axis][motorIndex][filterIndex];
        copyBiquadFilterCoeWeight(template, target);
    }
}

void initRpmFilter(void) {
    // Load RPM filter strength parameters from flash
    getParam("rpm_filter_strength", filterStrength, sizeof(filterStrength));

    rpmFilterTickTock = 0;
    motorLPFCutoffFrequency = 150.0f;
    rpmFilterMinFrequency = 100.0f;
    rpmFilterMaxFrequency = 0.48f * 1 / TICK_INTERVAL;  // 0.96 nyquist frequency
    rpmFilterFadeRangeFrequency = 50.0f;
    rpmFilterNotchQ = 5.0f;

    for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
        motorFrequency[i] = 0;
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
}

void rpmFilter(void) {
    // Extract gyro data from filter_data module
    rawGyroData[0] = getGyroScaled(0);
    rawGyroData[1] = getGyroScaled(1);
    rawGyroData[2] = getGyroScaled(2);

    // step 2: get rpm data from filter_data module
    float *rawMotorFrequency = getRawMotorFrequency();

    // step 3: filt rpm_hz
    for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
        motorFrequency[i] = bf_pt1FilterApply(&motorLPF[i], rawMotorFrequency[i]);
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
        postRpmGyroData[axis] = rawGyroData[axis];
        for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
            for (uint8_t j = 0; j < 3; j++) {
                postRpmGyroData[axis] =
                    applyDF1WeightedBiquadFilter(&rpmNotchFilter[axis][i][j], postRpmGyroData[axis]);
            }
        }
    }
}

// down sampling produce will take this data
float getRpmFilteredGyroData(uint8_t axis) { return postRpmGyroData[axis]; }
float *getPostRpmFilterGyroData(void) { return postRpmGyroData; }

// below are for blackbox logging
float *bbGetMotorFrequency(void) { return motorFrequency; }
float *bbGetGyroScaleData(void) { return rawGyroData; }
