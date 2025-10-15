#include <math.h>
#include <stdlib.h>
#include "filterNotch2p.h"

#ifndef M_PI_F
#define M_PI_F (float)3.14159265
#endif

static notch2pData_t gyroNotch[3];
static notch2pData_t gyroNotch2[3];
static notch2pData_t gyroNotch3[3];
static notch2pData_t accNotch[3];

static void notch2p_set_params(notch2pData_t* s, float sample_freq, float center_freq, float q) {
  if (s == NULL || sample_freq <= 0.0f || center_freq <= 0.0f || q <= 0.0f) {
    return;
  }

  float w0 = 2.0f * M_PI_F * center_freq / sample_freq;  // rad/sample (digital)
  float cos_w0 = cosf(w0);
  float alpha = sinf(w0) / (2.0f * q);

  float b0 = 1.0f;
  float b1 = -2.0f * cos_w0;
  float b2 = 1.0f;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * cos_w0;
  float a2 = 1.0f - alpha;

  // normalize by a0
  s->b0 = b0 / a0;
  s->b1 = b1 / a0;
  s->b2 = b2 / a0;
  s->a1 = a1 / a0;
  s->a2 = a2 / a0;
  s->delay_element_1 = 0.0f;
  s->delay_element_2 = 0.0f;
}

static float notch2p_apply(notch2pData_t* s, float sample) {
  float delay0 = sample - s->a1 * s->delay_element_1 - s->a2 * s->delay_element_2;
  if (!isfinite(delay0)) {
    delay0 = sample;
  }
  float out = delay0 * s->b0 + s->delay_element_1 * s->b1 + s->delay_element_2 * s->b2;
  s->delay_element_2 = s->delay_element_1;
  s->delay_element_1 = delay0;
  return out;
}

void filterInitNotchGyro(float sample_freq, float center_freq, float q) {
  for (uint8_t i = 0; i < 3; i++) {
    notch2p_set_params(&gyroNotch[i], sample_freq, center_freq, q);
  }
}

void filterInitNotchGyro2(float sample_freq, float center_freq, float q) {
  for (uint8_t i = 0; i < 3; i++) {
    notch2p_set_params(&gyroNotch2[i], sample_freq, center_freq, q);
  }
}

void filterInitNotchGyro3(float sample_freq, float center_freq, float q) {
  for (uint8_t i = 0; i < 3; i++) {
    notch2p_set_params(&gyroNotch3[i], sample_freq, center_freq, q);
  }
}

void applyAxis3fNotchGyro(Axis3f* in) {
  for (uint8_t i = 0; i < 3; i++) {
    float v1 = notch2p_apply(&gyroNotch[i], in->axis[i]);
    float v2 = notch2p_apply(&gyroNotch2[i], v1);
    in->axis[i] = notch2p_apply(&gyroNotch3[i], v2);
  }
}

void filterInitNotchAcc(float sample_freq, float center_freq, float q) {
  for (uint8_t i = 0; i < 3; i++) {
    notch2p_set_params(&accNotch[i], sample_freq, center_freq, q);
  }
}

void applyAxis3fNotchAcc(Axis3f* in) {
  for (uint8_t i = 0; i < 3; i++) {
    in->axis[i] = notch2p_apply(&accNotch[i], in->axis[i]);
  }
}


