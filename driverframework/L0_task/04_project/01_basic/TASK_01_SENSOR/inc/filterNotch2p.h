#ifndef __FILTER_NOTCH2P_H
#define __FILTER_NOTCH2P_H

#include <stdint.h>
#include <stdbool.h>
#include "sensorsTypes.h"

typedef struct {
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
  float delay_element_1;
  float delay_element_2;
} notch2pData_t;

void filterInitNotchGyro(float sample_freq, float center_freq, float q);
void applyAxis3fNotchGyro(Axis3f* in);
void filterInitNotchGyro2(float sample_freq, float center_freq, float q);
void filterInitNotchGyro3(float sample_freq, float center_freq, float q);

void filterInitNotchAcc(float sample_freq, float center_freq, float q);
void applyAxis3fNotchAcc(Axis3f* in);

#endif /* __FILTER_NOTCH2P_H */


