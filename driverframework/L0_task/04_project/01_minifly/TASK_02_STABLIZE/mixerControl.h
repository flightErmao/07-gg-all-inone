#ifndef __MIXER_CONTROL_H
#define __MIXER_CONTROL_H
#include "stabilizerTypes.h"

// bool powerControlTest(void);
void mixerControl(control_t* control);
void motorInit(void);

// void getMotorPWM(motorPWM_t* get);
// void setMotorPWM(bool enable, uint32_t m1_set, uint32_t m2_set, uint32_t m3_set, uint32_t m4_set);
#endif
