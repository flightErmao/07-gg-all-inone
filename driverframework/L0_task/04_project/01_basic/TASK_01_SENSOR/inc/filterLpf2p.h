#ifndef __FILTERLPF2P_H
#define __FILTERLPF2P_H
#include "sensorsTypes.h"

void filterInitLpf2AccGyro(void);
void filterInitLpfAngularAccel(float sample_freq, float cutoff_freq);
void applyAxis3fLpfGyro(Axis3f* in);
void applyAxis3fLpfAcc(Axis3f* in);
void applyAxis3fLpfAngularAccel(Axis3f* in);

#endif  //__FILTER_H
