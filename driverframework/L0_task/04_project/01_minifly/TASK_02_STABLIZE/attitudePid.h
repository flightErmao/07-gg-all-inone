#ifndef __ATTITUDE_PID_H
#define __ATTITUDE_PID_H

#include <stdbool.h>
#include "pidMinifly.h"
#include "sensorsTypes.h"
#include "stabilizerTypes.h"

extern PidObject pidAngleRoll;
extern PidObject pidAnglePitch;
extern PidObject pidAngleYaw;

extern PidObject pidRateRoll;
extern PidObject pidRatePitch;
extern PidObject pidRateYaw;

void attitudeControlInit(float rateDt, float angleDt);
bool attitudeControlTest(void);

void attitudeRatePID(const Axis3f *actualRate, const attitude_t *desiredRate, control_t *output);
void attitudeAnglePID(const attitude_t *actualAngle, const attitude_t *desiredAngle, attitude_t *outDesiredRate);
void attitudeControllerResetRollAttitudePID(void);
void attitudeControllerResetPitchAttitudePID(void);
void attitudeResetAllPID(void);
void attitudePIDwriteToConfigParam(void);

#endif /* __ATTITUDE_PID_H */
