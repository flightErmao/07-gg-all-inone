#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H
#include "stabilizerTypes.h"
#include "sensorsTypes.h"
#include "pidMinifly.h"
#include "rtconfig.h"
#include "debugPin.h"

void getAnglePidRollDebug(float* outP, float* outI, float* outD);
void getAnglePidPitchDebug(float* outP, float* outI, float* outD);
void getAnglePidYawDebug(float* outP, float* outI, float* outD);
void getRatePidRollDebug(float* outP, float* outI, float* outD);
void getRatePidPitchDebug(float* outP, float* outI, float* outD);
void getRatePidYawDebug(float* outP, float* outI, float* outD);
void getPx4RatePidInt(float* rollOutInt, float* pitchOutInt, float* yawOutInt);

void stateControlInit(void);

void getAngleDesired(attitude_t* get);
void getRateDesired(attitude_t* get);

void stateControl(const state_t* state, const setpoint_t* setpoint, control_t* control, const uint32_t tick);

#endif /*__STATE_CONTROL_H */
