#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H
#include "stabilizerTypes.h"
#include "sensorsTypes.h"
#include "pidMinifly.h"

/* Attitude/Rate PID interfaces (merged) */
void attitudeControlInit(float rateDt, float angleDt);
bool attitudeControlTest(void);
void attitudeRatePID(const Axis3f *actualRate, const attitude_t *desiredRate, control_t *output);
void attitudeAnglePID(const attitude_t *actualAngle, const attitude_t *desiredAngle, attitude_t *outDesiredRate);
void attitudeAnglePidFpv(const setpoint_t *setpoint, const attitude_t *actualAngle, attitude_t *desiredAngle, attitude_t *outDesiredRate);
void attitudeControllerResetRollAttitudePID(void);
void attitudeControllerResetPitchAttitudePID(void);
void attitudeResetAllPID(void);
void attitudePIDwriteToConfigParam(void);
void getAnglePidRollDebug(float* outP, float* outI, float* outD);
void getAnglePidPitchDebug(float* outP, float* outI, float* outD);
void getAnglePidYawDebug(float* outP, float* outI, float* outD);
void getRatePidRollDebug(float* outP, float* outI, float* outD);
void getRatePidPitchDebug(float* outP, float* outI, float* outD);
void getRatePidYawDebug(float* outP, float* outI, float* outD);

/* Expose PID objects for modules that tune or read them */
extern PidObject pidAngleRoll;
extern PidObject pidAnglePitch;
extern PidObject pidAngleYaw;
extern PidObject pidRateRoll;
extern PidObject pidRatePitch;
extern PidObject pidRateYaw;

void stateControlInit(void);
void stateControl(const state_t *state, const setpoint_t *setpoint, control_t *control, const uint32_t tick);
void getAngleDesired(attitude_t* get);
void getRateDesired(attitude_t *get);

#endif /*__STATE_CONTROL_H */
