#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H
#include "stabilizerTypes.h"

void stateControlInit(void);
void stateControl(const state_t *state, const setpoint_t *setpoint, control_t *control, const uint32_t tick);
void getAngleDesired(attitude_t* get);
void getRateDesired(attitude_t *get);

#endif /*__STATE_CONTROL_H */
