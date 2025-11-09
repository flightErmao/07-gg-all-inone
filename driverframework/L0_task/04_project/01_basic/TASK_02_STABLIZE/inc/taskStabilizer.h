#ifndef __TASK_STABILIZER_H__
#define __TASK_STABILIZER_H__

#include "stabilizerTypes.h"
#include "rtconfig.h"
#include "debugPin.h"

/**
 * @brief Get current setpoint
 * @param setpoint Pointer to setpoint structure to be filled with current values
 */
void commanderGetCurrentSetpoint(setpoint_t* setpoint);

/**
 * @brief Get control output
 * @param get Pointer to control structure to be filled with current control output
 */
void getControlOutput(control_t* get);

#endif /* __TASK_STABILIZER_H__ */

