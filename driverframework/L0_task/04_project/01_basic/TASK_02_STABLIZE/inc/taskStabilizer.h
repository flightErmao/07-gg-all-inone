#ifndef __TASK_STABILIZER_H__
#define __TASK_STABILIZER_H__

#include "stabilizerTypes.h"

/**
 * @brief Get current setpoint
 * @param setpoint Pointer to setpoint structure to be filled with current values
 */
void commanderGetCurrentSetpoint(setpoint_t* setpoint);

#endif /* __TASK_STABILIZER_H__ */

