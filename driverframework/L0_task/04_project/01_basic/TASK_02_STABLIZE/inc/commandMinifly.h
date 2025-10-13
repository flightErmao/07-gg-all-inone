#ifndef _COMMAND_MINIFLY_H_
#define _COMMAND_MINIFLY_H_

#include "stabilizerTypes.h"

// #if defined(PROJECT_MINIFLY_TASK06_RC_EN) || defined(PROJECT_FMT_TASK01_RC_EN)
#include "taskRc.h"

/**
 * @brief Acquire command from RC data
 * @param rc_data RC data structure
 * @param setpoint Setpoint structure to be filled
 */
void commanderGetSetpoint(const pilot_cmd_bus_t* rc_data, setpoint_t* setpoint);

/**
 * @brief Get current setpoint
 * @param setpoint Pointer to setpoint structure to be filled with current values
 */
void commanderGetCurrentSetpoint(setpoint_t* setpoint);

#endif

// #endif