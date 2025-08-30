#ifndef _COMMAND_MINIFLY_H_
#define _COMMAND_MINIFLY_H_

#include "stabilizerTypes.h"
#include "taskRc.h"
/**
 * @brief Acquire command from RC data
 * @param rc_data RC data structure
 * @param setpoint Setpoint structure to be filled
 */
void commanderGetSetpoint(const pilot_cmd_bus_t* rc_data, setpoint_t* setpoint);

#endif