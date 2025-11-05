/****************************************************************************
 *
 * Rate Control Task Header
 *
 ****************************************************************************/

#ifndef __TASK_RATE_CONTROL_H__
#define __TASK_RATE_CONTROL_H__

#include "rtconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

// Task configuration macros (defined in Kconfig)
#ifndef PROJECT_PX4_TASK_RATE_CONTROL_PERIOD_MS
#define PROJECT_PX4_TASK_RATE_CONTROL_PERIOD_MS 4
#endif

#ifndef PROJECT_PX4_TASK_RATE_CONTROL_PRIORITY
#define PROJECT_PX4_TASK_RATE_CONTROL_PRIORITY 5
#endif

#ifndef PROJECT_PX4_TASK_RATE_CONTROL_STACK_SIZE
#define PROJECT_PX4_TASK_RATE_CONTROL_STACK_SIZE 4096
#endif

#ifdef __cplusplus
}
#endif

#endif /* __TASK_RATE_CONTROL_H__ */

