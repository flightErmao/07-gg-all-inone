#ifndef TASK_MINIFLY_DSHOT_H
#define TASK_MINIFLY_DSHOT_H

#include <rtthread.h>
#include "rtconfig.h"

/* Conditional header includes */
#ifdef PROJECT_MINIFLY_TASK_DSHOT_MOTOR_FILTER_EN
#include "motorFilter.h"
#endif

#include "debugPin.h"

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_EN
#include "aMcnStabilize.h"
#endif

#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
#include "aMcnDshot.h"
#endif

#include "aMlogDshot.h"

/* API to get mapped motor values for external access */
void task_dshot_get_mapped(uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4);

#endif

