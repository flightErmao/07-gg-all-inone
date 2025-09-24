#ifndef TASK_MINIFLY_DSHOT_H
#define TASK_MINIFLY_DSHOT_H

#include "uMCN.h"
#include <rtthread.h>

typedef struct {
  uint16_t motor_val[4]; /* 0~65535 from mixerControl */
  uint32_t timestamp;
} dshot_cmd_bus_t;

/* uMCN topic extern (declared here, defined in taskDshot.c) */
MCN_DECLARE(dshot_cmd);

/* Publisher interface for mixerControl: publish raw 0~65535 values */
void task_dshot_publish_raw(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#endif

