#ifndef TASK_MINIFLY_DSHOT_H
#define TASK_MINIFLY_DSHOT_H

#include "uMCN.h"
#include <rtthread.h>

/* Publisher interface for mixerControl: publish raw 0~65535 values */
void task_dshot_publish_raw(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

/* API to get mapped motor values for external access */
void task_dshot_get_mapped(uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4);

void getMotorFreq(float* values);

#endif

