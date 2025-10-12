#ifndef TASK_MINIFLY_DSHOT_H
#define TASK_MINIFLY_DSHOT_H

#include <rtthread.h>

/* API to get mapped motor values for external access */
void task_dshot_get_mapped(uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4);

#endif

