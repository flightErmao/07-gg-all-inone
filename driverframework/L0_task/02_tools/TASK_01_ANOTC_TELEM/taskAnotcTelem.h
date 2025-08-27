#ifndef __TASK_ANOTC_TELEM_H__
#define __TASK_ANOTC_TELEM_H__

#include "protocolAtkpInterface.h"

typedef void (*sensor_data_send_func_t)(uint16_t count_ms);

void anotcTelemAddSensorFunc(sensor_data_send_func_t func);

void anotcMqStash(atkp_t *p);

#endif
