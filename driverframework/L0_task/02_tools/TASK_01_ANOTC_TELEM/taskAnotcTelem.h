#ifndef __TASK_ANOTC_TELEM_H__
#define __TASK_ANOTC_TELEM_H__

#include "protocolAtkpInterface.h"

/* 传感器数据发送函数类型定义 */
typedef void (*sensor_data_send_func_t)(uint16_t count_ms);

void anotcTelemAddSensorFunc(sensor_data_send_func_t func);

/* 消息队列接口 */
void anotcMqStash(atkp_t *p);

#endif
