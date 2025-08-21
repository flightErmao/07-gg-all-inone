#ifndef TASK_SENSOR_MINIFLY_H
#define TASK_SENSOR_MINIFLY_H

#include <rtthread.h>
#include "axis.h"

rt_err_t sensor_minifly_read_acc(Axis3f *out);
rt_err_t sensor_minifly_read_gyro(Axis3f *out);

#endif

