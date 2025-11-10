#ifndef __ESC_MONITOR_I2C_H__
#define __ESC_MONITOR_I2C_H__

#include <rtdef.h>

#include "I2cInterface.h"

rt_err_t esc_monitor_i2c_init(void);
const I2cInterface_t* esc_monitor_get_i2c_interface(void);
void esc_monitor_set_detection_result(rt_bool_t success);
rt_bool_t esc_monitor_get_detection_result(rt_bool_t* result);

#endif

