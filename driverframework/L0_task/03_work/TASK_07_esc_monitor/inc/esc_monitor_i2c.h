#ifndef __ESC_MONITOR_I2C_H__
#define __ESC_MONITOR_I2C_H__

#include <rtdef.h>

#include "I2cInterface.h"

rt_err_t esc_monitor_i2c_init(void);
const I2cInterface_t* esc_monitor_get_i2c_interface(void);

typedef enum {
  ESC_MONITOR_DETECTION_INDEX_FLOW_DONE = 0,
  ESC_MONITOR_DETECTION_INDEX_STATE1_HIGH,
  ESC_MONITOR_DETECTION_INDEX_STATE2_LOW,
  ESC_MONITOR_DETECTION_INDEX_STATE3_HIGH,
  ESC_MONITOR_DETECTION_INDEX_STAGE1_BUS_VOLTAGE_X10,
  ESC_MONITOR_DETECTION_INDEX_STAGE1_CURRENT_MA,
  ESC_MONITOR_DETECTION_INDEX_STAGE2_BUS_VOLTAGE_X10,
  ESC_MONITOR_DETECTION_INDEX_STAGE2_CURRENT_MA,
  ESC_MONITOR_DETECTION_INDEX_MAX
} esc_monitor_detection_index_e;

#define ESC_MONITOR_CURRENT_SCALE_MA 1000U
#define ESC_MONITOR_BUS_VOLTAGE_SCALE_X10 10U

rt_bool_t esc_monitor_get_detection_result(rt_uint16_t* results, rt_size_t length);

#endif

