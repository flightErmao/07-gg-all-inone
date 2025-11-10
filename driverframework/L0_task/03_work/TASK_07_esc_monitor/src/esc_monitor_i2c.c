#include <rtthread.h>

#include "esc_monitor_i2c.h"

static I2cInterface_t g_esc_i2c_interface = {0};
static rt_bool_t g_detection_result = RT_FALSE;
static rt_bool_t g_detection_valid = RT_FALSE;

rt_err_t esc_monitor_i2c_init(void) {
  rt_err_t result = get_i2c_interface(WORK_TASK_ESC_MONITOR_I2C_NAME,
                                      WORK_TASK_ESC_MONITOR_I2C_ADDR,
                                      &g_esc_i2c_interface);
  if (result != RT_EOK) {
    rt_kprintf("[ESC_MON_I2C] get interface fail\n");
    return result;
  }
  return RT_EOK;
}

const I2cInterface_t* esc_monitor_get_i2c_interface(void) {
  if (g_esc_i2c_interface.i2c_dev == RT_NULL) {
    return RT_NULL;
  }
  return &g_esc_i2c_interface;
}

void esc_monitor_set_detection_result(rt_bool_t success) {
  rt_base_t level = rt_hw_interrupt_disable();
  g_detection_result = success;
  g_detection_valid = RT_TRUE;
  rt_hw_interrupt_enable(level);
}

rt_bool_t esc_monitor_get_detection_result(rt_bool_t* result) {
  if (result == RT_NULL) {
    return RT_FALSE;
  }

  rt_base_t level = rt_hw_interrupt_disable();
  if (!g_detection_valid) {
    rt_hw_interrupt_enable(level);
    return RT_FALSE;
  }
  *result = g_detection_result;
  rt_hw_interrupt_enable(level);
  return RT_TRUE;
}

