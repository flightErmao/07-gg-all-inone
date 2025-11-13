#include <rtthread.h>

#define LOG_TAG "esc_i2c"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#include "esc_monitor_i2c.h"

static I2cInterface_t g_esc_i2c_interface = {0};

rt_err_t esc_monitor_i2c_init(void) {
  rt_err_t result = get_i2c_interface(WORK_TASK_ESC_MONITOR_I2C_NAME,
                                      WORK_TASK_ESC_MONITOR_I2C_ADDR,
                                      &g_esc_i2c_interface);
  if (result != RT_EOK) {
    LOG_E("get interface fail");
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
