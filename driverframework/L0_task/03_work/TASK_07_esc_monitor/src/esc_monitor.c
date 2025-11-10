#include <rtdevice.h>
#include <rtthread.h>

#include "esc_monitor_i2c.h"

#define ESC_MONITOR_THREAD_PRIORITY 7
#define ESC_MONITOR_THREAD_STACK_SIZE 1024
#define ESC_MONITOR_THREAD_TIMESLICE 5

#define ESC_MONITOR_PIN_OUTPUT_NAME "PB.14"
#define ESC_MONITOR_PIN_INPUT_NAME  "PB.15"

#define ESC_MONITOR_OUTPUT_LOW_DURATION_MS 3000
#define ESC_MONITOR_POST_DELAY_MS         2000
#define ESC_MONITOR_INPUT_LOW_DURATION_MS 2000
#define ESC_MONITOR_INPUT_SAMPLE_MS         10
#define ESC_MONITOR_CYCLE_INTERVAL_MS     1000

static rt_thread_t g_esc_monitor_thread = RT_NULL;
static rt_base_t g_pin_output = -1;
static rt_base_t g_pin_input = -1;

static rt_bool_t esc_monitor_check_pb15_low(rt_uint32_t duration_ms) {
  rt_uint32_t elapsed = 0;
  while (elapsed < duration_ms) {
    if (rt_pin_read(g_pin_input) != PIN_LOW) {
      return RT_FALSE;
    }
    rt_thread_mdelay(ESC_MONITOR_INPUT_SAMPLE_MS);
    elapsed += ESC_MONITOR_INPUT_SAMPLE_MS;
  }
  return RT_TRUE;
}

static void esc_monitor_thread_entry(void* parameter) {
  RT_UNUSED(parameter);

  while (1) {
    rt_pin_write(g_pin_output, PIN_LOW);
    rt_thread_mdelay(ESC_MONITOR_OUTPUT_LOW_DURATION_MS);

    rt_pin_write(g_pin_output, PIN_HIGH);
    rt_thread_mdelay(ESC_MONITOR_POST_DELAY_MS);

    rt_bool_t ok = esc_monitor_check_pb15_low(ESC_MONITOR_INPUT_LOW_DURATION_MS);
    esc_monitor_set_detection_result(ok);

    rt_thread_mdelay(ESC_MONITOR_CYCLE_INTERVAL_MS);
  }
}

static rt_err_t esc_monitor_io_init(void) {
  g_pin_output = rt_pin_get(ESC_MONITOR_PIN_OUTPUT_NAME);
  if (g_pin_output == PIN_NONE) {
    rt_kprintf("[ESC_MON] get %s fail\n", ESC_MONITOR_PIN_OUTPUT_NAME);
    return -RT_ERROR;
  }

  g_pin_input = rt_pin_get(ESC_MONITOR_PIN_INPUT_NAME);
  if (g_pin_input == PIN_NONE) {
    rt_kprintf("[ESC_MON] get %s fail\n", ESC_MONITOR_PIN_INPUT_NAME);
    return -RT_ERROR;
  }

  rt_pin_mode(g_pin_output, PIN_MODE_OUTPUT);
  rt_pin_write(g_pin_output, PIN_HIGH);

  rt_pin_mode(g_pin_input, PIN_MODE_INPUT_PULLUP);
  return RT_EOK;
}

static int esc_monitor_init(void) {
  if (esc_monitor_io_init() != RT_EOK) {
    return -RT_ERROR;
  }

  if (esc_monitor_i2c_init() != RT_EOK) {
    rt_kprintf("[ESC_MON] i2c init fail\n");
    return -RT_ERROR;
  }

  g_esc_monitor_thread = rt_thread_create("esc_mon", esc_monitor_thread_entry, RT_NULL,
                                          ESC_MONITOR_THREAD_STACK_SIZE,
                                          ESC_MONITOR_THREAD_PRIORITY,
                                          ESC_MONITOR_THREAD_TIMESLICE);
  if (g_esc_monitor_thread == RT_NULL) {
    rt_kprintf("[ESC_MON] thread create fail\n");
    return -RT_ERROR;
  }

  rt_thread_startup(g_esc_monitor_thread);
  rt_kprintf("[ESC_MON] start, I2C:%s, modbus reg:%d\n",
             WORK_TASK_ESC_MONITOR_I2C_NAME, WORK_TASK_ESC_MONITOR_MODBUS_REG_INDEX);
  return RT_EOK;
}

#ifdef WORK_TASK_ESC_MONITOR_EN
INIT_APP_EXPORT(esc_monitor_init);
#endif

