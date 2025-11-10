#include <rtdevice.h>
#include <rtthread.h>

#include "esc_monitor_i2c.h"

#define ESC_MONITOR_THREAD_PRIORITY 7
#define ESC_MONITOR_THREAD_STACK_SIZE 1024
#define ESC_MONITOR_THREAD_TIMESLICE 5

#define ESC_MONITOR_PIN_OUTPUT_NAME "PB.14"
#define ESC_MONITOR_PIN_INPUT_NAME "PB.15"

#define ESC_MONITOR_OUTPUT_LOW_DURATION_MS 1500
#define ESC_MONITOR_POST_DELAY_MS 1300
#define ESC_MONITOR_INPUT_LOW_DURATION_MS 2000
#define ESC_MONITOR_INPUT_SAMPLE_MS 10

static rt_thread_t g_esc_monitor_thread = RT_NULL;
static rt_base_t pinNameDroneOnOff_ = -1;
static rt_base_t pinNamePhoneOnV1P8_ = -1;
static rt_bool_t escMonitorResult_ = RT_FALSE;

static rt_bool_t esc_monitor_check_pb15_low(void) {
  rt_uint32_t elapsed = 0;
  uint16_t low_count = 0;
  uint16_t high_count = 0;
  while (elapsed < 1200) {
    if (rt_pin_read(pinNamePhoneOnV1P8_) != PIN_LOW) {
      high_count++;
    } else {
      low_count++;
    }
    rt_thread_mdelay(ESC_MONITOR_INPUT_SAMPLE_MS);
    elapsed += ESC_MONITOR_INPUT_SAMPLE_MS;
  }

  rt_kprintf("[ESC_MON] PB15 low_count: %d, high_count: %d\n", low_count, high_count);
  if (low_count > high_count) {
    return RT_TRUE;
  }
  return RT_FALSE;
}

static void esc_monitor_thread_entry(void* parameter) {
  RT_UNUSED(parameter);

  rt_thread_mdelay(1000);
  rt_pin_write(pinNameDroneOnOff_, PIN_LOW);
  rt_thread_mdelay(1500);
  rt_pin_write(pinNameDroneOnOff_, PIN_HIGH);

  escMonitorResult_ = esc_monitor_check_pb15_low();

  g_esc_monitor_thread = RT_NULL;
  return;
}

static rt_err_t esc_monitor_io_init(void) {
  pinNameDroneOnOff_ = rt_pin_get(ESC_MONITOR_PIN_OUTPUT_NAME);
  if (pinNameDroneOnOff_ == PIN_NONE) {
    rt_kprintf("[ESC_MON] get %s fail\n", ESC_MONITOR_PIN_OUTPUT_NAME);
    return -RT_ERROR;
  }

  pinNamePhoneOnV1P8_ = rt_pin_get(ESC_MONITOR_PIN_INPUT_NAME);
  if (pinNamePhoneOnV1P8_ == PIN_NONE) {
    rt_kprintf("[ESC_MON] get %s fail\n", ESC_MONITOR_PIN_INPUT_NAME);
    return -RT_ERROR;
  }

  rt_pin_mode(pinNameDroneOnOff_, PIN_MODE_OUTPUT);
  rt_pin_write(pinNameDroneOnOff_, PIN_HIGH);

  rt_pin_mode(pinNamePhoneOnV1P8_, PIN_MODE_INPUT_PULLUP);
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

  g_esc_monitor_thread = rt_thread_create("esc_mon", esc_monitor_thread_entry, RT_NULL, ESC_MONITOR_THREAD_STACK_SIZE,
                                          ESC_MONITOR_THREAD_PRIORITY, ESC_MONITOR_THREAD_TIMESLICE);
  if (g_esc_monitor_thread == RT_NULL) {
    rt_kprintf("[ESC_MON] thread create fail\n");
    return -RT_ERROR;
  }

  rt_thread_startup(g_esc_monitor_thread);
  rt_kprintf("[ESC_MON] start, I2C:%s, modbus reg:%d\n", WORK_TASK_ESC_MONITOR_I2C_NAME,
             WORK_TASK_ESC_MONITOR_MODBUS_REG_INDEX);
  return RT_EOK;
}

#ifdef WORK_TASK_ESC_MONITOR_EN
INIT_APP_EXPORT(esc_monitor_init);
#endif

static void cmdEscMonitor(int argc, char** argv) {
  rt_kprintf("DRONE ON/OFF LOW LEVEL %d begin \n", ESC_MONITOR_OUTPUT_LOW_DURATION_MS);
  rt_pin_write(pinNameDroneOnOff_, PIN_LOW);
  rt_thread_mdelay(ESC_MONITOR_OUTPUT_LOW_DURATION_MS);
  rt_pin_write(pinNameDroneOnOff_, PIN_HIGH);
  rt_kprintf("DRONE ON/OFF LOW LEVEL %d end \n", ESC_MONITOR_OUTPUT_LOW_DURATION_MS);
  escMonitorResult_ = esc_monitor_check_pb15_low();
}
MSH_CMD_EXPORT_ALIAS(cmdEscMonitor, cmdEscMonitor, ESC monitor command);

rt_bool_t esc_monitor_get_detection_result(rt_bool_t* result) {
  if (result == RT_NULL) {
    return RT_FALSE;
  }

  *result = escMonitorResult_;
  return RT_TRUE;
}