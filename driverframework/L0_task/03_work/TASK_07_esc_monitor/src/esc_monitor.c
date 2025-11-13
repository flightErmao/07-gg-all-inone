#include <rtdevice.h>
#include <rtthread.h>
#include <string.h>

#define LOG_TAG "esc_mon"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#include "esc_monitor_i2c.h"
#include "esc_monitor_ina226.h"
#include "INA226.h"

#define ESC_MONITOR_PIN_OUTPUT_NAME "PB.14"
#define ESC_MONITOR_PIN_INPUT_NAME "PB.15"

#define ESC_MONITOR_OUTPUT_LOW_DURATION_MS 1500
#define ESC_MONITOR_POST_DELAY_MS 1300
#define ESC_MONITOR_INPUT_LOW_DURATION_MS 2000
#define ESC_MONITOR_INPUT_SAMPLE_MS 10

typedef enum {
  PHONE_ON_V1P8_LOW_LEVEL = 0,
  PHONE_ON_V1P8_HIGH_LEVEL = 1,
} esc_monitor_state_e;

static rt_base_t pinNameDroneOnOff_ = -1;
static rt_base_t pinNamePhoneOnV1P8_ = -1;
static esc_monitor_ina_stage_result_t state1_ina226_result_ = {0};
static esc_monitor_ina_stage_result_t state2_ina226_result_ = {0};
static rt_bool_t esc_monitor_detection_[ESC_MONITOR_DETECTION_INDEX_MAX] = {RT_FALSE};
static rt_bool_t io_init_done_ = RT_FALSE;

void cmdEscMonResult(int argc, char** argv);
static rt_err_t esc_monitor_io_init(void);

static void esc_monitor_reset_detection_results(void) {
  memset(esc_monitor_detection_, 0, sizeof(esc_monitor_detection_));
  memset(&state1_ina226_result_, 0, sizeof(state1_ina226_result_));
  memset(&state2_ina226_result_, 0, sizeof(state2_ina226_result_));
}

static void esc_monitor_set_detection_result(esc_monitor_detection_index_e index, rt_bool_t value) {
  if (index >= ESC_MONITOR_DETECTION_INDEX_MAX) {
    return;
  }
  esc_monitor_detection_[index] = value ? RT_TRUE : RT_FALSE;
}

static esc_monitor_state_e esc_monitor_phone_v1p8(uint16_t times_span) {
  rt_uint32_t elapsed = 0;
  uint16_t low_count = 0;
  uint16_t high_count = 0;
  while (elapsed < times_span) {
    if (rt_pin_read(pinNamePhoneOnV1P8_) != PIN_LOW) {
      high_count++;
    } else {
      low_count++;
    }
    rt_thread_mdelay(ESC_MONITOR_INPUT_SAMPLE_MS);
    elapsed += ESC_MONITOR_INPUT_SAMPLE_MS;
  }

  LOG_I("times_span %d, PB15 low_count: %d, high_count: %d", times_span, low_count, high_count);
  if (low_count > high_count) {
    return PHONE_ON_V1P8_LOW_LEVEL;
  }
  return PHONE_ON_V1P8_HIGH_LEVEL;
}
static void cmdEscMonitor(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);

  if (esc_monitor_io_init() != RT_EOK) {
    LOG_E("ESC monitor IO init fail");
    return;
  } else {
    LOG_I("ESC monitor IO init success");
  }

  if (ina226_demo_init() != RT_EOK) {
    LOG_E("ina226 demo init fail");
    return;
  } else {
    LOG_I("ina226 init success");
  }

  LOG_I("ESC Phone_V1P8 monitor begin");

  esc_monitor_reset_detection_results();
  esc_monitor_ina226_reset();

  if (esc_monitor_ina226_start_stage(ESC_MONITOR_INA_STAGE_1) != RT_EOK) {
    LOG_W("INA226 stage1 start fail");
  }

  rt_pin_write(pinNameDroneOnOff_, PIN_LOW);
  esc_monitor_state_e state_1 = esc_monitor_phone_v1p8(1000);
  esc_monitor_set_detection_result(ESC_MONITOR_DETECTION_INDEX_STATE1_HIGH,
                                   (state_1 == PHONE_ON_V1P8_HIGH_LEVEL) ? RT_TRUE : RT_FALSE);

  rt_thread_mdelay(500);

  rt_pin_write(pinNameDroneOnOff_, PIN_HIGH);
  esc_monitor_state_e state_2 = esc_monitor_phone_v1p8(1200);
  esc_monitor_set_detection_result(ESC_MONITOR_DETECTION_INDEX_STATE2_LOW,
                                   (state_2 == PHONE_ON_V1P8_LOW_LEVEL) ? RT_TRUE : RT_FALSE);

  rt_thread_mdelay(1000);

  if (esc_monitor_ina226_start_stage(ESC_MONITOR_INA_STAGE_2) != RT_EOK) {
    LOG_W("INA226 stage2 start fail");
  }

  esc_monitor_state_e state_3 = esc_monitor_phone_v1p8(1000);
  esc_monitor_set_detection_result(ESC_MONITOR_DETECTION_INDEX_STATE3_HIGH,
                                   (state_3 == PHONE_ON_V1P8_HIGH_LEVEL) ? RT_TRUE : RT_FALSE);

  const esc_monitor_ina_stage_result_t* stage1_result = esc_monitor_ina226_get_stage_result(ESC_MONITOR_INA_STAGE_1);
  if (stage1_result != RT_NULL) {
    state1_ina226_result_ = *stage1_result;
    esc_monitor_set_detection_result(ESC_MONITOR_DETECTION_INDEX_STAGE1_ANALOG, state1_ina226_result_.threshold_pass);
  } else {
    esc_monitor_set_detection_result(ESC_MONITOR_DETECTION_INDEX_STAGE1_ANALOG, RT_FALSE);
  }

  const esc_monitor_ina_stage_result_t* stage2_result = esc_monitor_ina226_get_stage_result(ESC_MONITOR_INA_STAGE_2);
  if (stage2_result != RT_NULL) {
    state2_ina226_result_ = *stage2_result;
    esc_monitor_set_detection_result(ESC_MONITOR_DETECTION_INDEX_STAGE2_ANALOG, state2_ina226_result_.threshold_pass);
  } else {
    esc_monitor_set_detection_result(ESC_MONITOR_DETECTION_INDEX_STAGE2_ANALOG, RT_FALSE);
  }

  esc_monitor_set_detection_result(ESC_MONITOR_DETECTION_INDEX_FLOW_DONE, RT_TRUE);

  cmdEscMonResult(0, RT_NULL);
}

void cmdEscMonResult(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);

  rt_bool_t results[ESC_MONITOR_DETECTION_INDEX_MAX] = {RT_FALSE};
  if (!esc_monitor_get_detection_result(results, ESC_MONITOR_DETECTION_INDEX_MAX)) {
    LOG_W("get detection result failed");
    return;
  }

  LOG_I("Flow done: %s", results[ESC_MONITOR_DETECTION_INDEX_FLOW_DONE] ? "YES" : "NO");
  LOG_I("State1 high: %s", results[ESC_MONITOR_DETECTION_INDEX_STATE1_HIGH] ? "YES" : "NO");
  LOG_I("State2 low:  %s", results[ESC_MONITOR_DETECTION_INDEX_STATE2_LOW] ? "YES" : "NO");
  LOG_I("State3 high: %s", results[ESC_MONITOR_DETECTION_INDEX_STATE3_HIGH] ? "YES" : "NO");

  const esc_monitor_ina_stage_result_t* stage1 = esc_monitor_ina226_get_stage_result(ESC_MONITOR_INA_STAGE_1);
  if (stage1 == RT_NULL && state1_ina226_result_.valid) {
    stage1 = &state1_ina226_result_;
  }
  if (stage1 != RT_NULL) {
    float power = stage1->bus_voltage_median * stage1->shunt_current_median;
    LOG_I("Stage1 analog: flag=%s, pass=%s, Vmed=%.3f, Imed=%.3f, Pmed=%.3f, Ith=[%.3f, %.3f]",
          results[ESC_MONITOR_DETECTION_INDEX_STAGE1_ANALOG] ? "YES" : "NO", stage1->threshold_pass ? "YES" : "NO",
          stage1->bus_voltage_median, stage1->shunt_current_median, power, ESC_MONITOR_STAGE1_CURRENT_MIN,
          ESC_MONITOR_STAGE1_CURRENT_MAX);
  } else {
    LOG_I("Stage1 analog: flag=%s, no data, Ith=[%.3f, %.3f]",
          results[ESC_MONITOR_DETECTION_INDEX_STAGE1_ANALOG] ? "YES" : "NO", ESC_MONITOR_STAGE1_CURRENT_MIN,
          ESC_MONITOR_STAGE1_CURRENT_MAX);
  }

  const esc_monitor_ina_stage_result_t* stage2 = esc_monitor_ina226_get_stage_result(ESC_MONITOR_INA_STAGE_2);
  if (stage2 == RT_NULL && state2_ina226_result_.valid) {
    stage2 = &state2_ina226_result_;
  }
  if (stage2 != RT_NULL) {
    float power = stage2->bus_voltage_median * stage2->shunt_current_median;
    LOG_I("Stage2 analog: flag=%s, pass=%s, Vmed=%.3f, Imed=%.3f, Pmed=%.3f, Ith=[%.3f, %.3f]",
          results[ESC_MONITOR_DETECTION_INDEX_STAGE2_ANALOG] ? "YES" : "NO", stage2->threshold_pass ? "YES" : "NO",
          stage2->bus_voltage_median, stage2->shunt_current_median, power, ESC_MONITOR_STAGE2_CURRENT_MIN,
          ESC_MONITOR_STAGE2_CURRENT_MAX);
  } else {
    LOG_I("Stage2 analog: flag=%s, no data, Ith=[%.3f, %.3f]",
          results[ESC_MONITOR_DETECTION_INDEX_STAGE2_ANALOG] ? "YES" : "NO", ESC_MONITOR_STAGE2_CURRENT_MIN,
          ESC_MONITOR_STAGE2_CURRENT_MAX);
  }
}

static void esc_monitor_thread_entry(void* parameter) {
  RT_UNUSED(parameter);
  cmdEscMonitor(0, RT_NULL);
}

static rt_err_t esc_monitor_io_init(void) {
  if (io_init_done_) {
    return RT_EOK;
  }
  pinNameDroneOnOff_ = rt_pin_get(ESC_MONITOR_PIN_OUTPUT_NAME);
  if (pinNameDroneOnOff_ == PIN_NONE) {
    LOG_E("get %s fail", ESC_MONITOR_PIN_OUTPUT_NAME);
    return -RT_ERROR;
  }

  pinNamePhoneOnV1P8_ = rt_pin_get(ESC_MONITOR_PIN_INPUT_NAME);
  if (pinNamePhoneOnV1P8_ == PIN_NONE) {
    LOG_E("get %s fail", ESC_MONITOR_PIN_INPUT_NAME);
    return -RT_ERROR;
  }

  rt_pin_mode(pinNameDroneOnOff_, PIN_MODE_OUTPUT);
  rt_pin_write(pinNameDroneOnOff_, PIN_HIGH);

  rt_pin_mode(pinNamePhoneOnV1P8_, PIN_MODE_INPUT_PULLUP);
  io_init_done_ = RT_TRUE;
  return RT_EOK;
}

static int esc_monitor_init(void) {
#define ESC_MONITOR_THREAD_PRIORITY 7
#define ESC_MONITOR_THREAD_STACK_SIZE 2048
#define ESC_MONITOR_THREAD_TIMESLICE 5
  static struct rt_thread g_esc_monitor_tid = {0};
  static rt_uint8_t g_esc_monitor_stack[ESC_MONITOR_THREAD_STACK_SIZE] = {0};

  rt_err_t init_ret =
      rt_thread_init(&g_esc_monitor_tid, "esc_mon", esc_monitor_thread_entry, RT_NULL, g_esc_monitor_stack,
                     ESC_MONITOR_THREAD_STACK_SIZE, ESC_MONITOR_THREAD_PRIORITY, ESC_MONITOR_THREAD_TIMESLICE);
  if (init_ret != RT_EOK) {
    LOG_E("thread init fail");
    return -RT_ERROR;
  }

  rt_thread_startup(&g_esc_monitor_tid);
  LOG_I("start, I2C:%s, modbus reg:%d", WORK_TASK_ESC_MONITOR_I2C_NAME, WORK_TASK_ESC_MONITOR_MODBUS_REG_INDEX);
  return RT_EOK;
}

#ifdef WORK_TASK_ESC_MONITOR_EN
INIT_APP_EXPORT(esc_monitor_init);
// MSH_CMD_EXPORT_ALIAS(esc_monitor_init, esc_monitor_init, ESC monitor init);
#endif

MSH_CMD_EXPORT_ALIAS(cmdEscMonitor, cmdEscMonitor, ESC monitor command);
MSH_CMD_EXPORT_ALIAS(cmdEscMonResult, cmdEscMonResult, ESC monitor show result);

rt_bool_t esc_monitor_get_detection_result(rt_bool_t* results, rt_size_t length) {
  if (results == RT_NULL || length < ESC_MONITOR_DETECTION_INDEX_MAX) {
    return RT_FALSE;
  }

  rt_base_t level = rt_hw_interrupt_disable();
  for (rt_size_t i = 0; i < ESC_MONITOR_DETECTION_INDEX_MAX; ++i) {
    results[i] = esc_monitor_detection_[i];
  }
  rt_hw_interrupt_enable(level);
  return RT_TRUE;
}