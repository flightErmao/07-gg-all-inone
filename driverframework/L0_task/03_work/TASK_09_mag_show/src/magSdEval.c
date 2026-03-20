#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <math.h>

#include "magSdEval.h"
#include "rtconfig.h"
#include "qmc6308_debug.h"

#define LOG_TAG "mag_sd_eval"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#ifndef TASK_MAG_SD_EVAL_UART_NAME
#define TASK_MAG_SD_EVAL_UART_NAME "uart1"
#endif

#ifndef TASK_MAG_SD_EVAL_UART_BAUD
#define TASK_MAG_SD_EVAL_UART_BAUD 115200
#endif

#ifndef TASK_MAG_SD_EVAL_STREAM_PERIOD_MS
#define TASK_MAG_SD_EVAL_STREAM_PERIOD_MS 20
#endif

#define MAG_SD_EVAL_LINE_LEN 256

static rt_device_t g_eval_uart = RT_NULL;
static rt_bool_t g_uart_ready = RT_FALSE;
static volatile rt_bool_t g_stream_enabled = RT_FALSE;
static volatile rt_bool_t g_test_active = RT_FALSE;
static volatile uint32_t g_last_output_timestamp_ms = 0;
static volatile rt_bool_t g_config_emitted = RT_FALSE;

static rt_err_t mag_sd_eval_uart_init(void) {
  struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

  if (g_uart_ready == RT_TRUE && g_eval_uart != RT_NULL) {
    return RT_EOK;
  }

  g_eval_uart = rt_device_find(TASK_MAG_SD_EVAL_UART_NAME);
  if (g_eval_uart == RT_NULL) {
    LOG_E("cannot find eval uart: %s", TASK_MAG_SD_EVAL_UART_NAME);
    return -RT_ERROR;
  }

  config.baud_rate = TASK_MAG_SD_EVAL_UART_BAUD;
  if (rt_device_control(g_eval_uart, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK) {
    LOG_E("config eval uart failed: %s", TASK_MAG_SD_EVAL_UART_NAME);
    return -RT_ERROR;
  }

  if (rt_device_open(g_eval_uart, RT_DEVICE_FLAG_TX_BLOCKING) != RT_EOK) {
    LOG_E("open eval uart failed: %s", TASK_MAG_SD_EVAL_UART_NAME);
    return -RT_ERROR;
  }

  g_uart_ready = RT_TRUE;
  LOG_I("eval uart ready: %s @ %d", TASK_MAG_SD_EVAL_UART_NAME, TASK_MAG_SD_EVAL_UART_BAUD);
  return RT_EOK;
}

static void mag_sd_eval_uart_write(const char* line) {
  if (line == RT_NULL || g_uart_ready == RT_FALSE || g_eval_uart == RT_NULL) {
    return;
  }

  rt_device_write(g_eval_uart, 0, line, rt_strlen(line));
}

static void mag_sd_eval_emit_event(const char* event_name, const char* detail) {
  char line[MAG_SD_EVAL_LINE_LEN];
  uint32_t now_ms = rt_tick_get_millisecond();

  if (event_name == RT_NULL) {
    return;
  }

  if (detail != RT_NULL && detail[0] != '\0') {
    rt_snprintf(line, sizeof(line), "MAG_EVENT,%u,%s,%s\r\n", now_ms, event_name, detail);
  } else {
    rt_snprintf(line, sizeof(line), "MAG_EVENT,%u,%s\r\n", now_ms, event_name);
  }

  mag_sd_eval_uart_write(line);
}

static float mag_sd_eval_expected_lsb_from_range(rt_uint16_t range_g) {
  switch (range_g) {
    case 30:
      return 0.1f;
    case 12:
      return 0.04f;
    case 8:
      return 100.0f / 3750.0f;
    case 2:
      return 100.0f / 15000.0f;
    default:
      return 0.0f;
  }
}

static void mag_sd_eval_emit_config_snapshot(void) {
  char detail[256];
  qmc6308_debug_info_t info = {0};
  float expected_lsb = 0.0f;
  int lsb_consistent = 0;

  if (g_uart_ready == RT_FALSE || g_eval_uart == RT_NULL) {
    return;
  }

  if (qmc6308_get_debug_info(&info) != RT_EOK) {
    mag_sd_eval_emit_event("MAG_CONFIG", "driver=qmc6308,available=0");
    return;
  }

  expected_lsb = mag_sd_eval_expected_lsb_from_range(info.range_g);
  if (expected_lsb > 0.0f && fabsf(info.lsb_to_ut - expected_lsb) < 0.0005f) {
    lsb_consistent = 1;
  }

  rt_snprintf(detail, sizeof(detail),
              "driver=qmc6308,available=1,chip_id=0x%02X,ctl_reg_one=0x%02X,ctl_reg_two=0x%02X,ctl_reg_three=0x%02X,"
              "range_g=%u,odr_hz=%u,lsb_to_ut=%.6f,expected_lsb_to_ut=%.6f,lsb_per_g=%u,lsb_consistent=%d",
              info.chip_id,
              info.ctl_reg_one,
              info.ctl_reg_two,
              info.ctl_reg_three,
              info.range_g,
              info.odr_hz,
              info.lsb_to_ut,
              expected_lsb,
              info.lsb_per_g,
              lsb_consistent);
  mag_sd_eval_emit_event("MAG_CONFIG", detail);
}

rt_err_t magSdEvalStartStream(void) {
  char detail[64];
  rt_err_t ret = mag_sd_eval_uart_init();
  if (ret != RT_EOK) {
    return ret;
  }

  g_last_output_timestamp_ms = 0;
  g_stream_enabled = RT_TRUE;
  g_config_emitted = RT_FALSE;

  rt_snprintf(detail, sizeof(detail), "uart=%s,baud=%u,period_ms=%u",
              TASK_MAG_SD_EVAL_UART_NAME,
              TASK_MAG_SD_EVAL_UART_BAUD,
              TASK_MAG_SD_EVAL_STREAM_PERIOD_MS);
  mag_sd_eval_emit_event("STREAM_START", detail);
  mag_sd_eval_emit_config_snapshot();
  g_config_emitted = RT_TRUE;

  return RT_EOK;
}

rt_err_t magSdEvalStopStream(void) {
  if (g_stream_enabled == RT_TRUE && g_uart_ready == RT_TRUE) {
    mag_sd_eval_emit_event("STREAM_STOP", RT_NULL);
  }

  g_stream_enabled = RT_FALSE;
  g_test_active = RT_FALSE;
  return RT_EOK;
}

rt_err_t magSdEvalMarkTestStart(void) {
  rt_err_t ret = RT_EOK;

  if (g_stream_enabled == RT_FALSE) {
    ret = magSdEvalStartStream();
    if (ret != RT_EOK) {
      return ret;
    }
  }

  g_test_active = RT_TRUE;
  mag_sd_eval_emit_event("TEST_START", RT_NULL);
  return RT_EOK;
}

rt_err_t magSdEvalMarkTestStop(void) {
  if (g_stream_enabled == RT_FALSE || g_uart_ready == RT_FALSE) {
    return -RT_ERROR;
  }

  g_test_active = RT_FALSE;
  mag_sd_eval_emit_event("TEST_STOP", RT_NULL);
  return RT_EOK;
}

rt_bool_t magSdEvalIsStreamEnabled(void) {
  return g_stream_enabled;
}

rt_bool_t magSdEvalIsTestActive(void) {
  return g_test_active;
}

const char* magSdEvalGetUartName(void) {
  return TASK_MAG_SD_EVAL_UART_NAME;
}

rt_uint32_t magSdEvalGetBaudRate(void) {
  return TASK_MAG_SD_EVAL_UART_BAUD;
}

rt_uint32_t magSdEvalGetPeriodMs(void) {
  return TASK_MAG_SD_EVAL_STREAM_PERIOD_MS;
}

void magSdEvalOnRawSample(const mag_report_t* raw_data) {
  char line[MAG_SD_EVAL_LINE_LEN];
  uint32_t delta_ms;

  if (raw_data == RT_NULL || g_stream_enabled == RT_FALSE || g_uart_ready == RT_FALSE) {
    return;
  }

  if (g_config_emitted == RT_FALSE) {
    mag_sd_eval_emit_config_snapshot();
    g_config_emitted = RT_TRUE;
  }

  delta_ms = raw_data->timestamp_ms - g_last_output_timestamp_ms;
  if (g_last_output_timestamp_ms != 0 && delta_ms < TASK_MAG_SD_EVAL_STREAM_PERIOD_MS) {
    return;
  }

  g_last_output_timestamp_ms = raw_data->timestamp_ms;

  rt_snprintf(line, sizeof(line), "MAG_RAW,%u,%.3f,%.3f,%.3f\r\n",
              raw_data->timestamp_ms,
              raw_data->value_x,
              raw_data->value_y,
              raw_data->value_z);
  mag_sd_eval_uart_write(line);
}
