#include "mcnMagShow.h"
#include "rtconfig.h"

static void mag_format_float(char* out, rt_size_t len, float value) {
  if (out == RT_NULL || len == 0) {
    return;
  }
  int negative = (value < 0.0f);
  float abs_val = negative ? -value : value;
  long integer = (long)abs_val;
  float fractional = abs_val - (float)integer;
  long frac = (long)(fractional * 1000.0f + 0.5f);
  if (frac >= 1000) {
    integer += 1;
    frac -= 1000;
  }
  rt_snprintf(out, len, "%s%ld.%03ld", negative ? "-" : "", integer, frac);
}

/* MCN topic definition */
MCN_DEFINE(mag, sizeof(mag_report_t));

/* MCN subscriber node */
static McnNode_t mag_sub_node = RT_NULL;

/* Echo function for magnetometer data */
static int mag_echo(void* parameter) {
  mag_report_t mag_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &mag_data) != RT_EOK) {
    return -1;
  }

  char value_x_str[16], value_y_str[16], value_z_str[16], timestamp_str[16];
  mag_format_float(value_x_str, sizeof(value_x_str), mag_data.value_x);
  mag_format_float(value_y_str, sizeof(value_y_str), mag_data.value_y);
  mag_format_float(value_z_str, sizeof(value_z_str), mag_data.value_z);
  rt_snprintf(timestamp_str, sizeof(timestamp_str), "%u", mag_data.timestamp_ms);

  rt_kprintf("MAG[%s]: X=%s Y=%s Z=%s timestamp=%sms\n", TASK_MAG_DEVICE_NAME, value_x_str, value_y_str,
             value_z_str, timestamp_str);

  return 0;
}

/* Initialize MCN magnetometer reporting */
int mcnMagReportInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(mag), mag_echo);
  if (result != RT_EOK) {
    rt_kprintf("Failed to advertise mag topic: %d\n", result);
    return -1;
  }

  mag_sub_node = mcn_subscribe(MCN_HUB(mag), RT_NULL, RT_NULL);
  if (mag_sub_node == RT_NULL) {
    rt_kprintf("Failed to subscribe to mag topic\n");
    return -1;
  }

  return 0;
}

/* Publish magnetometer data to MCN */
int mcnMagReportPublish(const mag_report_t* mag_data) {
  if (!mag_data) {
    return -1;
  }

  return mcn_publish(MCN_HUB(mag), mag_data);
}

/* Acquire magnetometer data from MCN */
int mcnMagReportAcquire(mag_report_t* mag_data) {
  if (!mag_data) {
    return -1;
  }
  mcn_copy(MCN_HUB(mag), mag_sub_node, mag_data);

  return 0;
}

