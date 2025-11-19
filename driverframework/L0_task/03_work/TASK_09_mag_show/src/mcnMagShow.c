#include "mcnMagShow.h"
#include "rtconfig.h"
#include <math.h>

#define LOG_TAG "mag_mcn"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define MAG_REF_FIELD_NT 48909.8f
#define MAG_REF_FIELD_UT (MAG_REF_FIELD_NT / 1000.0f)
#define MAG_REF_FIELD_MGS (MAG_REF_FIELD_UT * 10.0f)

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

  /* Calculate magnitude (地磁长模长强度) */
  float magnitude = sqrtf(mag_data.value_x * mag_data.value_x + 
                         mag_data.value_y * mag_data.value_y + 
                         mag_data.value_z * mag_data.value_z);
  
  float relative_diff_percent = 0.0f;
  if (MAG_REF_FIELD_UT > 0.0f) {
    relative_diff_percent = ((magnitude - MAG_REF_FIELD_UT) / MAG_REF_FIELD_UT) * 100.0f;
  }

  LOG_I(
      "MAG[%s]: X=%.3f uT Y=%.3f uT Z=%.3f uT Magnitude=%.3f uT Ref=%.3f uT Diff=%.2f%% timestamp=%ums",
      TASK_MAG_DEVICE_NAME,
      mag_data.value_x,
      mag_data.value_y,
      mag_data.value_z,
      magnitude,
      MAG_REF_FIELD_UT,
      relative_diff_percent,
      mag_data.timestamp_ms);

  return 0;
}

/* Initialize MCN magnetometer reporting */
int mcnMagReportInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(mag), mag_echo);
  if (result != RT_EOK) {
    LOG_E("Failed to advertise mag topic: %d", result);
    return -1;
  }

  mag_sub_node = mcn_subscribe(MCN_HUB(mag), RT_NULL, RT_NULL);
  if (mag_sub_node == RT_NULL) {
    LOG_E("Failed to subscribe to mag topic");
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

