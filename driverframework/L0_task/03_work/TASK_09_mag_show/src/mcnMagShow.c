#include "mcnMagShow.h"
#include "rtconfig.h"
#include <math.h>

#define LOG_TAG "mag_mcn"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define MAG_REF_FIELD_NT 48909.8f
#define MAG_REF_FIELD_UT (MAG_REF_FIELD_NT / 1000.0f)
#define MAG_REF_FIELD_MGS (MAG_REF_FIELD_UT * 10.0f)

/* LSB to mGs conversion: 3000 LSB = 1G = 1000 mGs */
#define MAG_LSB_PER_G 3000.0f
#define MAG_LSB_TO_MGS (1000.0f / MAG_LSB_PER_G)  /* 1 LSB = 0.333333 mGs */

/* MCN topic definition */
MCN_DEFINE(mag, sizeof(mag_report_t));
MCN_DEFINE(mag_raw_data, sizeof(mag_report_t));

/* MCN subscriber node */
static McnNode_t mag_sub_node = RT_NULL;
static McnNode_t mag_raw_sub_node = RT_NULL;

/* Previous magnetometer data for difference calculation */
static mag_report_t prev_mag_data = {0};
static rt_bool_t has_prev_data = RT_FALSE;

/* Calculate magnitude */
static float calculate_magnitude(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

/* Echo function for magnetometer data */
static int mag_echo(void* parameter) {
  mag_report_t mag_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &mag_data) != RT_EOK) {
    return -1;
  }

  /* Calculate current magnitude */
  float magnitude = calculate_magnitude(mag_data.value_x, mag_data.value_y, mag_data.value_z);
  
  /* Calculate difference if previous data exists */
  float diff_x = 0.0f, diff_y = 0.0f, diff_z = 0.0f;
  float diff_magnitude = 0.0f;
  float diff_magnitude_percent = 0.0f;
  float prev_magnitude = 0.0f;
  
  if (has_prev_data) {
    /* Calculate axis differences */
    diff_x = mag_data.value_x - prev_mag_data.value_x;
    diff_y = mag_data.value_y - prev_mag_data.value_y;
    diff_z = mag_data.value_z - prev_mag_data.value_z;
    
    /* Calculate difference magnitude */
    diff_magnitude = calculate_magnitude(diff_x, diff_y, diff_z);
    
    /* Calculate previous magnitude */
    prev_magnitude = calculate_magnitude(prev_mag_data.value_x, 
                                         prev_mag_data.value_y, 
                                         prev_mag_data.value_z);
    
    /* Calculate percentage relative to previous magnitude */
    if (prev_magnitude > 0.001f) {
      diff_magnitude_percent = (diff_magnitude / prev_magnitude) * 100.0f;
    }
  }

  /* Print in 3 lines: previous, current, and difference */
  if (has_prev_data) {
    /* First line: Previous data */
    LOG_I("PREV[%s]: X=%.3f uT Y=%.3f uT Z=%.3f uT Magnitude=%.3f uT timestamp=%ums",
          TASK_MAG_DEVICE_NAME,
          prev_mag_data.value_x,
          prev_mag_data.value_y,
          prev_mag_data.value_z,
          prev_magnitude,
          prev_mag_data.timestamp_ms);
    
    /* Second line: Current data */
    LOG_I("CURR[%s]: X=%.3f uT Y=%.3f uT Z=%.3f uT Magnitude=%.3f uT timestamp=%ums",
          TASK_MAG_DEVICE_NAME,
          mag_data.value_x,
          mag_data.value_y,
          mag_data.value_z,
          magnitude,
          mag_data.timestamp_ms);
    
    /* Third line: Difference data */
    LOG_I("DIFF[%s]: X=%.3f uT Y=%.3f uT Z=%.3f uT Magnitude=%.3f uT (%.2f%%)",
          TASK_MAG_DEVICE_NAME,
          diff_x,
          diff_y,
          diff_z,
          diff_magnitude,
          diff_magnitude_percent);
  } else {
    /* First time, only print current data */
    LOG_I("CURR[%s]: X=%.3f uT Y=%.3f uT Z=%.3f uT Magnitude=%.3f uT timestamp=%ums",
          TASK_MAG_DEVICE_NAME,
          mag_data.value_x,
          mag_data.value_y,
          mag_data.value_z,
          magnitude,
          mag_data.timestamp_ms);
  }

  /* Update previous data */
  prev_mag_data = mag_data;
  has_prev_data = RT_TRUE;

  return 0;
}

/* Echo function for raw magnetometer data */
static int mag_raw_echo(void* parameter) {
  mag_report_t raw_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &raw_data) != RT_EOK) {
    return -1;
  }

  /* Convert LSB to mGs */
  float x_mgs = raw_data.value_x * MAG_LSB_TO_MGS;
  float y_mgs = raw_data.value_y * MAG_LSB_TO_MGS;
  float z_mgs = raw_data.value_z * MAG_LSB_TO_MGS;
  
  /* Calculate magnitude in mGs */
  float magnitude_mgs = calculate_magnitude(x_mgs, y_mgs, z_mgs);

  /* Print raw data: LSB values, mGs values and magnitude in one line */
  LOG_I("LSB: X=%.1f Y=%.1f Z=%.1f | mGs: X=%.1f Y=%.1f Z=%.1f Mag=%.1f",
        raw_data.value_x,
        raw_data.value_y,
        raw_data.value_z,
        x_mgs,
        y_mgs,
        z_mgs,
        magnitude_mgs);

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

/* Initialize MCN raw magnetometer data reporting */
int mcnMagRawDataInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(mag_raw_data), mag_raw_echo);
  if (result != RT_EOK) {
    LOG_E("Failed to advertise mag_raw_data topic: %d", result);
    return -1;
  }

  mag_raw_sub_node = mcn_subscribe(MCN_HUB(mag_raw_data), RT_NULL, RT_NULL);
  if (mag_raw_sub_node == RT_NULL) {
    LOG_E("Failed to subscribe to mag_raw_data topic");
    return -1;
  }

  return 0;
}

/* Publish raw magnetometer data to MCN */
int mcnMagRawDataPublish(const mag_report_t* raw_data) {
  if (!raw_data) {
    return -1;
  }

  return mcn_publish(MCN_HUB(mag_raw_data), raw_data);
}

/* Acquire raw magnetometer data from MCN */
int mcnMagRawDataAcquire(mag_report_t* raw_data) {
  if (!raw_data) {
    return -1;
  }
  mcn_copy(MCN_HUB(mag_raw_data), mag_raw_sub_node, raw_data);

  return 0;
}

