#include "mcnBaroShow.h"
#include "floatConvert.h"
#include "rtconfig.h"

/* MCN topic definition */
MCN_DEFINE(baro, sizeof(baro_report_t));

/* MCN subscriber node */
static McnNode_t baro_sub_node = RT_NULL;

/* Echo function for barometer data */
static int baro_echo(void* parameter) {
  baro_report_t baro_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &baro_data) != RT_EOK) {
    return -1;
  }

  char pressure_str[16], temperature_str[16], timestamp_str[16];
  float_to_string(baro_data.pressure_Pa, pressure_str, sizeof(pressure_str));
  float_to_string(baro_data.temperature_deg, temperature_str, sizeof(temperature_str));
  rt_snprintf(timestamp_str, sizeof(timestamp_str), "%u", baro_data.timestamp_ms);

  rt_kprintf("BARO[%s]: P=%sPa T=%sdeg timestamp=%sms\n", TASK_BARO_DEVICE_NAME, pressure_str, temperature_str,
             timestamp_str);

  return 0;
}

/* Initialize MCN barometer reporting */
int mcnBaroReportInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(baro), baro_echo);
  if (result != RT_EOK) {
    rt_kprintf("Failed to advertise baro topic: %d\n", result);
    return -1;
  }

  baro_sub_node = mcn_subscribe(MCN_HUB(baro), RT_NULL, RT_NULL);
  if (baro_sub_node == RT_NULL) {
    rt_kprintf("Failed to subscribe to baro topic\n");
    return -1;
  }

  return 0;
}

/* Publish barometer data to MCN */
int mcnBaroReportPublish(const baro_report_t* baro_data) {
  if (!baro_data) {
    return -1;
  }

  return mcn_publish(MCN_HUB(baro), baro_data);
}

/* Acquire barometer data from MCN */
int mcnBaroReportAcquire(baro_report_t* baro_data) {
  if (!baro_data) {
    return -1;
  }
  // if (mcn_poll(baro_sub_node)) {
  //     return mcn_copy(MCN_HUB(baro), baro_sub_node, baro_data);
  // }
  mcn_copy(MCN_HUB(baro), baro_sub_node, baro_data);

  return 0;
}