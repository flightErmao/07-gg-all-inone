#include "aMcnDshot.h"
#include "floatConvert.h"
#include "rtconfig.h"

/* ==================== RPM Data MCN ==================== */
#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN

MCN_DECLARE(rpm);
MCN_DEFINE(rpm, sizeof(rpm_data_bus_t));
static McnNode_t rpm_data_sub_node = RT_NULL;

/* Echo function for RPM data */
static int rpm_data_echo(void* parameter) {
  rpm_data_bus_t data;
  if (mcn_copy_from_hub((McnHub*)parameter, &data) != RT_EOK) {
    return -1;
  }
  
  char r1[16], r2[16], r3[16], r4[16];
  float_to_string(data.rpm[0], r1, sizeof(r1));
  float_to_string(data.rpm[1], r2, sizeof(r2));
  float_to_string(data.rpm[2], r3, sizeof(r3));
  float_to_string(data.rpm[3], r4, sizeof(r4));
  
  rt_kprintf("[aMcnDshot] RPM: %s, %s, %s, %s, ts: %lu\n", 
             r1, r2, r3, r4, data.timestamp);
  
  return 0;
}

/* Initialize MCN rpm data */
static int mcnRpmDataInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(rpm), rpm_data_echo);
  if (result != RT_EOK) {
    rt_kprintf("[aMcnDshot] Failed to advertise rpm topic: %d\n", result);
    return -1;
  }

  rpm_data_sub_node = mcn_subscribe(MCN_HUB(rpm), RT_NULL, RT_NULL);
  if (rpm_data_sub_node == RT_NULL) {
    rt_kprintf("[aMcnDshot] Failed to subscribe to rpm topic\n");
    return -1;
  }
  
  rt_kprintf("[aMcnDshot] RPM data MCN initialized\n");
  return 0;
}

/* Publish RPM data to MCN */
int mcnRpmDataPublish(const rpm_data_bus_t* rpm) {
  if (!rpm) {
    return -1;
  }

  return mcn_publish(MCN_HUB(rpm), rpm);
}

/* Acquire RPM data from MCN */
int mcnRpmDataAcquire(rpm_data_bus_t* rpm) {
  if (!rpm) {
    return -1;
  }

  if (rpm_data_sub_node == RT_NULL) {
    return -1;
  }
  return mcn_copy(MCN_HUB(rpm), rpm_data_sub_node, rpm);
}

INIT_COMPONENT_EXPORT(mcnRpmDataInit);
#endif
