#ifndef __ESC_MONITOR_INA226_H__
#define __ESC_MONITOR_INA226_H__

#include <rtdef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESC_MONITOR_INA226_SAMPLE_COUNT       100
#define ESC_MONITOR_INA226_SAMPLE_PERIOD_MS   10

#define ESC_MONITOR_STAGE1_CURRENT_MIN 0.020f
#define ESC_MONITOR_STAGE1_CURRENT_MAX 0.026f
#define ESC_MONITOR_STAGE2_CURRENT_MIN 0.045f
#define ESC_MONITOR_STAGE2_CURRENT_MAX 0.051f

typedef enum {
  ESC_MONITOR_INA_STAGE_1 = 0,
  ESC_MONITOR_INA_STAGE_2,
  ESC_MONITOR_INA_STAGE_MAX
} esc_monitor_ina_stage_e;

typedef struct {
  rt_bool_t valid;
  rt_uint16_t sample_count;
  float bus_voltage_median;
  float bus_voltage_mean;
  float shunt_current_median;
  float shunt_current_mean;
  rt_bool_t threshold_pass;
} esc_monitor_ina_stage_result_t;

void esc_monitor_ina226_reset(void);
rt_err_t esc_monitor_ina226_start_stage(esc_monitor_ina_stage_e stage);
rt_bool_t esc_monitor_ina226_stage_ready(esc_monitor_ina_stage_e stage);
const esc_monitor_ina_stage_result_t* esc_monitor_ina226_get_stage_result(esc_monitor_ina_stage_e stage);

#ifdef __cplusplus
}
#endif

#endif  // __ESC_MONITOR_INA226_H__


