#ifndef __QMC6308_DEBUG_H__
#define __QMC6308_DEBUG_H__

#include <rtdef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  rt_uint8_t chip_id;
  rt_uint8_t ctl_reg_one;
  rt_uint8_t ctl_reg_two;
  rt_uint8_t ctl_reg_three;
  rt_uint16_t range_g;
  rt_uint16_t odr_hz;
  float lsb_to_ut;
  rt_uint16_t lsb_per_g;
} qmc6308_debug_info_t;

rt_err_t qmc6308_get_debug_info(qmc6308_debug_info_t* info);

#ifdef __cplusplus
}
#endif

#endif /* __QMC6308_DEBUG_H__ */
