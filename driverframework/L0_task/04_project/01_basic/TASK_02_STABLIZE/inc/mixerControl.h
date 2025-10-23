#ifndef __MIXER_CONTROL_H
#define __MIXER_CONTROL_H

#include "stabilizerTypes.h"
#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Motor direction enumeration */
typedef enum {
  MOTOR_DIR_CW = 0,
  MOTOR_DIR_CCW = 1,
} motor_dir_e;

/* Motor PWM value structure with union for flexible access */
typedef union {
  struct {
    rt_uint16_t m1;
    rt_uint16_t m2;
    rt_uint16_t m3;
    rt_uint16_t m4;
  };
  rt_uint16_t val[4];
} motor_pwm_t;

/* Public functions */
void mixerControl(control_t* control);
void motorInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __MIXER_CONTROL_H */
