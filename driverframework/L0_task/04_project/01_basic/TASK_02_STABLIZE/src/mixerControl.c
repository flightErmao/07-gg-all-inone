#include "mixerControl.h"
#include "rtconfig.h"
#include <string.h>
#include "maths.h"
#include "aMcnStabilize.h"

#ifdef L2_DEVICE_03_MOTOR_01_PWM_EN
#include "motorsPwm.h"
#endif

#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
#include "actuator.h"
#endif

/* Private constants */
#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
#define MOTOR_DEVICE_NAME "aux_out"
#endif

/* Private variables */
static motor_dir_e front_right_dir_ = MOTOR_DIR_CCW;

#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
static rt_device_t motor_device_ = RT_NULL;
static bool motor_device_initialized_ = false;
#endif

/* Private function declarations */
#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
static int motor_device_init(void);
#endif
static uint16_t limit_thrust(int value);
static motor_pwm_t calculate_motor_mix(const control_t *control, int16_t yaw_value);

/* Function implementations */
#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
static int motor_device_init(void) {
  if (motor_device_initialized_) {
    return 0;
  }

  motor_device_ = rt_device_find(MOTOR_DEVICE_NAME);
  if (motor_device_ == RT_NULL) {
    rt_kprintf("[mixerControl] can't find device: %s\n", MOTOR_DEVICE_NAME);
    return -1;
  }

  rt_err_t ret = rt_device_open(motor_device_, RT_DEVICE_OFLAG_RDWR);
  if (ret != RT_EOK) {
    rt_kprintf("[mixerControl] open device failed: %d\n", ret);
    return -1;
  }

  motor_device_initialized_ = true;
  rt_kprintf("[mixerControl] device initialized successfully\n");
  return 0;
}
#endif

static uint16_t limit_thrust(int value) {
  if (value > UINT16_MAX) {
    value = UINT16_MAX;
  } else if (value < 0) {
    value = 0;
  }
  return (uint16_t)value;
}

static motor_pwm_t calculate_motor_mix(const control_t *control, int16_t yaw_value) {
  motor_pwm_t motor_pwm;
  
  /* Mix control inputs to motor outputs */
  motor_pwm.m1 = limit_thrust(control->thrust - control->roll - control->pitch + yaw_value);
  motor_pwm.m2 = limit_thrust(control->thrust - control->roll + control->pitch - yaw_value);
  motor_pwm.m3 = limit_thrust(control->thrust + control->roll + control->pitch + yaw_value);
  motor_pwm.m4 = limit_thrust(control->thrust + control->roll - control->pitch - yaw_value);
  
  return motor_pwm;
}

void mixerControl(control_t *control) {
  motor_pwm_t motor_pwm = {0};

  if (control == NULL) {
    rt_kprintf("[mixerControl] Error: control pointer is NULL\n");
    return;
  }

#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
  /* Initialize motor device if not already done */
  if (!motor_device_initialized_) {
    if (motor_device_init() != 0) {
      return;
    }
  }
#endif

  /* Calculate yaw value based on motor direction */
  int16_t yaw_value = (front_right_dir_ == MOTOR_DIR_CW) ? control->yaw : -control->yaw;

  /* Calculate motor PWM values */
  motor_pwm = calculate_motor_mix(control, yaw_value);

#ifdef PROJECT_MINIFLY_TASK_DSHOT_EN
  /* Publish motor values to DShot task via uMCN */
  mixer_data_t mixer_data_pub;
  mixer_data_pub.motor_val[0] = motor_pwm.m1;
  mixer_data_pub.motor_val[1] = motor_pwm.m2;
  mixer_data_pub.motor_val[2] = motor_pwm.m3;
  mixer_data_pub.motor_val[3] = motor_pwm.m4;
  mixer_data_pub.timestamp = rt_tick_get();
  mcnMixerPublish(&mixer_data_pub);

#elif defined(L2_DEVICE_03_MOTOR_01_PWM_EN)
  /* Set motor ratios */
  motorsSetRatio(MOTOR_M1, motor_pwm.m1);
  motorsSetRatio(MOTOR_M2, motor_pwm.m2);
  motorsSetRatio(MOTOR_M3, motor_pwm.m3);
  motorsSetRatio(MOTOR_M4, motor_pwm.m4);

#elif defined(L2_DEVICE_03_MOTOR_03_PWM_EN)
  /* Write all 4 motor values at once using channel selection mask */
  rt_device_write(motor_device_, ACT_CHAN_SEL_ALL, motor_pwm.val, ACT_WRITE_FROM_CONTROL);
#endif
}

void motorInit(void) {
#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
  if (motor_device_init() != 0) {
    rt_kprintf("[mixerControl] Failed to initialize motor device\n");
  }
#endif
}
