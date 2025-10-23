#include "mixerControl.h"
#include "rtconfig.h"
#include <string.h>
#include "maths.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "aMcnStabilize.h"

#ifdef L2_DEVICE_03_MOTOR_01_PWM_EN
#include "motorsPwm.h"
#endif

#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
#include "actuator.h"
#endif

typedef struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPWM_t;

typedef enum {
  CW = 0,
  CCW = 1,
} motorDir_e;

static motorDir_e frontRightDir = CCW;
static bool motorSetEnable = false;

#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
#define MOTOR_DEVICE_NAME "aux_out"
#define MAX_MOTOR_CHANNELS 4
static rt_device_t motor_device = RT_NULL;
static bool motor_device_initialized = false;
#endif

#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
/* Initialize motor device */
static int motor_device_init(void) {
  if (motor_device_initialized) {
    return 0;
  }

  motor_device = rt_device_find(MOTOR_DEVICE_NAME);
  if (motor_device == RT_NULL) {
    rt_kprintf("[mixerControl] can't find device: %s\n", MOTOR_DEVICE_NAME);
    return -1;
  }

  rt_err_t ret = rt_device_open(motor_device, RT_DEVICE_OFLAG_RDWR);
  if (ret != RT_EOK) {
    rt_kprintf("[mixerControl] open device failed: %d\n", ret);
    return -1;
  }

  motor_device_initialized = true;
  rt_kprintf("[mixerControl] device initialized successfully\n");
  return 0;
}

/* Set motor raw 16-bit value */
static int motor_set_raw_value(int motor_id, rt_uint16_t raw_value) {
  if (!motor_device_initialized) {
    if (motor_device_init() != 0) {
      return -1;
    }
  }

  if (motor_id < 1 || motor_id > MAX_MOTOR_CHANNELS) {
    rt_kprintf("[mixerControl] Error: Motor ID must be between 1-%d\n", MAX_MOTOR_CHANNELS);
    return -1;
  }

  /* Convert motor_id to channel index (0-based) */
  // int channel = motor_id - 1;
  // rt_uint16_t chan_sel = 1 << channel;

  // /* Write raw 16-bit value directly to specific channel */
  // rt_size_t written = rt_device_write(motor_device, chan_sel, &raw_value, sizeof(raw_value));
  // if (written != sizeof(raw_value)) {
  //   rt_kprintf("[mixerControl] write failed, written: %d\n", written);
  //   return -1;
  // }

  return 0;
}
#endif

static uint16_t limitThrust(int value) {
  if (value > UINT16_MAX) {
    value = UINT16_MAX;
  } else if (value < 0) {
    value = 0;
  }

  return (uint16_t)value;
}

void mixerControl(control_t *control) {
  motorPWM_t motorPWM = {0, 0, 0, 0};
  if (motorSetEnable) {
    return;
  }

  if (control == NULL) {
    rt_kprintf("[mixerControl] Error: control pointer is NULL\n");
    return;
  }

  // 注意，如果电机右前方是顺时钟，则下方control->yaw需要全部用相反的符合
  // 目前使用的电机顺序1~4，分别为右前，右后，左后，左前
  // 目前使用的右前方电机转向为逆时针
  int16_t yawValue = 0;
  if (frontRightDir == CW) {
    yawValue = control->yaw;
  } else {
    yawValue = -control->yaw;
  }

  motorPWM.m1 = limitThrust(control->thrust - control->roll - control->pitch + yawValue);
  motorPWM.m2 = limitThrust(control->thrust - control->roll + control->pitch - yawValue);
  motorPWM.m3 = limitThrust(control->thrust + control->roll + control->pitch + yawValue);
  motorPWM.m4 = limitThrust(control->thrust + control->roll - control->pitch - yawValue);

#ifdef PROJECT_MINIFLY_TASK_DSHOT_EN
  /* Publish raw 0~65535 motor values to DShot task via uMCN */
  mixer_data_t mixer_data_pub;
  mixer_data_pub.motor_val[0] = (uint16_t)motorPWM.m1;
  mixer_data_pub.motor_val[1] = (uint16_t)motorPWM.m2;
  mixer_data_pub.motor_val[2] = (uint16_t)motorPWM.m3;
  mixer_data_pub.motor_val[3] = (uint16_t)motorPWM.m4;
  mixer_data_pub.timestamp = rt_tick_get();
  mcnMixerPublish(&mixer_data_pub);

#elif defined(L2_DEVICE_03_MOTOR_01_PWM_EN)
  motorsSetRatio(MOTOR_M1, motorPWM.m1);
  motorsSetRatio(MOTOR_M2, motorPWM.m2);
  motorsSetRatio(MOTOR_M3, motorPWM.m3);
  motorsSetRatio(MOTOR_M4, motorPWM.m4);

#elif defined(L2_DEVICE_03_MOTOR_03_PWM_EN)

  /* Write raw 16-bit values directly to motor channels */
  motor_set_raw_value(1, (rt_uint16_t)motorPWM.m1);
  motor_set_raw_value(2, (rt_uint16_t)motorPWM.m2);
  motor_set_raw_value(3, (rt_uint16_t)motorPWM.m3);
  motor_set_raw_value(4, (rt_uint16_t)motorPWM.m4);
#endif
}

void motorInit(void) {
#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
  if (motor_device_init() != 0) {
    rt_kprintf("[mixerControl] Failed to initialize motor device\n");
  }
#endif
}
