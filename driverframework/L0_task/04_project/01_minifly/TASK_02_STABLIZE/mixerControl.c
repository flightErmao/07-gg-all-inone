#include "mixerControl.h"
#include "rtconfig.h"
#include <string.h>
#include "maths.h"
#include <rtthread.h>
#include <rtdevice.h>

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

static bool motorSetEnable = false;

#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
#define MOTOR_DEVICE_NAME "aux_out"
#define MAX_MOTOR_CHANNELS 4
#define DUTY_CYCLE_MIN 0.0f
#define DUTY_CYCLE_MAX 1.0f
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

/* Set motor duty cycle */
static int motor_set_duty_cycle(int motor_id, float duty_cycle) {
  if (!motor_device_initialized) {
    if (motor_device_init() != 0) {
      return -1;
    }
  }

  if (motor_id < 1 || motor_id > MAX_MOTOR_CHANNELS) {
    rt_kprintf("[mixerControl] Error: Motor ID must be between 1-%d\n", MAX_MOTOR_CHANNELS);
    return -1;
  }

  if (duty_cycle < DUTY_CYCLE_MIN || duty_cycle > DUTY_CYCLE_MAX) {
    rt_kprintf("[mixerControl] Error: Duty cycle must be between %.1f-%.1f\n", DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
    return -1;
  }

  /* Convert motor_id to channel index (0-based) */
  int channel = motor_id - 1;
  rt_uint16_t chan_sel = 1 << channel;

  /* Convert duty cycle to PWM value (1000-2000 range) */
  rt_uint16_t chan_val = (rt_uint16_t)(1000.0f + duty_cycle * 1000.0f);

  /* Write PWM value to specific channel */
  rt_size_t written = rt_device_write(motor_device, chan_sel, &chan_val, sizeof(chan_val));
  if (written != sizeof(chan_val)) {
    rt_kprintf("[mixerControl] write failed, written: %d\n", written);
    return -1;
  }

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
  if (motorSetEnable) {
    return;
  }

  if (control == NULL) {
    rt_kprintf("[mixerControl] Error: control pointer is NULL\n");
    return;
  }

  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;
  motorPWM_t motorPWM = {0, 0, 0, 0};

  motorPWM.m1 = limitThrust(control->thrust - r - p + control->yaw);
  motorPWM.m2 = limitThrust(control->thrust - r + p - control->yaw);
  motorPWM.m3 = limitThrust(control->thrust + r + p + control->yaw);
  motorPWM.m4 = limitThrust(control->thrust + r - p - control->yaw);

#ifdef L2_DEVICE_03_MOTOR_01_PWM_EN
  motorsSetRatio(MOTOR_M1, motorPWM.m1);
  motorsSetRatio(MOTOR_M2, motorPWM.m2);
  motorsSetRatio(MOTOR_M3, motorPWM.m3);
  motorsSetRatio(MOTOR_M4, motorPWM.m4);

#elif defined(L2_DEVICE_03_MOTOR_03_PWM_EN)

  if (motorPWM.m1 > UINT16_MAX || motorPWM.m2 > UINT16_MAX || motorPWM.m3 > UINT16_MAX || motorPWM.m4 > UINT16_MAX) {
    rt_kprintf("[mixerControl] Error: PWM value out of range\n");
    return;
  }

  float duty_m1 = scaleRangef((float)motorPWM.m1, 0.0f, 65535.0f, 0.0f, 1.0f);
  float duty_m2 = scaleRangef((float)motorPWM.m2, 0.0f, 65535.0f, 0.0f, 1.0f);
  float duty_m3 = scaleRangef((float)motorPWM.m3, 0.0f, 65535.0f, 0.0f, 1.0f);
  float duty_m4 = scaleRangef((float)motorPWM.m4, 0.0f, 65535.0f, 0.0f, 1.0f);

  motor_set_duty_cycle(1, duty_m1);
  motor_set_duty_cycle(2, duty_m2);
  motor_set_duty_cycle(3, duty_m3);
  motor_set_duty_cycle(4, duty_m4);
#endif
}

void motorInit(void) {
#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
  if (motor_device_init() != 0) {
    rt_kprintf("[mixerControl] Failed to initialize motor device\n");
  }
#endif
}

static void cmdMotorEnable(int argc, char **argv) {
  if (argc == 1) {
    /* No arguments - show current status */
    rt_kprintf("Motor control is currently %s\n", motorSetEnable ? "DISABLED" : "ENABLED");
    rt_kprintf("Usage: cmdMotorEnable [enable|disable]\n");
    return;
  }

  if (argc == 2) {
    if (strcmp(argv[1], "enable") == 0) {
      motorSetEnable = false; /* false means motor control is enabled */
      rt_kprintf("Motor control ENABLED , Motor test can't run\n");
    } else if (strcmp(argv[1], "disable") == 0) {
      motorSetEnable = true; /* true means motor control is disabled */
      rt_kprintf("Motor control DISABLED , Motor test can run!\n");
    } else {
      rt_kprintf("Invalid argument. Use 'enable' or 'disable'\n");
      rt_kprintf("Usage: cmdMotorEnable [enable|disable]\n");
    }
  } else {
    rt_kprintf("Too many arguments\n");
    rt_kprintf("Usage: cmdMotorEnable [enable|disable]\n");
  }
}

MSH_CMD_EXPORT_ALIAS(cmdMotorEnable, cmdMotorEnable, Enable motor control);