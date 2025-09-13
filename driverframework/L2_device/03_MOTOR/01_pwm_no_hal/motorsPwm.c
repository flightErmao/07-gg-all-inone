#include "motorsPwm.h"
#include <rtthread.h>
#include <rtdevice.h>

#ifdef RT_USING_FINSH
#include <finsh.h>
#include <stdlib.h>
#endif

/********************************************************************************
 * This program is for learning purposes only, unauthorized use is prohibited
 * ALIENTEK MiniFly
 * Motor driver code
 * ALIENTEK@ALIENTEK
 * Technical forum: www.openedv.com
 * Created: 2017/5/12
 * Version: V1.3
 * All rights reserved
 * Copyright(C) Guangzhou Xingyi Electronics Technology Co., Ltd. 2014-2024
 * All rights reserved
 ********************************************************************************/

typedef struct {
  const char *device_name;
  int channel;
} motor_pwm_binding_t;

static bool isInit = false;
uint32_t motor_ratios[] = {0, 0, 0, 0};
// static const uint32_t MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};
extern bool isExitFlip;

static motor_pwm_binding_t motor_bindings[NBR_OF_MOTORS] = {
    {"pwm4", 2}, /* M1 */
    {"pwm4", 1}, /* M2 */
    {"pwm2", 3}, /* M3 */
    {"pwm2", 1}, /* M4 */
};

static struct rt_device_pwm *motor_pwms[NBR_OF_MOTORS] = {RT_NULL, RT_NULL, RT_NULL, RT_NULL};
static rt_uint32_t pwm_period_ns_ = 0;

/* Calculate target PWM frequency and period (ns)
 * Bare metal frequency: TIM_CLOCK_HZ / (PRESCALE+1) / (PERIOD+1)
 */
static void motorsComputePwmPeriod(void) {
  /* Use same configuration as bare metal to map frequency */
  uint32_t pwm_freq_hz = TIM_CLOCK_HZ / (MOTORS_PWM_PRESCALE + 1) / (MOTORS_PWM_PERIOD + 1);
  if (pwm_freq_hz == 0) {
    /* Fallback to avoid division by zero */
    pwm_freq_hz = 20000; /* 20kHz */
  }
  /* ns period = 1e9 / frequency */
  pwm_period_ns_ = (rt_uint32_t)(1000000000ULL / pwm_freq_hz);
  if (pwm_period_ns_ == 0) {
    pwm_period_ns_ = 50000; /* 20kHz -> 50,000ns fallback */
  }
}

static rt_uint32_t ratioToPulseNs(uint16_t ratio16) {
  /* Map 16-bit duty cycle to pulse width time */
  return (rt_uint32_t)((((uint64_t)ratio16) * pwm_period_ns_) / UINT16_MAX);
}

int motorsInit(void) /* Motor initialization */
{
  motorsComputePwmPeriod();

  for (int i = 0; i < NBR_OF_MOTORS; i++) {
    const char *devname = motor_bindings[i].device_name;
    struct rt_device_pwm *pwm = (struct rt_device_pwm *)rt_device_find(devname);
    if (pwm == RT_NULL) {
      rt_kprintf("[motors] can't find %s for motor %d\n", devname, i);
      isInit = false;
      return -1;
    }

    motor_pwms[i] = pwm;

    /* Set default period with 0 duty cycle and enable channel */
    rt_err_t err = rt_pwm_set(motor_pwms[i], motor_bindings[i].channel, pwm_period_ns_, 0);
    if (err != RT_EOK) {
      rt_kprintf("[motors] rt_pwm_set failed dev=%s ch=%d err=%d\n", devname, motor_bindings[i].channel, err);
      isInit = false;
      return -1;
    }

    err = rt_pwm_enable(motor_pwms[i], motor_bindings[i].channel);
    if (err != RT_EOK) {
      rt_kprintf("[motors] rt_pwm_enable failed dev=%s ch=%d err=%d\n", devname, motor_bindings[i].channel, err);
      isInit = false;
      return -1;
    }
  }

  isInit = true;

  return 0;
}

/* Set motor PWM duty cycle */
void motorsSetRatio(uint32_t id, uint16_t ithrust) {
  if (!isInit) {
    return;
  }
  uint16_t ratio = ithrust;

#ifdef ENABLE_THRUST_BAT_COMPENSATED
  if (isExitFlip == true) {
    float thrust = ((float)ithrust / 65536.0f) * 60;
    float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
    float supply_voltage = pmGetBatteryVoltage();
    float percentage = volts / supply_voltage;
    percentage = percentage > 1.0f ? 1.0f : percentage;
    ratio = percentage * UINT16_MAX;
    motor_ratios[id] = ratio;
  }
#endif

  if (id < NBR_OF_MOTORS && motor_pwms[id] != RT_NULL) {
    rt_uint32_t pulse_ns = ratioToPulseNs(ratio);
    rt_pwm_set(motor_pwms[id], motor_bindings[id].channel, pwm_period_ns_, pulse_ns);
  }
}

/* Motor test */
void cmdMotorOrder(void) {
  if (isInit == false) {
    rt_kprintf("motor init fail\n");
    return;
  }
  uint16_t ratio = (uint16_t)((MOTORS_TEST_RATIO * UINT16_MAX) / 100);

  for (uint8_t i = 0; i < NBR_OF_MOTORS; i++) {
    motorsSetRatio(i, ratio);
    rt_thread_mdelay(MOTORS_TEST_ON_TIME_MS);
    motorsSetRatio(i, 0);
    rt_thread_mdelay(MOTORS_TEST_DELAY_TIME_MS);
  }
}

static void cmdMotorTest(int argc, char **argv) {
  uint32_t motor_id = 0;
  uint32_t pwm_value = 0;
  uint32_t duration_ms = 0;

  if (isInit == false) {
    rt_kprintf("motor init fail\n");
    return;
  }

  // Parse parameters
  if (argc >= 2) {
    motor_id = atoi(argv[1]);
  }
  if (argc >= 3) {
    pwm_value = atoi(argv[2]);
  }
  if (argc >= 4) {
    duration_ms = atoi(argv[3]);
  }

  // Parameter validation
  if (motor_id > 4) {
    rt_kprintf("Error: Motor ID must be between 0-4\n");
    rt_kprintf("Usage: motor_test [motor_id] [pwm_value] [duration_ms]\n");
    rt_kprintf("  motor_id: 0=all motors, 1-4=single motor\n");
    rt_kprintf("  pwm_value: 0-100 (percentage)\n");
    rt_kprintf("  duration_ms: Duration (ms), 0=continuous\n");
    return;
  }

  if (pwm_value > 100) {
    rt_kprintf("Error: PWM value must be between 0-100\n");
    return;
  }

  // Set motor PWM
  if (motor_id == 0) {
    // Test all motors
    if (pwm_value == 0) {
      for (int i = 0; i < NBR_OF_MOTORS; i++) {
        motorsSetRatio(i, 0);
      }
      rt_kprintf("All motors stopped\n");
    } else {
      uint16_t ratio = (uint16_t)((pwm_value * UINT16_MAX) / 100);
      for (int i = 0; i < NBR_OF_MOTORS; i++) {
        motorsSetRatio(i, ratio);
      }
      rt_kprintf("All motors set to %d%% power\n", pwm_value);
    }
  } else {
    // Test single motor
    uint16_t ratio = (uint16_t)((pwm_value * UINT16_MAX) / 100);
    motorsSetRatio(motor_id - 1, ratio);
    if (pwm_value == 0) {
      rt_kprintf("Motor %d stopped\n", motor_id);
    } else {
      rt_kprintf("Motor %d set to %d%% power\n", motor_id, pwm_value);
    }
  }

  // Auto stop after duration if specified
  if (duration_ms > 0) {
    rt_kprintf("Running for %d ms then auto stop...\n", duration_ms);
    rt_thread_mdelay(duration_ms);

    if (motor_id == 0) {
      for (int i = 0; i < NBR_OF_MOTORS; i++) {
        motorsSetRatio(i, 0);
      }
      rt_kprintf("All motors auto stopped\n");
    } else {
      motorsSetRatio(motor_id - 1, 0);
      rt_kprintf("Motor %d auto stopped\n", motor_id);
    }
  }
}

#ifdef L2_DEVICE_03_MOTOR_01_MSH_CMD_EN
INIT_COMPONENT_EXPORT(motorsInit);
MSH_CMD_EXPORT_ALIAS(cmdMotorTest, cmdMotorTest, motor test command);
MSH_CMD_EXPORT_ALIAS(cmdMotorOrder, cmdMotorOrder, motor order command);
#endif
