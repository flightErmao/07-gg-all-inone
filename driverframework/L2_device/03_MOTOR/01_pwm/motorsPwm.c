#include "motorsPwm.h"
// #include "mixerControl.h"
#include <rtthread.h>
#include <rtdevice.h>

/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 电机驱动代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 ********************************************************************************/

static bool isInit = false;
uint32_t motor_ratios[] = {0, 0, 0, 0};
static const uint32_t MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};

/* PWM 设备与通道映射：
 * 原裸机映射：
 * M1: TIM4 CH2 (PB7)  -> 设备 "pwm4", 通道 2
 * M2: TIM4 CH1 (PB6)  -> 设备 "pwm4", 通道 1
 * M3: TIM2 CH3 (PB10) -> 设备 "pwm2", 通道 3
 * M4: TIM2 CH1 (PA5)  -> 设备 "pwm2", 通道 1
 */
typedef struct {
  const char *device_name;
  int channel;
} motor_pwm_binding_t;

static motor_pwm_binding_t motor_bindings[NBR_OF_MOTORS] = {
    {"pwm4", 2}, /* M1 */
    {"pwm4", 1}, /* M2 */
    {"pwm2", 3}, /* M3 */
    {"pwm2", 1}, /* M4 */
};

static struct rt_device_pwm *motor_pwms[NBR_OF_MOTORS] = {RT_NULL, RT_NULL, RT_NULL, RT_NULL};

/* 计算目标 PWM 频率与周期(ns)
 * 裸机下频率: TIM_CLOCK_HZ / (PRESCALE+1) / (PERIOD+1)
 */
static rt_uint32_t pwm_period_ns = 0;

static void motorsComputePwmPeriod(void) {
  /* 使用与裸机相同的配置映射到频率 */
  uint32_t pwm_freq_hz = TIM_CLOCK_HZ / (MOTORS_PWM_PRESCALE + 1) / (MOTORS_PWM_PERIOD + 1);
  if (pwm_freq_hz == 0) {
    /* 兜底，避免除零 */
    pwm_freq_hz = 20000; /* 20kHz */
  }
  /* ns 周期 = 1e9 / 频率 */
  pwm_period_ns = (rt_uint32_t)(1000000000ULL / pwm_freq_hz);
  if (pwm_period_ns == 0) {
    pwm_period_ns = 50000; /* 20kHz -> 50,000ns 兜底 */
  }
}

static rt_uint32_t ratioToPulseNs(uint16_t ratio16) {
  /* 16位占空比映射到脉宽时间 */
  return (rt_uint32_t)((((uint64_t)ratio16) * pwm_period_ns) / UINT16_MAX);
}

void motorsInit(void) /*电机初始化*/
{
  motorsComputePwmPeriod();

  for (int i = 0; i < NBR_OF_MOTORS; i++) {
    const char *devname = motor_bindings[i].device_name;
    struct rt_device_pwm *pwm = (struct rt_device_pwm *)rt_device_find(devname);
    if (pwm == RT_NULL) {
      rt_kprintf("[motors] can't find %s for motor %d\n", devname, i);
      isInit = false;
      return;
    }

    motor_pwms[i] = pwm;

    /* 设置默认周期与0占空比，并使能通道 */
    rt_err_t err = rt_pwm_set(motor_pwms[i], motor_bindings[i].channel, pwm_period_ns, 0);
    if (err != RT_EOK) {
      rt_kprintf("[motors] rt_pwm_set failed dev=%s ch=%d err=%d\n", devname, motor_bindings[i].channel, err);
      isInit = false;
      return;
    }

    err = rt_pwm_enable(motor_pwms[i], motor_bindings[i].channel);
    if (err != RT_EOK) {
      rt_kprintf("[motors] rt_pwm_enable failed dev=%s ch=%d err=%d\n", devname, motor_bindings[i].channel, err);
      isInit = false;
      return;
    }
  }

  isInit = true;
}

/*电机测试*/
bool motorsTest(void) {
  int i;

  for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++) {
    motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
    rt_thread_mdelay(MOTORS_TEST_ON_TIME_MS);
    motorsSetRatio(MOTORS[i], 0);
    rt_thread_mdelay(MOTORS_TEST_DELAY_TIME_MS);
  }

  return isInit;
}

extern bool isExitFlip;

/*设置电机PWM占空比*/
void motorsSetRatio(uint32_t id, uint16_t ithrust) {
  if (isInit) {
    uint16_t ratio = ithrust;

#ifdef ENABLE_THRUST_BAT_COMPENSATED
    if (isExitFlip == true) /*500Hz*/
    {
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
      rt_pwm_set(motor_pwms[id], motor_bindings[id].channel, pwm_period_ns, pulse_ns);
    }
  }
}
