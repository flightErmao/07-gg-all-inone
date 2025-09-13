/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include <stdbool.h>
#include "actuator.h"

// #define rt_kprintf(...) console_printf(__VA_ARGS__)
#define rt_kprintf(...)

#define PWM_FREQ_50HZ (50)
#define PWM_FREQ_125HZ (125)
#define PWM_FREQ_250HZ (250)
#define PWM_FREQ_400HZ (400)

#define MAX_PWM_OUT_CHAN 4                   // AUX Out has 4 pwm channel
#define PWM_DEFAULT_FREQUENCY PWM_FREQ_50HZ  // pwm default frequqncy
#define PWM_MIN_VALUE 1000
#define PWM_MAX_VALUE 2000

#define PWM_DEVICE_NAME "pwm1"
#define PWM_CHANNEL_1 1
#define PWM_CHANNEL_2 2
#define PWM_CHANNEL_3 3
#define PWM_CHANNEL_4 4

static int pwm_freq_ = PWM_DEFAULT_FREQUENCY;
static rt_uint16_t pwm_fmu_duty_cyc_[MAX_PWM_OUT_CHAN] = {1000, 1000, 1000, 1000};
static struct rt_device_pwm* pwm_device_ = RT_NULL;
static rt_uint32_t pwm_period_ns_ = 0;
static bool pwm_initialized_ = false;

rt_inline void pwm_write_(uint8_t chan_id, rt_uint16_t pwm_val);

/* Validate PWM value and return clamped value */
static rt_uint16_t validate_pwm_value(rt_uint16_t val) {
  if (val == 0) {
    return 0;
  }

  if (val < PWM_MIN_VALUE || val > PWM_MAX_VALUE) {
    return PWM_MIN_VALUE;
  }
  return val;
}

/* PWM channel mapping - 1:1 mapping for 4 channels */
static rt_uint8_t pwm_channel_map[MAX_PWM_OUT_CHAN] = {
    PWM_CHANNEL_1,  // Channel 0 -> PWM1 CH1
    PWM_CHANNEL_2,  // Channel 1 -> PWM1 CH2
    PWM_CHANNEL_3,  // Channel 2 -> PWM1 CH3
    PWM_CHANNEL_4   // Channel 3 -> PWM1 CH4
};

/* Get timer clock frequency in Hz */
static rt_uint32_t get_timer_clock_freq(void) { return 100000000UL; /* 100MHz */ }

/* Initialize PWM device */
static rt_err_t pwm_hardware_init(void) {
  if (pwm_initialized_) {
    return RT_EOK;
  }

  /* Find PWM device */
  pwm_device_ = (struct rt_device_pwm*)rt_device_find(PWM_DEVICE_NAME);
  if (pwm_device_ == RT_NULL) {
    rt_kprintf("[PWM] can't find device: %s\n", PWM_DEVICE_NAME);
    return -RT_ERROR;
  }
  pwm_initialized_ = true;

  /* Calculate PWM period in nanoseconds using actual timer clock frequency */
  rt_uint32_t timer_clock = get_timer_clock_freq();
  pwm_period_ns_ = timer_clock / pwm_freq_; /* period = clock_freq / pwm_freq */

  /* Initialize all PWM channels */
  for (int i = 0; i < MAX_PWM_OUT_CHAN; i++) {
    rt_uint8_t channel = pwm_channel_map[i];

    /* Set PWM period and enable channel */
    // rt_err_t err = rt_pwm_set(pwm_device_, channel, pwm_period_ns_, pwm_fmu_duty_cyc_[i]);
    // if (err != RT_EOK) {
    //   rt_kprintf("[PWM] rt_pwm_set failed ch=%d err=%d\n", channel, err);
    //   return err;
    // }
    pwm_write_(i, pwm_fmu_duty_cyc_[i]);

    rt_err_t err = rt_pwm_enable(pwm_device_, channel);
    if (err != RT_EOK) {
      rt_kprintf("[PWM] rt_pwm_enable failed ch=%d err=%d\n", channel, err);
      return err;
    }
  }

  rt_kprintf("[PWM] hardware initialized successfully, freq=%dHz, period=%dns\n", pwm_freq_, pwm_period_ns_);
  return RT_EOK;
}

static rt_err_t pwm_set_frequency(uint16_t freq_to_set) {
  if (freq_to_set < PWM_FREQ_50HZ || freq_to_set > PWM_FREQ_400HZ) {
    /* invalid frequency */
    return RT_EINVAL;
  }

  pwm_freq_ = freq_to_set;

  /* Recalculate PWM period using actual timer clock frequency */
  rt_uint32_t timer_clock = get_timer_clock_freq();
  pwm_period_ns_ = timer_clock / pwm_freq_;

  //   /* Re-initialize PWM hardware with new frequency */
  //   if (pwm_initialized_ && pwm_device_ != RT_NULL) {
  //     for (int i = 0; i < MAX_PWM_OUT_CHAN; i++) {
  //       rt_uint8_t channel = pwm_channel_map[i];
  //       rt_pwm_set(pwm_device_, channel, pwm_period_ns_, pwm_fmu_duty_cyc_[i]);
  //     }
  //   }

  /* the timer compare value should be re-configured */
  for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
    pwm_write_(i, pwm_fmu_duty_cyc_[i]);
  }

  return RT_EOK;
}

rt_inline void pwm_write_(uint8_t chan_id, rt_uint16_t pwm_val) {
  /* Validate and store PWM value for readback */
  pwm_fmu_duty_cyc_[chan_id] = validate_pwm_value(pwm_val);

  /* Control actual PWM hardware */
  if (pwm_initialized_ && pwm_device_ != RT_NULL && chan_id < MAX_PWM_OUT_CHAN) {
    rt_uint8_t channel = pwm_channel_map[chan_id];
    rt_uint32_t pulse_ns = (rt_uint32_t)pwm_fmu_duty_cyc_[chan_id] * 1000; /* Convert us to ns */

    rt_pwm_set(pwm_device_, channel, pwm_period_ns_, pulse_ns);
  }
}

rt_inline void pwm_read_(uint8_t chan_id, rt_uint16_t* pwm_val) { *pwm_val = pwm_fmu_duty_cyc_[chan_id]; }

rt_err_t pwm_config(actuator_dev_t dev, const struct actuator_configure* cfg) {
  rt_kprintf("aux out configured: pwm frequency:%d\n", cfg->pwm_config.pwm_freq);

  if (pwm_set_frequency(cfg->pwm_config.pwm_freq) != RT_EOK) {
    return RT_ERROR;
  }

  /* Initialize PWM hardware */
  if (pwm_hardware_init() != RT_EOK) {
    return RT_ERROR;
  }

  /* update device configuration */
  //   dev->config = *cfg;

  return RT_EOK;
}

rt_err_t pwm_control(actuator_dev_t dev, int cmd, void* arg) {
  rt_err_t ret = RT_EOK;

  switch (cmd) {
    case ACT_CMD_CHANNEL_ENABLE:
      /* Initialize PWM hardware if not already done */
      if (!pwm_initialized_) {
        if (pwm_hardware_init() != RT_EOK) {
          return RT_ERROR;
        }
      }

      /* set to lowest pwm before open */
      if (pwm_initialized_ && pwm_device_ != RT_NULL) {
        for (int i = 0; i < MAX_PWM_OUT_CHAN; i++) {
          rt_uint8_t channel = pwm_channel_map[i];
          //   rt_pwm_set(pwm_device_, channel, pwm_period_ns_, 0);
          pwm_write_(i, pwm_fmu_duty_cyc_[i]);
          rt_pwm_enable(pwm_device_, channel);
        }
      }

      rt_kprintf("aux out enabled\n");
      break;
    case ACT_CMD_CHANNEL_DISABLE:
      /* Stop all PWM channels */
      if (pwm_initialized_ && pwm_device_ != RT_NULL) {
        for (int i = 0; i < MAX_PWM_OUT_CHAN; i++) {
          rt_uint8_t channel = pwm_channel_map[i];
          rt_pwm_disable(pwm_device_, channel);
        }
      }
      rt_kprintf("aux out disabled\n");
      break;
    default:
      ret = RT_EINVAL;
      break;
  }

  return ret;
}

rt_size_t pwm_read(actuator_dev_t dev, rt_uint16_t chan_sel, rt_uint16_t* chan_val, rt_size_t size) {
  rt_uint16_t* index = chan_val;
  rt_uint16_t pwm_val;

  for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
    if (chan_sel & (1 << i)) {
      pwm_read_(i, &pwm_val);
      *index = pwm_val;
      index++;
    }
  }

  return size;
}

rt_size_t pwm_write(actuator_dev_t dev, rt_uint16_t chan_sel, const rt_uint16_t* chan_val, rt_size_t size) {
  const rt_uint16_t* index = chan_val;
  rt_uint16_t val;

  for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
    if (chan_sel & (1 << i)) {
      val = *index;
      /* update pwm signal */
      pwm_write_(i, val);

      index++;
    }
  }

  return size;
}

const static struct actuator_ops _act_ops = {
    .act_config = pwm_config, .act_control = pwm_control, .act_read = pwm_read, .act_write = pwm_write};

static struct actuator_device act_dev = {.chan_mask = 0x0F, /* 4 channels: 0b00001111 */
                                         .range = {1000, 2000},
                                         .config = {.protocol = ACT_PROTOCOL_PWM,
                                                    .chan_num = MAX_PWM_OUT_CHAN,
                                                    .pwm_config = {.pwm_freq = 50},
                                                    .dshot_config = {0}},
                                         .ops = &_act_ops};

rt_err_t pwm_reg(void) {
  rt_err_t ret;

  /* register actuator hal device */
  ret = hal_actuator_register(&act_dev, "aux_out", RT_DEVICE_FLAG_RDWR, NULL);

  return ret;
}

#ifdef MOTOR_USING_PWM_EN
static int pwm_auto_start(void) {
  rt_err_t ret;
  ret = pwm_hardware_init();
  if (ret != RT_EOK) {
    rt_kprintf("[PWM] hardware init failed\n");
    return ret;
  }

  ret = pwm_reg();
  if (ret != RT_EOK) {
    rt_kprintf("[PWM] driver init failed\n");
    return ret;
  }

  rt_kprintf("[PWM] driver auto start success\n");
  return RT_EOK;
}

INIT_COMPONENT_EXPORT(pwm_auto_start);
#endif
