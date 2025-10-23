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
#include <string.h>
#include <stdlib.h>
#include "actuator.h"

// #define rt_kprintf(...) console_printf(__VA_ARGS__)
#define rt_kprintf(...)

#ifndef L2_DEVICE_03_MOTOR_03_PWM_DEFAULT_FREQUENCY
#define L2_DEVICE_03_MOTOR_03_PWM_DEFAULT_FREQUENCY 500
#endif

#ifndef L2_DEVICE_03_MOTOR_03_PWM_MIN_FREQUENCY
#define L2_DEVICE_03_MOTOR_03_PWM_MIN_FREQUENCY 50
#endif

#ifndef L2_DEVICE_03_MOTOR_03_PWM_MAX_FREQUENCY
#define L2_DEVICE_03_MOTOR_03_PWM_MAX_FREQUENCY 1000
#endif

#define MAX_PWM_OUT_CHAN 4                   // AUX Out has 4 pwm channel
#define PWM_DEFAULT_FREQUENCY L2_DEVICE_03_MOTOR_03_PWM_DEFAULT_FREQUENCY
#define PWM_MIN_FREQUENCY L2_DEVICE_03_MOTOR_03_PWM_MIN_FREQUENCY
#define PWM_MAX_FREQUENCY L2_DEVICE_03_MOTOR_03_PWM_MAX_FREQUENCY

#ifndef L2_DEVICE_03_MOTOR_03_PWM_DEVICE_NAME
#define L2_DEVICE_03_MOTOR_03_PWM_DEVICE_NAME "pwm1"
#endif

#ifndef L2_DEVICE_03_MOTOR_03_PWM_CHANNEL_MAPPING
#define L2_DEVICE_03_MOTOR_03_PWM_CHANNEL_MAPPING "1234"
#endif

#define PWM_DEVICE_NAME L2_DEVICE_03_MOTOR_03_PWM_DEVICE_NAME

static int pwm_freq_ = PWM_DEFAULT_FREQUENCY;
static rt_uint16_t pwm_fmu_duty_cyc_[MAX_PWM_OUT_CHAN] = {0, 0, 0, 0};
static struct rt_device_pwm* pwm_device_ = RT_NULL;
static rt_uint32_t pwm_period_ns_ = 0;
static bool pwm_initialized_ = false;

/* Motor channel remap configuration */
static rt_uint8_t motor_channel_remap_[MAX_PWM_OUT_CHAN] = {0, 1, 2, 3};  // Default: no remap
static bool remap_parsed_ = false;

/* PWM channel mapping configuration */
static rt_uint8_t pwm_channel_mapping_[MAX_PWM_OUT_CHAN] = {1, 2, 3, 4};  // Default: 1,2,3,4
static bool pwm_mapping_parsed_ = false;

rt_inline void setChanlValue(uint8_t chan_id, rt_uint16_t pwm_val);

/* Parse motor channel remap configuration */
static rt_err_t parse_motor_remap_config(void) {
  if (remap_parsed_) {
    return RT_EOK;  // Already parsed
  }

#ifdef L2_DEVICE_03_MOTOR_03_PWM_CHANNEL_REMAP
  const char* remap_str = L2_DEVICE_03_MOTOR_03_PWM_CHANNEL_REMAP;
  int len = strlen(remap_str);

  if (len != MAX_PWM_OUT_CHAN) {
    rt_kprintf("[PWM] Invalid remap string length: %d, expected: %d\n", len, MAX_PWM_OUT_CHAN);
    return -RT_ERROR;
  }

  // Parse each character in the remap string
  for (int i = 0; i < MAX_PWM_OUT_CHAN; i++) {
    char c = remap_str[i];
    if (c < '1' || c > '4') {
      rt_kprintf("[PWM] Invalid remap character: %c at position %d\n", c, i);
      return -RT_ERROR;
    }

    // Convert '1'-'4' to 0-3 (0-based index)
    motor_channel_remap_[i] = c - '1';
  }

  rt_kprintf("[PWM] Motor remap configured: %s\n", remap_str);
  rt_kprintf("[PWM] Logical->Physical mapping: ");
  for (int i = 0; i < MAX_PWM_OUT_CHAN; i++) {
    rt_kprintf("%d->%d ", i + 1, motor_channel_remap_[i] + 1);
  }
  rt_kprintf("\n");
#else
  // Use default mapping if not configured
  for (int i = 0; i < MAX_PWM_OUT_CHAN; i++) {
    motor_channel_remap_[i] = i;
  }
  rt_kprintf("[PWM] Using default motor mapping (no remap)\n");
#endif

  remap_parsed_ = true;
  return RT_EOK;
}

/* Parse PWM channel mapping configuration */
static rt_err_t parse_pwm_mapping_config(void) {
  if (pwm_mapping_parsed_) {
    return RT_EOK;  // Already parsed
  }

#ifdef L2_DEVICE_03_MOTOR_03_PWM_CHANNEL_MAPPING
  const char* mapping_str = L2_DEVICE_03_MOTOR_03_PWM_CHANNEL_MAPPING;
  int len = strlen(mapping_str);

  if (len != MAX_PWM_OUT_CHAN) {
    rt_kprintf("[PWM] Invalid mapping string length: %d, expected: %d\n", len, MAX_PWM_OUT_CHAN);
    return -RT_ERROR;
  }

  // Parse each character in the mapping string
  for (int i = 0; i < MAX_PWM_OUT_CHAN; i++) {
    char c = mapping_str[i];
    if (c < '1' || c > '4') {
      rt_kprintf("[PWM] Invalid mapping character: %c at position %d\n", c, i);
      return -RT_ERROR;
    }

    // Convert '1'-'4' to 1-4 (1-based channel numbers)
    pwm_channel_mapping_[i] = c - '0';
  }

  rt_kprintf("[PWM] PWM channel mapping configured: %s\n", mapping_str);
  rt_kprintf("[PWM] Logical->Physical mapping: ");
  for (int i = 0; i < MAX_PWM_OUT_CHAN; i++) {
    rt_kprintf("%d->%d ", i + 1, pwm_channel_mapping_[i]);
  }
  rt_kprintf("\n");
#else
  // Use default mapping if not configured
  for (int i = 0; i < MAX_PWM_OUT_CHAN; i++) {
    pwm_channel_mapping_[i] = i + 1;
  }
  rt_kprintf("[PWM] Using default PWM channel mapping (1,2,3,4)\n");
#endif

  pwm_mapping_parsed_ = true;
  return RT_EOK;
}

/* PWM channel mapping will be configured via Kconfig */

/* Get timer clock frequency in Hz */
static rt_uint32_t get_timer_clock_freq(void) { return 1000000000UL; }

/* Initialize PWM device */
static rt_err_t pwm_hardware_init(void) {
  if (pwm_initialized_) {
    return RT_EOK;
  }

  /* Parse motor remap configuration first */
  if (parse_motor_remap_config() != RT_EOK) {
    rt_kprintf("[PWM] Failed to parse motor remap configuration\n");
    return -RT_ERROR;
  }

  /* Parse PWM channel mapping configuration */
  if (parse_pwm_mapping_config() != RT_EOK) {
    rt_kprintf("[PWM] Failed to parse PWM channel mapping configuration\n");
    return -RT_ERROR;
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
    rt_uint8_t channel = pwm_channel_mapping_[i];

    /* Set PWM period and enable channel */
    setChanlValue(i, pwm_fmu_duty_cyc_[i]);

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
  if (freq_to_set < PWM_MIN_FREQUENCY || freq_to_set > PWM_MAX_FREQUENCY) {
    /* invalid frequency */
    rt_kprintf("[PWM] Invalid frequency: %d Hz, valid range: %d-%d Hz\n", freq_to_set, PWM_MIN_FREQUENCY,
               PWM_MAX_FREQUENCY);
    return RT_EINVAL;
  }

  pwm_freq_ = freq_to_set;

  /* Recalculate PWM period using actual timer clock frequency */
  rt_uint32_t timer_clock = get_timer_clock_freq();
  pwm_period_ns_ = timer_clock / pwm_freq_;

  /* the timer compare value should be re-configured */
  for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
    setChanlValue(i, pwm_fmu_duty_cyc_[i]);
  }

  return RT_EOK;
}

rt_inline void setChanlValue(uint8_t chan_id, rt_uint16_t pwm_val) {
  /* Store PWM value for readback */
  pwm_fmu_duty_cyc_[chan_id] = pwm_val;

  /* Control actual PWM hardware */
  if (pwm_initialized_ && pwm_device_ != RT_NULL && chan_id < MAX_PWM_OUT_CHAN) {
    /* Use remap to get physical channel */
    rt_uint8_t physical_channel = motor_channel_remap_[chan_id];
    rt_uint8_t channel = pwm_channel_mapping_[physical_channel];
    
    /* Convert 16-bit value (0-65535) to percentage (0.0-1.0) */
    float duty_cycle = (float)pwm_fmu_duty_cyc_[chan_id] / 65535.0f;
    
    /* Calculate pulse width in nanoseconds */
    rt_uint32_t pulse_ns = (rt_uint32_t)(duty_cycle * (float)pwm_period_ns_);

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
          rt_uint8_t physical_channel = motor_channel_remap_[i];
          rt_uint8_t channel = pwm_channel_mapping_[physical_channel];
          setChanlValue(i, pwm_fmu_duty_cyc_[i]);
          rt_pwm_enable(pwm_device_, channel);
        }
      }

      rt_kprintf("aux out enabled\n");
      break;
    case ACT_CMD_CHANNEL_DISABLE:
      /* Stop all PWM channels */
      if (pwm_initialized_ && pwm_device_ != RT_NULL) {
        for (int i = 0; i < MAX_PWM_OUT_CHAN; i++) {
          rt_uint8_t physical_channel = motor_channel_remap_[i];
          rt_uint8_t channel = pwm_channel_mapping_[physical_channel];
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
      setChanlValue(i, val);

      index++;
    }
  }

  return size;
}

const static struct actuator_ops _act_ops = {
    .act_config = pwm_config, .act_control = pwm_control, .act_read = pwm_read, .act_write = pwm_write};

static struct actuator_device act_dev = {.chan_mask = 0x0F, /* 4 channels: 0b00001111 */
                                         .range = {0, 65535},
                                         .config = {.protocol = ACT_PROTOCOL_PWM,
                                                    .chan_num = MAX_PWM_OUT_CHAN,
                                                    .pwm_config = {.pwm_freq = PWM_DEFAULT_FREQUENCY},
                                                    .dshot_config = {0}},
                                         .ops = &_act_ops};

rt_err_t pwm_reg(void) {
  rt_err_t ret;

  /* register actuator hal device */
  ret = hal_actuator_register(&act_dev, "aux_out", RT_DEVICE_FLAG_RDWR, NULL);

  return ret;
}

#ifdef L2_DEVICE_03_MOTOR_03_PWM_EN
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
