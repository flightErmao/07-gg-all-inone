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

#ifndef PWM_REG_H__
#define PWM_REG_H__

#include <rtconfig.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "actuator.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PWM output channel count */
#define MAX_PWM_OUT_CHAN 4  // AUX Out has 4 pwm channel

/* PWM frequency configuration (Hz) */
#ifndef L2_DEVICE_03_MOTOR_03_PWM_FREQUENCY
#define L2_DEVICE_03_MOTOR_03_PWM_FREQUENCY 500
#endif

/* PWM device name */
#ifndef L2_DEVICE_03_MOTOR_03_PWM_DEVICE_NAME
#define L2_DEVICE_03_MOTOR_03_PWM_DEVICE_NAME "pwm1"
#endif

/* PWM physical channels selection */
#ifndef L2_DEVICE_03_MOTOR_03_PWM_PHYSICAL_CHANNELS
#define L2_DEVICE_03_MOTOR_03_PWM_PHYSICAL_CHANNELS "1234"
#endif

/* Motor channel remap sequence */
#ifndef L2_DEVICE_03_MOTOR_03_PWM_CHANNEL_REMAP
#define L2_DEVICE_03_MOTOR_03_PWM_CHANNEL_REMAP "1234"
#endif

/* Forward declarations for actuator interface functions */
rt_err_t pwm_config(actuator_dev_t dev, const struct actuator_configure* cfg);
rt_err_t pwm_control(actuator_dev_t dev, int cmd, void* arg);
rt_size_t pwm_read(actuator_dev_t dev, rt_uint16_t chan_sel, rt_uint16_t* chan_val, rt_size_t size);
rt_size_t pwm_write(actuator_dev_t dev, rt_uint16_t chan_sel, const rt_uint16_t* chan_val, rt_size_t size);

#ifdef __cplusplus
}
#endif

#endif /* PWM_REG_H__ */

