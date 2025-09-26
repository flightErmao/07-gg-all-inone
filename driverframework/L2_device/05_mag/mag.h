/******************************************************************************
 * Copyright 2020-2023 The Firmament Authors. All Rights Reserved.
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

#ifndef __MAG_H__
#define __MAG_H__

#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

/* mag read pos */
#define MAG_RD_REPORT 1

/* mag command */
#define MAG_CMD_CHECK_READY 0x20

typedef struct {
  uint32_t timestamp_ms;
  int16_t value_x;
  int16_t value_y;
  int16_t value_z;
} mag_report_t;

struct mag_configure {
  rt_uint16_t osr; /* oversampling ratio */
};

struct mag_device {
  struct rt_device parent;
  const struct mag_ops* ops;
  uint8_t id;
};
typedef struct mag_device* mag_dev_t;

/* mag driver opeations */
struct mag_ops {
  /**
   * @brief mag meter control function (optional)
   * @param dev magmeter device
   * @param cmd operation command
   * @param arg command arguments
   */
  rt_err_t (*mag_control)(mag_dev_t dev, int cmd, void* arg);
  /**
   * @brief read magmeter report
   * @param dev magmeter device
   * @param report magmeter report buffer
   */
  rt_size_t (*mag_read)(mag_dev_t dev, mag_report_t* report);
};

rt_err_t hal_mag_register(mag_dev_t mag, const char* name, rt_uint32_t flag, void* data);

#ifdef __cplusplus
}
#endif

#endif
