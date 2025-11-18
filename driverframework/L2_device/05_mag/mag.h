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
  float value_x;  /* magnetometer value in uT */
  float value_y;  /* magnetometer value in uT */
  float value_z;  /* magnetometer value in uT */
} mag_report_t;

typedef struct mag_configure {
  rt_uint16_t range_g;      /* magnetometer range in Gauss (e.g., 2, 8, 12, 30) */
  rt_uint16_t odr_hz;        /* output data rate in Hz (e.g., 10, 50, 100, 200) */
  float lsb;                 /* least significant bit scale factor (uT/LSB) */
                              /* Note: For QMC6308, LSB values and GS conversion (from datasheet):
                               *   - ±30G range: 1000 LSB/G,  0.1 uT/LSB,    1 GS = 100 uT = 1000 LSB
                               *   - ±12G range: 2500 LSB/G,  0.04 uT/LSB,   1 GS = 100 uT = 2500 LSB
                               *   - ±8G range:  3750 LSB/G, 0.02667 uT/LSB, 1 GS = 100 uT = 3750 LSB
                               *   - ±2G range:  15000 LSB/G, 0.00667 uT/LSB, 1 GS = 100 uT = 15000 LSB
                               * Where 1 GS (Gauss) = 100 uT (microTesla) */
} mag_configure_t;

struct mag_device {
  struct rt_device parent;
  const struct mag_ops* ops;
  uint8_t id;
  mag_configure_t config;    /* magnetometer configuration */
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
