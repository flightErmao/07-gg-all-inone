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

#ifndef MPU6000_H__
#define MPU6000_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

// 错误码定义
#define MPU_EOK 0     // 无错误
#define MPU_ERROR -1  // 一般错误

// C接口函数声明
extern int drv_mpu6000_init(void);
extern int drv_mpu6000_set_delay(void (*delay_ms)(unsigned int));
extern int drv_mpu6000_set_spi_funcs(int8_t (*write_func)(uint8_t, uint8_t, uint8_t*, uint8_t),
                                     int8_t (*read_func)(uint8_t, uint8_t, uint8_t*, uint8_t));
extern int drv_mpu6000_read(int pos, void* data, int size);
extern int drv_mpu6000_get_accel(int16_t* ax, int16_t* ay, int16_t* az);
extern int drv_mpu6000_get_gyro(int16_t* gx, int16_t* gy, int16_t* gz);
extern int drv_mpu6000_get_temp(float* temp);
extern int drv_mpu6000_self_test(void);

// 原有的驱动初始化函数（保留兼容性）
rt_err_t mpu6000_drv_init(const char* spi_device_name, const char* gyro_device_name, const char* accel_device_name);

#ifdef __cplusplus
}
#endif

#endif
