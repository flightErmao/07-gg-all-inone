#ifndef __MPU6050_REG_H__
#define __MPU6050_REG_H__

#include <rtdevice.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化并注册MPU6050到IMU抽象层
rt_err_t mpu6050_register_on_bus(const char* i2c_device_name, const char* imu_name, uint8_t i2c_addr);

#ifdef __cplusplus
}
#endif

#endif  // __MPU6050_REG_H__