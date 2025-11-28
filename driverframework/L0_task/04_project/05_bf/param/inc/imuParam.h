#ifndef BF_IMU_PARAM_H
#define BF_IMU_PARAM_H

#include <stddef.h>
#include "uparam.h"

/* IMU 对齐参数名称定义 */
#define BF_IMU_PARAM_ALIGN_ACC "imu_align_acc"
#define BF_IMU_PARAM_ALIGN_GYRO "imu_align_gyro"
#define BF_IMU_PARAM_CUSTOM_ALIGN_ACC "imu_custom_align_acc"
#define BF_IMU_PARAM_CUSTOM_ALIGN_GYRO "imu_custom_align_gyro"

param_list *bfImuParam_list(void);
size_t bfImuParam_count(void);

#endif /* BF_IMU_PARAM_H */

