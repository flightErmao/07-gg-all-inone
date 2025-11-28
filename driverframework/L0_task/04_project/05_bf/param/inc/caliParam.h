#ifndef BF_IMU_CALI_OFFSET_PARAM_H
#define BF_IMU_CALI_OFFSET_PARAM_H

#include <stddef.h>
#include "uparam.h"

/* 注意：gyro 零偏值在运行时计算，不保存到参数系统 */
#define BF_IMU_CALI_OFFSET_PARAM_GYRO_OFFSET_YAW "cali_imu_gyro_offset_yaw"
#define BF_IMU_CALI_OFFSET_PARAM_ACC_OFFSET "cali_imu_acc_offset"

/* IMU 对齐参数（acc 和 gyro 共用） */
#define BF_IMU_CALI_PARAM_ALIGN_METHOD "imu_align_method"
#define BF_IMU_CALI_PARAM_CUSTOM_ALIGN "imu_custom_align"

param_list *bfImuCaliOffsetParam_list(void);
size_t bfImuCaliOffsetParam_count(void);

#endif /* BF_IMU_CALI_OFFSET_PARAM_H */

