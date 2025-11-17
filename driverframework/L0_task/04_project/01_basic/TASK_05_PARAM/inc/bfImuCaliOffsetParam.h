#ifndef BF_IMU_CALI_OFFSET_PARAM_H
#define BF_IMU_CALI_OFFSET_PARAM_H

#include <stddef.h>
#include "uparam.h"

/* 注意：gyro 零偏值在运行时计算，不保存到参数系统 */
#define BF_IMU_CALI_OFFSET_PARAM_GYRO_OFFSET_YAW "bf_imu_cali_gyro_offset_yaw"

param_list *bfImuCaliOffsetParam_list(void);
size_t bfImuCaliOffsetParam_count(void);

#endif /* BF_IMU_CALI_OFFSET_PARAM_H */

