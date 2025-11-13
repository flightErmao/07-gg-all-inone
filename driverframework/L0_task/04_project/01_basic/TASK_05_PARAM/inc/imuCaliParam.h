#ifndef IMU_CALI_PARAM_H
#define IMU_CALI_PARAM_H

#include <stddef.h>
#include "uparam.h"

#define IMU_CALI_PARAM_GYRO_BIAS "imu_cali_gyro_bias"
#define IMU_CALI_PARAM_ACC_BIAS "imu_cali_acc_bias"
#define IMU_CALI_PARAM_ORIENTATION "imu_cali_orientation"

param_list *imuCaliParam_list(void);
size_t imuCaliParam_count(void);

#endif /* IMU_CALI_PARAM_H */

