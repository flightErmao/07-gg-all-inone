/**
 * @file imu_raw_msg.h
 * 
 * IMU 原始数据消息类型定义（C 接口，供 C++ 使用）
 */

#ifndef IMU_RAW_MSG_H__
#define IMU_RAW_MSG_H__

#include <rtdef.h>
#include "uMCN.h"

/* IMU 原始数据消息类型 */
typedef struct {
  float accel[3];
  float gyro[3];
  rt_uint32_t seq;
} imu_raw_msg_t;

MCN_DECLARE(imu_raw);

#endif /* IMU_RAW_MSG_H__ */

