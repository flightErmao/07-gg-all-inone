/**
 * @file imu_mcn.h
 * 
 * IMU 原始数据消息类型定义（C 接口，供 C++ 使用）
 */

#ifndef IMU_RAW_MSG_H__
#define IMU_RAW_MSG_H__

#include <rtdef.h>
#include "uMCN.h"

/* 陀螺仪原始数据消息类型 */
typedef struct {
  float gyro[3];
  rt_uint32_t seq;
} gyro_raw_msg_t;

/* 加速度计原始数据消息类型 */
typedef struct {
  float accel[3];
  rt_uint32_t seq;
} acc_raw_msg_t;

MCN_DECLARE(gyro_raw);
MCN_DECLARE(acc_raw);

#endif /* IMU_RAW_MSG_H__ */

