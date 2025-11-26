/**
 * @file attitude_mcn.h
 *
 * 姿态数据消息类型定义（C 接口，供 C++ 使用）
 */

#ifndef ATTITUDE_MCN_H__
#define ATTITUDE_MCN_H__

#include <rtdef.h>
#include "uMCN.h"

/* 姿态数据消息类型（参考 Betaflight attitude.raw 和 attitude.values） */
typedef struct {
  int16_t raw[3];        // 原始角度值 [roll, pitch, yaw]（单位：十分之一度，centidegrees）
  float values[3];       // 浮点角度值 [roll, pitch, yaw]（单位：度）
  rt_uint32_t seq;       // 序列号（与 imu_raw 对应）
  rt_uint32_t timestamp; // 时间戳（微秒）
} attitude_msg_t;

MCN_DECLARE(attitude);

#endif /* ATTITUDE_MCN_H__ */

