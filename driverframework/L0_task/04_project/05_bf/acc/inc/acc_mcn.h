/**
 * @file acc_mcn.h
 *
 * 加速度计处理数据消息类型定义（C 接口，供 C++ 使用）
 */

#ifndef ACC_MCN_H__
#define ACC_MCN_H__

#include <rtdef.h>
#include "uMCN.h"

/* 加速度计处理数据消息类型 */
typedef struct {
  float acc_filtered[3];  // 处理后的加速度数据 [x, y, z]（对齐 → 校准 → Trim → PT2滤波）
  float acc_adc[3];       // 对齐、校准但未Trim和滤波的数据 [x, y, z]
  rt_uint32_t seq;        // 序列号（与 imu_raw 对应）
  rt_uint32_t timestamp;  // 时间戳（微秒）
} acc_filtered_msg_t;

MCN_DECLARE(acc);

#endif /* ACC_MCN_H__ */

