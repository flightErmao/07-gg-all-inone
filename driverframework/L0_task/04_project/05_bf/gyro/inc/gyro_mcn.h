/**
 * @file gyro_filtered_msg.h
 *
 * 滤波后陀螺仪数据消息类型定义（C 接口，供 C++ 使用）
 */

#ifndef GYRO_FILTERED_MSG_H__
#define GYRO_FILTERED_MSG_H__

#include <rtdef.h>
#include "uMCN.h"

/* 滤波后陀螺仪数据消息类型 */
typedef struct {
  float gyro_filtered[3];  // 滤波后的角速度数据 [x, y, z]（对应 gyro_adcf_）
  float gyro_adc[3];       // 对齐、校准但未滤波的数据 [x, y, z]（对应 gyro_adc_）
  rt_uint32_t seq;         // 序列号（与 imu_raw 对应）
} gyro_filtered_msg_t;

MCN_DECLARE(gyro);

#endif /* GYRO_FILTERED_MSG_H__ */
