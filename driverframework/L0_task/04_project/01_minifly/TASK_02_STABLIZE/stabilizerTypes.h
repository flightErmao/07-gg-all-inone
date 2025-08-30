#ifndef __STABILIZER_TYPES_H
#define __STABILIZER_TYPES_H
#include <stdbool.h>
#include "sensorsTypes.h"

#define RATE_5_HZ 5
#define RATE_10_HZ 10
#define RATE_25_HZ 25
#define RATE_50_HZ 50
#define RATE_100_HZ 100
#define RATE_200_HZ 200
#define RATE_250_HZ 250
#define RATE_500_HZ 500
#define RATE_1000_HZ 1000

#define MAIN_LOOP_RATE RATE_1000_HZ
#define MAIN_LOOP_DT (uint32_t)(1000 / MAIN_LOOP_RATE) /*单位ms*/

#define ATTITUDE_ESTIMAT_RATE RATE_250_HZ  // 姿态解算速率
#define ATTITUDE_ESTIMAT_DT (1.0 / RATE_250_HZ)

#define RATE_PID_RATE RATE_500_HZ  // 角速度环（内环）PID速率
#define RATE_PID_DT (1.0 / RATE_500_HZ)

#define ANGEL_PID_RATE ATTITUDE_ESTIMAT_RATE  // 角度环（外环）PID速率
#define ANGEL_PID_DT (1.0 / ATTITUDE_ESTIMAT_RATE)

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)

/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 结构体类型定义
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 ********************************************************************************/
// 支持GCC匿名联合体
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

typedef enum {
  modeDisable = 0, /*关闭模式*/
  modeVelocity,    /*速率模式*/
  modeAbs          /*绝对值模式*/
} mode_e;

typedef enum {
  CENTER = 0,
  FORWARD,
  BACK,
  LEFT,
  RIGHT,
} dir_e;

typedef struct {
  uint32_t timestamp; /*时间戳*/

  float roll;
  float pitch;
  float yaw;
} attitude_t;

typedef struct {
  uint32_t timestamp; /*时间戳*/

  float x;
  float y;
  float z;
} vec3_t;

typedef vec3_t point_t;
typedef vec3_t velocity_t;
typedef vec3_t acc_t;

/* Orientation as a quaternion */
typedef struct quaternion_s {
  uint32_t timestamp;

  union {
    struct {
      float q0;
      float q1;
      float q2;
      float q3;
    };
    struct {
      float x;
      float y;
      float z;
      float w;
    };
  };
} quaternion_t;

typedef struct toaMeasurement_s {
  int8_t senderId;
  float x, y, z;
  int64_t rx, tx;
} toaMeasurement_t;

typedef struct tdoaMeasurement_s {
  point_t anchorPosition[2];
  float distanceDiff;
  float stdDev;
} tdoaMeasurement_t;

typedef struct positionMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  float stdDev;
} positionMeasurement_t;

typedef struct distanceMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  float distance;
  float stdDev;
} distanceMeasurement_t;

typedef struct zRange_s {
  uint32_t timestamp;  // 时间戳
  float distance;      // 测量距离
  float quality;       // 可信度
} zRange_t;

/** Flow measurement**/
typedef struct flowMeasurement_s {
  uint32_t timestamp;
  union {
    struct {
      float dpixelx;  // Accumulated pixel count x
      float dpixely;  // Accumulated pixel count y
    };
    float dpixel[2];  // Accumulated pixel count
  };
  float stdDevX;  // Measurement standard deviation
  float stdDevY;  // Measurement standard deviation
  float dt;       // Time during which pixels were accumulated
} flowMeasurement_t;

typedef struct {
  Axis3f gyro_filter;
  attitude_t attitude;
  quaternion_t attitudeQuaternion;
  point_t position;
  velocity_t velocity;
  bool armed;
  uint8_t fly_mode;
} state_t;

typedef struct {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float thrust;
  dir_e flipDir; /*翻滚方向*/
} control_t;

typedef struct {
  mode_e x;
  mode_e y;
  mode_e z;
  mode_e roll;
  mode_e pitch;
  mode_e yaw;
} stabilizer_mode_t;

typedef struct {
  attitude_t attitude;      // deg
  attitude_t attitudeRate;  // deg/s
  point_t position;         // m
  velocity_t velocity;      // m/s
  stabilizer_mode_t mode;
  float thrust;
  bool armed;  // 解锁标志
} setpoint_t;

// 恢复GCC诊断设置
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#endif
