#ifndef __SENSORS_TYPES_H
#define __SENSORS_TYPES_H
#include "stdint.h"
#include "stdbool.h"

// 支持GCC匿名联合体
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

typedef union {
  struct {
    int16_t x;
    int16_t y;
    int16_t z;
  };
  int16_t axis[3];
} Axis3i16;

typedef union {
  struct {
    int32_t x;
    int32_t y;
    int32_t z;
  };
  int32_t axis[3];
} Axis3i32;

typedef union {
  struct {
    int64_t x;
    int64_t y;
    int64_t z;
  };
  int64_t axis[3];
} Axis3i64;

typedef union {
  struct {
    float x;
    float y;
    float z;
  };
  float axis[3];
} Axis3f;

typedef struct {
  uint32_t timestamp;
  Axis3i16 acc_raw;
  Axis3i16 gyro_raw;
  Axis3f acc_filter;
  Axis3f gyro_filter;
} sensorData_t;

// 恢复GCC诊断设置
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#endif /* __SENSORS_TYPES_H */
