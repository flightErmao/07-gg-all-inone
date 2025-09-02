#pragma once

#include <stdint.h>
#include <stddef.h>

// 状态码
#define MPU_EOK 0
#define MPU_ERROR -1

// 对外常量
#define M_PI_F 3.1415926f

#include "mpu6000_reg.h"

class Mpu6000 {
 public:
  // 基础读写与延迟函数指针类型（返回值为int，0为成功）
  typedef int (*WriteRegFn)(uint8_t reg, uint8_t val);
  typedef int (*ReadMultiRegFn)(uint8_t reg, uint8_t* buff, uint8_t len);
  typedef void (*DelayMsFn)(unsigned int ms);

  static Mpu6000& instance();

  // 配置底层函数
  int set_io_functions(WriteRegFn write_fn, ReadMultiRegFn read_fn, DelayMsFn delay_fn);

  // 初始化
  int init();

  // 读取原始与物理量数据
  int read_gyro_raw(int16_t gyr[3]);
  int read_gyro_rad(float gyr[3]);

  int read_accel_raw(int16_t acc[3]);
  int read_accel_m_s2(float acc[3]);

  int read_temperature(float* temp_c);

  // 可选：自检
  int self_test();

  // 新增：读取从 ACCEL_XOUT_H 起连续14字节（acc[6]+temp[2]+gyr[6]）
  int read_burst_imu(uint8_t out14[14]);

 private:
  Mpu6000();

  // 内部工具
  int write_checked(uint8_t reg, uint8_t val);
  int read_reg(uint8_t reg, uint8_t* val);  // 单字节读，底层由 read_multi_reg_ 包装
  int set_sample_rate(unsigned desired_hz);
  int set_dlpf_filter(uint16_t frequency_hz);
  int set_accel_range(unsigned max_g);
  int set_gyro_range(unsigned max_dps);

  // 旋转到FRD坐标，允许弱符号覆盖
  void rotate_to_frd(float* v3);

 private:
  // 底层函数
  WriteRegFn write_reg_;
  ReadMultiRegFn read_multi_reg_;
  DelayMsFn delay_ms_;

  // 参数缓存
  unsigned sample_rate_hz_;
  unsigned dlpf_freq_hz_;
  uint8_t product_id_;
  float gyro_range_scale_;
  float gyro_range_rad_s_;
  float accel_range_scale_;
  float accel_range_m_s2_;
};

// C接口（单例的薄封装）
extern "C" {
int drv_mpu6000_set_delay(void (*delay_ms)(unsigned int));
int drv_mpu6000_set_rw(int (*write_fn)(uint8_t, uint8_t), int (*read_fn)(uint8_t, uint8_t*, uint8_t));
int drv_mpu6000_set_spi_funcs(int8_t (*write_func)(uint8_t, uint8_t, uint8_t*, uint8_t),
                              int8_t (*read_func)(uint8_t, uint8_t, uint8_t*, uint8_t));
int drv_mpu6000_init(void);
int drv_mpu6000_read(int pos, void* data, int size);
}