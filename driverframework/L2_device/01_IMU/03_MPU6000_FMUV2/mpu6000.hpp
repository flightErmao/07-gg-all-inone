#pragma once

#include <stddef.h>
#include <stdint.h>

#include "mpu6000_reg.h"

class Mpu6000 {
 public:
  typedef int (*WriteRegFn)(uint8_t reg, uint8_t val);
  typedef int (*ReadMultiRegFn)(uint8_t reg, uint8_t* buff, uint8_t len);
  typedef void (*DelayMsFn)(unsigned int ms);

  static Mpu6000& instance();
  Mpu6000();
  ~Mpu6000();

  int set_io_functions(WriteRegFn write_fn, ReadMultiRegFn read_fn, DelayMsFn delay_fn);
  int init();
  int read_gyro_rad(float gyr[3]);
  int read_accel_m_s2(float acc[3]);
  int read_temperature(float* temp_c);
  int read_burst_imu(uint8_t out14[14]);
  DelayMsFn get_delay_ms() const { return delay_ms_; }

 private:
  int write_checked(uint8_t reg, uint8_t val);
  int read_reg(uint8_t reg, uint8_t* val);
  int read_gyro_raw(int16_t gyr[3]);
  int read_accel_raw(int16_t acc[3]);
  int set_sample_rate(unsigned desired_hz);
  int set_dlpf_filter(uint16_t frequency_hz);
  int set_accel_range(unsigned max_g);
  int set_gyro_range(unsigned max_dps);
  int probe();
  void rotate_to_frd(float* v3);

  WriteRegFn write_reg_;
  ReadMultiRegFn read_multi_reg_;
  DelayMsFn delay_ms_;

  uint8_t product_id_;
  unsigned dlpf_freq_hz_;

  float gyro_range_scale_;
  float accel_range_scale_;
};

extern "C" {
int drv_mpu6000_set_delay(void (*delay_ms)(unsigned int));
int drv_mpu6000_set_rw(int (*write_fn)(uint8_t, uint8_t), int (*read_fn)(uint8_t, uint8_t*, uint8_t));
int drv_mpu6000_init(void);
int drv_mpu6000_read(int pos, void* data, int size);
}