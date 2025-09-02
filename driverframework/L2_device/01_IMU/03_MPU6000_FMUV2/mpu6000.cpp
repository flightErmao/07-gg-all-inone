#include "mpu6000.hpp"

#include <string.h>

static inline int16_t be16_to_s16(uint8_t hi, uint8_t lo) { return (int16_t)((int16_t)((uint16_t)hi << 8) | lo); }

// 兼容旧式4参SPI函数指针的适配存根
static int8_t (*g_spi_write_legacy)(uint8_t, uint8_t, uint8_t*, uint8_t) = nullptr;
static int8_t (*g_spi_read_legacy)(uint8_t, uint8_t, uint8_t*, uint8_t) = nullptr;

static int write_adapter(uint8_t reg, uint8_t val) {
  if (!g_spi_write_legacy) return -1;
  uint8_t b = val;
  return g_spi_write_legacy(0, reg, &b, 1) == 0 ? 0 : -1;
}

static int read_adapter(uint8_t reg, uint8_t* buff, uint8_t len) {
  if (!g_spi_read_legacy) return -1;
  return g_spi_read_legacy(0, reg, buff, len) == 0 ? 0 : -1;
}

Mpu6000& Mpu6000::instance() {
  static Mpu6000 inst;
  return inst;
}

Mpu6000::Mpu6000()
    : write_reg_(nullptr),
      read_multi_reg_(nullptr),
      delay_ms_(nullptr),
      sample_rate_hz_(1000),
      dlpf_freq_hz_(MPU6000_DEFAULT_ONCHIP_FILTER_FREQ),
      product_id_(0),
      gyro_range_scale_(2000.0f / 32768.0f),
      gyro_range_rad_s_((2000.0f / 32768.0f) * (3.1415926f / 180.0f)),
      accel_range_scale_(MPU6000_ACCEL_DEFAULT_RANGE_G / 32768.0f),
      accel_range_m_s2_((MPU6000_ACCEL_DEFAULT_RANGE_G / 32768.0f) * MPU6000_ONE_G) {}

int Mpu6000::set_io_functions(WriteRegFn write_fn, ReadMultiRegFn read_fn, DelayMsFn delay_fn) {
  write_reg_ = write_fn;
  read_multi_reg_ = read_fn;
  delay_ms_ = delay_fn;
  return (write_reg_ && read_multi_reg_) ? MPU_EOK : MPU_ERROR;
}

int Mpu6000::read_reg(uint8_t reg, uint8_t* val) {
  if (!read_multi_reg_ || !val) return MPU_ERROR;
  uint8_t tmp = 0;
  int rc = read_multi_reg_(reg | DIR_READ, &tmp, 1);
  if (rc != 0) return MPU_ERROR;
  *val = tmp;
  return MPU_EOK;
}

int Mpu6000::write_checked(uint8_t reg, uint8_t val) {
  if (!write_reg_) return MPU_ERROR;
  int rc = write_reg_(reg, val);
  if (rc != 0) return MPU_ERROR;
  uint8_t readback = 0;
  if (read_reg(reg, &readback) != MPU_EOK) return MPU_ERROR;
  return (readback == val) ? MPU_EOK : MPU_ERROR;
}

int Mpu6000::set_sample_rate(unsigned desired_hz) {
  if (desired_hz == 0) desired_hz = 1000;
  unsigned base = 1000;  // 开启DLPF时采样基准为1kHz
  unsigned divider = 0;
  if (desired_hz < base) {
    divider = (base / desired_hz) - 1;
    if (divider > 255) divider = 255;
  }
  if (write_checked(MPUREG_SMPLRT_DIV, (uint8_t)divider) != MPU_EOK) return MPU_ERROR;
  sample_rate_hz_ = base / (divider + 1);
  return MPU_EOK;
}

int Mpu6000::set_dlpf_filter(uint16_t frequency_hz) {
  uint8_t cfg = 0;
  if (frequency_hz >= 256)
    cfg = BITS_DLPF_CFG_256HZ_NOLPF2;
  else if (frequency_hz >= 188)
    cfg = BITS_DLPF_CFG_188HZ;
  else if (frequency_hz >= 98)
    cfg = BITS_DLPF_CFG_98HZ;
  else if (frequency_hz >= 42)
    cfg = BITS_DLPF_CFG_42HZ;
  else if (frequency_hz >= 20)
    cfg = BITS_DLPF_CFG_20HZ;
  else if (frequency_hz >= 10)
    cfg = BITS_DLPF_CFG_10HZ;
  else
    cfg = BITS_DLPF_CFG_5HZ;
  if (write_checked(MPUREG_CONFIG, cfg) != MPU_EOK) return MPU_ERROR;
  dlpf_freq_hz_ = frequency_hz;
  return MPU_EOK;
}

int Mpu6000::set_accel_range(unsigned max_g) {
  uint8_t bits = 0;  // 默认2g
  if (max_g >= 16)
    bits = 0x18;
  else if (max_g >= 8)
    bits = 0x10;
  else if (max_g >= 4)
    bits = 0x08;
  else
    bits = 0x00;
  if (write_checked(MPUREG_ACCEL_CONFIG, bits) != MPU_EOK) return MPU_ERROR;
  unsigned range_g = 2;
  if (bits == 0x00)
    range_g = 2;
  else if (bits == 0x08)
    range_g = 4;
  else if (bits == 0x10)
    range_g = 8;
  else
    range_g = 16;
  accel_range_scale_ = (float)range_g / 32768.0f;
  accel_range_m_s2_ = accel_range_scale_ * MPU6000_ONE_G;
  return MPU_EOK;
}

int Mpu6000::set_gyro_range(unsigned max_dps) {
  uint8_t bits = BITS_FS_250DPS;
  unsigned range_dps = 250;
  if (max_dps >= 2000) {
    bits = BITS_FS_2000DPS;
    range_dps = 2000;
  } else if (max_dps >= 1000) {
    bits = BITS_FS_1000DPS;
    range_dps = 1000;
  } else if (max_dps >= 500) {
    bits = BITS_FS_500DPS;
    range_dps = 500;
  } else {
    bits = BITS_FS_250DPS;
    range_dps = 250;
  }
  if (write_checked(MPUREG_GYRO_CONFIG, bits) != MPU_EOK) return MPU_ERROR;
  gyro_range_scale_ = (float)range_dps / 32768.0f;
  gyro_range_rad_s_ = gyro_range_scale_ * (3.1415926f / 180.0f);
  return MPU_EOK;
}

void Mpu6000::rotate_to_frd(float* v3) { (void)v3; }

int Mpu6000::init() {
  if (!write_reg_ || !read_multi_reg_ || !delay_ms_) return MPU_ERROR;

  if (write_checked(MPUREG_PWR_MGMT_1, BIT_H_RESET) != MPU_EOK) return MPU_ERROR;
  delay_ms_(100);

  if (write_checked(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ) != MPU_EOK) return MPU_ERROR;
  delay_ms_(10);

  if (write_checked(MPUREG_USER_CTRL, BIT_I2C_IF_DIS) != MPU_EOK) return MPU_ERROR;
  if (write_checked(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN) != MPU_EOK) return MPU_ERROR;
  if (write_checked(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR) != MPU_EOK) return MPU_ERROR;

  if (set_dlpf_filter(98) != MPU_EOK) return MPU_ERROR;
  if (set_sample_rate(1000) != MPU_EOK) return MPU_ERROR;
  if (set_accel_range(MPU6000_ACCEL_DEFAULT_RANGE_G) != MPU_EOK) return MPU_ERROR;
  if (set_gyro_range(2000) != MPU_EOK) return MPU_ERROR;

  uint8_t whoami = 0;
  if (read_reg(MPUREG_WHOAMI, &whoami) != MPU_EOK) return MPU_ERROR;
  product_id_ = whoami;
  if (!(whoami == MPU_WHOAMI_6000 || whoami == ICM_WHOAMI_20608)) return MPU_ERROR;

  return MPU_EOK;
}

int Mpu6000::read_gyro_raw(int16_t gyr[3]) {
  if (!read_multi_reg_) return MPU_ERROR;
  uint8_t buf[8];
  if (read_multi_reg_(MPUREG_GYRO_XOUT_H | DIR_READ, buf, 6) != 0) return MPU_ERROR;
  gyr[0] = be16_to_s16(buf[0], buf[1]);
  gyr[1] = be16_to_s16(buf[2], buf[3]);
  gyr[2] = be16_to_s16(buf[4], buf[5]);
  return MPU_EOK;
}

int Mpu6000::read_gyro_rad(float gyr[3]) {
  int16_t raw[3];
  if (read_gyro_raw(raw) != MPU_EOK) return MPU_ERROR;
  gyr[0] = (float)raw[0] * gyro_range_rad_s_;
  gyr[1] = (float)raw[1] * gyro_range_rad_s_;
  gyr[2] = (float)raw[2] * gyro_range_rad_s_;
  rotate_to_frd(gyr);
  return MPU_EOK;
}

int Mpu6000::read_accel_raw(int16_t acc[3]) {
  if (!read_multi_reg_) return MPU_ERROR;
  uint8_t buf[8];
  if (read_multi_reg_(MPUREG_ACCEL_XOUT_H | DIR_READ, buf, 6) != 0) return MPU_ERROR;
  acc[0] = be16_to_s16(buf[0], buf[1]);
  acc[1] = be16_to_s16(buf[2], buf[3]);
  acc[2] = be16_to_s16(buf[4], buf[5]);
  return MPU_EOK;
}

int Mpu6000::read_accel_m_s2(float acc[3]) {
  int16_t raw[3];
  if (read_accel_raw(raw) != MPU_EOK) return MPU_ERROR;
  acc[0] = (float)raw[0] * accel_range_m_s2_;
  acc[1] = (float)raw[1] * accel_range_m_s2_;
  acc[2] = (float)raw[2] * accel_range_m_s2_;
  rotate_to_frd(acc);
  return MPU_EOK;
}

int Mpu6000::read_temperature(float* temp_c) {
  if (!read_multi_reg_ || !temp_c) return MPU_ERROR;
  uint8_t buf[2];
  if (read_multi_reg_(MPUREG_TEMP_OUT_H | DIR_READ, buf, 2) != 0) return MPU_ERROR;
  int16_t raw = be16_to_s16(buf[0], buf[1]);
  *temp_c = ((float)raw) / 340.0f + 36.53f;
  return MPU_EOK;
}

int Mpu6000::self_test() {
  uint8_t who = 0;
  if (read_reg(MPUREG_WHOAMI, &who) != MPU_EOK) return MPU_ERROR;
  return (who == MPU_WHOAMI_6000 || who == ICM_WHOAMI_20608) ? MPU_EOK : MPU_ERROR;
}

int Mpu6000::read_burst_imu(uint8_t out14[14]) {
  if (!read_multi_reg_ || !out14) return MPU_ERROR;
  return read_multi_reg_(MPUREG_ACCEL_XOUT_H | DIR_READ, out14, 14) == 0 ? MPU_EOK : MPU_ERROR;
}

extern "C" {

int drv_mpu6000_set_delay(void (*delay_ms)(unsigned int)) {
  return Mpu6000::instance().set_io_functions(nullptr, nullptr, delay_ms) == MPU_EOK ? MPU_EOK : MPU_ERROR;
}

int drv_mpu6000_set_rw(int (*write_fn)(uint8_t, uint8_t), int (*read_fn)(uint8_t, uint8_t*, uint8_t)) {
  Mpu6000& d = Mpu6000::instance();
  return d.set_io_functions(write_fn, read_fn, d.delay_ms_) == MPU_EOK ? MPU_EOK : MPU_ERROR;
}

int drv_mpu6000_set_spi_funcs(int8_t (*write_func)(uint8_t, uint8_t, uint8_t*, uint8_t),
                              int8_t (*read_func)(uint8_t, uint8_t, uint8_t*, uint8_t)) {
  g_spi_write_legacy = write_func;
  g_spi_read_legacy = read_func;
  // 复用适配器将函数装载到类
  Mpu6000& d = Mpu6000::instance();
  return d.set_io_functions(write_adapter, read_adapter, d.delay_ms_) == MPU_EOK ? MPU_EOK : MPU_ERROR;
}

int drv_mpu6000_init(void) { return Mpu6000::instance().init(); }

int drv_mpu6000_read(int pos, void* data, int size) {
  if (!data) return MPU_ERROR;
  Mpu6000& d = Mpu6000::instance();
  switch (pos) {
    case 0: {
      if (size < (int)(sizeof(float) * 3)) return MPU_ERROR;
      float* acc = (float*)data;
      return d.read_accel_m_s2(acc);
    }
    case 1: {
      if (size < (int)(sizeof(float) * 3)) return MPU_ERROR;
      float* gyr = (float*)data;
      return d.read_gyro_rad(gyr);
    }
    case 2: {
      if (size < (int)sizeof(float)) return MPU_ERROR;
      float* t = (float*)data;
      return d.read_temperature(t);
    }
    case 3: {
      if (size < 14) return MPU_ERROR;
      uint8_t* out = (uint8_t*)data;
      int rc = d.read_burst_imu(out);
      return (rc == MPU_EOK) ? 14 : MPU_ERROR;
    }
    default:
      return MPU_ERROR;
  }
}

}  // extern "C"