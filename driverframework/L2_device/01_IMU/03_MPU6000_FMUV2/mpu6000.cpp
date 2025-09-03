#include "mpu6000.hpp"

#include <string.h>

static inline int16_t be16_to_s16(uint8_t hi, uint8_t lo) { return (int16_t)((int16_t)((uint16_t)hi << 8) | lo); }

Mpu6000& Mpu6000::instance() {
  static Mpu6000 inst;
  return inst;
}

Mpu6000::Mpu6000()
    : write_reg_(nullptr),
      read_multi_reg_(nullptr),
      delay_ms_(nullptr),
      product_id_(0),
      dlpf_freq_hz_(MPU6000_DEFAULT_ONCHIP_FILTER_FREQ),
      gyro_range_scale_(0.0f),
      accel_range_scale_(0.0f) {}

Mpu6000::~Mpu6000() {}

int Mpu6000::set_io_functions(WriteRegFn write_fn, ReadMultiRegFn read_fn, DelayMsFn delay_fn) {
  write_reg_ = write_fn;
  read_multi_reg_ = read_fn;
  delay_ms_ = delay_fn;
  return (write_reg_ && read_multi_reg_) ? MPU_EOK : MPU_ERROR;
}

int Mpu6000::read_reg(uint8_t reg, uint8_t* val) {
  if (!read_multi_reg_ || !val) return MPU_ERROR;
  uint8_t tmp = 0;
  int rc = read_multi_reg_(reg, &tmp, 1);
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
  if (desired_hz == 0) {
    desired_hz = MPU6000_GYRO_DEFAULT_RATE;
  }

  uint8_t div = 1000 / desired_hz;

  if (div > 200) {
    div = 200;
  }

  if (div < 1) {
    div = 1;
  }

  return write_checked(MPUREG_SMPLRT_DIV, div - 1);
}

int Mpu6000::set_dlpf_filter(uint16_t frequency_hz) {
  uint8_t filter;

  if (frequency_hz == 0) {
    filter = BITS_DLPF_CFG_2100HZ_NOLPF;

  } else if (frequency_hz <= 5) {
    filter = BITS_DLPF_CFG_5HZ;

  } else if (frequency_hz <= 10) {
    filter = BITS_DLPF_CFG_10HZ;

  } else if (frequency_hz <= 21) {
    filter = BITS_DLPF_CFG_20HZ;

  } else if (frequency_hz <= 44) {
    filter = BITS_DLPF_CFG_42HZ;

  } else if (frequency_hz <= 98) {
    filter = BITS_DLPF_CFG_98HZ;

  } else if (frequency_hz <= 188) {
    filter = BITS_DLPF_CFG_188HZ;

  } else if (frequency_hz <= 260) {
    filter = BITS_DLPF_CFG_256HZ_NOLPF2;

  } else {
    filter = BITS_DLPF_CFG_2100HZ_NOLPF;
  }

  return write_checked(MPUREG_CONFIG, filter);
}

int Mpu6000::set_accel_range(unsigned max_g_in) {
  switch (product_id_) {
    case MPU6000ES_REV_C4:
    case MPU6000ES_REV_C5:
    case MPU6000_REV_C4:
    case MPU6000_REV_C5: {
      int rc = write_checked(MPUREG_ACCEL_CONFIG, (uint8_t)(1u << 3));
      if (rc != MPU_EOK) return MPU_ERROR;
      accel_range_scale_ = MPU6000_ONE_G / 4096.0f;
      return MPU_EOK;
    }
  }

  uint8_t afs_sel;
  float lsb_per_g;

  if (max_g_in > 8) {
    afs_sel = 3;
    lsb_per_g = 2048.0f;
  } else if (max_g_in > 4) {
    afs_sel = 2;
    lsb_per_g = 4096.0f;
  } else if (max_g_in > 2) {
    afs_sel = 1;
    lsb_per_g = 8192.0f;
  } else {
    afs_sel = 0;
    lsb_per_g = 16384.0f;
  }

  accel_range_scale_ = MPU6000_ONE_G / lsb_per_g;
  return write_checked(MPUREG_ACCEL_CONFIG, (uint8_t)(afs_sel << 3));
}

int Mpu6000::set_gyro_range(unsigned max_dps) {
  uint8_t fs_sel;

  if (max_dps <= 250) {
    fs_sel = BITS_FS_250DPS;
  } else if (max_dps <= 500) {
    fs_sel = BITS_FS_500DPS;
  } else if (max_dps <= 1000) {
    fs_sel = BITS_FS_1000DPS;
  } else {
    fs_sel = BITS_FS_2000DPS;
  }

  gyro_range_scale_ = (M_PI_F / 180.0f) / (32768.0f / (float)max_dps);
  return write_checked(MPUREG_GYRO_CONFIG, fs_sel);
}

void Mpu6000::rotate_to_frd(float* v3) { (void)v3; }

int Mpu6000::init() {
  if (!write_reg_ || !read_multi_reg_ || !delay_ms_) return MPU_ERROR;

  if (probe() != MPU_EOK) {
    return MPU_ERROR;
  }

  (void)read_reg(MPUREG_PRODUCT_ID, &product_id_);

  uint8_t tries = 5;
  uint8_t reg_val = 0;

  while (--tries != 0) {
    if (write_checked(MPUREG_PWR_MGMT_1, BIT_H_RESET) != MPU_EOK) {
      return MPU_ERROR;
    }
    delay_ms_(10);

    if (write_checked(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ) != MPU_EOK) {
      return MPU_ERROR;
    }
    delay_ms_(1);

    if (write_checked(MPUREG_USER_CTRL, BIT_I2C_IF_DIS) != MPU_EOK) {
      return MPU_ERROR;
    }

    int rc2 = read_reg(MPUREG_PWR_MGMT_1, &reg_val);
    if (rc2 == MPU_EOK && reg_val == MPU_CLK_SEL_PLLGYROZ) {
      break;
    }

    delay_ms_(2);
  }

  if (read_reg(MPUREG_PWR_MGMT_1, &reg_val) != MPU_EOK) {
    return MPU_ERROR;
  }
  if (reg_val != MPU_CLK_SEL_PLLGYROZ) {
    return MPU_ERROR;
  }

  delay_ms_(1);

  if (set_sample_rate(MPU6000_ACCEL_DEFAULT_RATE) != MPU_EOK) {
    return MPU_ERROR;
  }

  delay_ms_(1);

  if (set_dlpf_filter(MPU6000_DEFAULT_ONCHIP_FILTER_FREQ) != MPU_EOK) {
    return MPU_ERROR;
  }

  delay_ms_(1);

  (void)set_gyro_range(2000);
  delay_ms_(1);
  (void)set_accel_range(8);
  delay_ms_(1);

  if (write_checked(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN) != MPU_EOK) {
    return MPU_ERROR;
  }
  delay_ms_(1);
  if (write_checked(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR) != MPU_EOK) {
    return MPU_ERROR;
  }
  delay_ms_(1);

  return MPU_EOK;
}

int Mpu6000::read_gyro_raw(int16_t gyr[3]) {
  if (!read_multi_reg_) return MPU_ERROR;
  uint8_t buf[8];
  if (read_multi_reg_(MPUREG_GYRO_XOUT_H, buf, 6) != 0) return MPU_ERROR;
  gyr[0] = be16_to_s16(buf[0], buf[1]);
  gyr[1] = be16_to_s16(buf[2], buf[3]);
  gyr[2] = be16_to_s16(buf[4], buf[5]);
  return MPU_EOK;
}

int Mpu6000::read_gyro_rad(float gyr[3]) {
  int16_t raw[3];
  if (read_gyro_raw(raw) != MPU_EOK) return MPU_ERROR;
  gyr[0] = gyro_range_scale_ * (float)raw[0];
  gyr[1] = gyro_range_scale_ * (float)raw[1];
  gyr[2] = gyro_range_scale_ * (float)raw[2];
  rotate_to_frd(gyr);
  return MPU_EOK;
}

int Mpu6000::read_accel_raw(int16_t acc[3]) {
  if (!read_multi_reg_) return MPU_ERROR;
  uint8_t buf[8];
  if (read_multi_reg_(MPUREG_ACCEL_XOUT_H, buf, 6) != 0) return MPU_ERROR;
  acc[0] = be16_to_s16(buf[0], buf[1]);
  acc[1] = be16_to_s16(buf[2], buf[3]);
  acc[2] = be16_to_s16(buf[4], buf[5]);
  return MPU_EOK;
}

int Mpu6000::read_accel_m_s2(float acc[3]) {
  int16_t raw[3];
  if (read_accel_raw(raw) != MPU_EOK) return MPU_ERROR;
  acc[0] = accel_range_scale_ * (float)raw[0];
  acc[1] = accel_range_scale_ * (float)raw[1];
  acc[2] = accel_range_scale_ * (float)raw[2];
  rotate_to_frd(acc);
  return MPU_EOK;
}

int Mpu6000::read_temperature(float* temp_c) {
  if (!read_multi_reg_ || !temp_c) return MPU_ERROR;
  uint8_t buf[2];
  if (read_multi_reg_(MPUREG_TEMP_OUT_H, buf, 2) != 0) return MPU_ERROR;
  int16_t raw = be16_to_s16(buf[0], buf[1]);
  *temp_c = ((float)raw) / 340.0f + 36.53f;
  return MPU_EOK;
}

int Mpu6000::probe() {
  uint8_t who = 0;
  if (read_reg(MPUREG_WHOAMI, &who) != MPU_EOK) return MPU_ERROR;
  return (who == MPU_WHOAMI_6000 || who == ICM_WHOAMI_20608) ? MPU_EOK : MPU_ERROR;
}

int Mpu6000::read_burst_imu(uint8_t out14[14]) {
  if (!read_multi_reg_ || !out14) return MPU_ERROR;
  return read_multi_reg_(MPUREG_ACCEL_XOUT_H, out14, 14) == 0 ? MPU_EOK : MPU_ERROR;
}

extern "C" {

int drv_mpu6000_set_delay(void (*delay_ms)(unsigned int)) {
  return Mpu6000::instance().set_io_functions(nullptr, nullptr, delay_ms) == MPU_EOK ? MPU_EOK : MPU_ERROR;
}

int drv_mpu6000_set_rw(int (*write_fn)(uint8_t, uint8_t), int (*read_fn)(uint8_t, uint8_t*, uint8_t)) {
  Mpu6000& d = Mpu6000::instance();
  return d.set_io_functions(write_fn, read_fn, d.get_delay_ms()) == MPU_EOK ? MPU_EOK : MPU_ERROR;
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
      int rc3 = d.read_burst_imu(out);
      return (rc3 == MPU_EOK) ? 14 : MPU_ERROR;
    }
    default:
      return MPU_ERROR;
  }
}

}  // extern "C"