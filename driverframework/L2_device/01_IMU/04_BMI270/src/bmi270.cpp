#include "bmi270.hpp"
#include "bmi270_config_blob_data.h"
#include <string.h>

Bmi270& Bmi270::instance() {
  static Bmi270 inst;
  return inst;
}

Bmi270::Bmi270()
    : read_multi_reg_(nullptr),
      write_multi_reg_(nullptr),
      delay_ms_(nullptr),
      gpio_cs_control_(nullptr),
      gyro_cas_(0) {}

Bmi270::~Bmi270() {}

int Bmi270::setIoFunctions(ReadMultiRegFn read_fn, WriteMultiRegFn write_fn, DelayMsFn delay_fn,
                           GpioCsControlFn cs_fn) {
  read_multi_reg_ = read_fn;
  write_multi_reg_ = write_fn;
  delay_ms_ = delay_fn;
  gpio_cs_control_ = cs_fn;
  return (read_multi_reg_ && write_multi_reg_ && delay_ms_ && gpio_cs_control_) ? 0 : -1;
}

void Bmi270::resetToSpi() {
  if (gpio_cs_control_) {
    gpio_cs_control_(false);
  }
  if (delay_ms_) {
    delay_ms_(1);
  }
  if (gpio_cs_control_) {
    gpio_cs_control_(true);
  }
  if (delay_ms_) {
    delay_ms_(10);
  }
}

int Bmi270::readReg8(uint8_t reg, uint8_t* val) {
  if (!read_multi_reg_ || !val) return -1;
  uint8_t tmp[2] = {0};
  if (read_multi_reg_(reg, tmp, 1) != 0) return -1;
  *val = tmp[0];
  return 0;
}

int Bmi270::readReg16(uint8_t reg, uint16_t* out) {
  if (!read_multi_reg_ || !out) return -1;
  uint8_t tmp[2] = {0};
  if (read_multi_reg_(reg, tmp, 2) != 0) return -1;
  *out = (uint16_t)(((uint16_t)tmp[1] << 8) | tmp[0]);
  return 0;
}

int Bmi270::writeReg8(uint8_t reg, uint8_t val, unsigned delay_ms) {
  if (!write_multi_reg_) return -1;
  uint8_t v = val;
  if (write_multi_reg_(reg, &v, 1) != 0) return -1;
  if (delay_ms_ && delay_ms > 0) delay_ms_(delay_ms);
  return 0;
}

int Bmi270::writeReg16(uint8_t reg, uint16_t val, unsigned delay_ms) {
  if (!write_multi_reg_) return -1;
  uint8_t tmp[2];
  tmp[0] = (uint8_t)(val & 0xFFu);
  tmp[1] = (uint8_t)((val >> 8) & 0xFFu);
  if (write_multi_reg_(reg, tmp, 2) != 0) return -1;
  if (delay_ms_ && delay_ms > 0) delay_ms_(delay_ms);
  return 0;
}

int Bmi270::writeMultiDelay(uint8_t reg, const uint8_t* data, uint16_t len, unsigned delay_ms) {
  if (!write_multi_reg_) return -1;
  if (write_multi_reg_(reg, (uint8_t*)data, (uint16_t)len) != 0) return -1;
  if (delay_ms_ && delay_ms > 0) delay_ms_(delay_ms);
  return 0;
}

int Bmi270::detect() {
  resetToSpi();
  uint8_t id = 0;
  if (readReg8(BMI270_REG_CHIP_ID, &id) != 0) return -1;
  return (id == 0x24) ? 0 : -1;
}

int Bmi270::basicInit() {
  resetToSpi();
  if (writeReg8(BMI270_REG_CMD, BMI270_CMD_SOFTRESET, 100) != 0) return -1;
  resetToSpi();

  if (writeReg8(BMI270_REG_PWR_CONF, 0x0, 1) != 0) return -1;
  if (writeReg8(BMI270_REG_INIT_CTRL, 0x0, 1) != 0) return -1;
  if (writeMultiDelay(BMI270_REG_INIT_DATA, bmi270_config_file, (uint16_t)sizeof(bmi270_config_file), 10) != 0)
    return -1;
  if (writeReg8(BMI270_REG_INIT_CTRL, 0x1, 1) != 0) return -1;
  return 0;
}

int Bmi270::initConfig() {
  if (writeReg8(BMI270_REG_ACC_CONF,
                (uint8_t)((BMI270_ACC_CONF_HP << 7) | (BMI270_ACC_CONF_BWP << 4) | BMI270_ACC_CONF_ODR800), 1) != 0)
    return -1;
  if (writeReg8(BMI270_REG_ACC_RANGE, BMI270_ACC_RANGE_16G, 1) != 0) return -1;
  if (writeReg8(BMI270_REG_GYRO_CONF,
                (uint8_t)((BMI270_GYRO_CONF_FILTER_PERF << 7) | (BMI270_GYRO_CONF_NOISE_PERF << 6) |
                          (BMI270_GYRO_CONF_BWP << 4) | BMI270_GYRO_CONF_ODR3200),
                1) != 0)
    return -1;
  if (writeReg8(BMI270_REG_GYRO_RANGE, BMI270_GYRO_RANGE_2000DPS, 1) != 0) return -1;
  if (writeReg8(BMI270_REG_INT_MAP_DATA, BMI270_INT_MAP_DATA_DRDY_INT1, 1) != 0) return -1;
  if (writeReg8(BMI270_REG_INT1_IO_CTRL, BMI270_INT1_IO_CTRL_PINMODE, 1) != 0) return -1;
  if (writeReg8(BMI270_REG_PWR_CONF, BMI270_PWR_CONF, 1) != 0) return -1;
  if (writeReg8(BMI270_REG_PWR_CTRL, BMI270_PWR_CTRL, 1) != 0) return -1;
  return 0;
}

static int8_t compute_gyro_cas(uint8_t raw) {
  const uint8_t masked = (uint8_t)(raw & BMI270_GYRO_CAS_MASK);
  if ((masked & BMI270_GYRO_CAS_SIGN_BIT_MASK) == 0) {
    return (int8_t)(masked);
  }
  return (int8_t)(masked - 128);
}

int Bmi270::enableCas() {
  if (writeReg8(BMI270_REG_FEAT_PAGE, 1, 1) != 0) return -1;
  if (writeReg16(BMI270_REG_FEATURES_1_GEN_SET_1, BMI270_CONF_FEATURES_1_GEN_SET_1, 1) != 0) return -1;

  uint8_t offs6 = 0;
  if (readReg8(BMI270_REG_OFFSET_6, &offs6) != 0) return -1;
  if (writeReg8(BMI270_REG_OFFSET_6, (uint8_t)(offs6 | BMI270_GYRO_CONF_OFFSET_6), 1) != 0) return -1;

  if (writeReg8(BMI270_REG_FEAT_PAGE, 0, 1) != 0) return -1;

  uint8_t cas_factor = 0;
  if (readReg8(BMI270_REG_FEATURES_0_GYR_CAS, &cas_factor) != 0) return -1;
  gyro_cas_ = compute_gyro_cas(cas_factor);
  return 0;
}

int Bmi270::enableCrt() {
  if (writeReg8(BMI270_REG_PWR_CONF, 0x0, 10) != 0) return -1;
  if (writeReg8(BMI270_REG_OFFSET_6, 0x80, 10) != 0) return -1;
  if (writeReg8(BMI270_REG_PWR_CTRL, 0x04, 10) != 0) return -1;
  if (writeReg8(BMI270_REG_GYR_CRT_CONF, 0x04, 10) != 0) return -1;
  if (writeReg8(BMI270_REG_FEAT_PAGE, 0x01, 10) != 0) return -1;
  if (writeReg16(BMI270_REG_FEATURES_1_G_TRIG_1, 0x0100, 10) != 0) return -1;
  if (writeReg8(BMI270_REG_CMD, 0x02, 0) != 0) return -1;

  if (delay_ms_) {
    for (uint8_t i = 0; i < 10; i++) {
      delay_ms_(1000);
      uint8_t crt_check = 0;
      if (readReg8(BMI270_REG_GYR_CRT_CONF, &crt_check) != 0) return -1;
      if (crt_check == 0x00) break;
    }
    delay_ms_(10);
  }

  if (writeReg8(BMI270_REG_FEAT_PAGE, 0x00, 10) != 0) return -1;
  uint16_t gain_status = 0;
  if (readReg16(BMI270_REG_FEATURES_0_GYR_GAIN_STATUS, &gain_status) != 0) return -1;
  if (delay_ms_) {
    if (gain_status == 0x0000)
      delay_ms_(10);
    else
      delay_ms_(1);
  }
  return 0;
}

int Bmi270::init() {
  if (!read_multi_reg_ || !write_multi_reg_ || !delay_ms_ || !gpio_cs_control_) return -1;
  if (basicInit() != 0) return -1;
  if (initConfig() != 0) return -1;
  if (enableCas() != 0) return -1;
  return 0;
}

int Bmi270::calibrate() {
  if (!read_multi_reg_ || !write_multi_reg_ || !delay_ms_ || !gpio_cs_control_) return -1;
  if (basicInit() != 0) return -1;
  if (enableCrt() != 0) return -1;
  if (initConfig() != 0) return -1;
  if (enableCas() != 0) return -1;
  return 0;
}

int Bmi270::readBurstImu(uint8_t* out14) {
  if (!read_multi_reg_ || !out14) return -1;
  return (read_multi_reg_(BMI270_REG_ACC_DATA_X_LSB, out14, 14) == 0) ? 0 : -1;
}
