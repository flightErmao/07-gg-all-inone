#include "GoertekBaro_spa06_003.hpp"

#include <stdint.h>

#include <cmath>

#define PRESSURE_DEBUG 0
#define COEF_REG_NUM 21

GoertekBaro::GoertekBaro() {
  dev_mode_ = GoertekBaro_MODE_IDLE;
  pressure_mr_ = PM_RATE_32_HZ;
  pressure_osr_ = OSR_8_TIMES;
  temperature_mr_ = TMP_RATE_32_HZ;
  temperature_osr_ = OSR_8_TIMES;
}

int GoertekBaro::DebugInit() {
  bool status = 0;
  uint8_t who_am_i = 0x00;

  if (!ReadWhoAmI(&who_am_i)) {
    rt_kprintf("[GoertekBaro][Init][read who am i fail! value is %d]", who_am_i);
    return -1;
  } else {
    rt_kprintf("[GoertekBaro][Init][read who am i success! value is %d]", who_am_i);
  }

  if ((who_am_i != WHO_AM_I_001) && (who_am_i != WHO_AM_I_003)) {
    return -2;
  } else {
    if (who_am_i == WHO_AM_I_001) {
      baro_model_ = SPL06_001;
      rt_kprintf("[GoertekBaro][Init][the baro model is SPL06_001]");
    } else if (who_am_i == WHO_AM_I_003) {
      baro_model_ = SPA06_003;
      rt_kprintf("[GoertekBaro][Init][the baro model is SPA06_003]");
    }
  }

  status = ReadCalibCoefficient();

  if (status != true) {
    return -3;
  }
  delay_msec(50);

  status = ConfigParam();
  if (status != true) {
    return -4;
  }

  status = Resume();
  if (status != true) {
    return -5;
  }

  has_inited_ = true;

  return 0;
}

bool GoertekBaro::Init() {
  if (DebugInit()) {
    return false;
  } else {
    return true;
  }
}

bool GoertekBaro::Deinit() {
  bool status = 0;
  uint8_t who_am_i = 0x00;

  if (!ReadWhoAmI(&who_am_i)) {
    rt_kprintf("baro GoertekBaro WHO AM I %d", who_am_i);
    return false;
  }

  rt_kprintf("baro GoertekBaro WHO AM I %d", who_am_i);

  if ((who_am_i != WHO_AM_I_001) && (who_am_i != WHO_AM_I_003)) {
    return false;
  }
  status = Standby();
  if (status != true) {
    return false;
  }

  rt_kprintf("baro GoertekBaro mode is standy");

  has_inited_ = false;

  return true;
}

bool GoertekBaro::Read(BaroData& data) {
  uint8_t read_buf[10] = {0};
  int32_t pressure_raw;
  int32_t temperature_raw;
  bool status;
  float temperature_scaled;
  float pressure_scaled;

  if (!has_inited_) {
    return false;
  }

  // data.timestamp_ms = Timer::Now() / 1000;

#if (PRESSURE_DEBUG)
  status = i2c_read_mult_func(static_cast<uint8_t>(GoertekBaroRegister::PSR_B2_ADDR), read_buf, 10);

  for (int i = 0; i < 10; i++) {
    rt_kprintf(" production_reg  %x ", read_buf[i]);
  }
#else
  status = i2c_read_mult_func(static_cast<uint8_t>(GoertekBaroRegister::PSR_B2_ADDR), read_buf, 6);
#endif

  if (!status) {
    return false;
  }

  pressure_raw = GetTwosComplement(((read_buf[2]) + (read_buf[1] << 8) + (read_buf[0] << 16)), 24);
  temperature_raw = GetTwosComplement(((read_buf[5]) + (read_buf[4] << 8) + (read_buf[3] << 16)), 24);

#if (PRESSURE_DEBUG)
  rt_kprintf(" perssure %d temperature_raw %d ", pressure_raw, temperature_raw);
#endif

  temperature_scaled = temperature_raw / (float)tmperature_osr_scale_coeff_;
  pressure_scaled = pressure_raw / (float)prs_osr_scale_coeff_;

  if (baro_model_ == SPL06_001) {
    data.pressure_pa = calib_coeffs_.c00 +
                       pressure_scaled * (calib_coeffs_.c10 +
                                          pressure_scaled * (calib_coeffs_.c20 + pressure_scaled * calib_coeffs_.c30)) +
                       temperature_scaled * calib_coeffs_.c01 +
                       temperature_scaled * pressure_scaled * (calib_coeffs_.c11 + pressure_scaled * calib_coeffs_.c21);
    data.id = SPL06_001;
  } else if (baro_model_ == SPA06_003) {
    data.pressure_pa =
        calib_coeffs_.c00 +
        pressure_scaled *
            (calib_coeffs_.c10 +
             pressure_scaled *
                 (calib_coeffs_.c20 + pressure_scaled * (calib_coeffs_.c30 + pressure_scaled * calib_coeffs_.c40))) +
        temperature_scaled * calib_coeffs_.c01 +
        temperature_scaled * pressure_scaled *
            (calib_coeffs_.c11 + pressure_scaled * (calib_coeffs_.c21 + pressure_scaled * calib_coeffs_.c31));
    data.id = SPA06_003;
  }

  data.temperature_celsius = 0.5f * calib_coeffs_.c0 + calib_coeffs_.c1 * temperature_scaled;

  return true;
}

bool GoertekBaro::ReadWhoAmI(uint8_t* who_am_i) {
  bool status = i2c_read_func(static_cast<uint8_t>(GoertekBaroRegister::PRODUCT_ID_ADDR), who_am_i);

  return status;
}

bool GoertekBaro::Resume() {
  bool status =
      i2c_write_func(static_cast<uint8_t>(GoertekBaroRegister::MEAS_CFG_ADDR), GoertekBaro_MODE_BACKGROUND_ALL);

  if (!status) {
    return false;
  }

  dev_mode_ = GoertekBaro_MODE_BACKGROUND_ALL;

  return true;
}

bool GoertekBaro::Standby() {
  bool status = i2c_write_func(static_cast<uint8_t>(GoertekBaroRegister::MEAS_CFG_ADDR), GoertekBaro_MODE_IDLE);
  if (!status) {
    return false;
  }

  dev_mode_ = GoertekBaro_MODE_IDLE;

  return true;
}

int32_t GoertekBaro::GetTwosComplement(int32_t raw, uint8_t length) {
  return (raw & ((uint32_t)1 << (length - 1))) ? (raw - ((uint32_t)1 << length)) : raw;
}

bool GoertekBaro::ReadCalibCoefficient() {
  uint8_t read_buf[COEF_REG_NUM] = {0};

  bool status = i2c_read_mult_func(static_cast<uint8_t>(GoertekBaroRegister::COEF_ADDR), read_buf, COEF_REG_NUM);

  if (!status) {
    return false;
  }

#if (PRESSURE_DEBUG)
  for (int i = 0; i < COEF_REG_NUM; i++) {
    rt_kprintf(" Red_reg  %x ", read_buf[i]);
  }
#endif

  // compose coefficients from buffer content
  calib_coeffs_.c0 = GetTwosComplement(((uint32_t)read_buf[0] << 4) | (((uint32_t)read_buf[1] >> 4) & 0x0F), 12);
  calib_coeffs_.c1 = GetTwosComplement((((uint32_t)read_buf[1] & 0x0F) << 8) | (uint32_t)read_buf[2], 12);
  calib_coeffs_.c00 = GetTwosComplement(
      ((uint32_t)read_buf[3] << 12) | ((uint32_t)read_buf[4] << 4) | (((uint32_t)read_buf[5] >> 4) & 0x0F), 20);
  calib_coeffs_.c10 = GetTwosComplement(
      (((uint32_t)read_buf[5] & 0x0F) << 16) | ((uint32_t)read_buf[6] << 8) | (uint32_t)read_buf[7], 20);
  calib_coeffs_.c01 = GetTwosComplement(((uint32_t)read_buf[8] << 8) | (uint32_t)read_buf[9], 16);
  calib_coeffs_.c11 = GetTwosComplement(((uint32_t)read_buf[10] << 8) | (uint32_t)read_buf[11], 16);
  calib_coeffs_.c20 = GetTwosComplement(((uint32_t)read_buf[12] << 8) | (uint32_t)read_buf[13], 16);
  calib_coeffs_.c21 = GetTwosComplement(((uint32_t)read_buf[14] << 8) | (uint32_t)read_buf[15], 16);
  calib_coeffs_.c30 = GetTwosComplement(((uint32_t)read_buf[16] << 8) | (uint32_t)read_buf[17], 16);

  if (baro_model_ == SPA06_003) {
    calib_coeffs_.c31 = GetTwosComplement(((uint32_t)read_buf[18] << 4) | (uint32_t)read_buf[19] >> 4, 12);
    calib_coeffs_.c40 = GetTwosComplement(((uint32_t)read_buf[19] << 8) | (uint32_t)read_buf[20], 12);
  }

  return true;
}

bool GoertekBaro::ConfigParam() {
  uint8_t config = 0;

  // prepare a configration word for TMP_CFG register

  if (baro_model_ == SPL06_001) {
    tmperature_ext_ = TMP_EXT_MEMS;

    config = (uint8_t)tmperature_ext_;

    // set the TMP_RATE[2:0] -> 6:4
    config |= ((uint8_t)temperature_mr_);
  } else if (baro_model_ == SPA06_003) {
    // set the TMP_RATE[2:0] -> 6:4
    config |= ((uint8_t)temperature_mr_);
  }

  // set the TMP_PRC[3:0] -> 2:0
  config |= ((uint8_t)temperature_osr_);

  bool status = i2c_write_func(static_cast<uint8_t>(GoertekBaroRegister::TMP_CFG_ADDR), config);

  if (!status) {
    return false;
  }

  config = 0;

  config |= pressure_mr_;

  config |= pressure_osr_;

  status = i2c_write_func(static_cast<uint8_t>(GoertekBaroRegister::PRS_CFG_ADDR), config);

  if (!status) {
    return false;
  }

  config = 0;
  i2c_read_func(static_cast<uint8_t>(GoertekBaroRegister::CFG_REG_ADDR), &config);
  // if oversampling rate for temperature is greater than 8 times
  if (temperature_osr_ > OSR_8_TIMES) {
    config |= 0x08;
  } else {
    config &= ~0x08;
  }
  // if oversampling rate for pressure is greater than 8 times
  if (pressure_osr_ > OSR_8_TIMES) {
    config |= 0x04;
  } else {
    config &= ~0x04;
  }

  status = i2c_write_func(static_cast<uint8_t>(GoertekBaroRegister::CFG_REG_ADDR), config);
  if (!status) {
    return false;
  }

  tmperature_osr_scale_coeff_ = GetScalingCoef(temperature_osr_);
  prs_osr_scale_coeff_ = GetScalingCoef(pressure_osr_);

  return true;
}

GoertekBaro::ScalingCofficients GoertekBaro::GetScalingCoef(OverSampleRate osr) {
  ScalingCofficients scaling_coeff;

  switch (osr) {
    case OSR_1_TIME:
      scaling_coeff = OSR_SF_1;
      break;
    case OSR_2_TIMES:
      scaling_coeff = OSR_SF_2;
      break;
    case OSR_4_TIMES:
      scaling_coeff = OSR_SF_4;
      break;
    case OSR_8_TIMES:
      scaling_coeff = OSR_SF_8;
      break;
    case OSR_16_TIMES:
      scaling_coeff = OSR_SF_16;
      break;
    case OSR_32_TIMES:
      scaling_coeff = OSR_SF_32;
      break;
    case OSR_64_TIMES:
      scaling_coeff = OSR_SF_64;
      break;
    case OSR_128_TIMES:
      scaling_coeff = OSR_SF_128;
      break;
    default:
      scaling_coeff = OSR_SF_1;
      break;
  }

  return scaling_coeff;
}

void GoertekBaro::funcRegisterI2c(i2cReaddata read_func, i2cWritedata write_func, i2cReaddataMult read_mult_func) {
  i2c_read_func = read_func;
  i2c_write_func = write_func;
  i2c_read_mult_func = read_mult_func;
}

void GoertekBaro::funRegisterTimer(timeDelayMs delay_ms_func) { delay_msec = delay_ms_func; }

GoertekBaro::~GoertekBaro() {
  // Add any necessary cleanup code here
}