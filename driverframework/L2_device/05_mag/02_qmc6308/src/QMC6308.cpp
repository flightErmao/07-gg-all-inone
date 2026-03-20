#include "QMC6308.hpp"
#include "I2cInterface.h"

#include <rtdef.h>
#include <rtthread.h>
#include <rtdevice.h>

#undef LOG_TAG
#define LOG_TAG "qmc6308"
#ifndef LOG_LVL
#define LOG_LVL LOG_LVL_INFO
#endif

extern "C" {
#include <ulog.h>
}

#ifdef __cplusplus

/* LSB lookup table */
const QMC6308_LSB_Table QMC6308::lsb_table_[4] = {
  {30, QMC6308_LSB_PER_G_30G, QMC6308_LSB_TO_UT_30G},  /* ±30G: 1000 LSB/G, 0.1 uT/LSB */
  {12, QMC6308_LSB_PER_G_12G, QMC6308_LSB_TO_UT_12G},  /* ±12G: 2500 LSB/G, 0.04 uT/LSB */
  {8,  QMC6308_LSB_PER_G_8G,  QMC6308_LSB_TO_UT_8G},   /* ±8G:  3750 LSB/G, 0.02667 uT/LSB */
  {2,  QMC6308_LSB_PER_G_2G,  QMC6308_LSB_TO_UT_2G}    /* ±2G:  15000 LSB/G, 0.00667 uT/LSB */
};

QMC6308::QMC6308(I2cInterface_t* i2c_interface) : i2c_interface_(i2c_interface), chipid_(0), ctl_reg_one_(0), ctl_reg_two_(0), ctl_reg_three_(0) {
  // Initialize config to default values
  config_.range_g = 8;
  config_.odr_hz = 100;
  config_.lsb = QMC6308_LSB_TO_UT_8G;
}

int QMC6308::getChipID() {
  int ret = 0;
  for (int i = 0; i < 10; i++) {
    ret = i2c_read_reg8_mult_pack(*i2c_interface_, QMC6308_CHIP_ID_REG, &chipid_, 1);
    LOG_D("chip id = 0x%02X", chipid_);
    if (ret == 0) {
      break;
    }
  }
  if (ret != 0) {
    return 0;
  }
  if ((chipid_ & 0xf0) == 0) {
    return 0;
  }
  return 1;
}

void QMC6308::softReset() {
  uint8_t data = 0x80;
  i2c_write_reg8_mult_pack(*i2c_interface_, QMC6308_CTL_REG_TWO, &data, 1);
  rt_thread_mdelay(2);
  data = 0x00;
  i2c_write_reg8_mult_pack(*i2c_interface_, QMC6308_CTL_REG_TWO, &data, 1);
  rt_thread_mdelay(5);
}

const QMC6308_LSB_Table* QMC6308::getLSBTable(rt_uint16_t range_g) const {
  for (int i = 0; i < 4; i++) {
    if (lsb_table_[i].range_g == range_g) {
      return &lsb_table_[i];
    }
  }
  /* Default to 8G if not found */
  return &lsb_table_[QMC6308_RANGE_8G];
}

void QMC6308::updateConfig(rt_uint16_t range_g, rt_uint16_t odr_hz, float lsb) {
  config_.range_g = range_g;
  config_.odr_hz = odr_hz;
  config_.lsb = lsb;
}

void QMC6308::updateConfigFromRegisters(uint8_t reg_one_value, uint8_t reg_two_value) {
  rt_uint16_t range_g = 8;   /* default range */
  rt_uint16_t odr_hz = 100;  /* default ODR */
  float lsb = QMC6308_LSB_TO_UT_8G;

  /* Parse range from CTL_REG_TWO [3:2] bits */
  uint8_t range_bits = (reg_two_value >> 2) & 0x03;
  switch (range_bits) {
    case 0:
      range_g = 30;
      break; /* ±30G range */
    case 1:
      range_g = 12;
      break; /* ±12G range */
    case 2:
      range_g = 8;
      break; /* ±8G range */
    case 3:
      range_g = 2;
      break; /* ±2G range */
    default:
      range_g = 8;
      break; /* Default to 8G */
  }

  /* Get LSB conversion values from lookup table */
  const QMC6308_LSB_Table* lsb_table = getLSBTable(range_g);
  lsb = lsb_table->lsb_to_ut;

  /* Parse ODR from CTL_REG_ONE [3:2] bits */
  uint8_t odr_bits = (reg_one_value >> 2) & 0x03;
  switch (odr_bits) {
    case 0:
      odr_hz = 10;
      break;
    case 1:
      odr_hz = 50;
      break;
    case 2:
      odr_hz = 100;
      break;
    case 3:
      odr_hz = 200;
      break;
    default:
      odr_hz = 100;
      break;
  }

  /* Update configuration member variables */
  updateConfig(range_g, odr_hz, lsb);
}

/**
 * @brief Enable and configure QMC6308 magnetometer sensor
 *
 * 此函数配置以下寄存器：
 *
 * 1. QMC6308_CTL_REG_THREE (0x0D) - 控制寄存器三
 *    - 作用：配置软复位控制位 (SRCTRL)
 *    - 配置值：QMC6308_SET_SRCTRL_ON (0x40) - 软复位控制开启
 *
 * 2. QMC6308_CTL_REG_ONE (0x0A) - 控制寄存器一
 *    - 作用：配置工作模式、输出数据速率、过采样率和降采样率
 *    - 配置值：
 *      * MODE[1:0] = QMC6308_CONTINUOUS_MODE (0x03) - 连续测量模式
 *      * ODR[3:2] = QMC6308_SET_OUTPUT_DATA_RATE_100 (0x08) - 输出数据速率 100Hz
 *      * OSR[5:4] = QMC6308_SET_OVERSAMPLE_RATIO_8 (0x00) - 过采样率 8
 *      * DSR[7:6] = QMC6308_SET_DOWNSAMPLE_RATIO_1 (0x00) - 降采样率 1
 *    - 组合值：0xC3 (CONTINUOUS_MODE | ODR_100 | OSR_8 | DSR_1)
 *
 * 3. QMC6308_CTL_REG_TWO (0x0B) - 控制寄存器二
 *    - 作用：配置量程和设置/复位周期
 *    - 配置值：
 *      * RNG[3:2] = QMC6308_SET_RANGE_8G (0x08) - ±8G 量程
 *      * SRP[1:0] = QMC6308_SET_RESET_RESET_ON (0x00) - 复位开启
 *    - 组合值：0x08 (RANGE_8G | RESET_RESET_ON)
 *
 * @return 1 on success, 0 on failure
 */
int QMC6308::config() {
  int ret = 1; /* Default to success */

  /* 配置控制寄存器三 (0x0D) - 软复位控制 */
  uint8_t CONTROL_REG_THREE_VALUE = QMC6308_SET_SRCTRL_ON;
  ctl_reg_three_ = CONTROL_REG_THREE_VALUE;
  // Configure control register three - 软复位控制开启
  if (i2c_write_reg8_mult_pack(*i2c_interface_, QMC6308_CTL_REG_THREE, &CONTROL_REG_THREE_VALUE, 1) == 0) {
    LOG_D("[Mag]QMC6308 write control reg three success");
  } else {
    LOG_E("[Mag]QMC6308 write control reg three failed");
    return 0;
  }

  /* 配置控制寄存器一 (0x0A) - 工作模式、输出数据速率、过采样率、降采样率 */
  uint8_t CONTROL_REG_ONE_VALUE = QMC6308_CONTINUOUS_MODE | QMC6308_SET_OUTPUT_DATA_RATE_100 |
                                  QMC6308_SET_OVERSAMPLE_RATIO_8 | QMC6308_SET_DOWNSAMPLE_RATIO_1;
  ctl_reg_one_ = CONTROL_REG_ONE_VALUE;
  // Configure control register one - 连续模式 | 100Hz输出速率 | 过采样率8 | 降采样率1
  if (i2c_write_reg8_mult_pack(*i2c_interface_, QMC6308_CTL_REG_ONE, &CONTROL_REG_ONE_VALUE, 1) == 0) {
    LOG_D("[Mag]QMC6308 write control reg one success");
  } else {
    LOG_E("[Mag]QMC6308 write control reg one failed");
    return 0;
  }

  /* 配置控制寄存器二 (0x0B) - 量程和设置/复位周期 */
  uint8_t CONTROL_REG_TWO_VALUE = QMC6308_SET_RESET_RESET_ON | QMC6308_SET_RANGE_8G;
  ctl_reg_two_ = CONTROL_REG_TWO_VALUE;
  // Configure control register two - 复位开启 | ±8G量程
  if (i2c_write_reg8_mult_pack(*i2c_interface_, QMC6308_CTL_REG_TWO, &CONTROL_REG_TWO_VALUE, 1) == 0) {
    LOG_D("[Mag]QMC6308 write control reg two success");
  } else {
    LOG_E("[Mag]QMC6308 write control reg two failed");
    return 0;
  }

  /* 从寄存器值更新配置成员变量 */
  updateConfigFromRegisters(CONTROL_REG_ONE_VALUE, CONTROL_REG_TWO_VALUE);

  return ret;
}

int QMC6308::init() {
  if (getChipID() == 0) {
    return 0;
  }
  return config();
}

int QMC6308::readMagXYZ(int16_t* data) {
  int res;
  int t1 = 0;
  uint8_t rdy = 0;
  uint8_t mag_data[6];

  /* Check status register for data availability */
  while (!(rdy & 0x01) && (t1 < 5)) {
    res = i2c_read_reg8_mult_pack(*i2c_interface_, QMC6308_STATUS_REG, &rdy, 1);
    if (res != 0) {
      break;
    }
    if (rdy & 0x01) {
      break;  /* Data ready */
    }
    t1++;
    rt_thread_mdelay(1);  /* Small delay before retry */
  }

  res = i2c_read_reg8_mult_pack(*i2c_interface_, QMC6308_DATA_OUT_X_LSB_REG, mag_data, 6);
  if (res != 0) {
    return 0;
  }

  data[0] = (int16_t)(((mag_data[1]) << 8) | mag_data[0]);
  data[1] = (int16_t)(((mag_data[3]) << 8) | mag_data[2]);
  data[2] = (int16_t)(((mag_data[5]) << 8) | mag_data[4]);

  return 1;
}

#endif /* __cplusplus */

