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

QMC6308::QMC6308(I2cInterface_t* i2c_interface)
    : i2c_interface_(i2c_interface), chipid_(0) {
  // Initialize map to default layout
  map_.sign[AXIS_X] = 1;
  map_.sign[AXIS_Y] = 1;
  map_.sign[AXIS_Z] = 1;
  map_.map[AXIS_X] = AXIS_X;
  map_.map[AXIS_Y] = AXIS_Y;
  map_.map[AXIS_Z] = AXIS_Z;
  
  // Initialize config to default values
  config_.range_g = 8;
  config_.odr_hz = 100;
  config_.lsb = QMC6308_LSB_TO_UT_8G;
}

void QMC6308::setLayout(int layout) {
  switch (layout) {
    case 0:
      map_.sign[AXIS_X] = 1;
      map_.sign[AXIS_Y] = 1;
      map_.sign[AXIS_Z] = 1;
      map_.map[AXIS_X] = AXIS_X;
      map_.map[AXIS_Y] = AXIS_Y;
      map_.map[AXIS_Z] = AXIS_Z;
      break;
    case 1:
      map_.sign[AXIS_X] = -1;
      map_.sign[AXIS_Y] = 1;
      map_.sign[AXIS_Z] = 1;
      map_.map[AXIS_X] = AXIS_Y;
      map_.map[AXIS_Y] = AXIS_X;
      map_.map[AXIS_Z] = AXIS_Z;
      break;
    case 2:
      map_.sign[AXIS_X] = -1;
      map_.sign[AXIS_Y] = -1;
      map_.sign[AXIS_Z] = 1;
      map_.map[AXIS_X] = AXIS_X;
      map_.map[AXIS_Y] = AXIS_Y;
      map_.map[AXIS_Z] = AXIS_Z;
      break;
    case 3:
      map_.sign[AXIS_X] = 1;
      map_.sign[AXIS_Y] = -1;
      map_.sign[AXIS_Z] = 1;
      map_.map[AXIS_X] = AXIS_Y;
      map_.map[AXIS_Y] = AXIS_X;
      map_.map[AXIS_Z] = AXIS_Z;
      break;
    case 4:
      map_.sign[AXIS_X] = -1;
      map_.sign[AXIS_Y] = 1;
      map_.sign[AXIS_Z] = -1;
      map_.map[AXIS_X] = AXIS_X;
      map_.map[AXIS_Y] = AXIS_Y;
      map_.map[AXIS_Z] = AXIS_Z;
      break;
    case 5:
      map_.sign[AXIS_X] = 1;
      map_.sign[AXIS_Y] = 1;
      map_.sign[AXIS_Z] = -1;
      map_.map[AXIS_X] = AXIS_Y;
      map_.map[AXIS_Y] = AXIS_X;
      map_.map[AXIS_Z] = AXIS_Z;
      break;
    case 6:
      map_.sign[AXIS_X] = 1;
      map_.sign[AXIS_Y] = -1;
      map_.sign[AXIS_Z] = -1;
      map_.map[AXIS_X] = AXIS_X;
      map_.map[AXIS_Y] = AXIS_Y;
      map_.map[AXIS_Z] = AXIS_Z;
      break;
    case 7:
      map_.sign[AXIS_X] = -1;
      map_.sign[AXIS_Y] = -1;
      map_.sign[AXIS_Z] = -1;
      map_.map[AXIS_X] = AXIS_Y;
      map_.map[AXIS_Y] = AXIS_X;
      map_.map[AXIS_Z] = AXIS_Z;
      break;
    default:
      map_.sign[AXIS_X] = 1;
      map_.sign[AXIS_Y] = 1;
      map_.sign[AXIS_Z] = 1;
      map_.map[AXIS_X] = AXIS_X;
      map_.map[AXIS_Y] = AXIS_Y;
      map_.map[AXIS_Z] = AXIS_Z;
      break;
  }
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

int QMC6308::enable() {
  int ret = 1;  /* Default to success */
  uint8_t ctl_reg_one = 0;
  uint8_t ctl_reg_two = 0;
  rt_uint16_t range_g = 8;   /* default range */
  rt_uint16_t odr_hz = 100;  /* default ODR */
  float lsb = QMC6308_LSB_TO_UT_8G;

  if (chipid_ == 0x80) {
    uint8_t reg_data = 0x40;
    i2c_write_reg8_mult_pack(*i2c_interface_, 0x0d, &reg_data, 1);
    ctl_reg_two = 0x00;  /* Range: 0x00 could be 30G or 8G, default to 8G */
    ctl_reg_one = 0xc3;  /* Mode: CONTINUOUS | ODR: 100Hz | OSR: 8 | DSR: 1 */
    i2c_write_reg8_mult_pack(*i2c_interface_, QMC6308_CTL_REG_TWO, &ctl_reg_two, 1);
    i2c_write_reg8_mult_pack(*i2c_interface_, QMC6308_CTL_REG_ONE, &ctl_reg_one, 1);
    
    /* Parse range from CTL_REG_TWO [3:2] bits */
    uint8_t range_bits = (ctl_reg_two >> 2) & 0x03;
    switch (range_bits) {
      case 0: range_g = 30; break;  /* ±30G range */
      case 1: range_g = 12; break;  /* ±12G range */
      case 2: range_g = 8;  break;  /* ±8G range */
      case 3: range_g = 2;  break;  /* ±2G range */
      default: range_g = 8; break;  /* Default to 8G */
    }
    
    /* Get LSB conversion values from lookup table */
    const QMC6308_LSB_Table* lsb_table = getLSBTable(range_g);
    lsb = lsb_table->lsb_to_ut;
    
    /* Parse ODR from CTL_REG_ONE */
    uint8_t odr_bits = (ctl_reg_one >> 2) & 0x03;
    switch (odr_bits) {
      case 0: odr_hz = 10; break;
      case 1: odr_hz = 50; break;
      case 2: odr_hz = 100; break;
      case 3: odr_hz = 200; break;
      default: odr_hz = 100; break;
    }
  } else {
    uint8_t reg_data = 0x40;
    i2c_write_reg8_mult_pack(*i2c_interface_, 0x0d, &reg_data, 1);
    ctl_reg_two = 0x08;  /* Range: 8G (QMC6308_SET_RANGE_8G) */
    ctl_reg_one = 0x63;  /* Mode: CONTINUOUS | ODR: 100Hz | OSR: 4 | DSR: 1 */
    i2c_write_reg8_mult_pack(*i2c_interface_, QMC6308_CTL_REG_TWO, &ctl_reg_two, 1);
    i2c_write_reg8_mult_pack(*i2c_interface_, QMC6308_CTL_REG_ONE, &ctl_reg_one, 1);
    
    range_g = 8;
    /* Get LSB conversion values from lookup table for 8G range */
    const QMC6308_LSB_Table* lsb_table = getLSBTable(range_g);
    lsb = lsb_table->lsb_to_ut;
    
    /* Parse ODR from CTL_REG_ONE */
    uint8_t odr_bits = (ctl_reg_one >> 2) & 0x03;
    switch (odr_bits) {
      case 0: odr_hz = 10; break;
      case 1: odr_hz = 50; break;
      case 2: odr_hz = 100; break;
      case 3: odr_hz = 200; break;
      default: odr_hz = 100; break;
    }
  }

  /* Update configuration */
  config_.range_g = range_g;
  config_.odr_hz = odr_hz;
  config_.lsb = lsb;

  return ret;
}

int QMC6308::init() {
  if (getChipID() == 0) {
    return 0;
  }
  setLayout(1);
  return enable();
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

