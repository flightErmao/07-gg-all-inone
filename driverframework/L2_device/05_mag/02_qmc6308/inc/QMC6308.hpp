#ifndef __QMC6308_HPP__
#define __QMC6308_HPP__

#include <stdint.h>
#include <rtdef.h>

/* ============================================================================
 * QMC6308 寄存器定义
 * ============================================================================ */

/* ID 寄存器 */
#define QMC6308_CHIP_ID_REG 0x00

/* 状态寄存器 */
#define QMC6308_STATUS_REG 0x09

/* 配置寄存器 */
#define QMC6308_CTL_REG_ONE 0x0A   /* 控制寄存器一 */
#define QMC6308_CTL_REG_TWO 0x0B   /* 控制寄存器二 */
#define QMC6308_CTL_REG_THREE 0x0D /* 控制寄存器三 */

/* 数据输出寄存器 */
#define QMC6308_DATA_OUT_X_LSB_REG 0x01
#define QMC6308_DATA_OUT_X_MSB_REG 0x02
#define QMC6308_DATA_OUT_Y_LSB_REG 0x03
#define QMC6308_DATA_OUT_Y_MSB_REG 0x04
#define QMC6308_DATA_OUT_Z_LSB_REG 0x05
#define QMC6308_DATA_OUT_Z_MSB_REG 0x06

/* 磁力计工作模式 MODE[1:0] */
#define QMC6308_SUSPEND_MODE 0x00    /* 挂起模式 */
#define QMC6308_NORMAL_MODE 0x01     /* 正常模式 */
#define QMC6308_SINGLE_MODE 0x02     /* 单次测量模式 */
#define QMC6308_CONTINUOUS_MODE 0x03 /* 连续测量模式 */

/* 磁力计输出数据速率 ODR[3:2] */
#define QMC6308_SET_OUTPUT_DATA_RATE_10 0x00  /* 10 Hz */
#define QMC6308_SET_OUTPUT_DATA_RATE_50 0x04  /* 50 Hz */
#define QMC6308_SET_OUTPUT_DATA_RATE_100 0x08 /* 100 Hz */
#define QMC6308_SET_OUTPUT_DATA_RATE_200 0x0C /* 200 Hz */

/* 磁力计过采样率 OSR[5:4] */
#define QMC6308_SET_OVERSAMPLE_RATIO_8 0x00 /* 过采样率 8 */
#define QMC6308_SET_OVERSAMPLE_RATIO_4 0x10 /* 过采样率 4 */
#define QMC6308_SET_OVERSAMPLE_RATIO_2 0x20 /* 过采样率 2 */
#define QMC6308_SET_OVERSAMPLE_RATIO_1 0x30 /* 过采样率 1 */

/* 磁力计降采样率 DSR[7:6] */
#define QMC6308_SET_DOWNSAMPLE_RATIO_1 0x00 /* 降采样率 1 */
#define QMC6308_SET_DOWNSAMPLE_RATIO_2 0x40 /* 降采样率 2 */
#define QMC6308_SET_DOWNSAMPLE_RATIO_4 0x80 /* 降采样率 4 */
#define QMC6308_SET_DOWNSAMPLE_RATIO_8 0xC0 /* 降采样率 8 */

/* 磁力计设置/复位周期 SRP[1:0] */
#define QMC6308_SET_RESET_RESET_ON 0x00 /* 复位开启 */
#define QMC6308_SET_SET_ONLY_ON 0x01    /* 仅设置开启 */
#define QMC6308_SET_SET_RESET_OFF 0x02  /* 设置/复位关闭 */
#define QMC6308_SET_SET_RESET_OFF2 0x03 /* 设置/复位关闭2 */

/* 磁力计量程 RNG[3:2] */
#define QMC6308_SET_RANGE_30G 0x00 /* ±30G 量程 */
#define QMC6308_SET_RANGE_12G 0x04 /* ±12G 量程 */
#define QMC6308_SET_RANGE_8G 0x08  /* ±8G 量程 */
#define QMC6308_SET_RANGE_2G 0x0C  /* ±2G 量程 */

/* QMC6308 LSB 每高斯转换表
 * 根据数据手册：
 *   量程 = ±30G: 1000 LSB/G
 *   量程 = ±12G: 2500 LSB/G
 *   量程 = ±8G:  3750 LSB/G
 *   量程 = ±2G:  15000 LSB/G
 */
typedef enum {
  QMC6308_RANGE_30G = 0, /* ±30G 量程, 1000 LSB/G */
  QMC6308_RANGE_12G = 1, /* ±12G 量程, 2500 LSB/G */
  QMC6308_RANGE_8G = 2,  /* ±8G 量程,  3750 LSB/G */
  QMC6308_RANGE_2G = 3   /* ±2G 量程,  15000 LSB/G */
} qmc6308_range_e;

/* LSB 每高斯查找表 */
#define QMC6308_LSB_PER_G_30G 1000 /* ±30G 量程的 LSB/高斯 */
#define QMC6308_LSB_PER_G_12G 2500 /* ±12G 量程的 LSB/高斯 */
#define QMC6308_LSB_PER_G_8G 3750  /* ±8G 量程的 LSB/高斯 */
#define QMC6308_LSB_PER_G_2G 15000 /* ±2G 量程的 LSB/高斯 */

/* LSB 到 uT 转换 (1 高斯 = 100 uT) */
#define QMC6308_LSB_TO_UT_30G (100.0f / QMC6308_LSB_PER_G_30G) /* 0.1 uT/LSB */
#define QMC6308_LSB_TO_UT_12G (100.0f / QMC6308_LSB_PER_G_12G) /* 0.04 uT/LSB */
#define QMC6308_LSB_TO_UT_8G (100.0f / QMC6308_LSB_PER_G_8G)   /* 0.02667 uT/LSB */
#define QMC6308_LSB_TO_UT_2G (100.0f / QMC6308_LSB_PER_G_2G)   /* 0.00667 uT/LSB */

/* 软复位控制位 */
#define QMC6308_SET_SRCTRL_ON 0x40  /* 软复位控制开启 */
#define QMC6308_SET_SRCTRL_OFF 0x00 /* 软复位控制关闭 */

/* 默认延迟 */
#define QMC6308_DEFAULT_DELAY 200

/* 模式切换 */
#define QMC6308_MODE_SWITCH

/* 自检范围定义 */
#define QMC6308_SELFTEST_MAX_X (1800)
#define QMC6308_SELFTEST_MIN_X (120)
#define QMC6308_SELFTEST_MAX_Y (1800)
#define QMC6308_SELFTEST_MIN_Y (120)
#define QMC6308_SELFTEST_MAX_Z (1800)
#define QMC6308_SELFTEST_MIN_Z (120)

/* 芯片 ID 值 */
#define QMC6308_ID_VALUE 0x80

/* 轴枚举 */
enum { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2, AXIS_TOTAL };

/* QMC6308 配置结构体（从驱动导出） */
typedef struct {
  rt_uint16_t range_g; /* 磁力计量程，单位：高斯 (2, 8, 12, 30) */
  rt_uint16_t odr_hz;  /* 输出数据速率，单位：Hz (10, 50, 100, 200) */
  float lsb;           /* 最小有效位缩放因子 (uT/LSB) */
} qmc6308_config_t;

#ifdef __cplusplus

extern "C" {
#include "I2cInterface.h"
}

/* LSB per Gauss lookup table structure */
struct QMC6308_LSB_Table {
  rt_uint16_t range_g;        /* Range in Gauss */
  rt_uint16_t lsb_per_g;      /* LSB per Gauss */
  float lsb_to_ut;            /* LSB to uT conversion factor (uT/LSB) */
};

/**
 * @brief QMC6308 Magnetometer Driver Class
 * 
 * This class provides a C++ interface for the QMC6308 magnetometer sensor.
 * It handles initialization, configuration, and data reading operations.
 */
class QMC6308 {
public:
  /**
   * @brief Constructor
   * @param i2c_interface I2C interface structure
   */
  QMC6308(I2cInterface_t* i2c_interface);

  /**
   * @brief Destructor
   */
  ~QMC6308() = default;

  /**
   * @brief Initialize the sensor
   * @return 1 on success, 0 on failure
   */
  int init();

  /**
   * @brief Read magnetometer raw data (int16_t format)
   * @param data Output buffer for X, Y, Z axis data
   * @return 1 on success, 0 on failure
   */
  int readMagXYZ(int16_t* data);

  /**
   * @brief Get current configuration
   * @return Configuration structure
   */
  const qmc6308_config_t& getConfig() const { return config_; }
  uint8_t getChipIDValue() const { return chipid_; }
  uint8_t getCtlRegOneValue() const { return ctl_reg_one_; }
  uint8_t getCtlRegTwoValue() const { return ctl_reg_two_; }
  uint8_t getCtlRegThreeValue() const { return ctl_reg_three_; }

 private:
  /**
   * @brief Get chip ID
   * @return 1 on success, 0 on failure
   */
  int getChipID();

  /**
   * @brief Perform soft reset
   */
  void softReset();

  /**
   * @brief Enable and configure the sensor
   * @return 1 on success, 0 on failure
   */
  int config();

  /**
   * @brief Get LSB conversion values from lookup table
   * @param range_g Range in Gauss
   * @return Pointer to LSB table entry
   */
  const QMC6308_LSB_Table* getLSBTable(rt_uint16_t range_g) const;

  /**
   * @brief Update configuration member variables
   * @param range_g Range in Gauss
   * @param odr_hz Output data rate in Hz
   * @param lsb LSB to uT conversion factor
   */
  void updateConfig(rt_uint16_t range_g, rt_uint16_t odr_hz, float lsb);

  /**
   * @brief Update configuration member variables from register values
   * @param reg_one_value Control register one value
   * @param reg_two_value Control register two value
   */
  void updateConfigFromRegisters(uint8_t reg_one_value, uint8_t reg_two_value);

  // I2C interface
  I2cInterface_t* i2c_interface_;

  // Internal state
  uint8_t chipid_;
  uint8_t ctl_reg_one_;
  uint8_t ctl_reg_two_;
  uint8_t ctl_reg_three_;
  qmc6308_config_t config_;

  // LSB lookup table
  static const QMC6308_LSB_Table lsb_table_[4];
};

#endif /* __cplusplus */

#endif /* __QMC6308_HPP__ */

