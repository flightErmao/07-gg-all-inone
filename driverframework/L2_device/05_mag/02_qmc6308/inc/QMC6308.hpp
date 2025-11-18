#ifndef __QMC6308_HPP__
#define __QMC6308_HPP__

#include "qmc6308.h"
#include <stdint.h>
#include <rtdef.h>

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

private:
  /**
   * @brief Set axis layout mapping
   * @param layout Layout number (0-7)
   */
  void setLayout(int layout);

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
  int enable();

  /**
   * @brief Get LSB conversion values from lookup table
   * @param range_g Range in Gauss
   * @return Pointer to LSB table entry
   */
  const QMC6308_LSB_Table* getLSBTable(rt_uint16_t range_g) const;

  // I2C interface
  I2cInterface_t* i2c_interface_;

  // Internal state
  uint8_t chipid_;
  qmc6308_map map_;
  qmc6308_config_t config_;

  // LSB lookup table
  static const QMC6308_LSB_Table lsb_table_[4];
};

#endif /* __cplusplus */

#endif /* __QMC6308_HPP__ */

