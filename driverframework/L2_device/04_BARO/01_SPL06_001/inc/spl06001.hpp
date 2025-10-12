/*!
 * @file spl06_001.hpp
 * @brief SPL06 barometric pressure sensor C++ wrapper class header
 *
 * @detail
 * This file implements the C++ wrapper class for SPL06 barometric pressure sensor,
 * providing object-oriented interface for accessing SPL06 sensor functions including
 * pressure and temperature measurement, sensor configuration, etc.
 * This class is ported from the original C language driver while maintaining
 * the same functionality characteristics.
 *
 * @version 1.0.0
 * @date 2024
 */

#pragma once

#include <cstdint>
#include "spl06001Regs.h"

/*!
 * @brief SPL06 barometric pressure sensor C++ wrapper class
 *
 * This class provides complete functional interface for SPL06 barometric pressure sensor, including:
 * - Sensor initialization and configuration
 * - Pressure and temperature data reading
 * - Sensor parameter settings (oversampling rate, sampling rate, etc.)
 * - Calibration parameter reading and compensation calculation
 *
 * Uses singleton pattern to ensure only one sensor instance in the system.
 */
class spl06_001 {
 public:
  /*! @brief Delay function type definition */
  using delay_ms_func_t = void (*)(unsigned int);

  /*! @brief I2C read/write function type definition */
  using i2c_rw_func_t = int8_t (*)(uint8_t, uint8_t, uint8_t*, uint8_t);

  /*! @brief Bus operations structure containing I2C communication and delay function pointers */
  struct BusOps {
    i2c_rw_func_t bus_write;  /**< I2C write function pointer */
    i2c_rw_func_t bus_read;   /**< I2C read function pointer */
    delay_ms_func_t delay_ms; /**< Delay function pointer */
    uint8_t dev_addr;         /**< Device I2C address */
  };

  /*!
   * @brief Get singleton instance
   * @return Reference to spl06_001 class singleton
   */
  static spl06_001& instance();

  /*!
   * @brief Set bus operation functions
   * @param ops Structure containing I2C read/write and delay functions
   * @return 0 on success, -1 on failure
   */
  int8_t setBus(const BusOps& ops);

  /*!
   * @brief Initialize the sensor
   * @param osr_press Pressure oversampling rate (0-7)
   * @param mr_press Pressure measurement rate (0-7)
   * @param osr_temp Temperature oversampling rate (0-7)
   * @param mr_temp Temperature measurement rate (0-7)
   * @return 0 on success, -1 on failure
   */
  int init(uint8_t osr_press, uint8_t mr_press, uint8_t osr_temp, uint8_t mr_temp);

  /*!
   * @brief Probe if sensor exists
   * @return 0 if sensor exists, -1 if sensor not found
   */
  int probe();

  /*!
   * @brief Read pressure and temperature data (integer format)
   * @param pressure_pa Pressure value output pointer, unit: Pa
   * @param temperature_centi Temperature value output pointer, unit: 0.01 degree Centigrade
   * @return 0 on success, -1 on failure
   */
  int getPressureTemperature(int32_t* pressure_pa, int32_t* temperature_centi);

  /*!
   * @brief Read pressure and temperature data (floating point format)
   * @param pressure_pa Pressure value output pointer, unit: Pa
   * @param temperature_deg Temperature value output pointer, unit: degree Centigrade
   * @return 0 on success, -1 on failure
   */
  int getPressureTemperatureDouble(double* pressure_pa, double* temperature_deg);

  /*!
   * @brief Compute measurement time
   * @param meastime Measurement time output pointer, unit: 0.1ms
   * @return 0 on success, -1 on failure
   */
  int computeMeasTime(uint32_t* meastime);

 private:
  /*! @brief Private constructor for singleton pattern */
  spl06_001();

  /*! @brief Private destructor for singleton pattern */
  ~spl06_001();

  /*! @brief Calibration parameters structure for SPL06 sensor */
  struct CalibParam {
    int16_t c0;  /**< Temperature coefficient c0 */
    int16_t c1;  /**< Temperature coefficient c1 */
    int32_t c00; /**< Pressure coefficient c00 */
    int32_t c10; /**< Pressure coefficient c10 */
    int16_t c01; /**< Pressure coefficient c01 */
    int16_t c11; /**< Pressure coefficient c11 */
    int16_t c20; /**< Pressure coefficient c20 */
    int16_t c21; /**< Pressure coefficient c21 */
    int16_t c30; /**< Pressure coefficient c30 */
  } calib_{};

  uint32_t kP_ = 0; /**< Pressure scale factor variable */
  uint32_t kT_ = 0; /**< Temperature scale factor variable */

  uint8_t chip_id_ = 0;                   /**< Chip ID of the sensor */
  uint8_t dev_addr_ = SPL06_I2C_ADDRESS1; /**< Device I2C address */

  uint8_t oversamp_pressure_ = 0;      /**< Pressure oversampling variable */
  uint8_t oversamp_temperature_ = 0;   /**< Temperature oversampling variable */
  uint8_t samplerate_pressure_ = 0;    /**< Pressure sampling rate */
  uint8_t samplerate_temperature_ = 0; /**< Temperature sampling rate */

  delay_ms_func_t delay_ms_ = nullptr;    /**< Delay function pointer */
  i2c_rw_func_t i2c_bus_write_ = nullptr; /**< I2C write function pointer */
  i2c_rw_func_t i2c_bus_read_ = nullptr;  /**< I2C read function pointer */

  /*!
   * @brief Write data to the given register
   * @param addr Register address
   * @param data Data to write
   * @param len Number of bytes to write
   * @return 0 on success, -1 on failure
   */
  int8_t writeRegister(uint8_t addr, uint8_t* data, uint8_t len);

  /*!
   * @brief Read data from the given register
   * @param addr Register address
   * @param data Data buffer to read into
   * @param len Number of bytes to read
   * @return 0 on success, -1 on failure
   */
  int8_t readRegister(uint8_t addr, uint8_t* data, uint8_t len);

  /*!
   * @brief Get calibration parameters from sensor registers
   * @return 0 on success, -1 on failure
   */
  int8_t getCalibParam();

  /*!
   * @brief Configure sensor oversampling and sampling rates
   * @param osr_press Pressure oversampling rate
   * @param mr_press Pressure measurement rate
   * @param osr_temp Temperature oversampling rate
   * @param mr_temp Temperature measurement rate
   */
  void configure(uint8_t osr_press, uint8_t mr_press, uint8_t osr_temp, uint8_t mr_temp);

  /*!
   * @brief Set temperature oversampling setting
   * @param value Oversampling value (0-7)
   * @return 0 on success, -1 on failure
   */
  int8_t setOversampTemperature(uint8_t value);

  /*!
   * @brief Set pressure oversampling setting
   * @param value Oversampling value (0-7)
   * @return 0 on success, -1 on failure
   */
  int8_t setOversampPressure(uint8_t value);

  /*!
   * @brief Set temperature sampling rate
   * @param value Sampling rate value (0-7)
   * @return 0 on success, -1 on failure
   */
  int8_t setSamplerateTemperature(uint8_t value);

  /*!
   * @brief Set pressure sampling rate
   * @param value Sampling rate value (0-7)
   * @return 0 on success, -1 on failure
   */
  int8_t setSampleratePressure(uint8_t value);

  /*!
   * @brief Set power mode of the sensor
   * @param power_mode Power mode value
   * @return 0 on success, -1 on failure
   */
  int8_t setPowerMode(uint8_t power_mode);

  /*!
   * @brief Perform soft reset of the sensor
   * @return 0 on success, -1 on failure
   */
  int8_t setSoftRst();

  /*!
   * @brief Set working mode of the sensor
   * @param work_mode Work mode value (0-2)
   * @return 0 on success, -1 on failure
   */
  int8_t setWorkMode(uint8_t work_mode);

  /*!
   * @brief Get temperature oversampling setting
   * @param value Output pointer for oversampling value
   * @return 0 on success, -1 on failure
   */
  int8_t getOversampTemperature(uint8_t* value);

  /*!
   * @brief Get pressure oversampling setting
   * @param value Output pointer for oversampling value
   * @return 0 on success, -1 on failure
   */
  int8_t getOversampPressure(uint8_t* value);

  /*!
   * @brief Get temperature sampling rate
   * @param value Output pointer for sampling rate value
   * @return 0 on success, -1 on failure
   */
  int8_t getSamplerateTemperature(uint8_t* value);

  /*!
   * @brief Get pressure sampling rate
   * @param value Output pointer for sampling rate value
   * @return 0 on success, -1 on failure
   */
  int8_t getSampleratePressure(uint8_t* value);

  /*!
   * @brief Read uncompensated temperature from registers
   * @param temperature Output pointer for uncompensated temperature
   * @return 0 on success, -1 on failure
   */
  int8_t readRawTemperature(int32_t* temperature);

  /*!
   * @brief Compensate temperature to actual temperature (integer format)
   * @note Returns value in 0.01 degree Centigrade
   * @return Actual temperature as int32
   */
  int32_t compensateTemperatureInt32();

  /*!
   * @brief Compensate temperature to actual temperature (floating point format)
   * @note Returns value in degree Centigrade
   * @return Actual temperature as double
   */
  double compensateTemperatureDouble();

  /*!
   * @brief Read uncompensated pressure from registers
   * @param pressure Output pointer for uncompensated pressure
   * @return 0 on success, -1 on failure
   */
  int8_t readRawPressure(int32_t* pressure);

  /*!
   * @brief Read both uncompensated pressure and temperature from registers
   * @note Optimized function that reads all 6 data registers in a single burst read
   * @param pressure Output pointer for uncompensated pressure
   * @param temperature Output pointer for uncompensated temperature
   * @return 0 on success, -1 on failure
   */
  int8_t readRawPressureTemperature(int32_t* pressure, int32_t* temperature);

  /*!
   * @brief Compensate pressure to actual pressure (integer format)
   * @note Returns value in Pascal (Pa)
   * @return Actual pressure as int32
   */
  int32_t compensatePressureInt32();

  /*!
   * @brief Compensate pressure to actual pressure (floating point format)
   * @note Returns value in Pascal (Pa)
   * @return Actual pressure as double
   */
  double compensatePressureDouble();

  bool is_init_ = false; /**< Initialization status flag */
};
