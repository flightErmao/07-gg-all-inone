/*!
 * @file spl06_001.cpp
 * @brief SPL06 barometric pressure sensor C++ wrapper class implementation
 *
 * @detail
 * This file implements the C++ wrapper class for SPL06 barometric pressure sensor,
 * providing object-oriented interface for accessing SPL06 sensor functions.
 * This implementation is ported from the original C language driver (spl06.c)
 * while maintaining the same functionality characteristics.
 *
 * @version 1.0.0
 * @date 2024
 */

#include "spl06001.hpp"
#include <cstring>

/*! @brief Measurement time constants [unit:0.1ms] */
static const uint32_t meastime_list[] = {36, 52, 84, 148, 276, 532, 1044, 2068};

/*! @brief Sample rate constants [unit:Hz] */
static const uint32_t samplerate_list[] = {1, 2, 4, 8, 16, 32, 64, 128};

/*!
 * @brief Private constructor for singleton pattern
 * Initializes member variables to default values
 */
spl06_001::spl06_001() {}

/*!
 * @brief Private destructor for singleton pattern
 */
spl06_001::~spl06_001() {}

/*!
 * @brief Get singleton instance
 * @return Reference to spl06_001 class singleton
 */
spl06_001& spl06_001::instance() {
  static spl06_001 s_instance;
  return s_instance;
}

/*!
 * @brief Set bus operation functions
 * @param ops Structure containing I2C read/write and delay functions
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::setBus(const BusOps& ops) {
  if (ops.bus_write == nullptr || ops.bus_read == nullptr || ops.delay_ms == nullptr) {
    return -1;
  }
  i2c_bus_write_ = ops.bus_write;
  i2c_bus_read_ = ops.bus_read;
  delay_ms_ = ops.delay_ms;
  dev_addr_ = ops.dev_addr;
  return 0;
}

/*!
 * @brief Initialize the sensor
 * @param osr_press Pressure oversampling rate (0-7)
 * @param mr_press Pressure measurement rate (0-7)
 * @param osr_temp Temperature oversampling rate (0-7)
 * @param mr_temp Temperature measurement rate (0-7)
 * @return 0 on success, -1 on failure
 */
int spl06_001::init(uint8_t osr_press, uint8_t mr_press, uint8_t osr_temp, uint8_t mr_temp) {
  if (is_init_) return 0;
  if (getCalibParam() != 0) return -1;
  configure(osr_press, mr_press, osr_temp, mr_temp);
  if (setPowerMode(SPL06_NORMAL_MODE) != 0) return -1;
  is_init_ = true;
  return 0;
}

/*!
 * @brief Read pressure and temperature data (integer format)
 * @note Optimized to read both pressure and temperature data in a single I2C transaction
 * @param pressure_pa Pressure value output pointer, unit: Pa
 * @param temperature_centi Temperature value output pointer, unit: 0.01 degree Centigrade
 * @return 0 on success, -1 on failure
 */
int spl06_001::getPressureTemperature(int32_t* pressure_pa, int32_t* temperature_centi) {
  if (!is_init_) return -1;
  if (pressure_pa == nullptr || temperature_centi == nullptr) return -1;

  int32_t ut = 0, up = 0;
  // Optimized: Read both raw pressure and temperature in a single burst read
  if (readRawPressureTemperature(&up, &ut) != 0) return -1;

  // Compensate temperature
  int64_t left = ((int64_t)ut << 16);
  int32_t fTsc = (int32_t)(left / (int64_t)kT_);
  *temperature_centi = ((int32_t)calib_.c0 << 7) + ((int32_t)((int32_t)calib_.c1 * fTsc) >> 8);
  *temperature_centi = (*temperature_centi * 100) / 256;

  // Compensate pressure
  left = ((int64_t)up << 16);
  int32_t fPsc = (int32_t)(left / (int64_t)kP_);
  int32_t qua2 = ((int32_t)calib_.c20 << 8) + ((int32_t)((int32_t)calib_.c30 * fPsc) >> 8);
  int32_t qua3 = ((int32_t)calib_.c10 << 8) + (int32_t)(((int64_t)qua2 * (int64_t)fPsc) >> 16);
  *pressure_pa = ((int32_t)calib_.c00 << 8) + (int32_t)(((int64_t)qua3 * (int64_t)fPsc) >> 16);
  *pressure_pa += ((int32_t)((int32_t)calib_.c01 * fTsc) >> 8);
  qua2 = ((int32_t)calib_.c11 << 8) + ((int32_t)((int32_t)calib_.c21 * fPsc) >> 8);
  qua3 = (int32_t)(((int64_t)qua2 * (int64_t)fPsc) >> 16);
  *pressure_pa += (int32_t)(((int64_t)qua3 * (int64_t)fTsc) >> 16);
  *pressure_pa = (*pressure_pa >> 8);

  return 0;
}

/*!
 * @brief Read pressure and temperature data (floating point format)
 * @note Optimized to read both pressure and temperature data in a single I2C transaction
 * @param pressure_pa Pressure value output pointer, unit: Pa
 * @param temperature_deg Temperature value output pointer, unit: degree Centigrade
 * @return 0 on success, -1 on failure
 */
int spl06_001::getPressureTemperatureDouble(double* pressure_pa, double* temperature_deg) {
  if (!is_init_) return -1;
  if (pressure_pa == nullptr || temperature_deg == nullptr) return -1;

  int32_t ut = 0, up = 0;
  // Optimized: Read both raw pressure and temperature in a single burst read
  if (readRawPressureTemperature(&up, &ut) != 0) return -1;

  // Compensate temperature (floating point)
  double fTsc = (double)ut / (double)kT_;
  *temperature_deg = calib_.c0 * 0.5 + calib_.c1 * fTsc;

  // Compensate pressure (floating point)
  double fPsc = (double)up / (double)kP_;
  double qua2 = calib_.c10 + fPsc * (calib_.c20 + fPsc * calib_.c30);
  double qua3 = fTsc * fPsc * (calib_.c11 + fPsc * calib_.c21);
  *pressure_pa = calib_.c00 + fPsc * qua2 + fTsc * calib_.c01 + qua3;

  return 0;
}

/*!
 * @brief Compute measurement time
 * @param meastime Measurement time output pointer, unit: 0.1ms
 * @return 0 on success, -1 on failure
 */
int spl06_001::computeMeasTime(uint32_t* meastime) {
  if (meastime == nullptr) return -1;
  *meastime = (samplerate_list[samplerate_temperature_] * meastime_list[oversamp_temperature_] +
               samplerate_list[samplerate_pressure_] * meastime_list[oversamp_pressure_]) /
              10;
  return 0;
}

/*!
 * @brief Probe if sensor exists
 * @return 0 if sensor exists, -1 if sensor not found
 */
int spl06_001::probe() {
  uint8_t id = 0;
  const int max_retry = 5;
  for (int i = 0; i < max_retry; ++i) {
    if (readRegister(SPL06_REG_CHIP_ID, &id, 1) == 0 && id == SPL06_CHIP_ID_EXPECTED) {
      return 0;
    }
    if (delay_ms_) {
      delay_ms_(10);
    }
  }
  return -1;
}

/*!
 * @brief Configure sensor oversampling and sampling rates
 * @param osr_press Pressure oversampling rate
 * @param mr_press Pressure measurement rate
 * @param osr_temp Temperature oversampling rate
 * @param mr_temp Temperature measurement rate
 */
void spl06_001::configure(uint8_t osr_press, uint8_t mr_press, uint8_t osr_temp, uint8_t mr_temp) {
  setOversampPressure(osr_press);
  setSampleratePressure(mr_press);
  setOversampTemperature(osr_temp);
  setSamplerateTemperature(mr_temp);
}

/*!
 * @brief Write data to the given register
 * @param addr Register address
 * @param data Data to write
 * @param len Number of bytes to write
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::writeRegister(uint8_t addr, uint8_t* data, uint8_t len) {
  if (!i2c_bus_write_) return -1;
  return i2c_bus_write_(dev_addr_, addr, data, len);
}

/*!
 * @brief Read data from the given register
 * @param addr Register address
 * @param data Data buffer to read into
 * @param len Number of bytes to read
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::readRegister(uint8_t addr, uint8_t* data, uint8_t len) {
  if (!i2c_bus_read_) return -1;
  return i2c_bus_read_(dev_addr_, addr, data, len);
}

/*!
 * @brief Read uncompensated temperature from registers
 * @note Reads from registers 0x03, 0x04 and 0x05 in a single burst read
 * @note 0x03 -> MSB -> bit from 0 to 7
 * @note 0x04 -> LSB -> bit from 0 to 7
 * @note 0x05 -> XLSB -> bit from 0 to 7
 * @param temperature Output pointer for uncompensated temperature
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::readRawTemperature(int32_t* temperature) {
  uint8_t buf[3] = {0};
  // Optimized: Read all 3 temperature registers in a single burst read
  if (readRegister(SPL06_REG_TMP_MSB, buf, 3) != 0) return -1;
  int32_t ut = (int32_t)buf[0] << 16 | (int32_t)buf[1] << 8 | (int32_t)buf[2];
  *temperature = (ut & 0x800000) ? (0xFF000000 | ut) : ut;
  return 0;
}

/*!
 * @brief Compensate temperature to actual temperature (integer format)
 * @note Returns value in 0.01 degree Centigrade
 * @note Output value of "5123" equals 51.23 DegC
 * @return Actual temperature as int32
 */
int32_t spl06_001::compensateTemperatureInt32() {
  int32_t ut = 0;
  if (readRawTemperature(&ut) != 0) return 0;
  int64_t left = ((int64_t)ut << 16);
  int32_t fTsc = (int32_t)(left / (int64_t)kT_);
  int32_t temperature = ((int32_t)calib_.c0 << 7) + ((int32_t)((int32_t)calib_.c1 * fTsc) >> 8);
  temperature = (temperature * 100) / 256;
  return temperature;
}

/*!
 * @brief Compensate temperature to actual temperature (floating point format)
 * @note Returns value in degree Centigrade
 * @note Output value of "51.23" equals 51.23 DegC
 * @return Actual temperature as double
 */
double spl06_001::compensateTemperatureDouble() {
  int32_t ut = 0;
  if (readRawTemperature(&ut) != 0) return 0.0;
  double fTsc = (double)ut / (double)kT_;
  double temperature = calib_.c0 * 0.5 + calib_.c1 * fTsc;
  return temperature;
}

/*!
 * @brief Read uncompensated pressure from registers
 * @note Reads from registers 0x00, 0x01 and 0x02 in a single burst read
 * @note 0x00 -> MSB -> bit from 0 to 7
 * @note 0x01 -> LSB -> bit from 0 to 7
 * @note 0x02 -> XLSB -> bit from 0 to 7
 * @param pressure Output pointer for uncompensated pressure
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::readRawPressure(int32_t* pressure) {
  uint8_t buf[3] = {0};
  // Optimized: Read all 3 pressure registers in a single burst read
  if (readRegister(SPL06_REG_PRS_MSB, buf, 3) != 0) return -1;
  int32_t up = (int32_t)buf[0] << 16 | (int32_t)buf[1] << 8 | (int32_t)buf[2];
  *pressure = (up & 0x800000) ? (0xFF000000 | up) : up;
  return 0;
}

/*!
 * @brief Read both uncompensated pressure and temperature from registers
 * @note Optimized function that reads all 6 data registers (0x00-0x05) in a single burst read
 * @note This reduces I2C bus overhead when both pressure and temperature are needed
 * @param pressure Output pointer for uncompensated pressure
 * @param temperature Output pointer for uncompensated temperature
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::readRawPressureTemperature(int32_t* pressure, int32_t* temperature) {
  uint8_t buf[6] = {0};
  // Optimized: Read all 6 data registers (pressure + temperature) in a single burst read
  if (readRegister(SPL06_REG_PRS_MSB, buf, 6) != 0) return -1;

  // Parse pressure data (registers 0x00-0x02)
  int32_t up = (int32_t)buf[0] << 16 | (int32_t)buf[1] << 8 | (int32_t)buf[2];
  *pressure = (up & 0x800000) ? (0xFF000000 | up) : up;

  // Parse temperature data (registers 0x03-0x05)
  int32_t ut = (int32_t)buf[3] << 16 | (int32_t)buf[4] << 8 | (int32_t)buf[5];
  *temperature = (ut & 0x800000) ? (0xFF000000 | ut) : ut;

  return 0;
}

/*!
 * @brief Compensate pressure to actual pressure (integer format)
 * @note Returns value in Pascal (Pa)
 * @note Output value of "96386" equals 96386 Pa = 963.86 hPa = 963.86 millibar
 * @return Actual pressure as int32
 */
int32_t spl06_001::compensatePressureInt32() {
  int32_t ut = 0, up = 0;
  if (readRawTemperature(&ut) != 0) return 0;
  if (readRawPressure(&up) != 0) return 0;
  int64_t left = ((int64_t)ut << 16);
  int32_t fTsc = (int32_t)(left / (int64_t)kT_);
  left = ((int64_t)up << 16);
  int32_t fPsc = (int32_t)(left / (int64_t)kP_);
  int32_t qua2 = ((int32_t)calib_.c20 << 8) + ((int32_t)((int32_t)calib_.c30 * fPsc) >> 8);
  int32_t qua3 = ((int32_t)calib_.c10 << 8) + (int32_t)(((int64_t)qua2 * (int64_t)fPsc) >> 16);
  int32_t pressure = ((int32_t)calib_.c00 << 8) + (int32_t)(((int64_t)qua3 * (int64_t)fPsc) >> 16);
  pressure += ((int32_t)((int32_t)calib_.c01 * fTsc) >> 8);
  qua2 = ((int32_t)calib_.c11 << 8) + ((int32_t)((int32_t)calib_.c21 * fPsc) >> 8);
  qua3 = (int32_t)(((int64_t)qua2 * (int64_t)fPsc) >> 16);
  pressure += (int32_t)(((int64_t)qua3 * (int64_t)fTsc) >> 16);
  pressure = (pressure >> 8);
  return pressure;
}

/*!
 * @brief Compensate pressure to actual pressure (floating point format)
 * @note Returns value in Pascal (Pa)
 * @note Output value of "96386.2" equals 96386.2 Pa = 963.862 hPa
 * @return Actual pressure as double
 */
double spl06_001::compensatePressureDouble() {
  int32_t ut = 0, up = 0;
  if (readRawTemperature(&ut) != 0) return 0.0;
  if (readRawPressure(&up) != 0) return 0.0;
  double fTsc = (double)ut / (double)kT_;
  double fPsc = (double)up / (double)kP_;
  double qua2 = calib_.c10 + fPsc * (calib_.c20 + fPsc * calib_.c30);
  double qua3 = fTsc * fPsc * (calib_.c11 + fPsc * calib_.c21);
  double pressure = calib_.c00 + fPsc * qua2 + fTsc * calib_.c01 + qua3;
  return pressure;
}

/*!
 * @brief Get calibration parameters from sensor registers
 * @note Reads all calibration parameters in a single burst read from registers 0x10-0x21
 * @note Optimized to reduce I2C bus overhead by reading all 18 bytes at once
 *
 *  parameter | Register address
 *------------|-------------------------
 *	c0    |  0x10 and 0x11
 *	c1    |  0x11 and 0x12
 *	c00    |  0x13 and 0x14 and 0x15
 *	c10    |  0x16 and 0x17
 *	c01    |  0x18 and 0x19
 *	c11    |  0x1A and 0x1B
 *	c20    |  0x1C and 0x1D
 *	c21    |  0x1E and 0x1F
 *	c30    |  0x20 and 0x21
 *
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::getCalibParam() {
  uint8_t calib_data[18] = {0};

  // Optimized: Read all calibration parameters in a single burst read
  if (readRegister(SPL06_REG_CALIB_START, calib_data, 18) != 0) return -1;

  // Parse c0 (registers 0x10-0x11)
  calib_.c0 = (int16_t)calib_data[0] << 4 | calib_data[1] >> 4;
  if (calib_.c0 & 0x0800) calib_.c0 = (int16_t)(0xF000 | calib_.c0);

  // Parse c1 (registers 0x11-0x12)
  calib_.c1 = (int16_t)(calib_data[1] & 0x0F) << 8 | calib_data[2];
  if (calib_.c1 & 0x0800) calib_.c1 = (int16_t)(0xF000 | calib_.c1);

  // Parse c00 (registers 0x13-0x15)
  calib_.c00 = (int32_t)calib_data[3] << 12 | (int32_t)calib_data[4] << 4 | (int32_t)calib_data[5] >> 4;
  if (calib_.c00 & 0x080000) calib_.c00 = (int32_t)(0xFFF00000 | calib_.c00);

  // Parse c10 (registers 0x15-0x17)
  calib_.c10 = (int32_t)(calib_data[5] & 0x0F) << 16 | (int32_t)calib_data[6] << 8 | calib_data[7];
  if (calib_.c10 & 0x080000) calib_.c10 = (int32_t)(0xFFF00000 | calib_.c10);

  // Parse c01 (registers 0x18-0x19)
  calib_.c01 = (int16_t)calib_data[8] << 8 | calib_data[9];

  // Parse c11 (registers 0x1A-0x1B)
  calib_.c11 = (int16_t)calib_data[10] << 8 | calib_data[11];

  // Parse c20 (registers 0x1C-0x1D)
  calib_.c20 = (int16_t)calib_data[12] << 8 | calib_data[13];

  // Parse c21 (registers 0x1E-0x1F)
  calib_.c21 = (int16_t)calib_data[14] << 8 | calib_data[15];

  // Parse c30 (registers 0x20-0x21)
  calib_.c30 = (int16_t)calib_data[16] << 8 | calib_data[17];

  return 0;
}

/*!
 * @brief Set temperature oversampling setting
 * @note Sets the temperature oversampling setting in register 0x07 bits from 0 to 2
 * @param value Oversampling value (0-7)
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::setOversampTemperature(uint8_t value) {
  uint8_t v = 0;
  if (readRegister(SPL06_REG_TMP_CFG, &v, 1) != 0) return -1;
  v = SPL06_SET_BITS(v, SPL06_TMP_CFG_OVERSAMP_MSK, SPL06_TMP_CFG_OVERSAMP_POS, value);
  v |= SPL06_TMP_SOURCE_EXT;
  if (writeRegister(SPL06_REG_TMP_CFG, &v, 1) != 0) return -1;
  oversamp_temperature_ = value;
  static const uint32_t scalefactor_list[] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};
  kT_ = scalefactor_list[value & 0x07];
  if (value > SPL06_OVERSAMP_8X) {
    if (readRegister(SPL06_REG_CONFIG, &v, 1) != 0) return -1;
    v |= SPL06_TEMPERATURE_SHIFT;
    if (writeRegister(SPL06_REG_CONFIG, &v, 1) != 0) return -1;
  } else {
    if (readRegister(SPL06_REG_CONFIG, &v, 1) != 0) return -1;
    v &= (~SPL06_TEMPERATURE_SHIFT);
    if (writeRegister(SPL06_REG_CONFIG, &v, 1) != 0) return -1;
  }
  return 0;
}

/*!
 * @brief Set pressure oversampling setting
 * @note Sets the pressure oversampling setting in register 0x06 bits from 0 to 3
 * @param value Oversampling value (0-7)
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::setOversampPressure(uint8_t value) {
  uint8_t v = 0;
  if (readRegister(SPL06_REG_PRS_CFG, &v, 1) != 0) return -1;
  v = SPL06_SET_BITS(v, SPL06_PRS_CFG_OVERSAMP_MSK, SPL06_PRS_CFG_OVERSAMP_POS, value);
  if (writeRegister(SPL06_REG_PRS_CFG, &v, 1) != 0) return -1;
  oversamp_pressure_ = value;
  static const uint32_t scalefactor_list[] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};
  kP_ = scalefactor_list[value & 0x07];
  if (value > SPL06_OVERSAMP_8X) {
    if (readRegister(SPL06_REG_CONFIG, &v, 1) != 0) return -1;
    v |= SPL06_PRESSURE_SHIFT;
    if (writeRegister(SPL06_REG_CONFIG, &v, 1) != 0) return -1;
  } else {
    if (readRegister(SPL06_REG_CONFIG, &v, 1) != 0) return -1;
    v &= (~SPL06_PRESSURE_SHIFT);
    if (writeRegister(SPL06_REG_CONFIG, &v, 1) != 0) return -1;
  }
  return 0;
}

/*!
 * @brief Get temperature oversampling setting
 * @note Gets the temperature oversampling setting from register 0x07 bits from 0 to 2
 * @param value Output pointer for oversampling value
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::getOversampTemperature(uint8_t* value) {
  uint8_t v = 0;
  if (readRegister(SPL06_REG_TMP_CFG, &v, 1) != 0) return -1;
  *value = SPL06_GET_BITS(v, SPL06_TMP_CFG_OVERSAMP_MSK, SPL06_TMP_CFG_OVERSAMP_POS);
  oversamp_temperature_ = *value;
  return 0;
}

/*!
 * @brief Get pressure oversampling setting
 * @note Gets the pressure oversampling setting from register 0x06 bits from 0 to 3
 * @param value Output pointer for oversampling value
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::getOversampPressure(uint8_t* value) {
  uint8_t v = 0;
  if (readRegister(SPL06_REG_PRS_CFG, &v, 1) != 0) return -1;
  *value = SPL06_GET_BITS(v, SPL06_PRS_CFG_OVERSAMP_MSK, SPL06_PRS_CFG_OVERSAMP_POS);
  oversamp_pressure_ = *value;
  return 0;
}

/*!
 * @brief Set temperature sampling rate
 * @note Sets the temperature sampling rate in register 0x07 bits from 4 to 6
 * @param value Sampling rate value (0-7)
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::setSamplerateTemperature(uint8_t value) {
  uint8_t v = 0;
  if (readRegister(SPL06_REG_TMP_CFG, &v, 1) != 0) return -1;
  v = SPL06_SET_BITS(v, SPL06_TMP_CFG_SAMPLERATE_MSK, SPL06_TMP_CFG_SAMPLERATE_POS, value);
  if (writeRegister(SPL06_REG_TMP_CFG, &v, 1) != 0) return -1;
  samplerate_temperature_ = value;
  return 0;
}

/*!
 * @brief Set pressure sampling rate
 * @note Sets the pressure sampling rate in register 0x06 bits from 4 to 6
 * @param value Sampling rate value (0-7)
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::setSampleratePressure(uint8_t value) {
  uint8_t v = 0;
  if (readRegister(SPL06_REG_PRS_CFG, &v, 1) != 0) return -1;
  v = SPL06_SET_BITS(v, SPL06_PRS_CFG_SAMPLERATE_MSK, SPL06_PRS_CFG_SAMPLERATE_POS, value);
  if (writeRegister(SPL06_REG_PRS_CFG, &v, 1) != 0) return -1;
  samplerate_pressure_ = value;
  return 0;
}

/*!
 * @brief Get temperature sampling rate
 * @note Gets the temperature sampling rate from register 0x07 bits from 4 to 6
 * @param value Output pointer for sampling rate value
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::getSamplerateTemperature(uint8_t* value) {
  uint8_t v = 0;
  if (readRegister(SPL06_REG_TMP_CFG, &v, 1) != 0) return -1;
  *value = SPL06_GET_BITS(v, SPL06_TMP_CFG_SAMPLERATE_MSK, SPL06_TMP_CFG_SAMPLERATE_POS);
  samplerate_temperature_ = *value;
  return 0;
}

/*!
 * @brief Get pressure sampling rate
 * @note Gets the pressure sampling rate from register 0x06 bits from 4 to 6
 * @param value Output pointer for sampling rate value
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::getSampleratePressure(uint8_t* value) {
  uint8_t v = 0;
  if (readRegister(SPL06_REG_PRS_CFG, &v, 1) != 0) return -1;
  *value = SPL06_GET_BITS(v, SPL06_PRS_CFG_SAMPLERATE_MSK, SPL06_PRS_CFG_SAMPLERATE_POS);
  samplerate_pressure_ = *value;
  return 0;
}

/*!
 * @brief Set power mode of the sensor
 * @note Sets the operational mode in register 0x08 bit 0 to 2
 * @param power_mode Power mode value (0x00 = SLEEP_MODE, 0x07 = NORMAL_MODE)
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::setPowerMode(uint8_t power_mode) {
  if (power_mode > SPL06_NORMAL_MODE) return -1;
  return writeRegister(SPL06_REG_CTRL_MEAS, &power_mode, 1);
}

/*!
 * @brief Perform soft reset of the sensor
 * @note Writes 0x09 to register 0x0C to reset the device
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::setSoftRst() {
  uint8_t v = SPL06_SOFT_RESET_CODE;
  return writeRegister(SPL06_REG_RESET, &v, 1);
}

/*!
 * @brief Set working mode of the sensor
 * @note Sets predefined profiles for different working modes
 * @param work_mode Work mode value:
 *   - 0: SPL06_LOW_POWER_MODE
 *   - 1: SPL06_STANDARD_RESOLUTION_MODE
 *   - 2: SPL06_HIGH_RESOLUTION_MODE
 * @return 0 on success, -1 on failure
 */
int8_t spl06_001::setWorkMode(uint8_t work_mode) {
  // Map to predefined profiles (mirrors original C behavior)
  switch (work_mode) {
    case 0:
      samplerate_pressure_ = SPL06_SAMPLERATE_1;
      oversamp_pressure_ = SPL06_OVERSAMP_2X;
      samplerate_temperature_ = SPL06_SAMPLERATE_1;
      oversamp_temperature_ = SPL06_OVERSAMP_1X;
      break;
    case 1:
      samplerate_pressure_ = SPL06_SAMPLERATE_1;
      oversamp_pressure_ = SPL06_OVERSAMP_16X;
      samplerate_temperature_ = SPL06_SAMPLERATE_1;
      oversamp_temperature_ = SPL06_OVERSAMP_1X;
      break;
    case 2:
      samplerate_pressure_ = SPL06_SAMPLERATE_4;
      oversamp_pressure_ = SPL06_OVERSAMP_64X;
      samplerate_temperature_ = SPL06_SAMPLERATE_4;
      oversamp_temperature_ = SPL06_OVERSAMP_1X;
      break;
    default:
      return -1;
  }
  if (setOversampPressure(oversamp_pressure_) != 0) return -1;
  if (setSampleratePressure(samplerate_pressure_) != 0) return -1;
  if (setSamplerateTemperature(samplerate_temperature_) != 0) return -1;
  if (setOversampTemperature(oversamp_temperature_) != 0) return -1;
  return 0;
}
