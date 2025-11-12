/*

The MIT License

Copyright (c) 2014-2023 Korneliusz Jarzębski

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include <cmath>

#include <rtthread.h>

extern "C" {
#include "esc_monitor_i2c.h"
#define LOG_TAG "ina226"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include "INA226.h"

namespace {
static const I2cInterface_t* g_i2c_interface = nullptr;
static bool g_ina_initialized = false;
static INA226 g_ina_device;
}  // namespace

bool INA226::begin(uint8_t address)
{
  const I2cInterface_t* interface = esc_monitor_get_i2c_interface();
  if (interface == RT_NULL || interface->i2c_dev == RT_NULL) {
    LOG_E("begin: no I2C interface");
    return false;
  }

  g_i2c_interface = interface;
  inaAddress = address;
  return true;
}

bool INA226::configure(ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode)
{
    uint16_t config = 0;

    config |= (avg << 9 | busConvTime << 6 | shuntConvTime << 3 | mode);

    vBusMax = 36;
    vShuntMax = 0.08192f;

    writeRegister16(INA226_REG_CONFIG, config);

    return true;
}

bool INA226::calibrate(float rShuntValue, float iMaxExpected)
{
    uint16_t calibrationValue;
    rShunt = rShuntValue;

    float minimumLSB = iMaxExpected / 32767;

    currentLSB = (uint32_t)(minimumLSB * 100000000);
    currentLSB /= 100000000;
    currentLSB /= 0.0001;
    currentLSB = std::ceil(currentLSB);
    currentLSB *= 0.0001;

    powerLSB = currentLSB * 25;

    calibrationValue = (uint16_t)((0.00512) / (currentLSB * rShunt));

    writeRegister16(INA226_REG_CALIBRATION, calibrationValue);

    return true;
}

float INA226::getMaxPossibleCurrent(void)
{
    return (vShuntMax / rShunt);
}

float INA226::getMaxCurrent(void)
{
    float maxCurrent = (currentLSB * 32767);
    float maxPossible = getMaxPossibleCurrent();

    if (maxCurrent > maxPossible)
    {
        return maxPossible;
    } else
    {
        return maxCurrent;
    }
}

float INA226::getMaxShuntVoltage(void)
{
    float maxVoltage = getMaxCurrent() * rShunt;

    if (maxVoltage >= vShuntMax)
    {
        return vShuntMax;
    } else
    {
        return maxVoltage;
    }
}

float INA226::getMaxPower(void)
{
    return (getMaxCurrent() * vBusMax);
}

float INA226::readBusPower(void)
{
    return (readRegister16(INA226_REG_POWER) * powerLSB);
}

float INA226::readShuntCurrent(void)
{
    return (readRegister16(INA226_REG_CURRENT) * currentLSB);
}

int16_t INA226::readRawShuntCurrent(void)
{
    return readRegister16(INA226_REG_CURRENT);
}

float INA226::readShuntVoltage(void)
{
    float voltage;

    voltage = readRegister16(INA226_REG_SHUNTVOLTAGE);

    return (voltage * 0.0000025);
}

float INA226::readBusVoltage(void)
{
    int16_t voltage;

    voltage = readRegister16(INA226_REG_BUSVOLTAGE);

    return (voltage * 0.00125);
}

ina226_averages_t INA226::getAverages(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000111000000000;
    value >>= 9;

    return (ina226_averages_t)value;
}

ina226_busConvTime_t INA226::getBusConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000000111000000;
    value >>= 6;

    return (ina226_busConvTime_t)value;
}

ina226_shuntConvTime_t INA226::getShuntConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000000000111000;
    value >>= 3;

    return (ina226_shuntConvTime_t)value;
}

ina226_mode_t INA226::getMode(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000000000000111;

    return (ina226_mode_t)value;
}

void INA226::setMaskEnable(uint16_t mask)
{
    writeRegister16(INA226_REG_MASKENABLE, mask);
}

uint16_t INA226::getMaskEnable(void)
{
    return readRegister16(INA226_REG_MASKENABLE);
}

void INA226::enableShuntOverLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_SOL);
}

void INA226::enableShuntUnderLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_SUL);
}

void INA226::enableBusOvertLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_BOL);
}

void INA226::enableBusUnderLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_BUL);
}

void INA226::enableOverPowerLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_POL);
}

void INA226::enableConversionReadyAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_CNVR);
}

void INA226::disableAlerts(void)
{
    writeRegister16(INA226_REG_MASKENABLE, 0);
}

void INA226::setBusVoltageLimit(float voltage)
{
    uint16_t value = voltage / 0.00125;
    writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setShuntVoltageLimit(float voltage)
{
    uint16_t value = voltage / 0.0000025;
    writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setPowerLimit(float watts)
{
    uint16_t value = watts / powerLSB;
    writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setAlertInvertedPolarity(bool inverted)
{
    uint16_t temp = getMaskEnable();

    if (inverted)
    {
        temp |= INA226_BIT_APOL;
    } else
    {
        temp &= ~INA226_BIT_APOL;
    }

    setMaskEnable(temp);
}

void INA226::setAlertLatch(bool latch)
{
    uint16_t temp = getMaskEnable();

    if (latch)
    {
        temp |= INA226_BIT_LEN;
    } else
    {
        temp &= ~INA226_BIT_LEN;
    }

    setMaskEnable(temp);
}

bool INA226::isMathOverflow(void)
{
    return ((getMaskEnable() & INA226_BIT_OVF) == INA226_BIT_OVF);
}

bool INA226::isAlert(void)
{
    return ((getMaskEnable() & INA226_BIT_AFF) == INA226_BIT_AFF);
}

int16_t INA226::readRegister16(uint8_t reg)
{
    int16_t value;

    if (g_i2c_interface == nullptr || g_i2c_interface->i2c_dev == RT_NULL) {
      LOG_E("read reg 0x%02x: bus null", reg);
      return 0;
    }

    uint8_t buffer[2] = {0};

    if (i2c_read_reg8_mult_pack(*g_i2c_interface, reg, buffer, sizeof(buffer)) != RT_EOK) {
      LOG_E("read reg 0x%02x: transfer fail", reg);
      return 0;
    }

    value = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);

    return value;
}

void INA226::writeRegister16(uint8_t reg, uint16_t val)
{
  if (g_i2c_interface == nullptr || g_i2c_interface->i2c_dev == RT_NULL) {
    LOG_E("write reg 0x%02x: bus null", reg);
    return;
  }

  uint8_t buffer[2];
  buffer[0] = static_cast<uint8_t>((val >> 8) & 0xFF);
  buffer[1] = static_cast<uint8_t>(val & 0xFF);

  i2c_write_reg8_mult_pack(*g_i2c_interface, reg, buffer, sizeof(buffer));
}

static rt_err_t ina226_demo_init(void) {
  if (g_ina_initialized) {
    return RT_EOK;
  }

  const I2cInterface_t* interface = esc_monitor_get_i2c_interface();
  if (interface == RT_NULL || interface->i2c_dev == RT_NULL) {
    if (esc_monitor_i2c_init() != RT_EOK) {
      LOG_E("cmd: i2c init failed");
      return -RT_ERROR;
    }
  }

  if (!g_ina_device.begin(0x40)) {
    LOG_E("cmd: sensor begin failed");
    return -RT_ERROR;
  }

  if (!g_ina_device.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US,
                              INA226_MODE_SHUNT_BUS_CONT)) {
    LOG_E("cmd: configure failed");
    return -RT_ERROR;
  }

  if (!g_ina_device.calibrate(0.01f, 4.0f)) {
    LOG_E("cmd: calibrate failed");
    return -RT_ERROR;
  }

  g_ina_device.enableOverPowerLimitAlert();
  g_ina_device.setPowerLimit(0.130f);

  g_ina_initialized = true;
  return RT_EOK;
}

extern "C" void cmdIna226Demo(int argc, char** argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);

  if (ina226_demo_init() != RT_EOK) {
    return;
  }

  float bus_voltage = g_ina_device.readBusVoltage();
  float bus_power = g_ina_device.readBusPower();
  float shunt_voltage = g_ina_device.readShuntVoltage();
  float shunt_current = g_ina_device.readShuntCurrent();

  LOG_I("-----------------------------------------------");
  LOG_I("INA226 quick read");
  LOG_I("Bus voltage:   %.5f V", bus_voltage);
  LOG_I("Bus power:     %.5f W", bus_power);
  LOG_I("Shunt voltage: %.5f V", shunt_voltage);
  LOG_I("Shunt current: %.5f A", shunt_current);
  LOG_I("-----------------------------------------------");
}
MSH_CMD_EXPORT_ALIAS(cmdIna226Demo, cmdIna226Demo, INA226 quick read once);
