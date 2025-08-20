#include "mpu6050.hpp"
#include "mpu6050_regs.h"  // 添加寄存器定义头文件
#include <cstring>
#include <cstdio>

MPU6050::MPU6050() {
  // 替换为通用初始化，不依赖RTThread
  device_.user_data = nullptr;  // 用户可以在此存储I2C设备句柄
  dev_addr_ = MPU6500_ADDRESS_AD0_HIGH;
  is_init_ = false;
}

MPU6050::~MPU6050() {
  // 不再依赖RTThread关闭设备
}

// 单例实现
MPU6050& MPU6050::instance() {
  static MPU6050 s_instance;
  return s_instance;
}

// 设置延时函数指针
void MPU6050::setDelayMs(delay_ms_func_t func) { delay_ms_ = func; }

// 内部延时函数
void MPU6050::delayMs(unsigned int ms) {
  if (delay_ms_ != nullptr) {
    delay_ms_(ms);
  }
  // 移除对未定义的全局delay_ms函数的调用
}

int MPU6050::mpu6500Init() {
  if (is_init_) return MPU_EOK;

  // vTaskDelay(10);
  delayMs(10);

  // mpu6500Reset(); // 复位MPU6500（直接置位复位位，避免 reset() 内部固定 100ms 延时）
  i2cdevWriteBit(dev_addr_, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_DEVICE_RESET_BIT, 1);

  // vTaskDelay(20); // 等待寄存器复位
  delayMs(20);

  // 读取设备ID并校验（0x38 或 0x39）
  uint8_t temp = getDeviceID();
  if (temp == 0x38 || temp == 0x39) {
    std::printf("MPU9250 I2C connection [OK].\n");
  } else {
    std::printf("MPU9250 I2C connection [FAIL].\n");
    return MPU_ERROR;
  }

  // 唤醒与基础配置
  setSleepEnabled(false);  // 唤醒MPU6500
  delayMs(10);
  setClockSource(MPU6500_CLOCK_PLL_XGYRO);       // 设置X轴陀螺作为时钟
  delayMs(10);                                   // 等待时钟稳定
  setTempSensorEnabled(true);                    // 使能温度传感器
  setIntEnabled(false);                          // 关闭中断
  setI2CBypassEnabled(true);                     // 旁路模式
  setFullScaleGyroRange(SENSORS_GYRO_FS_CFG);    // 陀螺量程
  setFullScaleAccelRange(SENSORS_ACCEL_FS_CFG);  // 加速计量程
  setAccelDLPF(MPU6500_ACCEL_DLPF_BW_41);        // 加速度计DLPF

  // 采样率与陀螺DLPF
  setRate(0);                       // 1000Hz
  setDLPFMode(MPU6500_DLPF_BW_98);  // 陀螺DLPF

  is_init_ = true;
  return MPU_EOK;
}

void MPU6050::mpu6500SetWaitForExternalSensorEnabled(bool enabled) {
  i2cdevWriteBit(dev_addr_, MPU6500_RA_I2C_MST_CTRL, MPU6500_WAIT_FOR_ES_BIT, enabled);
}

void MPU6050::mpu6500SetInterruptMode(bool mode) {
  i2cdevWriteBit(dev_addr_, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_INT_LEVEL_BIT, mode);
}

void MPU6050::mpu6500SetInterruptDrive(bool drive) {
  i2cdevWriteBit(dev_addr_, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_INT_OPEN_BIT, drive);
}

void MPU6050::mpu6500SetInterruptLatch(bool latch) {
  i2cdevWriteBit(dev_addr_, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_LATCH_INT_EN_BIT, latch);
}

void MPU6050::mpu6500SetInterruptLatchClear(bool clear) {
  i2cdevWriteBit(dev_addr_, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_INT_RD_CLEAR_BIT, clear);
}

void MPU6050::mpu6500SetSlaveReadWriteTransitionEnabled(bool enabled) {
  i2cdevWriteBit(dev_addr_, MPU6500_RA_I2C_MST_CTRL, MPU6500_I2C_MST_P_NSR_BIT, enabled);
}

void MPU6050::mpu6500SetMasterClockSpeed(uint8_t speed) {
  i2cdevWriteBits(dev_addr_, MPU6500_RA_I2C_MST_CTRL, MPU6500_I2C_MST_CLK_BIT, MPU6500_I2C_MST_CLK_LENGTH, speed);
}

void MPU6050::mpu6500SetI2CMasterModeEnabled(bool enabled) {
  i2cdevWriteBit(dev_addr_, MPU6500_RA_USER_CTRL, MPU6500_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU6050::mpu6500SetIntDataReadyEnabled(bool enabled) {
  i2cdevWriteBit(dev_addr_, MPU6500_RA_INT_ENABLE, MPU6500_INTERRUPT_DATA_RDY_BIT, enabled);
}

int MPU6050::mpu6500SlaveSensorInit() {
  // mpu6500SetSlave4MasterDelay(19);  // 从机读取速率: 100Hz = (1000Hz / (1 + 9))

  setI2CBypassEnabled(false);  // 主机模式
  mpu6500SetWaitForExternalSensorEnabled(true);
  mpu6500SetInterruptMode(0);                        // 中断高电平有效
  mpu6500SetInterruptDrive(0);                       // 推挽输出
  mpu6500SetInterruptLatch(0);                       // 中断锁存模式(0=50us-pulse, 1=latch-until-int-cleared)
  mpu6500SetInterruptLatchClear(1);                  // 中断清除模式(0=status-read-only, 1=any-register-read)
  mpu6500SetSlaveReadWriteTransitionEnabled(false);  // 关闭从机读写传输
  mpu6500SetMasterClockSpeed(13);                    // 设置i2c速度400kHz

  mpu6500SetI2CMasterModeEnabled(true);  // 使能mpu6500主机模式
  mpu6500SetIntDataReadyEnabled(true);   // 数据就绪中断使能
}

int MPU6050::init() {
  if (mpu6500Init() != MPU_EOK) {
    return MPU_ERROR;
  }

  delayMs(150);

  // 初始化从机传感器
  if (mpu6500SlaveSensorInit() != MPU_EOK) {
    return MPU_ERROR;
  }

  return MPU_EOK;
}

// 实现read_data函数，用于读取传感器数据
int MPU6050::read_data(int pos, void* data, int size) {
  if (data == nullptr) {
    return 0;
  }

  if (!is_init_) {
    // 如果设备未初始化，返回错误
    return MPU_ERROR;
  }

  // 这里假设pos是传感器寄存器地址，从该地址开始读取size个字节
  int8_t result =
      i2cBusRead(dev_addr_, static_cast<uint8_t>(pos), static_cast<uint8_t>(size), static_cast<uint8_t*>(data));

  if (result != 0) {
    // I2C读取出错
    return MPU_ERROR;
  }

  return size;  // 返回实际读取的字节数
}

int8_t MPU6050::setI2cBusWrite(int8_t (*func)(uint8_t, uint8_t, uint8_t*, uint8_t)) {
  if (func == nullptr) {
    return -1;
  } else {
    i2c_bus_write_ = func;
  }
  return 0;
}

int8_t MPU6050::setI2cBusRead(int8_t (*func)(uint8_t, uint8_t, uint8_t*, uint8_t)) {
  if (func == nullptr) {
    return -1;
  } else {
    i2c_bus_read_ = func;
  }
  return 0;
}

int8_t MPU6050::i2cBusRead(uint8_t devAddress, uint8_t memAddress, uint8_t len, uint8_t* data) {
  if (i2c_bus_read_) {
    return i2c_bus_read_(devAddress, memAddress, data, len);
  }
  return -1;
}

int8_t MPU6050::i2cBusWrite(uint8_t devAddress, uint8_t memAddress, uint8_t len, uint8_t* data) {
  if (i2c_bus_write_) {
    return i2c_bus_write_(devAddress, memAddress, data, len);
  }
  return -1;
}

int8_t MPU6050::i2cBusReadByte(uint8_t devAddress, uint8_t memAddress, uint8_t* data) {
  return i2cBusRead(devAddress, memAddress, 1, data);
}

int8_t MPU6050::i2cBusWriteByte(uint8_t devAddress, uint8_t memAddress, uint8_t data) {
  return i2cBusWrite(devAddress, memAddress, 1, &data);
}

void MPU6050::setI2cAddr(uint8_t addr) { dev_addr_ = addr; }

// 新增位操作函数
int8_t MPU6050::i2cdevReadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t* data) {
  uint8_t b;
  int8_t status = i2cBusReadByte(devAddr, regAddr, &b);
  if (status == 0) {
    *data = (b >> bitNum) & 0x01;
  }
  return status;
}

int8_t MPU6050::i2cdevReadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data) {
  uint8_t b;
  uint8_t mask;
  int8_t status = i2cBusReadByte(devAddr, regAddr, &b);
  if (status == 0) {
    mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    *data = b;
  }
  return status;
}

int8_t MPU6050::i2cdevWriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
  uint8_t b;
  int8_t status = i2cBusReadByte(devAddr, regAddr, &b);
  if (status == 0) {
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    status = i2cBusWriteByte(devAddr, regAddr, b);
  }
  return status;
}

int8_t MPU6050::i2cdevWriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
  uint8_t b;
  uint8_t mask;
  int8_t status = i2cBusReadByte(devAddr, regAddr, &b);
  if (status == 0) {
    mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);
    data &= mask;
    b &= ~mask;
    b |= data;
    status = i2cBusWriteByte(devAddr, regAddr, b);
  }
  return status;
}

// 公开的数据获取函数
uint8_t MPU6050::getAccelerometer(int16_t* ax, int16_t* ay, int16_t* az) {
  uint8_t buf[6];
  uint8_t res = i2cBusRead(dev_addr_, MPU6500_RA_ACCEL_XOUT_H, 6, buf);
  if (res == 0) {
    *ax = ((uint16_t)buf[0] << 8) | buf[1];
    *ay = ((uint16_t)buf[2] << 8) | buf[3];
    *az = ((uint16_t)buf[4] << 8) | buf[5];
  }
  return res;
}

uint8_t MPU6050::getGyroscope(int16_t* gx, int16_t* gy, int16_t* gz) {
  uint8_t buf[6];
  uint8_t res = i2cBusRead(dev_addr_, MPU6500_RA_GYRO_XOUT_H, 6, buf);
  if (res == 0) {
    *gx = ((uint16_t)buf[0] << 8) | buf[1];
    *gy = ((uint16_t)buf[2] << 8) | buf[3];
    *gz = ((uint16_t)buf[4] << 8) | buf[5];
  }
  return res;
}

uint8_t MPU6050::getTemperature(float* temp) {
  uint8_t buf[2];
  uint8_t res = i2cBusRead(dev_addr_, MPU6500_RA_TEMP_OUT_H, 2, buf);
  if (res == 0) {
    int16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    *temp = 36.53f + ((float)raw) / 340.0f;
  }
  return res;
}

uint8_t MPU6050::selfTest() {
  uint8_t data;
  uint8_t res = i2cBusReadByte(dev_addr_, MPU6500_RA_WHO_AM_I, &data);
  if (res == 0) {
    return (data == 0x68 || data == 0x69) ? 0 : 1;  // 检查设备ID是否正确
  }
  return 1;
}

// 内部配置函数实现
void MPU6050::reset() {
  i2cdevWriteBit(dev_addr_, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_DEVICE_RESET_BIT, 1);
  delayMs(100);  // 等待复位完成
}

uint8_t MPU6050::getDeviceID() {
  i2cdevReadBits(dev_addr_, MPU6500_RA_WHO_AM_I, MPU6500_WHO_AM_I_BIT, MPU6500_WHO_AM_I_LENGTH, buffer);
  return buffer[0];
}

void MPU6050::setSleepEnabled(bool enabled) {
  i2cdevWriteBit(dev_addr_, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_SLEEP_BIT, enabled ? 1 : 0);
}

void MPU6050::setClockSource(uint8_t source) {
  i2cdevWriteBits(dev_addr_, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_CLKSEL_BIT, MPU6500_PWR1_CLKSEL_LENGTH, source);
}

void MPU6050::setTempSensorEnabled(bool enabled) {
  i2cdevWriteBit(dev_addr_, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_TEMP_DIS_BIT, enabled ? 0 : 1);
}

void MPU6050::setIntEnabled(bool enabled) { i2cBusWriteByte(dev_addr_, MPU6500_RA_INT_ENABLE, enabled ? 0x01 : 0x00); }

void MPU6050::setI2CBypassEnabled(bool enabled) {
  i2cdevWriteBit(dev_addr_, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_I2C_BYPASS_EN_BIT, enabled ? 1 : 0);
}

void MPU6050::setFullScaleGyroRange(uint8_t range) {
  i2cdevWriteBits(dev_addr_, MPU6500_RA_GYRO_CONFIG, MPU6500_GCONFIG_FS_SEL_BIT, 2, range);
}

void MPU6050::setFullScaleAccelRange(uint8_t range) {
  i2cdevWriteBits(dev_addr_, MPU6500_RA_ACCEL_CONFIG, MPU6500_ACONFIG_AFS_SEL_BIT, 2, range);
}

void MPU6050::setAccelDLPF(uint8_t bandwidth) {
  i2cBusWriteByte(dev_addr_, MPU6500_RA_ACCEL_CONFIG_2, bandwidth & 0x07);
}

void MPU6050::setDLPFMode(uint8_t mode) {
  i2cdevWriteBits(dev_addr_, MPU6500_RA_CONFIG, MPU6500_CFG_DLPF_CFG_BIT, 3, mode);
}

uint8_t MPU6050::setRate(uint16_t rate) {
  uint8_t data;
  if (rate == 0) {
    // 特殊处理: 0 表示 1000Hz => 分频寄存器为 0
    data = 0;
  } else {
    if (rate > 1000 || rate < 4) return 1;
    data = (uint8_t)(1000 / rate - 1);
  }
  return i2cBusWriteByte(dev_addr_, MPU6500_RA_SMPLRT_DIV, data);
}

// 兼容C接口（基于单例）
extern "C" int drv_mpu6050_init() { return MPU6050::instance().init(); }

extern "C" int drv_mpu6050_set_i2c_addr(uint8_t addr) {
  MPU6050::instance().setI2cAddr(addr);
  return MPU_EOK;
}

extern "C" int drv_mpu6050_set_delay(void (*delay_ms_cb)(unsigned int)) {
  MPU6050::instance().setDelayMs(delay_ms_cb);
  return MPU_EOK;
}

extern "C" int drv_mpu6050_set_i2c_funcs(int8_t (*write_func)(uint8_t, uint8_t, uint8_t*, uint8_t),
                                         int8_t (*read_func)(uint8_t, uint8_t, uint8_t*, uint8_t)) {
  int8_t w = MPU6050::instance().setI2cBusWrite(write_func);
  int8_t r = MPU6050::instance().setI2cBusRead(read_func);
  return (w == 0 && r == 0) ? MPU_EOK : MPU_ERROR;
}

extern "C" int drv_mpu6050_read(int pos, void* data, int size) {
  return MPU6050::instance().read_data(pos, data, size);
}

extern "C" int drv_mpu6050_get_accel(int16_t* ax, int16_t* ay, int16_t* az) {
  uint8_t res = MPU6050::instance().getAccelerometer(ax, ay, az);
  return (res == 0) ? MPU_EOK : MPU_ERROR;
}

extern "C" int drv_mpu6050_get_gyro(int16_t* gx, int16_t* gy, int16_t* gz) {
  uint8_t res = MPU6050::instance().getGyroscope(gx, gy, gz);
  return (res == 0) ? MPU_EOK : MPU_ERROR;
}

extern "C" int drv_mpu6050_get_temp(float* temp) {
  uint8_t res = MPU6050::instance().getTemperature(temp);
  return (res == 0) ? MPU_EOK : MPU_ERROR;
}
