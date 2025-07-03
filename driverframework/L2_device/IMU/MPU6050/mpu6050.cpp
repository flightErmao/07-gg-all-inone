#include "mpu6050.hpp"
#include <cstring>
#include <rtdbg.h>

MPU6050::MPU6050(const char* i2c_device_name) {
  imu_i2c_dev_ = rt_device_find(i2c_device_name);
  RT_ASSERT(imu_i2c_dev_ != nullptr);
  dev_addr_ = MPU6500_ADDRESS_AD0_HIGH;
  is_init_ = false;
}

MPU6050::~MPU6050() {
  if (imu_i2c_dev_) {
    rt_device_close(imu_i2c_dev_);
    imu_i2c_dev_ = nullptr;
  }
}

rt_err_t MPU6050::init() {
  if (is_init_) return RT_EOK;
  RT_TRY(rt_device_open(imu_i2c_dev_, RT_DEVICE_OFLAG_RDWR));
  // 复位、唤醒、配置寄存器等
  // ... 迁移原mpu6050_init流程 ...
  is_init_ = true;
  return RT_EOK;
}

rt_err_t MPU6050::config(const imu_configure* cfg) {
  RT_ASSERT(cfg != nullptr);
  config_ = *cfg;
  // 可根据cfg设置寄存器
  return RT_EOK;
}

rt_size_t MPU6050::read_data(rt_off_t pos, void* data, rt_size_t size) {
  if (data == nullptr) return 0;
  i2cBusRead(dev_addr_, MPU6500_RA_ACCEL_XOUT_H, size, static_cast<uint8_t*>(data));
  return size;
}

void MPU6050::setI2cBusWrite(int8_t (*func)(uint8_t, uint8_t, uint8_t*, uint8_t)) { i2c_bus_write_ = func; }
void MPU6050::setI2cBusRead(int8_t (*func)(uint8_t, uint8_t, uint8_t*, uint8_t)) { i2c_bus_read_ = func; }

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

void setI2cAddr(uint8_t addr) { dev_addr_ = addr; }

// 兼容C接口
extern "C" rt_err_t drv_mpu6050_i2c_init(const char* i2c_device_name, const char* device_name) {
  static MPU6050 mpu(i2c_device_name);
  return mpu.init();
}

// ... 其他成员函数和寄存器操作可继续迁移 ...
// ... existing code ...
