#include "rtdevice.h"

#include "mpu6050_regs.h"  // 添加寄存器定义头文件
#include "mpu6050.hpp"
#include "i2c_interface.h"

#define IMU_CONFIGURE                                                   \
  {                                                                     \
      3200,                 /* gyro ODR at 3.2KHz */                    \
      IMU_GYRO_MODE_NORMAL, /* NORMAL MODE (approximate ~751Hz DLPF) */ \
      800,                  /* accel ODR at 800Hz */                    \
      IMU_ACC_MODE_OSR2,    /* 178 dlpf */                              \
      GYRO_SCALE_2000DPS,                                               \
      ACC_SCALE_16G,                                                    \
      IMU_TEMP_SCALE,                                                   \
      IMU_TEMP_OFFSET,                                                  \
  }

static I2cInterface_t i2c_interface;

static rt_size_t mpu6050_read_data(imu_dev_t imu, rt_off_t pos, void *data, rt_size_t size) {
    if (data == NULL) {
        return 0;
    }
    drv_mpu6050_read(pos, data, size);
    return size;
}


const static struct imu_ops mpu6050_dev = {
        NULL,
        mpu6050_read_data,
};

static struct imu_device imu_dev = {
    .ops = &mpu6050_dev,
    .config = IMU_CONFIGURE,
};

static rt_err_t mpu6050_i2c_init(const char* i2c_device_name, uint8_t i2c_addr) {
  rt_err_t result = get_i2c_interface(i2c_device_name, i2c_addr, &i2c_interface);
  if (result != RT_EOK) {
    return result;
  }

  MPU6050::instance().setI2cAddr(i2c_addr);
  MPU6050::instance().setI2cBusWrite(i2c_write_reg8_mult_pack);
  MPU6050::instance().setI2cBusRead(i2c_read_reg8_mult_pack);
  MPU6050::instance().setDelayMs(delay_ms);

  int ret = MPU6050::instance().init();
  if (ret != 0) {
    return RT_ERROR;
  }

  hal_imu_register(&imu_dev, MPU6050_DEVICE_NAME_ID1, RT_DEVICE_FLAG_RDWR, RT_NULL);

  return RT_EOK;
}

static int mpu6050_init_auto(void) { return mpu6050_i2c_init(MPU6050_I2C_NAME_ID1, MPU6050_I2C_ADDRESS_ID1); }

#ifdef BSP_USING_MPU6050
INIT_COMPONENT_EXPORT(mpu6050_init_auto);
#endif