#include "rtdevice.h"
#include "rtthread.h"

#include "mpu6050_regs.h"  // 添加寄存器定义头文件
#include "mpu6050.hpp"
#include "i2c_interface.h"
#include "imu.h"  // 添加IMU相关常量定义

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

static int8_t i2cBusRead_wrap(uint8_t devAddress, uint8_t memAddress, uint8_t* data, uint8_t len) {
  return i2c_read_reg8_mult_pack(i2c_interface, memAddress, data, len);
}
static int8_t i2cBusWrite_wrap(uint8_t devAddress, uint8_t memAddress, uint8_t* data, uint8_t len) {
  return i2c_write_reg8_mult_pack(i2c_interface, memAddress, data, len);
}
static void delay_ms_wrap(unsigned int ms) { rt_thread_mdelay(ms); }

static rt_size_t mpu6050_read_data(imu_dev_t imu, rt_off_t pos, void *data, rt_size_t size) {
    if (data == NULL) {
        return 0;
    }
    return drv_mpu6050_read(pos, data, size);
}

const static struct imu_ops mpu6050_dev = {
        NULL,
        mpu6050_read_data,
};

static struct imu_device imu_dev = {
    .ops = &mpu6050_dev,
    .config = IMU_CONFIGURE,
};

static rt_err_t mpu6050_init(const char* i2c_device_name, uint8_t i2c_addr) {
  rt_err_t result = get_i2c_interface(i2c_device_name, i2c_addr, &i2c_interface);
  if (result != RT_EOK) {
    return result;
  }
  
  drv_mpu6050_set_i2c_funcs(i2cBusWrite_wrap, i2cBusRead_wrap);
  drv_mpu6050_set_delay(delay_ms_wrap);
  
  int ret = drv_mpu6050_init();
  if (ret != 0) {
    return RT_ERROR;
  }

  hal_imu_register(&imu_dev, SENSOR_NAME_MPU6050_MINIFLY, RT_DEVICE_FLAG_RDWR, RT_NULL);

  return RT_EOK;
}

static int mpu6050_init_auto(void) { return mpu6050_init(SENSOR_I2C_NAME_MPU6050_MINIFLY, SENSOR_MPU6050_MINIFLY_I2C_ADDR); }

#ifdef BSP_USING_MPU6050_MINIFLY
INIT_COMPONENT_EXPORT(mpu6050_init_auto);
#endif