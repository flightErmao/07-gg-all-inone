#ifndef IMU_H__
#define IMU_H__

#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GYRO_SCALE_2000DPS 2000.0f / (1 << 15)  // 16.384 dps/lsb scalefactor for 2000dps sensors
#define GYRO_SCALE_2048DPS 2048.0f / (1 << 15)  //     16 dps/lsb scalefactor for 2048dps sensors
#define GYRO_SCALE_4000DPS 4000.0f / (1 << 15)  //  8.192 dps/lsb scalefactor for 4000dps sensors

#define ACC_SCALE_2G 2.0f / (1 << 15)    // 16.384 g/lsb scalefactor for 2g sensors
#define ACC_SCALE_4G 4.0f / (1 << 15)    //  8.192 g/lsb scalefactor for 4g sensors
#define ACC_SCALE_8G 8.0f / (1 << 15)    //  4.096 g/lsb scalefactor for 8g sensors
#define ACC_SCALE_16G 16.0f / (1 << 15)  //  2.048 g/lsb scalefactor for 16g sensors

/*TO Do:need check the value in 270 datasheet*/
#define IMU_TEMP_SCALE 512
#define IMU_TEMP_OFFSET 23.0f

enum imu_mode {
  IMU_GYRO_MODE_NORMAL = 0,
  IMU_GYRO_MODE_OSR2,
  IMU_GYRO_MODE_OSR4,
  IMU_ACC_MODE_NORMAL,
  IMU_ACC_MODE_OSR2,
  IMU_ACC_MODE_OSR4,
};

enum imu_pos {
  IMU_POS_GYRO = 0, /* gyro data */
  IMU_POS_ACC_GYRO, /* accel and gyro data */
  IMU_POS_TEMP,     /* ASIC temperature data */
};

/* default config for accel sensor */
#define IMU_CONFIG_DEFAULT          \
  {                                 \
      4000, /* gyro ODR at 4KHz */  \
      500,  /* 500Hz dlpf */        \
      1000, /* accel ODR at 1KHz */ \
      125,  /* 100Hz dlpf */        \
  }

typedef struct imu_configure {
  // TODO: add gain(needed by bmi270 if we don't wanna trigger CRT everytime we power up the sensor)
  rt_uint32_t gyro_odr_hz;
  rt_uint16_t gyro_dlpf_mode; /* can be one of some modes (e.g. Normal/OSR2/OSR4 mode ) or just frequency number (e.g.
                                 100Hz/343Hz) */
  // TODO: figure out: why odr is 32bit and why dlpf is 16bit?
  rt_uint32_t accel_odr_hz;
  rt_uint16_t accel_dlpf_mode; /* can be one of some modes (e.g. Normal/OSR2/OSR4 mode ) or just frequency number (e.g.
                                  100Hz/343Hz) */

  float gyro_scale_factor;
  float acc_scale_factor;

  rt_uint16_t temp_scale;
  float temp_offset;
} imu_configure_t;

struct imu_device {
  struct rt_device parent;
  const struct imu_ops* ops;
  imu_configure_t config;
};
typedef struct imu_device* imu_dev_t;

struct imu_ops {
  /**
   * @brief imu configuration function (optional) (TODO: apply user settings (and gain) after read it from eeprom?)
   * @param dev imu device
   * @param cfg imu configuration
   */
  rt_err_t (*imu_config)(imu_dev_t dev, const imu_configure_t* cfg);
  // /**
  //  * @brief imu control function (optional) (TODO: trigger CRT and store gain to eeprom?)
  //  * @param dev imu device
  //  * @param cmd operation command
  //  * @param arg command argument (optional)
  // */
  // rt_err_t (*imu_control)(imu_dev_t dev, int cmd, void* arg);
  /**
   * @brief imu read data function
   * @param dev imu device
   * @param pos read pos, sent by upper layer. can be used to identify the data type to read, e.g, raw data or scaled
   * data
   * @param data read data buffer. normally it's a pointer to float[9]
   * @param size read data size
   */
  rt_err_t (*imu_read)(imu_dev_t dev, rt_off_t pos, void* data, rt_size_t size);
};

rt_err_t hal_imu_register(imu_dev_t imu, const char* name, rt_uint32_t flag, void* data);

#ifdef __cplusplus
}
#endif

#endif  // IMU_H__