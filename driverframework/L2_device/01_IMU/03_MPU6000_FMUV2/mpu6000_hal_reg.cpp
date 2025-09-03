#include "rtdevice.h"
#include "rtthread.h"
#include "imu.h"
#include "mpu6000.hpp"
#include <string.h>

#include "spi_interface.hpp"

#ifdef SENSOR_MPU6000_DEBUGPIN_EN
#include "debugPin.h"
#endif

static SpiInterface g_spi_;

static int write_reg_wrap(uint8_t reg, uint8_t val) { return g_spi_.write_reg(reg, val); }

static int read_multi_wrap(uint8_t reg, uint8_t *buff, uint8_t len) { return g_spi_.read_multi(reg, buff, len); }

static void delay_ms_wrap(unsigned int ms) { rt_thread_mdelay(ms); }

static int8_t mpu6000_read_data(imu_dev_t imu, rt_off_t pos, void* data, rt_size_t size) {
  if (data == RT_NULL) {
    return -RT_EINVAL;
  }

  int8_t read_size = drv_mpu6000_read(3, data, size);
  return read_size;
}

const static struct imu_ops mpu6000_dev = {
    .imu_config = RT_NULL,
    .imu_read = mpu6000_read_data,
};

static struct imu_device imu_dev = {
    .ops = &mpu6000_dev, .config = {0},  // 初始化为空配置
};

static rt_err_t mpu6000_init(const char *spi_bus_name, const char *spi_slave_name, const char *cs_pin_name,
                             int spi_max_hz, const char *imu_name) {
  if (!g_spi_.init(spi_bus_name, spi_slave_name, cs_pin_name)) {
    rt_kprintf("init spi failed\r\n");
    return RT_ERROR;
  }

  if (!g_spi_.configure((RT_SPI_MODE_3 | RT_SPI_MSB) & RT_SPI_MODE_MASK, spi_max_hz)) {
    return RT_ERROR;
  }

  drv_mpu6000_set_fn(write_reg_wrap, read_multi_wrap, delay_ms_wrap);

  int ret = drv_mpu6000_init();
  if (ret != 0) {
    return RT_ERROR;
  }

  hal_imu_register(&imu_dev, imu_name, RT_DEVICE_FLAG_RDWR, RT_NULL);

  return RT_EOK;
}

extern "C" {
static int mpu6000_init_auto(void) {
  return mpu6000_init(SENSOR_SPI_NAME_MPU6000, SENSOR_SPI_SLAVE_NAME_MPU6000, SENSOR_MPU6000_SPI_CS_PIN_NAME,
                      SENSOR_MPU6000_SPI_MAX_HZ, SENSOR_NAME_MPU6000);
}
#ifdef BSP_USING_MPU6000
INIT_COMPONENT_EXPORT(mpu6000_init_auto);
#endif
}