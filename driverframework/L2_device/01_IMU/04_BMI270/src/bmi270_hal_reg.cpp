#include "bmi270.hpp"
#include "spi_interface.hpp"
#include "pinInterface.h"

extern "C" {
#include "imu.h"
#include "rtconfig.h"
#include "rtdevice.h"
#include "rtthread.h"
}

#ifdef SENSOR_BMI270_DEBUGPIN_EN
#include "debugPin.h"
#endif

static SpiInterface g_spi_;
static rt_base_t g_cs_pin_ = -1;

static int read_multi_wrap(uint8_t reg, uint8_t *buff, uint8_t len) { return g_spi_.readMultiReg16(reg, buff, len); }
static int write_multi_wrap(uint8_t reg, uint8_t *buff, uint16_t len) { return g_spi_.writeMultiReg8(reg, buff, len); }
static void delay_ms_wrap(unsigned int ms) { rt_thread_mdelay(ms); }
static void gpio_cs_control_wrap(bool state) {
  if (g_cs_pin_ >= 0) {
    rt_pin_write(g_cs_pin_, state ? PIN_HIGH : PIN_LOW);
  }
}

static int8_t bmi270_read_data(imu_dev_t imu, rt_off_t pos, void *data, rt_size_t size) {
  if (data == RT_NULL) {
    return -RT_EINVAL;
  }
  if (size < 14) {
    return -RT_EINVAL;
  }
  uint8_t *out = (uint8_t *)data;
  int rc = Bmi270::instance().readBurstImu(out);
  return (rc == 0) ? 14 : -RT_ERROR;
}

static void gpioCsControlInit(const char *cs_pin_name) {
  g_cs_pin_ = parse_pin_name_from_config(cs_pin_name);
  rt_pin_mode(g_cs_pin_, PIN_MODE_OUTPUT);
}

static rt_err_t bmi270_control(imu_dev_t dev, int cmd, void *arg) {
  if (cmd == IMU_CMD_CALIBRATE) {
    return (Bmi270::instance().calibrate() == 0) ? RT_EOK : RT_ERROR;
  }
  return -RT_ENOSYS;
}

const static struct imu_ops bmi270_dev_ops = {
    .imu_config = RT_NULL,
    .imu_control = bmi270_control,
    .imu_read = bmi270_read_data,
};

static struct imu_device bmi270_dev = {
    .ops = &bmi270_dev_ops,
    .config = {0},
};

static void setImuParam(void) {
  bmi270_dev.config.gyro_scale_factor = (float)2000.0f / (1 << 15);
  bmi270_dev.config.acc_scale_factor = (float)16.0f / (1 << 15);
  bmi270_dev.config.temp_scale = IMU_TEMP_SCALE;
  bmi270_dev.config.temp_offset = IMU_TEMP_OFFSET;
}

static rt_err_t bmi270_init(const char *spi_bus_name, const char *spi_slave_name, const char *cs_pin_name,
                            int spi_max_hz, const char *imu_name) {
  if (!g_spi_.init(spi_bus_name, spi_slave_name, cs_pin_name)) {
    rt_kprintf("bmi270 spi init failed\r\n");
    return RT_ERROR;
  }

  if (!g_spi_.configure((RT_SPI_MODE_3 | RT_SPI_MSB) & RT_SPI_MODE_MASK, spi_max_hz)) {
    return RT_ERROR;
  }

  gpioCsControlInit(cs_pin_name);

  Bmi270 &bmi270 = Bmi270::instance();
  if (bmi270.setIoFunctions(read_multi_wrap, write_multi_wrap, delay_ms_wrap, gpio_cs_control_wrap) != 0) {
    return RT_ERROR;
  }
  if (bmi270.detect() != 0) {
    return RT_ERROR;
  }
  if (bmi270.configure() != 0) {
    return RT_ERROR;
  }
  setImuParam();

  hal_imu_register(&bmi270_dev, imu_name, RT_DEVICE_FLAG_RDWR, RT_NULL);
  return RT_EOK;
}

extern "C" {
static int bmi270_init_auto(void) {
  return bmi270_init(SENSOR_SPI_NAME_BMI270, SENSOR_SPI_SLAVE_NAME_BMI270, SENSOR_BMI270_SPI_CS_PIN,
                     SENSOR_BMI270_SPI_MAX_HZ, SENSOR_NAME_BMI270);
}
#ifdef BSP_USING_BMI270
INIT_ENV_EXPORT(bmi270_init_auto);
#endif
}
