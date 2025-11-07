#include "bmi270_px4_wrapper.h"

extern "C" {
#include <rtconfig.h>
}

#include "BMI270.hpp"

using namespace bmi270_px4;

static void fill_runtime_config(RuntimeConfig &runtime, const px4_bmi270_config_t *cfg) {
  runtime.spi_bus_name = (cfg && cfg->spi_bus_name) ? cfg->spi_bus_name : SENSOR_SPI_NAME_BMI270_PX4;
  runtime.spi_device_name = (cfg && cfg->spi_device_name) ? cfg->spi_device_name : SENSOR_SPI_SLAVE_NAME_BMI270_PX4;
  runtime.cs_pin_name = (cfg && cfg->cs_pin_name) ? cfg->cs_pin_name : SENSOR_BMI270_PX4_SPI_CS_PIN;
  runtime.int_pin_name = (cfg && cfg->int_pin_name) ? cfg->int_pin_name : SENSOR_BMI270_PX4_INT_PIN;
  runtime.spi_max_hz = (cfg && cfg->spi_max_hz) ? cfg->spi_max_hz : SENSOR_BMI270_PX4_SPI_MAX_HZ;
  runtime.imu_device_name = (cfg && cfg->imu_device_name) ? cfg->imu_device_name : SENSOR_NAME_BMI270_PX4;

  uint8_t watermark = (cfg && cfg->fifo_watermark_samples) ? cfg->fifo_watermark_samples : SENSOR_BMI270_PX4_FIFO_WM;
  if (watermark == 0) {
    watermark = 4;
  } else if (watermark > 32) {
    watermark = 32;
  }
  runtime.fifo_watermark_samples = watermark;
}

rt_err_t px4_bmi270_init(const px4_bmi270_config_t *cfg) {
  RuntimeConfig runtime{};
  fill_runtime_config(runtime, cfg);
  return BMI270::instance().init(runtime);
}

bool px4_bmi270_is_ready(void) {
  return BMI270::instance().initialized();
}

int px4_bmi270_read_raw(uint8_t *buffer, rt_size_t size) {
  return BMI270::instance().readRaw(buffer, size);
}


