#pragma once

extern "C" {
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_gpio.h"
#include "drv_spi.h"
#include "rtconfig.h"
}

#include <ctype.h>
#include <stdint.h>

class SpiInterface {
 public:
  SpiInterface();
  ~SpiInterface();

  bool init(const char *spi_bus_name, const char *spi_slave_name, const char *cs_pin_name);
  bool configure(rt_uint16_t mode, rt_uint32_t max_hz);

  // 简化接口，适配 MPU6000 的读写函数签名（返回0成功，非0失败）
  int write_reg(uint8_t reg, uint8_t val);
  int read_multi(uint8_t reg, uint8_t *buff, uint8_t len);

 private:
  int spi_read_reg_wrapper(uint8_t *cmd, uint8_t cmd_length, uint8_t *data, uint16_t data_len);
  int spi_write_reg_wrapper(uint8_t *cmd, uint8_t cmd_length, uint8_t *data, uint16_t data_len);

  rt_spi_device *spi_device_;
};