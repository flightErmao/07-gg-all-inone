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

#ifndef SPI_INTERFACE_USERBUF_MAX
#define SPI_INTERFACE_USERBUF_MAX 64
#endif

class SpiInterface {
 public:
  SpiInterface();
  ~SpiInterface();

  bool init(const char *spi_bus_name, const char *spi_slave_name, const char *cs_pin_name);

  // 公共：配置与打开
  bool configure(rt_uint16_t mode, rt_uint32_t max_hz);
  rt_err_t open(rt_uint16_t oflag = RT_DEVICE_OFLAG_RDWR);

  int write(const uint8_t *tx_buffer, size_t length);
  int read(uint8_t *rx_buffer, size_t length);

  // 简化接口，适配 MPU6000 的读写函数签名（返回0成功，非0失败）
  int write_reg(uint8_t reg, uint8_t val);
  int read_multi(uint8_t reg, uint8_t *buff, uint8_t len);

  rt_spi_device *device() const;

 private:
  static rt_base_t parse_pin_name_from_config(const char *pin_name);

  int transfer(const uint8_t *tx_buffer, uint8_t *rx_buffer, size_t length);

  rt_spi_device *_spi_device;
};