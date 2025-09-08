#include "spi_interface.hpp"
#include <string.h>
#include "pinInterface.h"

SpiInterface::SpiInterface() : spi_device_(RT_NULL) {}

SpiInterface::~SpiInterface() {}

bool SpiInterface::init(const char *spi_bus_name, const char *spi_slave_name, const char *cs_pin_name) {
  if (spi_bus_name == RT_NULL || spi_slave_name == RT_NULL) {
    return false;
  }

  rt_base_t cs_pin = parse_pin_name_from_config(cs_pin_name);
  rt_pin_mode(cs_pin, PIN_MODE_OUTPUT);
  if (rt_hw_spi_device_attach(spi_bus_name, spi_slave_name, cs_pin) != RT_EOK) {
    return false;
  }

  rt_device_t dev = rt_device_find(spi_slave_name);
  if (dev == RT_NULL) {
    return false;
  }

  spi_device_ = (rt_spi_device *)dev;

  if (rt_device_open((rt_device_t)spi_device_, RT_DEVICE_OFLAG_RDWR) != RT_EOK) {
    return false;
  }

  return true;
}

bool SpiInterface::configure(rt_uint16_t mode, rt_uint32_t max_hz) {
  if (spi_device_ == RT_NULL) {
    return false;
  }
  struct rt_spi_configuration cfg;
  cfg.data_width = 8;
  cfg.mode = mode;
  cfg.max_hz = max_hz;
  return (rt_spi_configure(spi_device_, &cfg) == RT_EOK);
}

int SpiInterface::spi_read_reg_wrapper(uint8_t *cmd, uint8_t cmd_length, uint8_t *data, uint16_t data_len) {
  if (spi_device_ == RT_NULL) return -RT_ERROR;

  struct rt_spi_message msg1, msg2;
  msg1.send_buf = cmd;
  msg1.recv_buf = RT_NULL;
  msg1.length = cmd_length;
  msg1.cs_take = 1;
  msg1.cs_release = (data != RT_NULL && data_len > 0) ? 0 : 1;

  if (data != RT_NULL && data_len > 0) {
    msg1.next = &msg2;
    msg2.send_buf = RT_NULL;
    msg2.recv_buf = data;
    msg2.length = data_len;
    msg2.cs_take = 0;
    msg2.cs_release = 1;
    msg2.next = RT_NULL;
  } else {
    msg1.next = RT_NULL;
  }

  rt_spi_transfer_message(spi_device_, &msg1);
  return 0;
}

int SpiInterface::spi_write_reg_wrapper(uint8_t *cmd, uint8_t cmd_length, uint8_t *data, uint16_t data_len) {
  if (spi_device_ == RT_NULL) return -RT_ERROR;

  struct rt_spi_message msg1, msg2;
  msg1.send_buf = cmd;
  msg1.recv_buf = RT_NULL;
  msg1.length = cmd_length;
  msg1.cs_take = 1;
  msg1.cs_release = (data != RT_NULL && data_len > 0) ? 0 : 1;

  if (data != RT_NULL && data_len > 0) {
    msg1.next = &msg2;
    msg2.send_buf = data;
    msg2.recv_buf = RT_NULL;
    msg2.length = data_len;
    msg2.cs_take = 0;
    msg2.cs_release = 1;
    msg2.next = RT_NULL;
  } else {
    msg1.next = RT_NULL;
  }

  rt_spi_transfer_message(spi_device_, &msg1);
  return 0;
}

int SpiInterface::write_reg(uint8_t reg, uint8_t val) {
  uint8_t cmd = reg;
  return spi_write_reg_wrapper(&cmd, 1, &val, 1);
}

int SpiInterface::read_multi(uint8_t reg, uint8_t *buff, uint8_t len) {
  if (buff == RT_NULL || len == 0) return -1;
  uint8_t cmd = (uint8_t)(reg | 0x80u);
  return spi_read_reg_wrapper(&cmd, 1, buff, len);
}