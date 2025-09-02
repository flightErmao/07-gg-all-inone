#include "spi_interface.hpp"

#include <string.h>

SpiInterface::SpiInterface() : _spi_device(RT_NULL) {}

SpiInterface::~SpiInterface() {}

rt_base_t SpiInterface::parse_pin_name_from_config(const char *pin_name) {
  if (pin_name == RT_NULL) {
    return (rt_base_t)(2 * 16 + 15);  // 默认PC15
  }

  const char *s = pin_name;
  if (s[0] == 'P' || s[0] == 'p') {
    s++;
  }

  if (s[0] == '\0' || !isalpha((unsigned char)s[0])) {
    return (rt_base_t)(2 * 16 + 15);
  }
  char port_char = (char)toupper((unsigned char)s[0]);
  s++;

  int pin_num = 0;
  int has_digit = 0;
  while (*s) {
    if (!isdigit((unsigned char)*s)) break;
    has_digit = 1;
    pin_num = pin_num * 10 + (*s - '0');
    s++;
  }
  if (!has_digit) {
    return (rt_base_t)(2 * 16 + 15);
  }

  if (port_char < 'A' || port_char > 'Z') {
    return (rt_base_t)(2 * 16 + 15);
  }
  int port_index = port_char - 'A';

  return (rt_base_t)(port_index * 16 + pin_num);
}

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

  _spi_device = (rt_spi_device *)dev;
  return true;
}

bool SpiInterface::configure(rt_uint16_t mode, rt_uint32_t max_hz) {
  if (_spi_device == RT_NULL) {
    return false;
  }
  struct rt_spi_configuration cfg;
  cfg.data_width = 8;
  cfg.mode = mode;
  cfg.max_hz = max_hz;
  _spi_device->config.data_width = cfg.data_width;
  _spi_device->config.mode = cfg.mode & RT_SPI_MODE_MASK;
  _spi_device->config.max_hz = cfg.max_hz;
  return (rt_spi_configure(_spi_device, &cfg) == RT_EOK);
}

rt_err_t SpiInterface::open(rt_uint16_t oflag) {
  if (_spi_device == RT_NULL) {
    return -RT_ERROR;
  }
  return rt_device_open((rt_device_t)_spi_device, oflag);
}

int SpiInterface::transfer(const uint8_t *tx_buffer, uint8_t *rx_buffer, size_t length) {
  if (_spi_device == RT_NULL) {
    return -RT_ERROR;
  }

  int total_transferred = 0;
  while (length > 0) {
    size_t chunk = length > SPI_INTERFACE_USERBUF_MAX ? SPI_INTERFACE_USERBUF_MAX : length;
    int ret = rt_spi_transfer(_spi_device, tx_buffer, rx_buffer, chunk);
    if (ret != (int)chunk) {
      return (total_transferred > 0) ? total_transferred : ret;
    }

    total_transferred += ret;
    length -= chunk;
    if (tx_buffer) {
      tx_buffer += chunk;
    }
    if (rx_buffer) {
      rx_buffer += chunk;
    }
  }
  return total_transferred;
}

int SpiInterface::write(const uint8_t *tx_buffer, size_t length) { return transfer(tx_buffer, RT_NULL, length); }

int SpiInterface::read(uint8_t *rx_buffer, size_t length) { return transfer(RT_NULL, rx_buffer, length); }

int SpiInterface::write_reg(uint8_t reg, uint8_t val) {
  uint8_t buf[2];
  buf[0] = reg;
  buf[1] = val;
  int ret = write(buf, 2);
  return (ret == 2) ? 0 : -1;
}

int SpiInterface::read_multi(uint8_t reg, uint8_t *buff, uint8_t len) {
  if (buff == RT_NULL) {
    return -1;
  }
  if ((size_t)(len + 1) > SPI_INTERFACE_USERBUF_MAX) {
    return -1;
  }
  uint8_t tx_buf[SPI_INTERFACE_USERBUF_MAX];
  uint8_t rx_buf[SPI_INTERFACE_USERBUF_MAX];
  tx_buf[0] = (uint8_t)(reg | 0x80u);
  memset(&tx_buf[1], 0xFF, len);
  int ret = transfer(tx_buf, rx_buf, (size_t)len + 1);
  if (ret != (int)((size_t)len + 1)) {
    return -1;
  }
  memcpy(buff, &rx_buf[1], len);
  return 0;
}

rt_spi_device *SpiInterface::device() const { return _spi_device; }