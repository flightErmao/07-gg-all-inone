#include "spi_interface.hpp"
#include <string.h>
#include "pinInterface.h"

#undef LOG_TAG
#define LOG_TAG "spi.if"
#ifndef LOG_LVL
#define LOG_LVL LOG_LVL_INFO
#endif
#include <ulog.h>

SpiInterface::SpiInterface() : spi_device_(RT_NULL), cs_pin_(PIN_NONE) {}

SpiInterface::~SpiInterface() {}

bool SpiInterface::init(const char *spi_bus_name, const char *spi_slave_name, const char *cs_pin_name) {
  if (spi_bus_name == RT_NULL || spi_slave_name == RT_NULL) {
    LOG_E("spi init invalid args, bus=%p slave=%p", spi_bus_name, spi_slave_name);
    return false;
  }

  rt_base_t cs_pin = parse_pin_name_from_config(cs_pin_name);
  cs_pin_ = cs_pin;
  rt_pin_mode(cs_pin, PIN_MODE_OUTPUT);
  rt_pin_write(cs_pin, PIN_HIGH);
  
#ifdef SOC_FAMILY_AT32
  void *cs_gpio_x;
  uint16_t cs_gpio_pin;
  get_gpio_from_pin_index(cs_pin, &cs_gpio_x, &cs_gpio_pin);
  
  if (rt_hw_spi_device_attach(spi_bus_name, spi_slave_name, (gpio_type*)cs_gpio_x, cs_gpio_pin) != RT_EOK) {
    LOG_E("attach spi device failed, bus=%s dev=%s cs=%s", spi_bus_name, spi_slave_name,
          cs_pin_name ? cs_pin_name : "<null>");
    return false;
  }
#elif defined(SOC_FAMILY_STM32)
  if (rt_hw_spi_device_attach(spi_bus_name, spi_slave_name, cs_pin) != RT_EOK) {
    LOG_E("attach spi device failed, bus=%s dev=%s cs=%s(%d)", spi_bus_name, spi_slave_name,
          cs_pin_name ? cs_pin_name : "<null>", (int)cs_pin);
    return false;
  }
#else
  #error "Unsupported SOC family"
#endif

  rt_device_t dev = rt_device_find(spi_slave_name);
  if (dev == RT_NULL) {
    LOG_E("spi device not found after attach, dev=%s", spi_slave_name);
    return false;
  }

  spi_device_ = (rt_spi_device *)dev;

  if (rt_device_open((rt_device_t)spi_device_, RT_DEVICE_OFLAG_RDWR) != RT_EOK) {
    LOG_E("open spi device failed, dev=%s", spi_slave_name);
    return false;
  }

  return true;
}

bool SpiInterface::configure(rt_uint16_t mode, rt_uint32_t max_hz) {
  if (spi_device_ == RT_NULL) {
    LOG_E("configure before init");
    return false;
  }
  struct rt_spi_configuration cfg;
  cfg.data_width = 8;
  cfg.mode = mode;
  cfg.max_hz = max_hz;
  if (rt_spi_configure(spi_device_, &cfg) != RT_EOK) {
    LOG_E("spi configure failed, mode=0x%X hz=%u", mode, max_hz);
    return false;
  }

  // On STM32H7, HAL_SPI_Init re-runs MSP and may switch the NSS pin back to AF mode.
  // Re-force the attached CS pin to GPIO output so RT-Thread software CS can toggle it.
  if (cs_pin_ != PIN_NONE) {
    rt_pin_mode(cs_pin_, PIN_MODE_OUTPUT);
    if (mode & RT_SPI_CS_HIGH) {
      rt_pin_write(cs_pin_, PIN_LOW);
    } else {
      rt_pin_write(cs_pin_, PIN_HIGH);
    }
  }

  return true;
}

int SpiInterface::spi_read_reg_wrapper(uint8_t *cmd, uint8_t cmd_length, uint8_t *data, uint16_t data_len) {
  if (spi_device_ == RT_NULL) return -RT_ERROR;
  if (cmd == RT_NULL || cmd_length == 0 || data == RT_NULL || data_len == 0) return -RT_ERROR;

  uint8_t dummy_stack[32];
  uint8_t *dummy_tx = dummy_stack;
  rt_bool_t use_heap = RT_FALSE;

  if (data_len > sizeof(dummy_stack)) {
    dummy_tx = static_cast<uint8_t *>(rt_malloc(data_len));
    if (dummy_tx == RT_NULL) {
      LOG_E("alloc dummy tx failed, len=%d", data_len);
      return -RT_ENOMEM;
    }
    use_heap = RT_TRUE;
  }

  memset(dummy_tx, 0xFF, data_len);

  struct rt_spi_message msg1, msg2;
  msg1.send_buf = cmd;
  msg1.recv_buf = RT_NULL;
  msg1.length = cmd_length;
  msg1.cs_take = 1;
  msg1.cs_release = 0;
  msg1.next = &msg2;

  msg2.send_buf = dummy_tx;
  msg2.recv_buf = data;
  msg2.length = data_len;
  msg2.cs_take = 0;
  msg2.cs_release = 1;
  msg2.next = RT_NULL;

  if (rt_spi_transfer_message(spi_device_, &msg1) != RT_NULL) {
    LOG_E("spi read transfer failed, cmd_len=%d data_len=%d", cmd_length, data_len);
    if (use_heap) {
      rt_free(dummy_tx);
    }
    return -RT_ERROR;
  }

  if (use_heap) {
    rt_free(dummy_tx);
  }

  return RT_EOK;
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

  if (rt_spi_transfer_message(spi_device_, &msg1) != RT_NULL) {
    LOG_E("spi write transfer failed, cmd_len=%d data_len=%d", cmd_length, data_len);
    return -RT_ERROR;
  }
  return RT_EOK;
}

int SpiInterface::write_reg(uint8_t reg, uint8_t val) {
  uint8_t cmd = reg;
  return spi_write_reg_wrapper(&cmd, 1, &val, 1);
}

int SpiInterface::readMultiReg8(uint8_t reg, uint8_t *buff, uint8_t len) {
  if (buff == RT_NULL || len == 0) return -1;
  uint8_t cmd = (uint8_t)(reg | 0x80u);
  return spi_read_reg_wrapper(&cmd, 1, buff, len);
}

int SpiInterface::readMultiReg16(uint8_t reg, uint8_t *buff, uint8_t len) {
  if (buff == RT_NULL || len == 0) return -1;
  uint8_t cmd[2] = {0};
  cmd[0] = (uint8_t)(reg | 0x80u);
  return spi_read_reg_wrapper(cmd, 2, buff, len);
}

int SpiInterface::writeMultiReg8(uint8_t reg, uint8_t *buff, uint16_t len) {
  if (buff == RT_NULL || len == 0) return -1;
  uint8_t cmd = reg;
  return spi_write_reg_wrapper(&cmd, 1, buff, len);
}

int SpiInterface::transfer(uint8_t *send_buf, uint8_t *recv_buf, uint16_t len) {
  if (spi_device_ == RT_NULL || len == 0) {
    return -RT_ERROR;
  }

  rt_size_t transferred = rt_spi_transfer(spi_device_, send_buf, recv_buf, len);
  return (transferred == len) ? RT_EOK : -RT_ERROR;
}
