#include "I2cInterface.h"
#include <string.h>

#define IIC_MAX_WRITE_COUNT 512

// internal prototypes
static rt_err_t i2cdevRead(rt_device_t bus, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t* data);
static rt_err_t i2cdevWrite(rt_device_t bus, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t* data);
static rt_err_t i2cdevRead16(rt_device_t bus, uint8_t devAddress, uint16_t memAddress, uint16_t len, uint8_t* data);
static rt_err_t i2cdevWrite16(rt_device_t bus, uint8_t devAddress, uint16_t memAddress, uint16_t len, uint8_t* data);

// 8-bit register R/W
static rt_err_t i2cdevRead(rt_device_t bus, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t* data) {
  struct rt_i2c_msg msg[2] = {0};
  RT_ASSERT(bus != RT_NULL);

  msg[0].addr = devAddress;
  msg[0].flags = RT_I2C_WR;
  msg[0].len = 1;
  msg[0].buf = &memAddress;

  msg[1].addr = devAddress;
  msg[1].flags = RT_I2C_RD;
  msg[1].len = len;
  msg[1].buf = data;

  if (rt_i2c_transfer((struct rt_i2c_bus_device*)bus, msg, 2) != 2) {
    rt_kprintf("I2C read data failed, reg = 0x%02x.\n", memAddress);
    return RT_ERROR;
  }
  return RT_EOK;
}

static rt_err_t i2cdevWrite(rt_device_t bus, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t* data) {
  struct rt_i2c_msg msgs[1] = {0};
  if (len > IIC_MAX_WRITE_COUNT - 1) {
    rt_kprintf("[i2cdevWrite] too large len in once write\n");
    return RT_ERROR;
  }
  rt_uint8_t buff[IIC_MAX_WRITE_COUNT] = {0};

  RT_ASSERT(bus != RT_NULL);

  buff[0] = memAddress;
  rt_memcpy(&buff[1], data, len);

  msgs[0].addr = devAddress;
  msgs[0].flags = RT_I2C_WR;
  msgs[0].buf = buff;
  msgs[0].len = len + 1;

  if (rt_i2c_transfer((struct rt_i2c_bus_device*)bus, msgs, 1) != 1) {
    rt_kprintf("I2C write data failed, reg = 0x%02x.\n", memAddress);
    return RT_ERROR;
  }
  return RT_EOK;
}

// 16-bit register R/W
static rt_err_t i2cdevRead16(rt_device_t bus, uint8_t devAddress, uint16_t memAddress, uint16_t len, uint8_t* data) {
  struct rt_i2c_msg msg[2] = {0};
  rt_uint8_t addr_buf[2] = {0};
  RT_ASSERT(bus != RT_NULL);

  addr_buf[0] = (memAddress >> 8) & 0xFF;
  addr_buf[1] = memAddress & 0xFF;

  msg[0].addr = devAddress;
  msg[0].flags = RT_I2C_WR;
  msg[0].len = 2;
  msg[0].buf = addr_buf;

  msg[1].addr = devAddress;
  msg[1].flags = RT_I2C_RD;
  msg[1].len = len;
  msg[1].buf = data;

  if (rt_i2c_transfer((struct rt_i2c_bus_device*)bus, msg, 2) != 2) {
    rt_kprintf("I2C read data failed, reg = 0x%04x.\n", memAddress);
    return RT_ERROR;
  }
  return RT_EOK;
}

static rt_err_t i2cdevWrite16(rt_device_t bus, uint8_t devAddress, uint16_t memAddress, uint16_t len, uint8_t* data) {
  struct rt_i2c_msg msgs[1] = {0};
  if (len > IIC_MAX_WRITE_COUNT - 2) {
    rt_kprintf("[i2cdevWrite16] too large len in once write\n");
    return RT_ERROR;
  }
  rt_uint8_t buff[IIC_MAX_WRITE_COUNT] = {0};

  RT_ASSERT(bus != RT_NULL);

  buff[0] = (memAddress >> 8) & 0xFF;
  buff[1] = memAddress & 0xFF;
  rt_memcpy(&buff[2], data, len);

  msgs[0].addr = devAddress;
  msgs[0].flags = RT_I2C_WR;
  msgs[0].buf = buff;
  msgs[0].len = len + 2;

  if (rt_i2c_transfer((struct rt_i2c_bus_device*)bus, msgs, 1) != 1) {
    rt_kprintf("I2C write data failed, reg = 0x%04x.\n", memAddress);
    return RT_ERROR;
  }
  return RT_EOK;
}

// public API
int8_t i2c_read_reg8_mult_pack(I2cInterface_t i2c_interface, uint8_t register_addr, uint8_t* data, uint8_t len) {
  return i2cdevRead(i2c_interface.i2c_dev, i2c_interface.i2c_addr, register_addr, len, data);
}

int8_t i2c_write_reg8_mult_pack(I2cInterface_t i2c_interface, uint8_t register_addr, uint8_t* data, uint8_t len) {
  return i2cdevWrite(i2c_interface.i2c_dev, i2c_interface.i2c_addr, register_addr, len, data);
}

int8_t i2c_read_reg16_mult_pack(I2cInterface_t i2c_interface, uint16_t register_addr, uint8_t* data, uint8_t len) {
  return i2cdevRead16(i2c_interface.i2c_dev, i2c_interface.i2c_addr, register_addr, len, data);
}

int8_t i2c_write_reg16_mult_pack(I2cInterface_t i2c_interface, uint16_t register_addr, uint8_t* data, uint8_t len) {
  return i2cdevWrite16(i2c_interface.i2c_dev, i2c_interface.i2c_addr, register_addr, len, data);
}

rt_err_t get_i2c_interface(const char* i2c_device_name, uint8_t i2c_addr, I2cInterface_t* i2c_interface) {
  i2c_interface->i2c_dev = rt_device_find(i2c_device_name);
  RT_ASSERT(i2c_interface->i2c_dev != RT_NULL);
  RT_ASSERT(rt_device_open(i2c_interface->i2c_dev, RT_DEVICE_OFLAG_RDWR) == RT_EOK);
  i2c_interface->i2c_addr = i2c_addr;

  if (strstr(i2c_device_name, "i2c1") != RT_NULL || strstr(i2c_device_name, "I2C1") != RT_NULL) {
#ifdef BSP_I2C1_SPEED
    set_i2c_speed(i2c_interface->i2c_dev, BSP_I2C1_SPEED);
#endif
  } else if (strstr(i2c_device_name, "i2c2") != RT_NULL || strstr(i2c_device_name, "I2C2") != RT_NULL) {
#ifdef BSP_I2C2_SPEED
    set_i2c_speed(i2c_interface->i2c_dev, BSP_I2C2_SPEED);
#endif
  }

  return RT_EOK;
}

rt_err_t set_i2c_speed(rt_device_t i2c_dev, uint32_t speed) {
  RT_ASSERT(i2c_dev != RT_NULL);

  I2cSpeedConfig_t config = {
      .speed = speed,
      .duty_cycle = 0,
      .timeout_ms = 1000
  };

  return configure_i2c_speed(i2c_dev, &config);
}

rt_err_t configure_i2c_speed(rt_device_t i2c_dev, const I2cSpeedConfig_t* config) {
  RT_ASSERT(i2c_dev != RT_NULL);
  RT_ASSERT(config != RT_NULL);

  rt_kprintf("Configuring I2C speed to %d Hz (%.1f KHz)\n", config->speed, config->speed / 1000.0f);

  struct rt_i2c_bus_device* i2c_bus = (struct rt_i2c_bus_device*)i2c_dev;

  if (i2c_bus && i2c_bus->ops && i2c_bus->ops->i2c_bus_control) {
    /* use hardware driver control to apply speed */
    rt_err_t ret = i2c_bus->ops->i2c_bus_control(i2c_bus, 0x01, (void*)&config->speed);
    if (ret == RT_EOK) {
      rt_kprintf("I2C speed set successfully via hardware driver: %d Hz\n", config->speed);
    } else {
      rt_kprintf("Failed to set I2C speed via hardware driver: %d\n", ret);
      return ret;
    }
  } else {
    rt_kprintf("I2C device does not support control interface\n");
    return -RT_ENOSYS;
  }

  return RT_EOK;
}

rt_err_t set_i2c_standard_speed(rt_device_t i2c_dev) {
  return set_i2c_speed(i2c_dev, I2C_IF_SPEED_STANDARD);
}

rt_err_t set_i2c_fast_speed(rt_device_t i2c_dev) {
  return set_i2c_speed(i2c_dev, I2C_IF_SPEED_FAST);
}

rt_err_t set_i2c_fast_plus_speed(rt_device_t i2c_dev) {
  return set_i2c_speed(i2c_dev, I2C_IF_SPEED_FAST_PLUS);
}

rt_err_t set_i2c_high_speed(rt_device_t i2c_dev) {
  return set_i2c_speed(i2c_dev, I2C_IF_SPEED_HIGH);
}

// test helper remains unchanged