#include "i2c_interface.h"

#define IIC_MAX_WIRTE_COUNT 512

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
    rt_kprintf("I2C read data failed, reg = 0x%02x. \n", memAddress);
    return RT_ERROR;
  }
  return RT_EOK;
}

static rt_err_t i2cdevWrite(rt_device_t bus, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t* data) {
  struct rt_i2c_msg msgs[1] = {0};
  if (len > IIC_MAX_WIRTE_COUNT - 1) {
    rt_kprintf("[i2cdevWrite] too large len in once write\n");
    return RT_ERROR;
  }
  rt_uint8_t buff[IIC_MAX_WIRTE_COUNT] = {0};

  RT_ASSERT(bus != RT_NULL);

  buff[0] = memAddress;
  rt_memcpy(&buff[1], data, len);

  msgs[0].addr = devAddress;
  msgs[0].flags = RT_I2C_WR;
  msgs[0].buf = buff;
  msgs[0].len = len + 1;

  if (rt_i2c_transfer((struct rt_i2c_bus_device*)bus, msgs, 1) != 1) {
    rt_kprintf("I2C write data failed, reg = 0x%2x. \n", memAddress);
    return RT_ERROR;
  }
  return RT_EOK;
}

int8_t i2c_read_reg8_mult_pack(I2cInterface_t i2c_interface, uint8_t register_addr, uint8_t* data, uint8_t len) {
  int8_t ret_function = i2cdevRead(i2c_interface.i2c_dev, i2c_interface.i2c_addr, register_addr, len, data);
  return ret_function;
}

int8_t i2c_write_reg8_mult_pack(I2cInterface_t i2c_interface, uint8_t register_addr, uint8_t* data, uint8_t len) {
  int8_t ret_function = i2cdevWrite(i2c_interface.i2c_dev, i2c_interface.i2c_addr, register_addr, len, data);
  return ret_function;
}

rt_err_t get_i2c_interface(const char* i2c_device_name, uint8_t i2c_addr, I2cInterface_t* i2c_interface) {
  i2c_interface->i2c_dev = rt_device_find(i2c_device_name);
  RT_ASSERT(i2c_interface->i2c_dev != RT_NULL);
  RT_ASSERT(rt_device_open(i2c_interface->i2c_dev, RT_DEVICE_OFLAG_RDWR) == RT_EOK);
  i2c_interface->i2c_addr = i2c_addr;
  return RT_EOK;
}
