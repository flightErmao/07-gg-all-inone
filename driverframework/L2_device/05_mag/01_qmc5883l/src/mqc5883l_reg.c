#include "mqc5883l_reg.h"

#include <rtthread.h>

#include "QMC5883L.h"
#include "hal_mag.h"
#include "rtdevice.h"

#define DBG_TAG "mqc5883l"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define IIC_MAX_WIRTE_COUNT 512

static rt_device_t i2c_qmc5883l_dev = NULL;
static struct mag_device qmc5883l_dev;

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

static uint8_t qmc5883l_i2c_read_block_pack(uint8_t addr, uint8_t* data, uint8_t len) {
  uint8_t ret_func = 0;
  rt_err_t ret = i2cdevRead(i2c_qmc5883l_dev, SENSOR_QMC5883l_I2C_ADDR, addr, len, data);
  if (ret == RT_EOK) {
    ret_func = 1;
  }
  return ret_func;
}

static uint8_t qmc5883l_i2c_write_reg_pack(uint8_t addr, uint8_t data) {
  uint8_t ret_func = 0;
  rt_err_t ret = i2cdevWrite(i2c_qmc5883l_dev, SENSOR_QMC5883l_I2C_ADDR, addr, 1, &data);
  if (ret == RT_EOK) {
    ret_func = 1;
  }
  return ret_func;
}

static rt_err_t sensor_init(void) {
  rt_err_t ret_func = RT_EOK;

  qmc5883l_read_block = (mqc5883l_i2c_read_block)qmc5883l_i2c_read_block_pack;
  qmc5883l_write_reg = (mqc5883l_i2c_write_reg)qmc5883l_i2c_write_reg_pack;

  if (qmc5883l_init() == 0) {
    ret_func = RT_ERROR;
  }
  return ret_func;
}

static rt_err_t mag_control(mag_dev_t mag, int cmd, void* arg) { return RT_EOK; }

static rt_size_t mag_read(mag_dev_t mag, mag_report_t* report) {
  rt_size_t size = 0;
  float data[3] = {0};
  qmc5883l_read_mag_xyz((float*)data);
  report->value_x = data[0];
  report->value_y = data[1];
  report->value_z = data[2];
  size = sizeof(mag_report_t);
  return size;
}

static rt_err_t _i2c_device_init(const char* i2c_dev_name) {
  rt_err_t ret = RT_EOK;
  rt_device_t i2c_dev = RT_NULL;

  i2c_dev = rt_device_find(i2c_dev_name);
  if (i2c_dev == RT_NULL) {
    LOG_E("can't find %s device!", i2c_dev_name);
    return RT_ERROR;
  }
  RT_ASSERT(i2c_dev != NULL);
  RT_ASSERT(rt_device_open(i2c_dev, RT_DEVICE_OFLAG_RDWR) == RT_EOK);
  i2c_qmc5883l_dev = i2c_dev;

  return ret;
}

static struct mag_ops _mag_ops = {.mag_control = mag_control, .mag_read = mag_read};

rt_err_t drv_mqc5883l_init(const char* i2c_device_name, const char* device_name) {
  rt_err_t ret = RT_EOK;
  qmc5883l_dev.ops = &_mag_ops;

  RT_ASSERT(_i2c_device_init(i2c_device_name) != RT_ERROR);
  ret = sensor_init();
  if (ret == RT_EOK) {
    RT_ASSERT(hal_mag_register(&qmc5883l_dev, device_name, RT_DEVICE_FLAG_RDWR, RT_NULL) == RT_EOK);
  }

  return ret;
}

static int drv_mqc5883l_reg(void) { return drv_mqc5883l_init(SENSOR_I2C_NAME_QMC5883l, SENSOR_NAME_QMC5883l); }

#ifdef BSP_USING_MAG_QMC5883l
INIT_COMPONENT_EXPORT(drv_mqc5883l_reg);
#endif
