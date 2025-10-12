#include "rtdevice.h"
#include "rtthread.h"

#include "spl06001.hpp"
#include "spl06001HalConf.h"
#include "I2cInterface.h"
#include "barometer.h"

static I2cInterface_t i2c_interface;

static int8_t i2cBusRead_wrap(uint8_t devAddress, uint8_t memAddress, uint8_t* data, uint8_t len) {
  return i2c_read_reg8_mult_pack(i2c_interface, memAddress, data, len);
}
static int8_t i2cBusWrite_wrap(uint8_t devAddress, uint8_t memAddress, uint8_t* data, uint8_t len) {
  return i2c_write_reg8_mult_pack(i2c_interface, memAddress, data, len);
}
static void delay_ms_wrap(unsigned int ms) { rt_thread_mdelay(ms); }

static rt_err_t baro_control(baro_dev_t baro, int cmd, void* arg) { return RT_EOK; }

static rt_size_t baro_read(baro_dev_t baro, baro_report_t* report) {
  double p_pa = 0.0, t_deg = 0.0;
  if (spl06_001::instance().getPressureTemperatureDouble(&p_pa, &t_deg) != 0) return 0;
  report->timestamp_ms = rt_tick_get();
  report->pressure_Pa = (float)p_pa;
  report->temperature_deg = (float)t_deg;
  return sizeof(baro_report_t);
}

static struct baro_ops _baro_ops = {.baro_control = baro_control, .baro_read = baro_read};

rt_err_t spl06_001_hal_init(const char* i2c_device_name, uint8_t i2c_addr) {
  static struct baro_device baro_dev = {.ops = &_baro_ops};
  rt_err_t result = get_i2c_interface(i2c_device_name, i2c_addr, &i2c_interface);
  if (result != RT_EOK) {
    return result;
  }

  spl06_001::BusOps ops{};
  ops.bus_read = i2cBusRead_wrap;
  ops.bus_write = i2cBusWrite_wrap;
  ops.delay_ms = delay_ms_wrap;
  ops.dev_addr = i2c_addr;

  if (spl06_001::instance().setBus(ops) != 0) {
    return RT_ERROR;
  }
  if (spl06_001::instance().probe() != 0) {
    return RT_ERROR;
  }

  if (spl06_001::instance().init(_OSR_P, _MR_P, _OSR_T, _MR_T) != 0) {
    return RT_ERROR;
  }

  if (hal_baro_register(&baro_dev, SENSOR_SPL06_001_DEVICE_NAME, RT_DEVICE_FLAG_RDWR, RT_NULL) != RT_EOK) {
    return RT_ERROR;
  }

  return RT_EOK;
}

static int spl06_001_init_auto(void) {
  return spl06_001_hal_init(SENSOR_SPL06_001_I2C_NAME, SENSOR_SPL06_001_I2C_ADDR);
}

#ifdef BSP_USING_SPL06_001
INIT_COMPONENT_EXPORT(spl06_001_init_auto);
#endif
