#include "mqc6308_reg.h"

#include <rtthread.h>

#include "mag.h"
#include "qmc6308.h"
#include "QMC6308.hpp"
#include "I2cInterface.h"
#include "rtdevice.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DBG_TAG "mqc6308"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static struct mag_device qmc6308_dev;
static QMC6308* qmc6308_instance = NULL;
static I2cInterface_t i2c_interface;

static rt_err_t sensor_init(const char* i2c_device_name, uint8_t i2c_addr) {
  rt_err_t ret_func = RT_EOK;

  /* Initialize I2C interface */
  rt_err_t result = get_i2c_interface(i2c_device_name, i2c_addr, &i2c_interface);
  if (result != RT_EOK) {
    LOG_E("Failed to get I2C interface: %s, addr: 0x%02X", i2c_device_name, i2c_addr);
    return RT_ERROR;
  }

  /* Create QMC6308 instance */
  qmc6308_instance = new QMC6308(&i2c_interface);
  if (qmc6308_instance == NULL) {
    LOG_E("Failed to create QMC6308 instance");
    return RT_ERROR;
  }

  /* Initialize the sensor */
  if (qmc6308_instance->init() == 0) {
    LOG_E("QMC6308 initialization failed");
    delete qmc6308_instance;
    qmc6308_instance = NULL;
    ret_func = RT_ERROR;
  }

  return ret_func;
}

static rt_err_t mag_control(mag_dev_t mag, int cmd, void* arg) { return RT_EOK; }

static rt_size_t mag_read(mag_dev_t mag, mag_report_t* report) {
  rt_size_t size = 0;

  if (qmc6308_instance == NULL) {
    return 0;
  }

  int16_t data[3] = {0};
  if (qmc6308_instance->readMagXYZ(data) == 0) {
    return 0;
  }

  report->value_x = (float)data[0];
  report->value_y = (float)data[1];
  report->value_z = (float)data[2];
  size = sizeof(mag_report_t);
  return size;
}

static struct mag_ops _mag_ops = {.mag_control = mag_control, .mag_read = mag_read};

rt_err_t drv_mqc6308_init(const char* i2c_device_name, const char* device_name) {
  rt_err_t ret = RT_EOK;
  qmc6308_dev.ops = &_mag_ops;

  ret = sensor_init(i2c_device_name, SENSOR_QMC6308_I2C_ADDR);
  if (ret == RT_EOK && qmc6308_instance != NULL) {
    /* Initialize device configuration from QMC6308 instance */
    const qmc6308_config_t& config = qmc6308_instance->getConfig();
    qmc6308_dev.config.range_g = config.range_g;
    qmc6308_dev.config.odr_hz = config.odr_hz;
    qmc6308_dev.config.lsb = config.lsb;
    
    LOG_I("QMC6308 config: range=%dG, ODR=%dHz, LSB=%.3f uT/LSB", 
          qmc6308_dev.config.range_g, 
          qmc6308_dev.config.odr_hz, 
          qmc6308_dev.config.lsb);
    
    RT_ASSERT(hal_mag_register(&qmc6308_dev, device_name, RT_DEVICE_FLAG_RDWR, RT_NULL) == RT_EOK);
  }

  return ret;
}

static int drv_mqc6308_reg(void) { return drv_mqc6308_init(SENSOR_I2C_NAME_QMC6308, SENSOR_NAME_QMC6308); }

#ifdef BSP_USING_MAG_QMC6308
INIT_COMPONENT_EXPORT(drv_mqc6308_reg);
#endif

#ifdef __cplusplus
}
#endif
