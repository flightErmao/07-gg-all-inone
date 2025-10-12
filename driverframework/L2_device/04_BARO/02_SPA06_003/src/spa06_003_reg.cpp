#include <rtthread.h>

#include "GoertekBaro_spa06_003.hpp"
#include "barometer.h"
// #include "systime.h"

#define DBG_TAG "spa06_003"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <new>

#define IIC_MAX_WIRTE_COUNT 512

typedef struct {
  rt_device_t i2c_dev;
  GoertekBaro spa06_003;
  struct baro_device baro_dev;
} spa06_003_t;

#ifdef SENSOR_SPA06_003_NUM1
static spa06_003_t spa06_003_num1_;
#endif
#ifdef SENSOR_SPA06_003_NUM2
static spa06_003_t spa06_003_num2_;
#endif

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

#ifdef SENSOR_SPA06_003_NUM1
static bool i2c_wirte_reg8_one_pack_num1(uint8_t register_addr, uint8_t data) {
  rt_err_t ret_function = i2cdevWrite(spa06_003_num1_.i2c_dev, SENSOR_SPA06_003_NUM1_I2C_ADDR, register_addr, 1, &data);
  bool ret_fun = false;
  if (ret_function == RT_EOK) {
    ret_fun = true;
  }
  return ret_fun;
}
static bool i2c_read_reg8_one_pack_num1(uint8_t register_addr, uint8_t* data) {
  rt_err_t ret_read = i2cdevRead(spa06_003_num1_.i2c_dev, SENSOR_SPA06_003_NUM1_I2C_ADDR, register_addr, 1, data);
  bool ret_fun = false;
  if (ret_read == RT_EOK) {
    ret_fun = true;
  }
  return ret_fun;
}
static bool i2c_read_reg8_mult_pack_num1(uint8_t register_addr, uint8_t* data, uint8_t len) {
  rt_err_t ret_read = i2cdevRead(spa06_003_num1_.i2c_dev, SENSOR_SPA06_003_NUM1_I2C_ADDR, register_addr, len, data);
  bool ret_fun = false;
  if (ret_read == RT_EOK) {
    ret_fun = true;
  }
  return ret_fun;
}
#endif

#ifdef SENSOR_SPA06_003_NUM2
static bool i2c_wirte_reg8_one_pack_num2(uint8_t register_addr, uint8_t data) {
  bool ret_fun = false;
  rt_err_t ret_function = i2cdevWrite(spa06_003_num2_.i2c_dev, SENSOR_SPA06_003_NUM2_I2C_ADDR, register_addr, 1, &data);
  if (ret_function == RT_EOK) {
    ret_fun = true;
  }
  return ret_fun;
}
static bool i2c_read_reg8_one_pack_num2(uint8_t register_addr, uint8_t* data) {
  rt_err_t ret_read = i2cdevRead(spa06_003_num2_.i2c_dev, SENSOR_SPA06_003_NUM2_I2C_ADDR, register_addr, 1, data);
  bool ret_fun = false;
  if (ret_read == RT_EOK) {
    ret_fun = true;
  }
  return ret_fun;
}
static bool i2c_read_reg8_mult_pack_num2(uint8_t register_addr, uint8_t* data, uint8_t len) {
  rt_err_t ret_read = i2cdevRead(spa06_003_num2_.i2c_dev, SENSOR_SPA06_003_NUM2_I2C_ADDR, register_addr, len, data);
  bool ret_fun = false;
  if (ret_read == RT_EOK) {
    ret_fun = true;
  }
  return ret_fun;
}
#endif

static void delay_ms_pack(uint16_t ms) { rt_thread_mdelay(ms); }

static rt_err_t lowlevel_init(uint8_t id) {
  rt_err_t fun_ret = RT_ERROR;

  if (id == 0) {
#ifdef SENSOR_SPA06_003_NUM1
    // Ensure C++ constructor is executed even if global constructors are not run
    new (&spa06_003_num1_.spa06_003) GoertekBaro();
    spa06_003_num1_.spa06_003.funcRegisterI2c(i2c_read_reg8_one_pack_num1, i2c_wirte_reg8_one_pack_num1,
                                              i2c_read_reg8_mult_pack_num1);
    spa06_003_num1_.spa06_003.funRegisterTimer(delay_ms_pack);
    bool ret = spa06_003_num1_.spa06_003.Init();
    if (ret == true) {
      fun_ret = RT_EOK;
    }
#endif
  } else if (id == 1) {
#ifdef SENSOR_SPA06_003_NUM2
    // Ensure C++ constructor is executed even if global constructors are not run
    new (&spa06_003_num2_.spa06_003) GoertekBaro();
    spa06_003_num2_.spa06_003.funcRegisterI2c(i2c_read_reg8_one_pack_num2, i2c_wirte_reg8_one_pack_num2,
                                              i2c_read_reg8_mult_pack_num2);
    spa06_003_num2_.spa06_003.funRegisterTimer(delay_ms_pack);
    bool ret = spa06_003_num2_.spa06_003.Init();
    if (ret == true) {
      fun_ret = RT_EOK;
    }
#endif
  }

  return fun_ret;
}

static rt_err_t baro_control(baro_dev_t baro, int cmd, void* arg) { return RT_EOK; }

// static float _baro_presure_to_height(float baro_pres, float baro_temp, float& prev_baro_pres) {
//   constexpr float baro_sea_level = 101325.f;
//   constexpr float coeff = baro_sea_level / (1.293 * 273.15 * 9.80665);
//   float air_temperature = baro_temp;
//   float meters_per_pa = (air_temperature + 273.15f) / baro_pres * coeff;
//   float height = (baro_pres - baro_sea_level) * meters_per_pa;
//   height += (baro_pres - prev_baro_pres) * meters_per_pa;
//   prev_baro_pres = baro_pres;
//   return height;
// }

static rt_size_t baro_read(baro_dev_t baro, baro_report_t* report) {
  rt_size_t size = 0;
  BaroData spa06_003_data_temp;

  if (baro->id == 0) {
#ifdef SENSOR_SPA06_003_NUM1
    // static float prev_baro_pres_num1 = 0;
    spa06_003_num1_.spa06_003.Read(spa06_003_data_temp);
    // report->altitude_m = _baro_presure_to_height(spa06_003_data_temp.pressure_pa,
    //                                              spa06_003_data_temp.temperature_celsius, prev_baro_pres_num1);
#endif
  }

  else if (baro->id == 1) {
#ifdef SENSOR_SPA06_003_NUM2
    // static float prev_baro_pres_num2 = 0;
    spa06_003_num2_.spa06_003.Read(spa06_003_data_temp);
    // report->altitude_m = _baro_presure_to_height(spa06_003_data_temp.pressure_pa,
    //                                              spa06_003_data_temp.temperature_celsius, prev_baro_pres_num2);
#endif
  } else {
    return 0;
  }

  report->timestamp_ms = spa06_003_data_temp.timestamp_ms;
  report->temperature_deg = spa06_003_data_temp.temperature_celsius;
  report->pressure_Pa = spa06_003_data_temp.pressure_pa;
  size = sizeof(baro_report_t);
  return size;
}

static struct baro_ops _baro_ops = {.baro_control = baro_control, .baro_read = baro_read};

static rt_err_t _i2c_dev_init(const char* i2c_dev_name, uint8_t id) {
  rt_err_t ret = RT_EOK;
  rt_device_t i2c_dev = RT_NULL;

  i2c_dev = rt_device_find(i2c_dev_name);
  if (i2c_dev == RT_NULL) {
    LOG_E("can't find %s device!", i2c_dev_name);
    return RT_ERROR;
  }
  RT_ASSERT(i2c_dev != NULL);
  RT_ASSERT(rt_device_open(i2c_dev, RT_DEVICE_OFLAG_RDWR) == RT_EOK);

  if (id == 0) {
#ifdef SENSOR_SPA06_003_NUM1
    spa06_003_num1_.i2c_dev = i2c_dev;
#endif
  } else if (id == 1) {
#ifdef SENSOR_SPA06_003_NUM2
    spa06_003_num2_.i2c_dev = i2c_dev;
#endif
  }

  return ret;
}

static void _drv_hal_reg(uint8_t id, const char* baro_device_name) {
  if (id == 0) {
#ifdef SENSOR_SPA06_003_NUM1
    spa06_003_num1_.baro_dev.ops = &_baro_ops;
    spa06_003_num1_.baro_dev.id = id;
    RT_ASSERT(hal_baro_register(&spa06_003_num1_.baro_dev, baro_device_name, RT_DEVICE_FLAG_RDWR, RT_NULL) == RT_EOK);
#endif
  } else if (id == 1) {
#ifdef SENSOR_SPA06_003_NUM2
    spa06_003_num2_.baro_dev.ops = &_baro_ops;
    spa06_003_num2_.baro_dev.id = id;
    RT_ASSERT(hal_baro_register(&spa06_003_num2_.baro_dev, baro_device_name, RT_DEVICE_FLAG_RDWR, RT_NULL) == RT_EOK);
#endif
  }
}

rt_err_t drv_spa06_003_init(const char* i2c_dev_ice_name, const char* baro_device_name, uint8_t id) {
  RT_ASSERT(_i2c_dev_init(i2c_dev_ice_name, id) == RT_EOK);
  /* device low-level initialization */
  RT_ASSERT(lowlevel_init(id) == RT_EOK);
  /* register barometer device */
  _drv_hal_reg(id, baro_device_name);

  return RT_EOK;
}

static int _drv_spa06_003_reg(void) {
#ifdef SENSOR_SPA06_003_NUM1
  drv_spa06_003_init(SENSOR_SPA06_003_NUM1_I2C_NAME, SENSOR_SPA06_003_NUM1_DEVICE_NAME, SENSOR_SPA06_003_NUM1_ID);
#endif

#ifdef SENSOR_SPA06_003_NUM2
  drv_spa06_003_init(SENSOR_SPA06_003_NUM2_I2C_NAME, SENSOR_SPA06_003_NUM2_DEVICE_NAME, SENSOR_SPA06_003_NUM2_ID);
#endif
  return 0;
}

#ifdef BSP_USING_SPA06_003
INIT_COMPONENT_EXPORT(_drv_spa06_003_reg);
#endif
