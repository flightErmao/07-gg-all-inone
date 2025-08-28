#include <drivers/dev_i2c.h>
#include "barometer.h"
#include "spl06.h"
#include "i2c_interface.h"

#define DBG_TAG "spl16_001"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

typedef struct {
  struct baro_device baro_dev;
  I2cInterface_t i2c_interface;
  struct spl06_t spl06;
} spl06_reg_t;

#ifdef SPL16001_ID1
static spl06_reg_t spl06_reg_id1 = {0};
#endif

static rt_err_t baro_control(baro_dev_t baro, int cmd, void* arg) { return RT_EOK; }

static rt_size_t baro_read(baro_dev_t baro, baro_report_t* report) {
  spl06_reg_t* reg = (spl06_reg_t*)baro;
  rt_size_t size = sizeof(baro_report_t);
  int32_t pressure = 0, temperature = 0;
  int8_t ret = spl06_read_pressure_temperature(&pressure, &temperature);
  report->timestamp_ms = (uint32_t)(rt_tick_get() * 1000 / RT_TICK_PER_SECOND);
  if (ret == 0) {
    report->pressure_Pa = (float)(pressure / 100.0f);
    report->temperature_deg = (float)(temperature / 100.0f);
    report->altitude_m = 0.0f;  // 可根据需要后续计算
  } else {
    report->pressure_Pa = 0.0f;
    report->temperature_deg = 0.0f;
    report->altitude_m = 0.0f;
  }
  return size;
}

static struct baro_ops _baro_ops = {.baro_control = baro_control, .baro_read = baro_read};

static int8_t i2c_read_reg8_pack(uint8_t device_addr, uint8_t register_addr, uint8_t* data, uint8_t len,
                                 spl06_reg_t* reg) {
  return i2c_read_reg8_mult_pack(reg->i2c_interface, register_addr, data, len);
}

static int8_t i2c_write_reg8_pack(uint8_t device_addr, uint8_t register_addr, uint8_t* data, uint8_t len,
                                  spl06_reg_t* reg) {
  return i2c_write_reg8_mult_pack(reg->i2c_interface, register_addr, data, len);
}

static void spl_delay_pack(uint16_t ms) { rt_thread_mdelay(ms); }

static rt_err_t lowlevel_init(spl06_reg_t* reg, uint8_t dev_addr) {
  reg->spl06.dev_addr = dev_addr;
  reg->spl06.bus_read = (int8_t (*)(uint8_t, uint8_t, uint8_t*, uint8_t))i2c_read_reg8_pack;
  reg->spl06.bus_write = (int8_t (*)(uint8_t, uint8_t, uint8_t*, uint8_t))i2c_write_reg8_pack;
  reg->spl06.delay_msec = spl_delay_pack;
  reg->baro_dev.ops = &_baro_ops;

  int ret_init = spl06_init(&reg->spl06);
  int ret_work_mode = spl06_set_work_mode(SPL06_LOW_POWER_MODE);
  int ret_power_mode = spl06_set_power_mode(SPL06_NORMAL_MODE);

  if (ret_init || ret_work_mode || ret_power_mode) {
    while (1) {
      rt_kprintf("sensor baro spl16_001 init failed\n");
      rt_thread_mdelay(2000);
    }
  }
  return RT_EOK;
}

rt_err_t drv_spl16_001_init(const char* i2c_device_name, const char* baro_device_name, uint8_t i2c_addr,
                            spl06_reg_t* reg) {
  RT_ASSERT(get_i2c_interface(i2c_device_name, i2c_addr, &reg->i2c_interface) == RT_EOK);
  RT_ASSERT(lowlevel_init(reg, i2c_addr) == RT_EOK);
  RT_ASSERT(hal_baro_register(&reg->baro_dev, baro_device_name, RT_DEVICE_FLAG_RDWR, RT_NULL) == RT_EOK);
  return RT_EOK;
}

#ifdef SPL16001_ID1
static int drv_spl16_001_reg_id1(void) {
  return drv_spl16_001_init(SPL16001_I2C_NAME_ID1, SPL16001_DEVICE_NAME_ID1, SPL16001_I2C_ADDRESS_ID1, &spl06_reg_id1);
}
INIT_COMPONENT_EXPORT(drv_spl16_001_reg_id1);
#endif