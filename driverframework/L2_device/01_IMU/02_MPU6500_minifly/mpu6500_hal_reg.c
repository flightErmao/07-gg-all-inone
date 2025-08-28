#include "rtdevice.h"
#include "rtthread.h"

#include "mpu6500_regs.h"
#include "mpu6500.hpp"
#include "I2cInterface.h"
#include "imu.h"
#include <drv_gpio.h>
#ifdef SENSOR_MPU6500_MINIFLY_DEBUGPIN_EN
#include "debugPin.h"
#endif

#define IMU_CONFIGURE                                                                                  \
  {                                                                                                    \
      3200,           IMU_GYRO_MODE_NORMAL, 800, IMU_ACC_MODE_OSR2, GYRO_SCALE_2000DPS, ACC_SCALE_16G, \
      IMU_TEMP_SCALE, IMU_TEMP_OFFSET,                                                                 \
  }

#ifndef mpu6500_INT_EVENT_FLAG
#define mpu6500_INT_EVENT_FLAG (1u << 0)
#endif

#ifndef SENSOR_mpu6500_MINIFLY_INT_PIN
#define SENSOR_mpu6500_MINIFLY_INT_PIN GET_PIN(A, 4)
#endif

static struct rt_event mpu6500_int_event;
static rt_bool_t mpu6500_int_event_inited = RT_FALSE;

static void mpu6500_int_isr(void* parameter) {
#ifdef SENSOR_MPU6500_MINIFLY_DEBUGPIN_EN
  DEBUG_PIN_DEBUG2_TOGGLE();
#endif
  rt_event_send(&mpu6500_int_event, mpu6500_INT_EVENT_FLAG);
}

static rt_err_t mpu6500_interrupt_event_init(void) {
  if (!mpu6500_int_event_inited) {
    rt_err_t err = rt_event_init(&mpu6500_int_event, "mpu6_e", RT_IPC_FLAG_FIFO);
    if (err != RT_EOK) {
      return err;
    }
    mpu6500_int_event_inited = RT_TRUE;
  }
  return RT_EOK;
}

static rt_err_t mpu6500_interrupt_gpio_init(void) {
  rt_pin_mode(SENSOR_mpu6500_MINIFLY_INT_PIN, PIN_MODE_INPUT_PULLDOWN);
  rt_err_t err = rt_pin_attach_irq(SENSOR_mpu6500_MINIFLY_INT_PIN, PIN_IRQ_MODE_RISING, mpu6500_int_isr, RT_NULL);
  if (err != RT_EOK) {
    return err;
  }
  err = rt_pin_irq_enable(SENSOR_mpu6500_MINIFLY_INT_PIN, PIN_IRQ_ENABLE);
  if (err != RT_EOK) {
    return err;
  }
  return RT_EOK;
}

static I2cInterface_t i2c_interface;

static int8_t i2cBusRead_wrap(uint8_t devAddress, uint8_t memAddress, uint8_t* data, uint8_t len) {
  return i2c_read_reg8_mult_pack(i2c_interface, memAddress, data, len);
}
static int8_t i2cBusWrite_wrap(uint8_t devAddress, uint8_t memAddress, uint8_t* data, uint8_t len) {
  return i2c_write_reg8_mult_pack(i2c_interface, memAddress, data, len);
}
static void delay_ms_wrap(unsigned int ms) { rt_thread_mdelay(ms); }

static int8_t mpu6500_read_data(imu_dev_t imu, rt_off_t pos, void* data, rt_size_t size) {
  if (data == NULL) {
    return -RT_EINVAL;
  }

#ifdef SENSOR_MPU6500_MINIFLY_DEBUGPIN_EN
  DEBUG_PIN_DEBUG0_HIGH();
#endif
  if (mpu6500_int_event_inited) {
    rt_event_recv(&mpu6500_int_event, mpu6500_INT_EVENT_FLAG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                  RT_WAITING_FOREVER, RT_NULL);
  }

#ifdef SENSOR_MPU6500_MINIFLY_DEBUGPIN_EN
  DEBUG_PIN_DEBUG0_LOW();
  DEBUG_PIN_DEBUG1_HIGH();
#endif
  int8_t read_size = drv_mpu6500_read(pos, data, size);
#ifdef SENSOR_MPU6500_MINIFLY_DEBUGPIN_EN
  DEBUG_PIN_DEBUG1_LOW();
#endif
  return read_size;
}

const static struct imu_ops mpu6500_dev = {
    .imu_config = NULL,
    .imu_read = mpu6500_read_data,
};

static struct imu_device imu_dev = {
    .ops = &mpu6500_dev,
    .config = IMU_CONFIGURE,
};

static rt_err_t mpu6500_init(const char* i2c_device_name, uint8_t i2c_addr) {
  rt_err_t result = get_i2c_interface(i2c_device_name, i2c_addr, &i2c_interface);
  if (result != RT_EOK) {
    return result;
  }

  drv_mpu6500_set_i2c_funcs(i2cBusWrite_wrap, i2cBusRead_wrap);
  drv_mpu6500_set_delay(delay_ms_wrap);

  int ret = drv_mpu6500_init();
  if (ret != 0) {
    return RT_ERROR;
  }

  result = mpu6500_interrupt_event_init();
  if (result != RT_EOK) {
    return result;
  }
  result = mpu6500_interrupt_gpio_init();
  if (result != RT_EOK) {
    return result;
  }

  hal_imu_register(&imu_dev, SENSOR_NAME_MPU6500_MINIFLY, RT_DEVICE_FLAG_RDWR, RT_NULL);

  return RT_EOK;
}

static int mpu6500_init_auto(void) {
  return mpu6500_init(SENSOR_I2C_NAME_MPU6500_MINIFLY, SENSOR_MPU6500_MINIFLY_I2C_ADDR);
}

#ifdef BSP_USING_MPU6500_MINIFLY
INIT_COMPONENT_EXPORT(mpu6500_init_auto);
#endif