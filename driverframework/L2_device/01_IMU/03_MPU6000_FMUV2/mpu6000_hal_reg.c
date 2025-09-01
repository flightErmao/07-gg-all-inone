#include "rtdevice.h"
#include "rtthread.h"
#include "imu.h"
#include "mpu6000.h"
#include <string.h>

#ifdef SENSOR_MPU6000_DEBUGPIN_EN
#include "debugPin.h"
#endif

#define IMU_CONFIGURE                                                                                  \
  {                                                                                                    \
      3200,           IMU_GYRO_MODE_NORMAL, 800, IMU_ACC_MODE_OSR2, GYRO_SCALE_2000DPS, ACC_SCALE_16G, \
      IMU_TEMP_SCALE, IMU_TEMP_OFFSET,                                                                 \
  }

#ifndef MPU6000_INT_EVENT_FLAG
#define MPU6000_INT_EVENT_FLAG (1u << 0)
#endif

#ifndef SENSOR_MPU6000_INT_PIN
#define SENSOR_MPU6000_INT_PIN SENSOR_MPU6000_INT_PIN
#endif

static struct rt_event mpu6000_int_event;
static rt_bool_t mpu6000_int_event_inited = RT_FALSE;

static void mpu6000_int_isr(void* parameter) {
#ifdef SENSOR_MPU6000_DEBUGPIN_EN
  DEBUG_PIN_DEBUG2_TOGGLE();
#endif
  rt_event_send(&mpu6000_int_event, MPU6000_INT_EVENT_FLAG);
}

static rt_err_t mpu6000_interrupt_event_init(void) {
  if (!mpu6000_int_event_inited) {
    rt_err_t err = rt_event_init(&mpu6000_int_event, "mpu6_e", RT_IPC_FLAG_FIFO);
    if (err != RT_EOK) {
      return err;
    }
    mpu6000_int_event_inited = RT_TRUE;
  }
  return RT_EOK;
}

static rt_err_t mpu6000_interrupt_gpio_init(void) {
  rt_pin_mode(SENSOR_MPU6000_INT_PIN, PIN_MODE_INPUT_PULLDOWN);
  rt_err_t err = rt_pin_attach_irq(SENSOR_MPU6000_INT_PIN, PIN_IRQ_MODE_RISING, mpu6000_int_isr, RT_NULL);
  if (err != RT_EOK) {
    return err;
  }
  err = rt_pin_irq_enable(SENSOR_MPU6000_INT_PIN, PIN_IRQ_ENABLE);
  if (err != RT_EOK) {
    return err;
  }
  return RT_EOK;
}

static rt_device_t spi_device;

static int8_t spiBusRead_wrap(uint8_t devAddress, uint8_t memAddress, uint8_t* data, uint8_t len) {
  // SPI读取实现
  rt_uint8_t cmd = 0x80 | memAddress; // 读命令，最高位为1
  rt_spi_send_then_recv((struct rt_spi_device*)spi_device, &cmd, 1, data, len);
  return 0;
}

static int8_t spiBusWrite_wrap(uint8_t devAddress, uint8_t memAddress, uint8_t* data, uint8_t len) {
  // SPI写入实现
  rt_uint8_t send_buffer[256]; // 假设最大256字节
  send_buffer[0] = memAddress; // 写命令，最高位为0
  if (len > 0) {
    memcpy(&send_buffer[1], data, len);
  }
  rt_device_write(spi_device, 0, send_buffer, len + 1);
  return 0;
}

static void delay_ms_wrap(unsigned int ms) { 
  rt_thread_mdelay(ms); 
}

static int8_t mpu6000_read_data(imu_dev_t imu, rt_off_t pos, void* data, rt_size_t size) {
  if (data == NULL) {
    return -RT_EINVAL;
  }

#ifdef SENSOR_MPU6000_DEBUGPIN_EN
  DEBUG_PIN_DEBUG0_HIGH();
#endif
  if (mpu6000_int_event_inited) {
    rt_event_recv(&mpu6000_int_event, MPU6000_INT_EVENT_FLAG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                  RT_WAITING_FOREVER, RT_NULL);
  }

#ifdef SENSOR_MPU6000_DEBUGPIN_EN
  DEBUG_PIN_DEBUG0_LOW();
  DEBUG_PIN_DEBUG1_HIGH();
#endif
  int8_t read_size = drv_mpu6000_read(pos, data, size);
#ifdef SENSOR_MPU6000_DEBUGPIN_EN
  DEBUG_PIN_DEBUG1_LOW();
#endif
  return read_size;
}

const static struct imu_ops mpu6000_dev = {
    .imu_config = NULL,
    .imu_read = mpu6000_read_data,
};

static struct imu_device imu_dev = {
    .ops = &mpu6000_dev,
    .config = IMU_CONFIGURE,
};

static rt_err_t mpu6000_init(const char* spi_device_name) {
  spi_device = rt_device_find(spi_device_name);
  if (spi_device == RT_NULL) {
    console_printf("spi device %s not found!\r\n", spi_device_name);
    return RT_EEMPTY;
  }

  /* config spi */
  {
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible Modes 3 */
    cfg.max_hz = 3000000;

    struct rt_spi_device* spi_device_t = (struct rt_spi_device*)spi_device;

    spi_device_t->config.data_width = cfg.data_width;
    spi_device_t->config.mode = cfg.mode & RT_SPI_MODE_MASK;
    spi_device_t->config.max_hz = cfg.max_hz;
    rt_spi_configure(spi_device_t, &cfg);
  }

  /* 打开SPI设备 */
  rt_device_open(spi_device, RT_DEVICE_OFLAG_RDWR);

  /* 设置驱动函数 */
  drv_mpu6000_set_spi_funcs(spiBusWrite_wrap, spiBusRead_wrap);
  drv_mpu6000_set_delay(delay_ms_wrap);

  /* 初始化MPU6000 */
  int ret = drv_mpu6000_init();
  if (ret != 0) {
    return RT_ERROR;
  }

  /* 初始化中断事件 */
  rt_err_t result = mpu6000_interrupt_event_init();
  if (result != RT_EOK) {
    return result;
  }
  
  /* 初始化中断GPIO */
  result = mpu6000_interrupt_gpio_init();
  if (result != RT_EOK) {
    return result;
  }

  /* 注册IMU设备 */
  hal_imu_register(&imu_dev, SENSOR_NAME_MPU6000, RT_DEVICE_FLAG_RDWR, RT_NULL);

  return RT_EOK;
}

static int mpu6000_init_auto(void) {
  return mpu6000_init(SENSOR_SPI_NAME_MPU6000);
}

#ifdef BSP_USING_MPU6000
INIT_COMPONENT_EXPORT(mpu6000_init_auto);
#endif