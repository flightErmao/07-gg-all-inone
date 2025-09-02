#include "rtdevice.h"
#include "rtthread.h"
#include "imu.h"
#include "mpu6000.hpp"
#include <string.h>

#include "spi_interface.hpp"

#ifdef SENSOR_MPU6000_DEBUGPIN_EN
#include "debugPin.h"
#endif

#ifndef MPU6000_INT_EVENT_FLAG
#define MPU6000_INT_EVENT_FLAG (1u << 0)
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

static SpiInterface g_spi;  // 全局静态 SPI 对象
static rt_device_t spi_device;

static int8_t spiBusRead_wrap(uint8_t devAddress, uint8_t memAddress, uint8_t* data, uint8_t len) {
  (void)devAddress;
  if ((size_t)(len + 1) > SPI_INTERFACE_USERBUF_MAX) {
    return -1;
  }
  uint8_t tx_buf[SPI_INTERFACE_USERBUF_MAX];
  uint8_t rx_buf[SPI_INTERFACE_USERBUF_MAX];
  tx_buf[0] = (rt_uint8_t)(0x80 | memAddress);
  memset(&tx_buf[1], 0xFF, len);
  int ret = g_spi.transfer(tx_buf, rx_buf, (size_t)len + 1);
  if (ret != (int)((size_t)len + 1)) {
    return -1;
  }
  memcpy(data, &rx_buf[1], len);
  return 0;
}

static int8_t spiBusWrite_wrap(uint8_t devAddress, uint8_t memAddress, uint8_t* data, uint8_t len) {
  (void)devAddress;
  uint8_t tmp[SPI_INTERFACE_USERBUF_MAX];
  if ((size_t)(len + 1) > sizeof(tmp)) {
    return -1;
  }
  tmp[0] = memAddress;  // 写命令，高位为0
  if (len > 0) {
    memcpy(&tmp[1], data, len);
  }
  int ret = g_spi.write(tmp, (size_t)len + 1);
  return (ret == (int)((size_t)len + 1)) ? 0 : -1;
}

static void delay_ms_wrap(unsigned int ms) { rt_thread_mdelay(ms); }

static int8_t mpu6000_read_data(imu_dev_t imu, rt_off_t pos, void* data, rt_size_t size) {
  if (data == RT_NULL) {
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
    .imu_config = RT_NULL,
    .imu_read = mpu6000_read_data,
};

static struct imu_device imu_dev = {
    .ops = &mpu6000_dev,
    .config = IMU_CONFIGURE,
};

static rt_err_t mpu6000_init(const char* spi_device_name) {
  (void)spi_device_name;  // 不再使用此参数，转由 Kconfig 提供完整参数

  // 初始化 SPI 接口（总线名、从设备名、CS 引脚名）
  if (!g_spi.init(SENSOR_SPI_NAME_MPU6000, SENSOR_SPI_SLAVE_NAME_MPU6000, SENSOR_MPU6000_SPI_CS_PIN_NAME)) {
    console_printf("init spi failed: bus=%s slave=%s cs=%s\r\n", SENSOR_SPI_NAME_MPU6000, SENSOR_SPI_SLAVE_NAME_MPU6000,
                   SENSOR_MPU6000_SPI_CS_PIN_NAME);
    return RT_ERROR;
  }

  // 配置 SPI 时钟与模式，并打开设备
  if (!g_spi.configure(RT_SPI_MODE_3 | RT_SPI_MSB, SENSOR_MPU6000_SPI_MAX_HZ)) {
    return RT_ERROR;
  }
  if (g_spi.open(RT_DEVICE_OFLAG_RDWR) != RT_EOK) {
    return RT_ERROR;
  }

  // 设置 MPU 驱动侧的 SPI 访问函数与延迟
  drv_mpu6000_set_spi_funcs(spiBusWrite_wrap, spiBusRead_wrap);
  drv_mpu6000_set_delay(delay_ms_wrap);

  // 初始化底层传感器
  int ret = drv_mpu6000_init();
  if (ret != 0) {
    return RT_ERROR;
  }

  // 初始化中断事件与GPIO
  rt_err_t result = mpu6000_interrupt_event_init();
  if (result != RT_EOK) {
    return result;
  }
  result = mpu6000_interrupt_gpio_init();
  if (result != RT_EOK) {
    return result;
  }

  // 注册IMU设备
  hal_imu_register(&imu_dev, SENSOR_NAME_MPU6000, RT_DEVICE_FLAG_RDWR, RT_NULL);

  return RT_EOK;
}

extern "C" {
static int mpu6000_init_auto(void) { return mpu6000_init(SENSOR_SPI_NAME_MPU6000); }
#ifdef BSP_USING_MPU6000
INIT_COMPONENT_EXPORT(mpu6000_init_auto);
#endif
}