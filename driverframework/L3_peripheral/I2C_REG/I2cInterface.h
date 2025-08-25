#ifndef __I2C_INTERFACE_H__
#define __I2C_INTERFACE_H__

#include <rtdevice.h>  // 必须加在最前面

#ifdef __cplusplus
extern "C" {
#endif

// I2C标准速度定义 (重命名避免与STM32 HAL冲突)
#define I2C_IF_SPEED_STANDARD     100000    // 100 KHz
#define I2C_IF_SPEED_FAST         400000    // 400 KHz  
#define I2C_IF_SPEED_FAST_PLUS   1000000   // 1 MHz
#define I2C_IF_SPEED_HIGH         3400000   // 3.4 MHz

// I2C占空比定义
#define I2C_IF_DUTY_CYCLE_2       2         // 50% 占空比
#define I2C_IF_DUTY_CYCLE_16_9    16        // 16/9 占空比 (仅用于快速模式)

typedef struct {
  rt_device_t i2c_dev;
  uint8_t i2c_addr;
} I2cInterface_t;

// I2C速度配置结构体
typedef struct {
  uint32_t speed;           // I2C速度 (Hz)
  uint8_t duty_cycle;       // 占空比
  uint32_t timeout_ms;      // 超时时间 (ms)
} I2cSpeedConfig_t;

// 函数声明
int8_t i2c_read_reg8_mult_pack(I2cInterface_t i2c_interface, uint8_t register_addr, uint8_t* data, uint8_t len);
int8_t i2c_write_reg8_mult_pack(I2cInterface_t i2c_interface, uint8_t register_addr, uint8_t* data, uint8_t len);
int8_t i2c_read_reg16_mult_pack(I2cInterface_t i2c_interface, uint16_t register_addr, uint8_t* data, uint8_t len);
int8_t i2c_write_reg16_mult_pack(I2cInterface_t i2c_interface, uint16_t register_addr, uint8_t* data, uint8_t len);

rt_err_t get_i2c_interface(const char* i2c_device_name, uint8_t i2c_addr, I2cInterface_t* i2c_interface);
rt_err_t set_i2c_speed(rt_device_t i2c_dev, uint32_t speed);
rt_err_t configure_i2c_speed(rt_device_t i2c_dev, const I2cSpeedConfig_t* config);

// 便捷的速率设置函数
rt_err_t set_i2c_standard_speed(rt_device_t i2c_dev);
rt_err_t set_i2c_fast_speed(rt_device_t i2c_dev);
rt_err_t set_i2c_fast_plus_speed(rt_device_t i2c_dev);
rt_err_t set_i2c_high_speed(rt_device_t i2c_dev);

#ifdef __cplusplus
}
#endif

#endif 