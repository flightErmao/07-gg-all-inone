#ifndef __I2C_INTERFACE_H__
#define __I2C_INTERFACE_H__

#include <rtdevice.h>  // 必须加在最前面

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  rt_device_t i2c_dev;
  uint8_t i2c_addr;
} I2cInterface_t;

int8_t i2c_read_reg8_mult_pack(I2cInterface_t i2c_interface, uint8_t register_addr, uint8_t* data, uint8_t len);
int8_t i2c_write_reg8_mult_pack(I2cInterface_t i2c_interface, uint8_t register_addr, uint8_t* data, uint8_t len);
rt_err_t get_i2c_interface(const char* i2c_device_name, uint8_t i2c_addr, I2cInterface_t* i2c_interface);

#ifdef __cplusplus
}
#endif

#endif