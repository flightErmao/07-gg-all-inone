#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__

#include <rtdevice.h>
#include <rtthread.h>

rt_err_t uart_config_by_device_name(const char* device_name, rt_uint32_t baud_rate, struct serial_configure* config);

#endif