#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f4xx.h"

// 配置参数存储地址 - 使用STM32F4的扇区11（最后一个扇区）
#define CONFIG_PARAM_ADDR 0x080E0000

// 其他配置宏定义
#define CONFIG_PARAM_SIZE 1024  // 配置参数占用空间大小

#endif /* __CONFIG_H */ 