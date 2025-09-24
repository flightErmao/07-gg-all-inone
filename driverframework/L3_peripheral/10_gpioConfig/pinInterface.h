#ifndef __PIN_INTERFACE_H__
#define __PIN_INTERFACE_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 从配置字符串解析引脚名称
 * @param pin_name 引脚名称字符串，格式如 "PC13", "PA5" 等
 * @return 解析后的引脚编号，解析失败时返回默认值 PC15
 */
rt_base_t parse_pin_name_from_config(const char *pin_name);

#ifdef SOC_FAMILY_AT32
/**
 * @brief 从 rt_base_t 引脚编号中提取 GPIO 端口和引脚号 (AT32)
 * @param pin_index 引脚编号
 * @param gpio_x GPIO 端口指针
 * @param gpio_pin GPIO 引脚位
 */
void get_gpio_from_pin_index(rt_base_t pin_index, void **gpio_x, uint16_t *gpio_pin);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __PIN_INTERFACE_H__ */