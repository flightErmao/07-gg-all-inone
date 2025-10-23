#ifndef DEBUG_PIN_H
#define DEBUG_PIN_H

#include <stdint.h>
#include <rtthread.h>
#include <rtconfig.h>

/* 调试引脚宏定义 - 基于Kconfig配置 */
/* 当模块未启用时，所有宏都定义为空，不会报错 */
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN
/* 引脚解析函数声明 */
rt_base_t parse_pin_name_from_config(const char* pin_name);

void debug_pin_set_high(uint8_t pin_index);
void debug_pin_set_low(uint8_t pin_index);
void debug_pin_toggle(uint8_t pin_index);

/* DEBUG0宏定义 */
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_0_PIN
#define DEBUG_PIN_DEBUG0_HIGH() debug_pin_set_high(0)  // DEBUG0置1
#define DEBUG_PIN_DEBUG0_LOW() debug_pin_set_low(0)    // DEBUG0置0
#define DEBUG_PIN_DEBUG0_TOGGLE() debug_pin_toggle(0)  // DEBUG0翻转
#else
#define DEBUG_PIN_DEBUG0_HIGH()    // DEBUG0未配置
#define DEBUG_PIN_DEBUG0_LOW()     // DEBUG0未配置
#define DEBUG_PIN_DEBUG0_TOGGLE()  // DEBUG0未配置
#endif
#else
/* 模块未启用时，提供空的宏定义 */
#define DEBUG_PIN_DEBUG0_HIGH()    // 模块未启用
#define DEBUG_PIN_DEBUG0_LOW()     // 模块未启用
#define DEBUG_PIN_DEBUG0_TOGGLE()  // 模块未启用
#endif

/* DEBUG1宏定义 */
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_1_PIN
#define DEBUG_PIN_DEBUG1_HIGH() debug_pin_set_high(1)  // DEBUG1置1
#define DEBUG_PIN_DEBUG1_LOW() debug_pin_set_low(1)    // DEBUG1置0
#define DEBUG_PIN_DEBUG1_TOGGLE() debug_pin_toggle(1)  // DEBUG1翻转
#else
#define DEBUG_PIN_DEBUG1_HIGH()    // DEBUG1未配置
#define DEBUG_PIN_DEBUG1_LOW()     // DEBUG1未配置
#define DEBUG_PIN_DEBUG1_TOGGLE()  // DEBUG1未配置
#endif
#else
#define DEBUG_PIN_DEBUG1_HIGH()    // 模块未启用
#define DEBUG_PIN_DEBUG1_LOW()     // 模块未启用
#define DEBUG_PIN_DEBUG1_TOGGLE()  // 模块未启用
#endif

/* DEBUG2宏定义 */
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_2_PIN
#define DEBUG_PIN_DEBUG2_HIGH() debug_pin_set_high(2)  // DEBUG2置1
#define DEBUG_PIN_DEBUG2_LOW() debug_pin_set_low(2)    // DEBUG2置0
#define DEBUG_PIN_DEBUG2_TOGGLE() debug_pin_toggle(2)  // DEBUG2翻转
#else
#define DEBUG_PIN_DEBUG2_HIGH()    // DEBUG2未配置
#define DEBUG_PIN_DEBUG2_LOW()     // DEBUG2未配置
#define DEBUG_PIN_DEBUG2_TOGGLE()  // DEBUG2未配置
#endif
#else
#define DEBUG_PIN_DEBUG2_HIGH()    // 模块未启用
#define DEBUG_PIN_DEBUG2_LOW()     // 模块未启用
#define DEBUG_PIN_DEBUG2_TOGGLE()  // 模块未启用
#endif

/* DEBUG3宏定义 */
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_3_PIN
#define DEBUG_PIN_DEBUG3_HIGH() debug_pin_set_high(3)  // DEBUG3置1
#define DEBUG_PIN_DEBUG3_LOW() debug_pin_set_low(3)    // DEBUG3置0
#define DEBUG_PIN_DEBUG3_TOGGLE() debug_pin_toggle(3)  // DEBUG3翻转
#else
#define DEBUG_PIN_DEBUG3_HIGH()    // DEBUG3未配置
#define DEBUG_PIN_DEBUG3_LOW()     // DEBUG3未配置
#define DEBUG_PIN_DEBUG3_TOGGLE()  // DEBUG3未配置
#endif
#else
#define DEBUG_PIN_DEBUG3_HIGH()    // 模块未启用
#define DEBUG_PIN_DEBUG3_LOW()     // 模块未启用
#define DEBUG_PIN_DEBUG3_TOGGLE()  // 模块未启用
#endif

#endif  // DEBUG_PIN_H