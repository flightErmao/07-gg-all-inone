#ifndef DEBUG_PIN_H
#define DEBUG_PIN_H

#include <stdint.h>
#include <rtthread.h>
#include <rtconfig.h>

/* 引脚解析函数声明 */
rt_base_t parse_pin_name_from_config(const char* pin_name);

/* 调试引脚宏定义 - 基于Kconfig配置 */
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_0_EN
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_0_PIN
#define DEBUG_PIN_DEBUG0_HIGH() debug_pin_set_high(0)  // DEBUG0置1
#define DEBUG_PIN_DEBUG0_LOW() debug_pin_set_low(0)    // DEBUG0置0
#define DEBUG_PIN_DEBUG0_TOGGLE() debug_pin_toggle(0)  // DEBUG0翻转
#else
#define DEBUG_PIN_DEBUG0_HIGH() debug_pin_set_high(0)  // DEBUG0置1
#define DEBUG_PIN_DEBUG0_LOW() debug_pin_set_low(0)    // DEBUG0置0
#define DEBUG_PIN_DEBUG0_TOGGLE() debug_pin_toggle(0)  // DEBUG0翻转
#endif
#else
#define DEBUG_PIN_DEBUG0_HIGH()    // DEBUG0未启用
#define DEBUG_PIN_DEBUG0_LOW()     // DEBUG0未启用
#define DEBUG_PIN_DEBUG0_TOGGLE()  // DEBUG0未启用
#endif

#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_1_EN
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_1_PIN
#define DEBUG_PIN_DEBUG1_HIGH() debug_pin_set_high(1)  // DEBUG1置1
#define DEBUG_PIN_DEBUG1_LOW() debug_pin_set_low(1)    // DEBUG1置0
#define DEBUG_PIN_DEBUG1_TOGGLE() debug_pin_toggle(1)  // DEBUG1翻转
#else
#define DEBUG_PIN_DEBUG1_HIGH() debug_pin_set_high(1)  // DEBUG1置1
#define DEBUG_PIN_DEBUG1_LOW() debug_pin_set_low(1)    // DEBUG1置0
#define DEBUG_PIN_DEBUG1_TOGGLE() debug_pin_toggle(1)  // DEBUG1翻转
#endif
#else
#define DEBUG_PIN_DEBUG1_HIGH()    // DEBUG1未启用
#define DEBUG_PIN_DEBUG1_LOW()     // DEBUG1未启用
#define DEBUG_PIN_DEBUG1_TOGGLE()  // DEBUG1未启用
#endif

#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_2_EN
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_2_PIN
#define DEBUG_PIN_DEBUG2_HIGH() debug_pin_set_high(2)  // DEBUG2置1
#define DEBUG_PIN_DEBUG2_LOW() debug_pin_set_low(2)    // DEBUG2置0
#define DEBUG_PIN_DEBUG2_TOGGLE() debug_pin_toggle(2)  // DEBUG2翻转
#else
#define DEBUG_PIN_DEBUG2_HIGH() debug_pin_set_high(2)  // DEBUG2置1
#define DEBUG_PIN_DEBUG2_LOW() debug_pin_set_low(2)    // DEBUG2置0
#define DEBUG_PIN_DEBUG2_TOGGLE() debug_pin_toggle(2)  // DEBUG2翻转
#endif
#else
#define DEBUG_PIN_DEBUG2_HIGH()    // DEBUG2未启用
#define DEBUG_PIN_DEBUG2_LOW()     // DEBUG2未启用
#define DEBUG_PIN_DEBUG2_TOGGLE()  // DEBUG2未启用
#endif

#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_3_EN
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_3_PIN
#define DEBUG_PIN_DEBUG3_HIGH() debug_pin_set_high(3)  // DEBUG3置1
#define DEBUG_PIN_DEBUG3_LOW() debug_pin_set_low(3)    // DEBUG3置0
#define DEBUG_PIN_DEBUG3_TOGGLE() debug_pin_toggle(3)  // DEBUG3翻转
#else
#define DEBUG_PIN_DEBUG3_HIGH() debug_pin_set_high(3)  // DEBUG3置1
#define DEBUG_PIN_DEBUG3_LOW() debug_pin_set_low(3)    // DEBUG3置0
#define DEBUG_PIN_DEBUG3_TOGGLE() debug_pin_toggle(3)  // DEBUG3翻转
#endif
#else
#define DEBUG_PIN_DEBUG3_HIGH()    // DEBUG3未启用
#define DEBUG_PIN_DEBUG3_LOW()     // DEBUG3未启用
#define DEBUG_PIN_DEBUG3_TOGGLE()  // DEBUG3未启用
#endif

#endif  // DEBUG_PIN_H