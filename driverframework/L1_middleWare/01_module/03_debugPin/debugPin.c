#include "debugPin.h"
#include <stdio.h>
#include <rtthread.h>
#include <rtconfig.h>
#include <ctype.h>
#include <drv_gpio.h>
#include "pinInterface.h"

#define DEBUG_PIN_COUNT 4

/* 调试引脚配置结构体 */
typedef struct {
  rt_base_t rt_pin;
  int enabled;
} debug_pin_config_t;

/* 调试引脚配置数组 */
static debug_pin_config_t debug_pins[4] = {0};

/**
 * @brief 初始化调试引脚
 */
static int debug_pin_init(void) {
  printf("[DEBUG_PIN] 开始初始化调试引脚\n");

  // 初始化DEBUG0引脚
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_0_EN
  debug_pins[0].enabled = 1;
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_0_PIN
  debug_pins[0].rt_pin = parse_pin_name_from_config(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_0_PIN);
#else
  debug_pins[0].rt_pin = parse_pin_name_from_config("PC15");
#endif

  rt_pin_mode(debug_pins[0].rt_pin, PIN_MODE_OUTPUT);
  rt_pin_write(debug_pins[0].rt_pin, PIN_LOW);

  printf("[DEBUG_PIN] DEBUG0引脚初始化完成，引脚: %ld\n", debug_pins[0].rt_pin);
#endif

  // 初始化DEBUG1引脚
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_1_EN
  debug_pins[1].enabled = 1;
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_1_PIN
  debug_pins[1].rt_pin = parse_pin_name_from_config(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_1_PIN);
#else
  debug_pins[1].rt_pin = parse_pin_name_from_config("PB0");
#endif

  rt_pin_mode(debug_pins[1].rt_pin, PIN_MODE_OUTPUT);
  rt_pin_write(debug_pins[1].rt_pin, PIN_LOW);

  printf("[DEBUG_PIN] DEBUG1引脚初始化完成，引脚: %ld\n", debug_pins[1].rt_pin);
#endif

  // 初始化DEBUG2引脚
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_2_EN
  debug_pins[2].enabled = 1;
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_2_PIN
  debug_pins[2].rt_pin = parse_pin_name_from_config(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_2_PIN);
#else
  debug_pins[2].rt_pin = parse_pin_name_from_config("PB1");
#endif

  rt_pin_mode(debug_pins[2].rt_pin, PIN_MODE_OUTPUT);
  rt_pin_write(debug_pins[2].rt_pin, PIN_LOW);

  printf("[DEBUG_PIN] DEBUG2引脚初始化完成，引脚: %ld\n", debug_pins[2].rt_pin);
#endif

  // 初始化DEBUG3引脚
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_3_EN
  debug_pins[3].enabled = 1;
#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_3_PIN
  debug_pins[3].rt_pin = parse_pin_name_from_config(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_3_PIN);
#else
  debug_pins[3].rt_pin = parse_pin_name_from_config("PA8");
#endif

  rt_pin_mode(debug_pins[3].rt_pin, PIN_MODE_OUTPUT);
  rt_pin_write(debug_pins[3].rt_pin, PIN_LOW);

  printf("[DEBUG_PIN] DEBUG3引脚初始化完成，引脚: %ld\n", debug_pins[3].rt_pin);
#endif

  printf("[DEBUG_PIN] 调试引脚初始化完成\n");
  return 0;
}

/**
 * @brief 设置指定调试引脚为高电平
 * @param pin_index 引脚索引 (0-3)
 */
void debug_pin_set_high(uint8_t pin_index) {
  if (pin_index < DEBUG_PIN_COUNT && debug_pins[pin_index].enabled) {
    rt_pin_write(debug_pins[pin_index].rt_pin, PIN_HIGH);
  }
}

/**
 * @brief 设置指定调试引脚为低电平
 * @param pin_index 引脚索引 (0-3)
 */
void debug_pin_set_low(uint8_t pin_index) {
  if (pin_index < DEBUG_PIN_COUNT && debug_pins[pin_index].enabled) {
    rt_pin_write(debug_pins[pin_index].rt_pin, PIN_LOW);
  }
}

/**
 * @brief 翻转指定调试引脚状态
 * @param pin_index 引脚索引 (0-3)
 */
void debug_pin_toggle(uint8_t pin_index) {
  if (pin_index < DEBUG_PIN_COUNT && debug_pins[pin_index].enabled) {
    rt_pin_write(debug_pins[pin_index].rt_pin,
                 rt_pin_read(debug_pins[pin_index].rt_pin) == PIN_HIGH ? PIN_LOW : PIN_HIGH);
  }
}

/**
 * @brief 查询指定调试引脚是否启用
 * @param pin_index 引脚索引 (0-3)
 * @return int 1表示启用，0表示未启用
 */
int debug_pin_is_enabled(uint8_t pin_index) {
  if (pin_index < DEBUG_PIN_COUNT) {
    return debug_pins[pin_index].enabled;
  }
  return 0;
}

/**
 * @brief 获取指定调试引脚的RT-Thread引脚句柄
 * @param pin_index 引脚索引 (0-3)
 * @return rt_base_t 引脚句柄，如果引脚未启用则返回0
 */
rt_base_t debug_pin_get_rt_pin(uint8_t pin_index) {
  if (pin_index < DEBUG_PIN_COUNT && debug_pins[pin_index].enabled) {
    return debug_pins[pin_index].rt_pin;
  }
  return 0;
}

#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN
INIT_BOARD_EXPORT(debug_pin_init);
#endif