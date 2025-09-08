#include "pinInterface.h"
#include <ctype.h>

/**
 * @brief 从配置字符串解析引脚名称
 * @param pin_name 引脚名称字符串，格式如 "PC13", "PA5" 等
 * @return 解析后的引脚编号，解析失败时返回默认值 PC15
 */
rt_base_t parse_pin_name_from_config(const char *pin_name) {
  if (pin_name == RT_NULL) {
    return (rt_base_t)(2 * 16 + 15);  // 默认PC15
  }

  const char *s = pin_name;
  if (s[0] == 'P' || s[0] == 'p') {
    s++;
  }

  if (s[0] == '\0' || !isalpha((unsigned char)s[0])) {
    return (rt_base_t)(2 * 16 + 15);
  }
  char port_char = (char)toupper((unsigned char)s[0]);
  s++;

  int pin_num = 0;
  int has_digit = 0;
  while (*s) {
    if (!isdigit((unsigned char)*s)) break;
    has_digit = 1;
    pin_num = pin_num * 10 + (*s - '0');
    s++;
  }
  if (!has_digit) {
    return (rt_base_t)(2 * 16 + 15);
  }

  if (port_char < 'A' || port_char > 'Z') {
    return (rt_base_t)(2 * 16 + 15);
  }
  int port_index = port_char - 'A';

  return (rt_base_t)(port_index * 16 + pin_num);
}