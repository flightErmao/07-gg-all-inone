#include <stddef.h>
#include <stdint.h>

void float_to_string(float num, char *buf, size_t buf_size) {
  int pos = 0;

  // 处理负数
  if (num < 0) {
    buf[pos++] = '-';
    num = -num;
  }

  // 处理整数部分
  int int_part = (int)num;
  if (int_part == 0) {
    buf[pos++] = '0';
  } else {
    char temp[16];
    int temp_pos = 0;
    while (int_part > 0) {
      temp[temp_pos++] = '0' + (int_part % 10);
      int_part /= 10;
    }
    // 反转整数部分
    while (temp_pos > 0) {
      buf[pos++] = temp[--temp_pos];
    }
  }

  // 添加小数点
  buf[pos++] = '.';

  // 处理小数部分
  float frac_part = num - (int)num;
  frac_part *= 1000;                       // 保留三位小数
  int frac_int = (int)(frac_part + 0.5f);  // 四舍五入

  // 提取小数部分的三位
  buf[pos++] = '0' + (frac_int / 100);
  buf[pos++] = '0' + ((frac_int / 10) % 10);
  buf[pos++] = '0' + (frac_int % 10);

  // 添加字符串结束符
  buf[pos] = '\0';
}