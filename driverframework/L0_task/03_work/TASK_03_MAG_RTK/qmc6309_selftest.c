#include "qmc6309_selftest.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <stdarg.h>

/**
 * @brief 写入单个寄存器
 */
static int qmc6309_write_reg(I2cInterface_t* i2c_interface, uint8_t reg, uint8_t val) {
  int8_t ret = i2c_write_reg8_mult_pack(*i2c_interface, reg, &val, 1);
  return (ret == 0) ? QMC6309_OK : QMC6309_FAIL;
}

/**
 * @brief 读取寄存器块
 */
static int qmc6309_read_block(I2cInterface_t* i2c_interface, uint8_t reg, uint8_t* data, uint8_t len) {
  int8_t ret = i2c_read_reg8_mult_pack(*i2c_interface, reg, data, len);
  return (ret == 0) ? QMC6309_OK : QMC6309_FAIL;
}

/**
 * @brief 延时函数
 */
static void qmc6309_delay(uint32_t ms) {
  rt_thread_mdelay(ms);
}

/**
 * @brief 辅助输出函数（支持可变参数）
 */
static void qmc6309_output_helper(qmc6309_output_func_t output_func, const char* format, ...) {
  if (output_func != RT_NULL) {
    va_list args;
    va_start(args, format);
    char buffer[128];
    rt_vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    output_func(buffer);
  }
}

/**
 * @brief QMC6309 软复位（通过写 CTL_REG_TWO bit7）
 */
static void qmc6309_soft_reset(I2cInterface_t* i2c_interface) {
  uint8_t reg_val = 0;
  qmc6309_read_block(i2c_interface, QMC6309_CTL_REG_TWO, &reg_val, 1);
  reg_val |= 0x80;  /* Set bit 7 for soft reset */
  qmc6309_write_reg(i2c_interface, QMC6309_CTL_REG_TWO, reg_val);
  qmc6309_delay(10);
}

/**
 * @brief QMC6309 自测功能（按照 QMC6308 官方流程）
 * @param i2c_interface I2C 接口结构体指针
 * @param output_func 输出函数指针，用于输出自测过程中的信息（可为 NULL）
 * @return 1 表示成功，0 表示失败
 */
int qmc6309_self_test(I2cInterface_t* i2c_interface, qmc6309_output_func_t output_func) {
  int selftest_result = QMC6309_FAIL;
  int hdata_a[3];  /* 第一次读取的数据（正常模式） */
  int hdata_b[3];  /* 第二次读取的数据（自测模式） */
  int hdata[3];    /* 差值的绝对值 */
  uint8_t rx_buf[8] = {0};
  int t1 = 0;
  int check_result = 0;
  int ret = QMC6309_FAIL;
  int retry_num = 0;
  uint8_t reg_bak_data[3] = {0};  /* 保存寄存器值 */

  if (i2c_interface == RT_NULL) {
    rt_kprintf("[QMC6309_SELFTEST] i2c_interface is NULL\n");
    return QMC6309_FAIL;
  }

  qmc6309_output_helper(output_func, "SELFTEST START\r\n");

  /* 保存当前寄存器值 */
  qmc6309_read_block(i2c_interface, QMC6309_CTL_REG_THREE, &reg_bak_data[0], 1);
  qmc6309_read_block(i2c_interface, QMC6309_CTL_REG_ONE, &reg_bak_data[1], 1);
  qmc6309_read_block(i2c_interface, QMC6309_CTL_REG_TWO, &reg_bak_data[2], 1);

  for (retry_num = 0; retry_num < 3; retry_num++) {
    qmc6309_output_helper(output_func, "RETRY:%d\r\n", retry_num + 1);
    
    /* 软复位 */
    qmc6309_soft_reset(i2c_interface);
    
    /* 写 CTL_REG_THREE = 0x40 */
    if (qmc6309_write_reg(i2c_interface, QMC6309_CTL_REG_THREE, 0x40) != QMC6309_OK) {
      qmc6309_output_helper(output_func, "WRITE CTL_REG_THREE FAIL\r\n");
      continue;
    }
    qmc6309_delay(1);
    
    /* 写 CTL_REG_TWO = 0x00 */
    if (qmc6309_write_reg(i2c_interface, QMC6309_CTL_REG_TWO, 0x00) != QMC6309_OK) {
      qmc6309_output_helper(output_func, "WRITE CTL_REG_TWO FAIL\r\n");
      continue;
    }
    qmc6309_delay(1);
    
    /* 写 CTL_REG_ONE = 0x03 (continuous mode) */
    if (qmc6309_write_reg(i2c_interface, QMC6309_CTL_REG_ONE, 0x03) != QMC6309_OK) {
      qmc6309_output_helper(output_func, "WRITE CTL_REG_ONE FAIL\r\n");
      continue;
    }
    qmc6309_delay(1);
    
    /* 检查状态寄存器 bit[1:0] = 0x03，等待数据就绪 */
    t1 = 0;
    rx_buf[0] = 0x00;
    while (!(rx_buf[0] & QMC6309_STATUS_DRDY_MASK) && (t1 < 20)) {
      ret = qmc6309_read_block(i2c_interface, QMC6309_STATUS_REG, rx_buf, 1);
      if (ret != QMC6309_OK) {
        break;
      }
      qmc6309_output_helper(output_func, "STATUS1:0x%02X\r\n", rx_buf[0]);
      t1++;
      qmc6309_delay(1);
    }
    
    if (t1 >= 20) {
      qmc6309_output_helper(output_func, "DRDY1 TIMEOUT STATUS:0x%02X\r\n", rx_buf[0]);
      continue;
    }
    
    /* 读取第一次数据（正常模式） */
    ret = qmc6309_read_block(i2c_interface, QMC6309_DATA_OUT_X_LSB, rx_buf, 6);
    if (ret != QMC6309_OK) {
      qmc6309_output_helper(output_func, "READ DATA_A FAIL\r\n");
      continue;
    }
    
    /* 组合16位数据（LSB在前，MSB在后） */
    hdata_a[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
    hdata_a[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    hdata_a[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
    qmc6309_output_helper(output_func, "DATA_A:X=%d Y=%d Z=%d\r\n", hdata_a[0], hdata_a[1], hdata_a[2]);
    
    /* 写 CTL_REG_TWO = 0x40 (enter self-test function) */
    if (qmc6309_write_reg(i2c_interface, QMC6309_CTL_REG_TWO, 0x40) != QMC6309_OK) {
      qmc6309_output_helper(output_func, "ENTER SELFTEST MODE FAIL\r\n");
      continue;
    }
    qmc6309_delay(20);
    
    /* 再次检查状态寄存器 bit[1:0] = 0x03，等待数据就绪 */
    t1 = 0;
    rx_buf[0] = 0x00;
    while (!(rx_buf[0] & QMC6309_STATUS_DRDY_MASK) && (t1 < 20)) {
      ret = qmc6309_read_block(i2c_interface, QMC6309_STATUS_REG, rx_buf, 1);
      if (ret != QMC6309_OK) {
        break;
      }
      qmc6309_output_helper(output_func, "STATUS2:0x%02X\r\n", rx_buf[0]);
      t1++;
      qmc6309_delay(1);
    }
    
    if (t1 >= 20) {
      qmc6309_output_helper(output_func, "DRDY2 TIMEOUT STATUS:0x%02X\r\n", rx_buf[0]);
      continue;
    }
    
    /* 读取第二次数据（自测模式） */
    ret = qmc6309_read_block(i2c_interface, QMC6309_DATA_OUT_X_LSB, rx_buf, 6);
    if (ret != QMC6309_OK) {
      qmc6309_output_helper(output_func, "READ DATA_B FAIL\r\n");
      continue;
    }
    
    /* 组合16位数据（LSB在前，MSB在后） */
    hdata_b[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
    hdata_b[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    hdata_b[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
    qmc6309_output_helper(output_func, "DATA_B:X=%d Y=%d Z=%d\r\n", hdata_b[0], hdata_b[1], hdata_b[2]);
    
    /* 计算差值的绝对值 */
    hdata[0] = QMC6309_ABS(hdata_a[0] - hdata_b[0]);
    hdata[1] = QMC6309_ABS(hdata_a[1] - hdata_b[1]);
    hdata[2] = QMC6309_ABS(hdata_a[2] - hdata_b[2]);
    qmc6309_output_helper(output_func, "DELTA_ABS:X=%d Y=%d Z=%d\r\n", hdata[0], hdata[1], hdata[2]);
    
    /* 检查每个轴的差值是否在范围内 */
    check_result = 0;
    if ((hdata[0] < QMC6309_SELFTEST_MAX_X) && (hdata[0] > QMC6309_SELFTEST_MIN_X)) {
      check_result++;
    }
    if ((hdata[1] < QMC6309_SELFTEST_MAX_Y) && (hdata[1] > QMC6309_SELFTEST_MIN_Y)) {
      check_result++;
    }
    if ((hdata[2] < QMC6309_SELFTEST_MAX_Z) && (hdata[2] > QMC6309_SELFTEST_MIN_Z)) {
      check_result++;
    }
    
    if (check_result >= 3) {
      qmc6309_output_helper(output_func, "RANGE:OK (MIN:120 MAX:1800)\r\n");
      selftest_result = QMC6309_OK;
      break;
    } else {
      qmc6309_output_helper(output_func, "RANGE:FAIL (MIN:120 MAX:1800)\r\n");
      selftest_result = QMC6309_FAIL;
    }
  }

  /* 恢复寄存器值 */
  qmc6309_write_reg(i2c_interface, QMC6309_CTL_REG_THREE, reg_bak_data[0]);
  qmc6309_write_reg(i2c_interface, QMC6309_CTL_REG_ONE, reg_bak_data[1]);
  qmc6309_write_reg(i2c_interface, QMC6309_CTL_REG_TWO, reg_bak_data[2]);

  if (selftest_result == QMC6309_OK) {
    qmc6309_output_helper(output_func, "SELFTEST OK\r\n");
  } else {
    qmc6309_output_helper(output_func, "SELFTEST FAIL\r\n");
  }

  return selftest_result;
}

