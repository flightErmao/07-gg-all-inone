#include "qmc6309_selftest.h"
#include <rtthread.h>
#include <rtdevice.h>

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
 * @brief QMC6309 自测功能
 * @param i2c_interface I2C 接口结构体指针
 * @return 1 表示成功，0 表示失败
 */
int qmc6309_self_test(I2cInterface_t* i2c_interface) {
  int selftest_result = 0;
  int selftest_retry = 0;
  signed char st_data[3];
  unsigned char abs_data[3];
  unsigned char rdy = 0x00;
  int t1 = 0;
  int ret = QMC6309_FAIL;

  if (i2c_interface == RT_NULL) {
    rt_kprintf("[QMC6309_SELFTEST] i2c_interface is NULL\n");
    return QMC6309_FAIL;
  }

  while ((selftest_result == 0) && (selftest_retry < 3)) {
    selftest_retry++;
    
    /* 初始化控制寄存器 */
    if (qmc6309_write_reg(i2c_interface, QMC6309_CTL_REG_ONE, 0x00) != QMC6309_OK) {
      continue;
    }
    qmc6309_delay(2);
    
    if (qmc6309_write_reg(i2c_interface, QMC6309_CTL_REG_TWO, 0x00) != QMC6309_OK) {
      continue;
    }
    qmc6309_delay(2);
    
    /* 启动自测模式 */
    if (qmc6309_write_reg(i2c_interface, QMC6309_CTL_REG_ONE, 0x03) != QMC6309_OK) {
      continue;
    }
    qmc6309_delay(20);
    
    /* 触发自测 */
    if (qmc6309_write_reg(i2c_interface, 0x0e, 0x80) != QMC6309_OK) {
      continue;
    }
    
    /* 等待数据就绪 */
    rdy = 0x00;
    t1 = 0;
    while (!(rdy & QMC6309_STATUS_DRDY)) {
      qmc6309_delay(5);
      ret = qmc6309_read_block(i2c_interface, QMC6309_STATUS_REG, &rdy, 1);
      
      if (ret != QMC6309_OK) {
        break;
      }
      
      if (t1++ > 50) {
        break;
      }
    }

    /* 读取自测数据 */
    if (rdy & QMC6309_STATUS_DRDY) {
      ret = qmc6309_read_block(i2c_interface, QMC6309_DATA_OUT_ST_X, (unsigned char*)st_data, 3);
      if (ret == QMC6309_FAIL) {
        continue;
      }
    } else {
      rt_kprintf("[QMC6309_SELFTEST] drdy fail! retry:%d\n", selftest_retry);
      continue;
    }

    /* 计算绝对值 */
    abs_data[0] = QMC6309_ABS(st_data[0]);
    abs_data[1] = QMC6309_ABS(st_data[1]);
    abs_data[2] = QMC6309_ABS(st_data[2]);

    /* 检查自测数据是否在有效范围内 */
    if (((abs_data[0] < QMC6309_SELFTEST_MAX_X) && (abs_data[0] > QMC6309_SELFTEST_MIN_X)) &&
        ((abs_data[1] < QMC6309_SELFTEST_MAX_Y) && (abs_data[1] > QMC6309_SELFTEST_MIN_Y)) &&
        ((abs_data[2] < QMC6309_SELFTEST_MAX_Z) && (abs_data[2] > QMC6309_SELFTEST_MIN_Z))) {
      rt_kprintf("[QMC6309_SELFTEST] OK! status[0x%02x] data[%d %d %d]\n", 
                 rdy, st_data[0], st_data[1], st_data[2]);
      selftest_result = 1;
    } else {
      rt_kprintf("[QMC6309_SELFTEST] fail! status[0x%02x] data[%d %d %d]\n", 
                 rdy, st_data[0], st_data[1], st_data[2]);
      selftest_result = 0;
    }
  }

  return selftest_result;
}

