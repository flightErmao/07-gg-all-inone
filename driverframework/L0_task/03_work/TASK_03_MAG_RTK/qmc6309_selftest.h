#ifndef __QMC6309_SELFTEST_H__
#define __QMC6309_SELFTEST_H__

#include <rtthread.h>
#include "I2cInterface.h"

#ifdef __cplusplus
extern "C" {
#endif

/* QMC6309 寄存器定义 */
#define QMC6309_STATUS_REG         0x09  /* Status register */
#define QMC6309_CTL_REG_ONE        0x0A  /* Control register one */
#define QMC6309_CTL_REG_TWO        0x0B  /* Control register two */
#define QMC6309_CTL_REG_THREE      0x0D  /* Control register three */
#define QMC6309_SELFTEST_TRIGGER   0x0E  /* Self-test trigger register */
#define QMC6309_DATA_OUT_ST_X      0x13  /* Self-test data output X register */

/* 自测阈值定义 */
#define QMC6309_SELFTEST_MIN_X     1
#define QMC6309_SELFTEST_MAX_X     50
#define QMC6309_SELFTEST_MIN_Y     1
#define QMC6309_SELFTEST_MAX_Y     50
#define QMC6309_SELFTEST_MIN_Z     1
#define QMC6309_SELFTEST_MAX_Z     50

/* 状态位定义 */
#define QMC6309_STATUS_DRDY        0x04  /* Data ready bit (bit 2) */
#define QMC6309_STATUS_OVFL        0x02  /* Overflow bit */

/* 宏定义 */
#define QMC6309_ABS(x)              ((x) < 0 ? -(x) : (x))
#define QMC6309_FAIL                0
#define QMC6309_OK                  1

/**
 * @brief 输出函数类型定义
 * @param str 要输出的字符串（已格式化）
 */
typedef void (*qmc6309_output_func_t)(const char* str);

/**
 * @brief QMC6309 自测功能
 * @param i2c_interface I2C 接口结构体指针
 * @param output_func 输出函数指针，用于输出自测过程中的信息（可为 NULL）
 * @return 1 表示成功，0 表示失败
 */
int qmc6309_self_test(I2cInterface_t* i2c_interface, qmc6309_output_func_t output_func);

#ifdef __cplusplus
}
#endif

#endif /* __QMC6309_SELFTEST_H__ */

