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

/* 数据输出寄存器（16位数据，每个轴2字节） */
#define QMC6309_DATA_OUT_X_LSB     0x01  /* X axis LSB */
#define QMC6309_DATA_OUT_X_MSB     0x02  /* X axis MSB */
#define QMC6309_DATA_OUT_Y_LSB     0x03  /* Y axis LSB */
#define QMC6309_DATA_OUT_Y_MSB     0x04  /* Y axis MSB */
#define QMC6309_DATA_OUT_Z_LSB     0x05  /* Z axis LSB */
#define QMC6309_DATA_OUT_Z_MSB     0x06  /* Z axis MSB */

/* 自测阈值定义（差值范围，单位：LSB） */
#define QMC6309_SELFTEST_MIN_X     120
#define QMC6309_SELFTEST_MAX_X     1800
#define QMC6309_SELFTEST_MIN_Y     120
#define QMC6309_SELFTEST_MAX_Y     1800
#define QMC6309_SELFTEST_MIN_Z     120
#define QMC6309_SELFTEST_MAX_Z     1800

/* 状态位定义 */
#define QMC6309_STATUS_DRDY_MASK   0x03  /* Data ready mask (bit[1:0]) */

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

