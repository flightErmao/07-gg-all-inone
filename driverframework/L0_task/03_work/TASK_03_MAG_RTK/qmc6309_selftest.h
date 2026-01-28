#ifndef __QMC6309_SELFTEST_H__
#define __QMC6309_SELFTEST_H__

#include <rtthread.h>
#include "I2cInterface.h"

#ifdef __cplusplus
extern "C" {
#endif

/* QMC6309 寄存器定义 */
#define QMC6309_CTL_REG_ONE        0x09
#define QMC6309_CTL_REG_TWO        0x0A
#define QMC6309_STATUS_REG         0x06
#define QMC6309_DATA_OUT_ST_X      0x00

/* 自测阈值定义 */
#define QMC6309_SELFTEST_MIN_X     100
#define QMC6309_SELFTEST_MAX_X     500
#define QMC6309_SELFTEST_MIN_Y     100
#define QMC6309_SELFTEST_MAX_Y     500
#define QMC6309_SELFTEST_MIN_Z     100
#define QMC6309_SELFTEST_MAX_Z     500

/* 状态位定义 */
#define QMC6309_STATUS_DRDY        0x04

/* 宏定义 */
#define QMC6309_ABS(x)              ((x) < 0 ? -(x) : (x))
#define QMC6309_FAIL                0
#define QMC6309_OK                  1

/**
 * @brief QMC6309 自测功能
 * @param i2c_interface I2C 接口结构体
 * @return 1 表示成功，0 表示失败
 */
int qmc6309_self_test(I2cInterface_t* i2c_interface);

#ifdef __cplusplus
}
#endif

#endif /* __QMC6309_SELFTEST_H__ */

