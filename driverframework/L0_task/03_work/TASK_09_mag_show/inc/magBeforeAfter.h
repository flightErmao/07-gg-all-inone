/****************************************************************************
 *
 * Magnetometer Before/After Magnetic Field Detection Module
 * Monitors magnetometer data and detects magnetic field interference
 *
 ****************************************************************************/

#ifndef __MAG_BEFORE_AFTER_H__
#define __MAG_BEFORE_AFTER_H__

#include <rtdef.h>
#include "mag.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 受磁前后数据结构体 */
typedef struct {
    /* 受磁前数据 */
    float before_x;          /* X轴 (uT) */
    float before_y;          /* Y轴 (uT) */
    float before_z;          /* Z轴 (uT) */
    float before_magnitude;  /* 模长 (uT) */
    float before_diff_percent; /* 差值百分比 (%) */
    
    /* 受磁后数据 */
    float after_x;           /* X轴 (uT) */
    float after_y;           /* Y轴 (uT) */
    float after_z;           /* Z轴 (uT) */
    float after_magnitude;   /* 模长 (uT) */
    float after_diff_percent; /* 差值百分比 (%) */
    
    /* 差值 */
    float diff_x;            /* X轴差值 (uT) */
    float diff_y;            /* Y轴差值 (uT) */
    float diff_z;            /* Z轴差值 (uT) */
    float diff_magnitude;    /* 模长差值 (uT) */
    
    uint32_t timestamp_ms;   /* 时间戳 */
} mag_before_after_t;

/**
 * @brief 初始化磁力计受磁前后检测模块
 * @return RT_EOK 成功，其他值失败
 */
rt_err_t magBeforeAfterInit(void);

/**
 * @brief 启动磁力计受磁前后检测线程
 * @return RT_EOK 成功，其他值失败
 */
rt_err_t magBeforeAfterStart(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAG_BEFORE_AFTER_H__ */

