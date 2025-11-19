/****************************************************************************
 *
 * Magnetometer Calibration Module
 * Provides hard-iron and soft-iron calibration for magnetometer
 *
 ****************************************************************************/

#ifndef __MAG_CALIBRATION_H__
#define __MAG_CALIBRATION_H__

#include <rtdef.h>
#include "mag.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 磁力计校准参数结构体 */
typedef struct {
    /* 硬铁校准（偏移量） */
    float offset_x;  /* X轴偏移量 (uT) */
    float offset_y;  /* Y轴偏移量 (uT) */
    float offset_z;  /* Z轴偏移量 (uT) */
    
    /* 软铁校准（缩放因子） */
    float scale_x;   /* X轴缩放因子 */
    float scale_y;   /* Y轴缩放因子 */
    float scale_z;   /* Z轴缩放因子 */
    
    /* 校准状态 */
    rt_bool_t is_calibrated;  /* 是否已校准 */
} mag_calibration_t;

/* 校准数据采集结构体 */
typedef struct {
    float min_x, max_x;  /* X轴最小/最大值 */
    float min_y, max_y;  /* Y轴最小/最大值 */
    float min_z, max_z;  /* Z轴最小/最大值 */
    rt_uint32_t sample_count;  /* 采样计数 */
    rt_bool_t is_collecting;   /* 是否正在采集 */
} mag_calib_collect_t;

/**
 * @brief 初始化磁力计校准模块
 */
void magCalibrationInit(void);

/**
 * @brief 获取校准参数
 * @param calib 输出参数，校准参数结构体指针
 * @return RT_EOK 成功，其他值失败
 */
rt_err_t magCalibrationGet(mag_calibration_t* calib);

/**
 * @brief 设置校准参数
 * @param calib 输入参数，校准参数结构体指针
 * @return RT_EOK 成功，其他值失败
 */
rt_err_t magCalibrationSet(const mag_calibration_t* calib);

/**
 * @brief 应用校准到原始数据
 * @param raw 输入，原始磁力计数据
 * @param calibrated 输出，校准后的磁力计数据
 * @return RT_EOK 成功，其他值失败
 */
rt_err_t magCalibrationApply(const mag_report_t* raw, mag_report_t* calibrated);

/**
 * @brief 开始校准数据采集
 */
void magCalibrationStartCollect(void);

/**
 * @brief 停止校准数据采集
 */
void magCalibrationStopCollect(void);

/**
 * @brief 更新校准数据采集（添加一个采样点）
 * @param mag_data 磁力计数据
 */
void magCalibrationUpdateCollect(const mag_report_t* mag_data);

/**
 * @brief 计算校准参数（基于采集的数据）
 * @return RT_EOK 成功，其他值失败
 */
rt_err_t magCalibrationCalculate(void);

/**
 * @brief 清除校准参数
 */
void magCalibrationClear(void);

/**
 * @brief 获取采集状态
 * @return 采集结构体指针
 */
mag_calib_collect_t* magCalibrationGetCollectStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAG_CALIBRATION_H__ */

