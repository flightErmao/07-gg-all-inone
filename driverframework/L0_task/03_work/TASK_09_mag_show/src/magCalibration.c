/****************************************************************************
 *
 * Magnetometer Calibration Module Implementation
 *
 ****************************************************************************/

#include "magCalibration.h"
#include <rtthread.h>
#include <string.h>
#include <math.h>
#include <ulog.h>

#undef LOG_TAG
#define LOG_TAG "mag_calib"
#ifndef LOG_LVL
#define LOG_LVL LOG_LVL_INFO
#endif

/* 校准参数存储（默认值：无校准） */
static mag_calibration_t mag_calibration = {
    .offset_x = 0.0f,
    .offset_y = 0.0f,
    .offset_z = 0.0f,
    .scale_x = 1.0f,
    .scale_y = 1.0f,
    .scale_z = 1.0f,
    .is_calibrated = RT_FALSE
};

/* 校准数据采集 */
static mag_calib_collect_t calib_collect = {
    .min_x = 0.0f,
    .max_x = 0.0f,
    .min_y = 0.0f,
    .max_y = 0.0f,
    .min_z = 0.0f,
    .max_z = 0.0f,
    .sample_count = 0,
    .is_collecting = RT_FALSE
};

/* 初始化磁力计校准模块 */
void magCalibrationInit(void) {
    memset(&mag_calibration, 0, sizeof(mag_calibration_t));
    mag_calibration.scale_x = 1.0f;
    mag_calibration.scale_y = 1.0f;
    mag_calibration.scale_z = 1.0f;
    mag_calibration.is_calibrated = RT_FALSE;
    
    memset(&calib_collect, 0, sizeof(mag_calib_collect_t));
    LOG_I("Magnetometer calibration module initialized");
}

/* 获取校准参数 */
rt_err_t magCalibrationGet(mag_calibration_t* calib) {
    if (calib == RT_NULL) {
        return -RT_ERROR;
    }
    
    memcpy(calib, &mag_calibration, sizeof(mag_calibration_t));
    return RT_EOK;
}

/* 设置校准参数 */
rt_err_t magCalibrationSet(const mag_calibration_t* calib) {
    if (calib == RT_NULL) {
        return -RT_ERROR;
    }
    
    memcpy(&mag_calibration, calib, sizeof(mag_calibration_t));
    LOG_I("Calibration parameters updated");
    LOG_I("  Offset: X=%.3f, Y=%.3f, Z=%.3f uT", 
          mag_calibration.offset_x, 
          mag_calibration.offset_y, 
          mag_calibration.offset_z);
    LOG_I("  Scale:  X=%.3f, Y=%.3f, Z=%.3f", 
          mag_calibration.scale_x, 
          mag_calibration.scale_y, 
          mag_calibration.scale_z);
    
    return RT_EOK;
}

/* 应用校准到原始数据 */
rt_err_t magCalibrationApply(const mag_report_t* raw, mag_report_t* calibrated) {
    if (raw == RT_NULL || calibrated == RT_NULL) {
        return -RT_ERROR;
    }
    
    if (!mag_calibration.is_calibrated) {
        /* 未校准，直接复制原始数据 */
        memcpy(calibrated, raw, sizeof(mag_report_t));
        return RT_EOK;
    }
    
    /* 应用硬铁校准（减去偏移量） */
    float calib_x = raw->value_x - mag_calibration.offset_x;
    float calib_y = raw->value_y - mag_calibration.offset_y;
    float calib_z = raw->value_z - mag_calibration.offset_z;
    
    /* 应用软铁校准（乘以缩放因子） */
    calibrated->value_x = calib_x * mag_calibration.scale_x;
    calibrated->value_y = calib_y * mag_calibration.scale_y;
    calibrated->value_z = calib_z * mag_calibration.scale_z;
    
    /* 保留时间戳 */
    calibrated->timestamp_ms = raw->timestamp_ms;
    
    return RT_EOK;
}

/* 开始校准数据采集 */
void magCalibrationStartCollect(void) {
    calib_collect.min_x = 1e6f;
    calib_collect.max_x = -1e6f;
    calib_collect.min_y = 1e6f;
    calib_collect.max_y = -1e6f;
    calib_collect.min_z = 1e6f;
    calib_collect.max_z = -1e6f;
    calib_collect.sample_count = 0;
    calib_collect.is_collecting = RT_TRUE;
    LOG_I("Started magnetometer calibration data collection");
}

/* 停止校准数据采集 */
void magCalibrationStopCollect(void) {
    calib_collect.is_collecting = RT_FALSE;
    LOG_I("Stopped magnetometer calibration data collection");
    LOG_I("Collected %d samples", calib_collect.sample_count);
}

/* 更新校准数据采集 */
void magCalibrationUpdateCollect(const mag_report_t* mag_data) {
    if (!calib_collect.is_collecting || mag_data == RT_NULL) {
        return;
    }
    
    /* 更新最小/最大值 */
    if (mag_data->value_x < calib_collect.min_x) {
        calib_collect.min_x = mag_data->value_x;
    }
    if (mag_data->value_x > calib_collect.max_x) {
        calib_collect.max_x = mag_data->value_x;
    }
    
    if (mag_data->value_y < calib_collect.min_y) {
        calib_collect.min_y = mag_data->value_y;
    }
    if (mag_data->value_y > calib_collect.max_y) {
        calib_collect.max_y = mag_data->value_y;
    }
    
    if (mag_data->value_z < calib_collect.min_z) {
        calib_collect.min_z = mag_data->value_z;
    }
    if (mag_data->value_z > calib_collect.max_z) {
        calib_collect.max_z = mag_data->value_z;
    }
    
    calib_collect.sample_count++;
}

/* 计算校准参数 */
rt_err_t magCalibrationCalculate(void) {
    if (calib_collect.sample_count < 10) {
        LOG_E("Not enough samples for calibration (need at least 10, got %d)", 
              calib_collect.sample_count);
        return -RT_ERROR;
    }
    
    /* 计算硬铁偏移量（中心点） */
    mag_calibration.offset_x = (calib_collect.min_x + calib_collect.max_x) / 2.0f;
    mag_calibration.offset_y = (calib_collect.min_y + calib_collect.max_y) / 2.0f;
    mag_calibration.offset_z = (calib_collect.min_z + calib_collect.max_z) / 2.0f;
    
    /* 计算软铁缩放因子（使各轴范围相等） */
    float range_x = calib_collect.max_x - calib_collect.min_x;
    float range_y = calib_collect.max_y - calib_collect.min_y;
    float range_z = calib_collect.max_z - calib_collect.min_z;
    
    /* 使用平均范围作为参考 */
    float avg_range = (range_x + range_y + range_z) / 3.0f;
    
    if (range_x > 0.001f) {
        mag_calibration.scale_x = avg_range / range_x;
    } else {
        mag_calibration.scale_x = 1.0f;
    }
    
    if (range_y > 0.001f) {
        mag_calibration.scale_y = avg_range / range_y;
    } else {
        mag_calibration.scale_y = 1.0f;
    }
    
    if (range_z > 0.001f) {
        mag_calibration.scale_z = avg_range / range_z;
    } else {
        mag_calibration.scale_z = 1.0f;
    }
    
    mag_calibration.is_calibrated = RT_TRUE;
    
    LOG_I("Calibration calculated successfully:");
    LOG_I("  Samples: %d", calib_collect.sample_count);
    LOG_I("  X range: [%.3f, %.3f] uT", calib_collect.min_x, calib_collect.max_x);
    LOG_I("  Y range: [%.3f, %.3f] uT", calib_collect.min_y, calib_collect.max_y);
    LOG_I("  Z range: [%.3f, %.3f] uT", calib_collect.min_z, calib_collect.max_z);
    LOG_I("  Offset: X=%.3f, Y=%.3f, Z=%.3f uT", 
          mag_calibration.offset_x, 
          mag_calibration.offset_y, 
          mag_calibration.offset_z);
    LOG_I("  Scale:  X=%.3f, Y=%.3f, Z=%.3f", 
          mag_calibration.scale_x, 
          mag_calibration.scale_y, 
          mag_calibration.scale_z);
    
    return RT_EOK;
}

/* 清除校准参数 */
void magCalibrationClear(void) {
    memset(&mag_calibration, 0, sizeof(mag_calibration_t));
    mag_calibration.scale_x = 1.0f;
    mag_calibration.scale_y = 1.0f;
    mag_calibration.scale_z = 1.0f;
    mag_calibration.is_calibrated = RT_FALSE;
    LOG_I("Calibration parameters cleared");
}

/* 获取采集状态 */
mag_calib_collect_t* magCalibrationGetCollectStatus(void) {
    return &calib_collect;
}

