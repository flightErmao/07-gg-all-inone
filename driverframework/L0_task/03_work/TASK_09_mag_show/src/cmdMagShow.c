/****************************************************************************
 *
 * MAG Command Interface
 * Provides magnetometer attribute query commands via MSH
 *
 ****************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <string.h>
#include <math.h>
#include <ulog.h>

#include "mag.h"
#include "mcnMagShow.h"
#include "magCalibration.h"
#include "rtconfig.h"

#ifdef RT_USING_FINSH

#undef LOG_TAG
#define LOG_TAG "mag_cmd"
#ifndef LOG_LVL
#define LOG_LVL LOG_LVL_DBG
#endif

/* Show magnetometer device attributes */
static void cmd_mag_show(int argc, char **argv)
{
    const char* device_name = TASK_MAG_DEVICE_NAME;
    rt_device_t dev = rt_device_find(device_name);
    
    if (dev == RT_NULL) {
        LOG_W("Device '%s' not found!", device_name);
        return;
    }
    
    /* Open device if not already open */
    if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK) {
        LOG_E("Failed to open device '%s'", device_name);
        return;
    }
    
    mag_dev_t mag_dev = (mag_dev_t)dev;
    
    LOG_I("Device Attributes:");
    LOG_I("  Device Name:   %s", device_name);
    LOG_I("  Device ID:     %d", mag_dev->id);
    LOG_I("  Range:         ±%d G", mag_dev->config.range_g);
    LOG_I("  ODR:           %d Hz", mag_dev->config.odr_hz);
    LOG_I("  LSB:           %.6f uT/LSB", mag_dev->config.lsb);
    
    /* Calculate conversion factors */
    if (mag_dev->config.lsb > 0.0f) {
        LOG_I("  LSB to mGs:    %.6f mGs/LSB", mag_dev->config.lsb * 10.0f); /* 1 uT = 10 mGs */
    }
    
    /* Read current magnetometer data */
    mag_report_t mag_data = {0};
    rt_size_t size = rt_device_read(dev, MAG_RD_REPORT, (void*)&mag_data, 1);
    
    if (size > 0) {
        float lsb_to_ut = (mag_dev->config.lsb > 0.0f) ? mag_dev->config.lsb : 0.1f;
        mag_data.value_x *= lsb_to_ut;
        mag_data.value_y *= lsb_to_ut;
        mag_data.value_z *= lsb_to_ut;
        
        float magnitude = sqrtf(mag_data.value_x * mag_data.value_x + 
                               mag_data.value_y * mag_data.value_y + 
                               mag_data.value_z * mag_data.value_z);
        
        LOG_I("Current Data:");
        LOG_I("  X:             %.3f uT (%.3f mGs)", mag_data.value_x, mag_data.value_x * 10.0f);
        LOG_I("  Y:             %.3f uT (%.3f mGs)", mag_data.value_y, mag_data.value_y * 10.0f);
        LOG_I("  Z:             %.3f uT (%.3f mGs)", mag_data.value_z, mag_data.value_z * 10.0f);
        LOG_I("  Magnitude:     %.3f uT (%.3f mGs)", magnitude, magnitude * 10.0f);
        LOG_I("  Timestamp:     %u ms", mag_data.timestamp_ms);
    } else {
        LOG_W("Failed to read magnetometer data");
    }
}

MSH_CMD_EXPORT_ALIAS(cmd_mag_show, mag_show, Show magnetometer device attributes and current data);

/* Show magnetic unit conversions and Earth field references */
static void cmd_mag_unit_info(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    LOG_I("磁单位换算速查：");
    LOG_I("  1 G（高斯）   = 100 uT（微特斯拉） = 1000 mGs（毫高斯）");
    LOG_I("  1 uT（微特斯拉） = 0.01 G = 10 mGs");
    LOG_I("  1 mGs（毫高斯） = 0.001 G = 0.1 uT");

    LOG_I("地球各区域典型地磁场强度（仅供参考）：");
    LOG_I("  赤道地区：     250 ~ 350 mGs（25 ~ 35 uT）");
    LOG_I("  中纬度地区：   400 ~ 500 mGs（40 ~ 50 uT）");
    LOG_I("  极地地区：     550 ~ 650 mGs（55 ~ 65 uT）");
    LOG_I("  全球平均：     300 ~ 600 mGs（30 ~ 60 uT）");

    LOG_I("这些数值可帮助快速判断当前磁传感器数据是否处于合理范围。");
}

MSH_CMD_EXPORT_ALIAS(cmd_mag_unit_info, mag_info, Show magnetic unit conversions and Earth field reference);

/* ============================================================================
 * 磁力计校准命令
 * ============================================================================ */

/* 开始校准数据采集 */
static void cmd_mag_calib_start(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    
    magCalibrationStartCollect();
    LOG_I("磁力计校准数据采集已开始");
    LOG_I("请缓慢旋转设备，确保覆盖所有方向（至少10秒）");
    LOG_I("完成后使用 'mag_calib_stop' 停止采集");
}

MSH_CMD_EXPORT_ALIAS(cmd_mag_calib_start, mag_calib_start, Start magnetometer calibration data collection);

/* 停止校准数据采集 */
static void cmd_mag_calib_stop(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    
    magCalibrationStopCollect();
}

MSH_CMD_EXPORT_ALIAS(cmd_mag_calib_stop, mag_calib_stop, Stop magnetometer calibration data collection);

/* 计算校准参数 */
static void cmd_mag_calib_calc(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    
    rt_err_t ret = magCalibrationCalculate();
    if (ret == RT_EOK) {
        LOG_I("校准参数计算成功！");
    } else {
        LOG_E("校准参数计算失败！");
    }
}

MSH_CMD_EXPORT_ALIAS(cmd_mag_calib_calc, mag_calib_calc, Calculate magnetometer calibration parameters);

/* 显示校准状态 */
static void cmd_mag_calib_status(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    
    mag_calibration_t calib;
    magCalibrationGet(&calib);
    
    mag_calib_collect_t* collect = magCalibrationGetCollectStatus();
    
    LOG_I("=== 磁力计校准状态 ===");
    LOG_I("校准状态: %s", calib.is_calibrated ? "已校准" : "未校准");
    
    if (calib.is_calibrated) {
        LOG_I("硬铁偏移量 (uT):");
        LOG_I("  X: %.3f", calib.offset_x);
        LOG_I("  Y: %.3f", calib.offset_y);
        LOG_I("  Z: %.3f", calib.offset_z);
        LOG_I("软铁缩放因子:");
        LOG_I("  X: %.3f", calib.scale_x);
        LOG_I("  Y: %.3f", calib.scale_y);
        LOG_I("  Z: %.3f", calib.scale_z);
    }
    
    LOG_I("数据采集状态: %s", collect->is_collecting ? "进行中" : "已停止");
    if (collect->is_collecting || collect->sample_count > 0) {
        LOG_I("采样数量: %d", collect->sample_count);
        LOG_I("X轴范围: [%.3f, %.3f] uT", collect->min_x, collect->max_x);
        LOG_I("Y轴范围: [%.3f, %.3f] uT", collect->min_y, collect->max_y);
        LOG_I("Z轴范围: [%.3f, %.3f] uT", collect->min_z, collect->max_z);
    }
}

MSH_CMD_EXPORT_ALIAS(cmd_mag_calib_status, mag_calib_status, Show magnetometer calibration status);

/* 清除校准参数 */
static void cmd_mag_calib_clear(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    
    magCalibrationClear();
    LOG_I("校准参数已清除");
}

MSH_CMD_EXPORT_ALIAS(cmd_mag_calib_clear, mag_calib_clear, Clear magnetometer calibration parameters);

/* ============================================================================
 * 航向角计算命令
 * ============================================================================ */

/* 计算并显示航向角 */
static void cmd_mag_heading(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    
    const char* device_name = TASK_MAG_DEVICE_NAME;
    rt_device_t dev = rt_device_find(device_name);
    
    if (dev == RT_NULL) {
        LOG_W("Device '%s' not found!", device_name);
        return;
    }
    
    /* Open device if not already open */
    if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK) {
        LOG_E("Failed to open device '%s'", device_name);
        return;
    }
    
    mag_dev_t mag_dev = (mag_dev_t)dev;
    
    /* 读取原始磁力计数据 */
    mag_report_t raw_data = {0};
    rt_size_t size = rt_device_read(dev, MAG_RD_REPORT, (void*)&raw_data, 1);
    
    if (size == 0) {
        LOG_E("Failed to read magnetometer data");
        return;
    }
    
    /* 应用LSB缩放 */
    float lsb_to_ut = (mag_dev->config.lsb > 0.0f) ? mag_dev->config.lsb : 0.1f;
    raw_data.value_x *= lsb_to_ut;
    raw_data.value_y *= lsb_to_ut;
    raw_data.value_z *= lsb_to_ut;
    
    /* 应用校准 */
    mag_report_t calibrated_data = {0};
    magCalibrationApply(&raw_data, &calibrated_data);
    
    /* 计算航向角（使用水平面X和Y分量）
     * 航向角 = atan2(Y, X) * 180 / PI
     * 范围：-180° 到 +180°
     * 0° = 北，90° = 东，-90° = 西，±180° = 南
     */
    float heading_rad = atan2f(calibrated_data.value_y, calibrated_data.value_x);
    float heading_deg = heading_rad * 180.0f / 3.14159265358979323846f;
    
    /* 转换为0-360度范围 */
    if (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }
    
    /* 计算磁力大小 */
    float magnitude = sqrtf(calibrated_data.value_x * calibrated_data.value_x + 
                           calibrated_data.value_y * calibrated_data.value_y + 
                           calibrated_data.value_z * calibrated_data.value_z);
    
    /* 计算水平面磁力大小 */
    float horizontal_mag = sqrtf(calibrated_data.value_x * calibrated_data.value_x + 
                                calibrated_data.value_y * calibrated_data.value_y);
    
    /* 计算俯仰角（相对于水平面的角度） */
    float pitch_deg = atan2f(calibrated_data.value_z, horizontal_mag) * 180.0f / 3.14159265358979323846f;
    
    LOG_I("=== 磁力计航向角信息 ===");
    LOG_I("原始数据 (uT):");
    LOG_I("  X: %.3f, Y: %.3f, Z: %.3f", raw_data.value_x, raw_data.value_y, raw_data.value_z);
    
    mag_calibration_t calib;
    magCalibrationGet(&calib);
    if (calib.is_calibrated) {
        LOG_I("校准后数据 (uT):");
        LOG_I("  X: %.3f, Y: %.3f, Z: %.3f", 
              calibrated_data.value_x, calibrated_data.value_y, calibrated_data.value_z);
    }
    
    LOG_I("航向角: %.2f° (%.2f rad)", heading_deg, heading_rad);
    LOG_I("  0° = 北, 90° = 东, 180° = 南, 270° = 西");
    LOG_I("俯仰角: %.2f°", pitch_deg);
    LOG_I("磁力大小: %.3f uT (%.3f mGs)", magnitude, magnitude * 10.0f);
    LOG_I("水平磁力: %.3f uT (%.3f mGs)", horizontal_mag, horizontal_mag * 10.0f);
}

MSH_CMD_EXPORT_ALIAS(cmd_mag_heading, mag_heading, Calculate and show magnetometer heading angle);

/* ============================================================================
 * 磁力计受磁前后对比命令
 * ============================================================================ */

/* 参考地磁场强度 (uT) */
#define MAG_REF_FIELD_UT 48.910f

/* 存储受磁前数据 */
static struct {
    float x;                  /* X轴 (uT) */
    float y;                  /* Y轴 (uT) */
    float z;                  /* Z轴 (uT) */
    float magnitude;          /* 模长 (uT) */
    float diff_percent;       /* 差值百分比 (%) */
    uint32_t timestamp_ms;    /* 时间戳 */
    rt_bool_t is_valid;       /* 数据是否有效 */
} mag_before_data = {0};

/* 存储受磁后数据 */
static struct {
    float x;                  /* X轴 (uT) */
    float y;                  /* Y轴 (uT) */
    float z;                  /* Z轴 (uT) */
    float magnitude;          /* 模长 (uT) */
    float diff_percent;       /* 差值百分比 (%) */
    uint32_t timestamp_ms;    /* 时间戳 */
    rt_bool_t is_valid;       /* 数据是否有效 */
} mag_after_data = {0};

/* 计算模长 */
static float calculate_magnitude(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

/* 计算差值百分比 */
static float calculate_diff_percent(float magnitude) {
    if (MAG_REF_FIELD_UT > 0.0f) {
        return ((magnitude - MAG_REF_FIELD_UT) / MAG_REF_FIELD_UT) * 100.0f;
    }
    return 0.0f;
}

/* 记录受磁前数据 */
static void cmd_mag_before(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    
    mag_report_t mag_data = {0};
    
    /* 使用mcnMagReportAcquire获取数据 */
    if (mcnMagReportAcquire(&mag_data) != 0) {
        LOG_E("Failed to acquire magnetometer data");
        return;
    }
    
    /* 计算模长和差值百分比 */
    float mag_magnitude = calculate_magnitude(mag_data.value_x, 
                                               mag_data.value_y, 
                                               mag_data.value_z);
    float diff_percent = calculate_diff_percent(mag_magnitude);
    
    /* 保存受磁前数据 */
    mag_before_data.x = mag_data.value_x;
    mag_before_data.y = mag_data.value_y;
    mag_before_data.z = mag_data.value_z;
    mag_before_data.magnitude = mag_magnitude;
    mag_before_data.diff_percent = diff_percent;
    mag_before_data.timestamp_ms = mag_data.timestamp_ms;
    mag_before_data.is_valid = RT_TRUE;
    
    LOG_I("受磁前数据记录完成！");
    LOG_I("  X=%.3f uT Y=%.3f uT Z=%.3f uT", 
          mag_before_data.x, mag_before_data.y, mag_before_data.z);
    LOG_I("  Magnitude=%.3f uT Diff=%.2f%%", 
          mag_before_data.magnitude, mag_before_data.diff_percent);
}

MSH_CMD_EXPORT_ALIAS(cmd_mag_before, mag_before, Record magnetometer data before magnetic interference);

/* 记录受磁后数据 */
static void cmd_mag_after(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    
    mag_report_t mag_data = {0};
    
    /* 使用mcnMagReportAcquire获取数据 */
    if (mcnMagReportAcquire(&mag_data) != 0) {
        LOG_E("Failed to acquire magnetometer data");
        return;
    }
    
    /* 计算模长和差值百分比 */
    float mag_magnitude = calculate_magnitude(mag_data.value_x, 
                                               mag_data.value_y, 
                                               mag_data.value_z);
    float diff_percent = calculate_diff_percent(mag_magnitude);
    
    /* 保存受磁后数据 */
    mag_after_data.x = mag_data.value_x;
    mag_after_data.y = mag_data.value_y;
    mag_after_data.z = mag_data.value_z;
    mag_after_data.magnitude = mag_magnitude;
    mag_after_data.diff_percent = diff_percent;
    mag_after_data.timestamp_ms = mag_data.timestamp_ms;
    mag_after_data.is_valid = RT_TRUE;
    
    LOG_I("受磁后数据记录完成！");
    LOG_I("  X=%.3f uT Y=%.3f uT Z=%.3f uT", 
          mag_after_data.x, mag_after_data.y, mag_after_data.z);
    LOG_I("  Magnitude=%.3f uT Diff=%.2f%%", 
          mag_after_data.magnitude, mag_after_data.diff_percent);
}

MSH_CMD_EXPORT_ALIAS(cmd_mag_after, mag_after, Record magnetometer data after magnetic interference);

/* 显示受磁前后对比结果 */
static void cmd_mag_result(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    
    /* 检查数据是否有效 */
    if (!mag_before_data.is_valid) {
        LOG_W("受磁前数据未记录，请先使用 'mag_before' 命令记录数据");
        return;
    }
    
    if (!mag_after_data.is_valid) {
        LOG_W("受磁后数据未记录，请先使用 'mag_after' 命令记录数据");
        return;
    }
    
    /* 计算差值 */
    float diff_x = mag_after_data.x - mag_before_data.x;
    float diff_y = mag_after_data.y - mag_before_data.y;
    float diff_z = mag_after_data.z - mag_before_data.z;
    float diff_magnitude = mag_after_data.magnitude - mag_before_data.magnitude;
    
    /* 计算差值百分比 */
    float diff_magnitude_percent = 0.0f;
    if (mag_before_data.magnitude > 0.001f) {
        diff_magnitude_percent = (diff_magnitude / mag_before_data.magnitude) * 100.0f;
    }
    
    LOG_I("=== 磁力计受磁前后对比结果 ===");
    
    /* 第一行：受磁前数据 */
    LOG_I("BEFORE: X=%.3f uT Y=%.3f uT Z=%.3f uT Magnitude=%.3f uT Diff=%.2f%% timestamp=%ums",
          mag_before_data.x,
          mag_before_data.y,
          mag_before_data.z,
          mag_before_data.magnitude,
          mag_before_data.diff_percent,
          mag_before_data.timestamp_ms);
    
    /* 第二行：受磁后数据 */
    LOG_I("AFTER:  X=%.3f uT Y=%.3f uT Z=%.3f uT Magnitude=%.3f uT Diff=%.2f%% timestamp=%ums",
          mag_after_data.x,
          mag_after_data.y,
          mag_after_data.z,
          mag_after_data.magnitude,
          mag_after_data.diff_percent,
          mag_after_data.timestamp_ms);
    
    /* 第三行：差值数据 */
    LOG_I("DIFF:   X=%.3f uT Y=%.3f uT Z=%.3f uT Magnitude=%.3f uT (%.2f%%) timestamp=%ums",
          diff_x,
          diff_y,
          diff_z,
          diff_magnitude,
          diff_magnitude_percent,
          mag_after_data.timestamp_ms);
}

MSH_CMD_EXPORT_ALIAS(cmd_mag_result, mag_result, Show magnetometer before/after comparison result);

#endif /* RT_USING_FINSH */

