#include "rateCtrlAngularVelocity.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "rate_ang_vel"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cstring>

#include "../workqueueManage.h"
#include "bfImuFilterInit.h"

extern "C" {
#include "param.h"
#include "bfImuFilterParam.h"
}

// IMU 采样频率（Hz）- 假设从 IMU 发布频率推算
#ifndef PROJECT_BF_RATE_CTRL_IMU_SAMPLE_RATE_HZ
#define PROJECT_BF_RATE_CTRL_IMU_SAMPLE_RATE_HZ 800.0f
#endif

// GyroCalibration 实现
GyroCalibration::GyroCalibration()
    : calibration_complete_(false),
      calibrating_(false),
      calibration_cycles_remaining_(0),
      calibration_cycles_total_(0),
      movement_threshold_(0.0f),
      yaw_offset_centidegrees_(0)
{
    std::memset(calibration_sum_, 0, sizeof(calibration_sum_));
    std::memset(gyro_zero_, 0, sizeof(gyro_zero_));
}

void GyroCalibration::startCalibration(float sample_rate_hz, uint32_t calibration_duration_ms,
                                       float movement_threshold, int16_t yaw_offset_centidegrees)
{
    calibration_complete_ = false;
    calibrating_ = true;
    movement_threshold_ = movement_threshold;
    yaw_offset_centidegrees_ = yaw_offset_centidegrees;
    
    // 计算校准周期数：(duration_ms * sample_rate_hz) / 1000
    calibration_cycles_total_ = static_cast<uint32_t>((calibration_duration_ms * sample_rate_hz) / 1000.0f);
    if (calibration_cycles_total_ < 1) {
        calibration_cycles_total_ = 1;
    }
    calibration_cycles_remaining_ = calibration_cycles_total_;
    
    std::memset(calibration_sum_, 0, sizeof(calibration_sum_));
    for (int i = 0; i < 3; i++) {
        calibration_var_[i].clear();
        gyro_zero_[i] = 0.0f;
    }
}

bool GyroCalibration::isOnFirstCycle() const
{
    return calibration_cycles_remaining_ == calibration_cycles_total_;
}

bool GyroCalibration::isOnFinalCycle() const
{
    return calibration_cycles_remaining_ == 1;
}

bool GyroCalibration::updateCalibration(const float gyro_raw[3])
{
    if (!calibrating_ || calibration_complete_) {
        return false;
    }
    
    bool cal_failed = false;
    
    for (int axis = 0; axis < 3; axis++) {
        // 在第一个周期重置统计
        if (isOnFirstCycle()) {
            calibration_sum_[axis] = 0.0f;
            calibration_var_[axis].clear();
            gyro_zero_[axis] = 0.0f;
        }
        
        // 累加值和统计标准差
        calibration_sum_[axis] += gyro_raw[axis];
        calibration_var_[axis].push(gyro_raw[axis]);
        
        // 在最后一个周期计算零偏值
        if (isOnFinalCycle()) {
            float stddev = calibration_var_[axis].standardDeviation();
            
            // 检查标准差（运动检测）
            if (movement_threshold_ > 0.0f && stddev > movement_threshold_) {
                cal_failed = true;
            } else {
                // 计算零偏值（平均值）
                gyro_zero_[axis] = calibration_sum_[axis] / calibration_cycles_total_;
                
                // 对 yaw 轴应用偏移（centidegrees 转成 float）
                if (axis == 2) {  // Z 轴是 yaw
                    gyro_zero_[axis] -= (static_cast<float>(yaw_offset_centidegrees_) / 100.0f);
                }
            }
        }
    }
    
    // 如果检测到运动，重新开始校准
    if (cal_failed) {
        calibration_cycles_remaining_ = calibration_cycles_total_;
        std::memset(calibration_sum_, 0, sizeof(calibration_sum_));
        for (int i = 0; i < 3; i++) {
            calibration_var_[i].clear();
        }
        return false;
    }
    
    // 在最后一个周期，校准完成
    if (isOnFinalCycle()) {
        calibrating_ = false;
        calibration_complete_ = true;
        calibration_cycles_remaining_ = 0;
        return true;  // 校准完成
    }
    
    // 减少剩余周期数
    calibration_cycles_remaining_--;
    return false;  // 校准进行中
}

void GyroCalibration::getGyroZero(float gyro_zero[3]) const
{
    std::memcpy(gyro_zero, gyro_zero_, sizeof(gyro_zero_));
}

void GyroCalibration::setGyroZero(const float gyro_zero[3])
{
    std::memcpy(gyro_zero_, gyro_zero, sizeof(gyro_zero_));
    // 设置零偏值后，标记为已校准完成
    calibration_complete_ = true;
    calibrating_ = false;
}

void GyroCalibration::applyZeroOffset(const float gyro_raw[3], float gyro_corrected[3]) const
{
    if (!calibration_complete_) {
        // 如果未校准完成，直接返回原始值
        std::memcpy(gyro_corrected, gyro_raw, sizeof(float) * 3);
        return;
    }
    
    for (int i = 0; i < 3; i++) {
        gyro_corrected[i] = gyro_raw[i] - gyro_zero_[i];
    }
}

RateCtrlAngularVelocity::RateCtrlAngularVelocity()
    : imu_node_(RT_NULL),
      calibration_started_(false)
{
    std::memset(&latest_imu_, 0, sizeof(latest_imu_));
    rt_work_init(&work_, RateCtrlAngularVelocity::workHandler, this);
}

rt_err_t RateCtrlAngularVelocity::init()
{
    rt_err_t ret = wq_workqueue_manage_init();
    if (ret != RT_EOK) {
        LOG_E("workqueue init failed (%d)", ret);
        return ret;
    }

    imu_node_ = mcn_subscribe(MCN_HUB(imu_raw), RT_NULL, RT_NULL);
    if (imu_node_ == RT_NULL) {
        LOG_E("subscribe imu topic failed");
        return -RT_ERROR;
    }

    ret = mcn_register_async_cb(imu_node_, RateCtrlAngularVelocity::asyncCallback, this);
    if (ret != RT_EOK) {
        LOG_E("register imu callback failed (%d)", ret);
        return ret;
    }

    // 初始化滤波器（从参数系统读取配置）
    initFilters();

    // 注意：gyro 零偏值在运行时计算，不保存到参数系统
    // 每次启动时都会重新校准

    LOG_I("RateCtrlAngularVelocity initialized");
    return RT_EOK;
}

void RateCtrlAngularVelocity::initFilters()
{
    // 从参数系统读取采样频率
    float sample_rate_hz = PROJECT_BF_RATE_CTRL_IMU_SAMPLE_RATE_HZ;
    float param_sample_rate = 0.0f;
    if (getParam("bf_imu_filter_sample_rate_hz", &param_sample_rate, sizeof(param_sample_rate)) == RT_EOK) {
        if (param_sample_rate > 0.0f) {
            sample_rate_hz = param_sample_rate;
        }
    }
    
    // 初始化 LPF 滤波器（参考 gyroInitFilters）
    bfImuFilterInitLpf(&lpf1_filter_, &lpf2_filter_, sample_rate_hz);
    
    if (lpf1_filter_.isEnabled()) {
        LOG_I("LPF1 filter initialized and enabled");
    }
    if (lpf2_filter_.isEnabled()) {
        LOG_I("LPF2 filter initialized and enabled");
    }
}

void RateCtrlAngularVelocity::workHandler(struct rt_work* work, void* parameter)
{
    RT_UNUSED(work);
    if (parameter == RT_NULL) {
        return;
    }

    static_cast<RateCtrlAngularVelocity*>(parameter)->handleWork();
}

void RateCtrlAngularVelocity::asyncCallback(const void* data, void* user_data)
{
    if ((data == RT_NULL) || (user_data == RT_NULL)) {
        return;
    }

    RateCtrlAngularVelocity* instance = static_cast<RateCtrlAngularVelocity*>(user_data);
    std::memcpy(&instance->latest_imu_, data, sizeof(instance->latest_imu_));

    if (wq_add_work(&instance->work_) != RT_EOK) {
        LOG_E("submit angular velocity work failed");
    }
}

void RateCtrlAngularVelocity::handleWork()
{
    float gyro_raw[3] = {latest_imu_.gyro[0], latest_imu_.gyro[1], latest_imu_.gyro[2]};
    
    // 如果没有校准，开始校准
    if (!calibration_started_ && !gyro_calibration_.isCalibrationComplete()) {
        // 从 Kconfig 获取校准参数
        uint32_t calibration_duration_ms = CONFIG_PROJECT_BF_RATE_CTRL_GYRO_CALIBRATION_DURATION_MS;
        float movement_threshold = static_cast<float>(CONFIG_PROJECT_BF_RATE_CTRL_GYRO_MOVEMENT_THRESHOLD);
        int16_t yaw_offset = static_cast<int16_t>(CONFIG_PROJECT_BF_RATE_CTRL_GYRO_OFFSET_YAW);
        
        gyro_calibration_.startCalibration(
            PROJECT_BF_RATE_CTRL_IMU_SAMPLE_RATE_HZ,
            calibration_duration_ms,
            movement_threshold,
            yaw_offset);
        calibration_started_ = true;
        LOG_I("Starting gyro calibration: duration=%u ms, threshold=%.1f", 
              calibration_duration_ms, movement_threshold);
    }
    
    // 如果正在校准，更新校准
    if (gyro_calibration_.isCalibrating()) {
        bool cal_complete = gyro_calibration_.updateCalibration(gyro_raw);
        
        if (cal_complete) {
            // 校准完成，零偏值在运行时使用，不保存到参数系统
            float gyro_zero[3];
            gyro_calibration_.getGyroZero(gyro_zero);
            LOG_I("Gyro calibration complete! Zero=[%.3f, %.3f, %.3f] (runtime only, not saved)", 
                  gyro_zero[0], gyro_zero[1], gyro_zero[2]);
        } else {
            // 校准进行中，显示进度
            LOG_D("Calibrating... (use raw gyro)");
            return;  // 校准期间不处理数据
        }
    }
    
    // 应用零偏值校正
    float gyro_corrected[3];
    gyro_calibration_.applyZeroOffset(gyro_raw, gyro_corrected);
    
    // 应用 LPF1 滤波器（如果使能）
    float gyro_lpf1[3];
    if (lpf1_filter_.isEnabled()) {
        lpf1_filter_.apply(gyro_corrected, gyro_lpf1);
    } else {
        std::memcpy(gyro_lpf1, gyro_corrected, sizeof(gyro_corrected));
    }
    
    // 应用 LPF2 滤波器（如果使能）
    float gyro_filtered[3];
    if (lpf2_filter_.isEnabled()) {
        lpf2_filter_.apply(gyro_lpf1, gyro_filtered);
    } else {
        std::memcpy(gyro_filtered, gyro_lpf1, sizeof(gyro_lpf1));
    }

    // Log filtered angular velocity
    LOG_D("seq:%u angular_velocity(%.3f, %.3f, %.3f)",
        latest_imu_.seq,
        gyro_filtered[0],
        gyro_filtered[1],
        gyro_filtered[2]);
}

