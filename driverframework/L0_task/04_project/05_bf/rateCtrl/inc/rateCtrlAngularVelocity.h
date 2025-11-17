#ifndef RATE_CTRL_ANGULAR_VELOCITY_H__
#define RATE_CTRL_ANGULAR_VELOCITY_H__

#include <rtthread.h>
#include <uMCN.h>
#include <ipc/workqueue.h>
#include <cstdint>
#include <cmath>

extern "C" {
#include "imu_raw_msg.h"
}

#include "bfGyroLpfFilter.hpp"

// 简化的标准差统计结构（用于校准运动检测）
class DeviationStats {
public:
    DeviationStats() : sum_(0.0f), sum_sq_(0.0f), count_(0) {}

    void clear() {
        sum_ = 0.0f;
        sum_sq_ = 0.0f;
        count_ = 0;
    }

    void push(float value) {
        sum_ += value;
        sum_sq_ += value * value;
        count_++;
    }

    float standardDeviation() const {
        if (count_ < 2) {
            return 0.0f;
        }
        float mean = sum_ / count_;
        float variance = (sum_sq_ / count_) - (mean * mean);
        return (variance > 0.0f) ? std::sqrt(variance) : 0.0f;
    }

private:
    float sum_;
    float sum_sq_;
    uint32_t count_;
};

// 陀螺仪校准类（从 gyro.c 提取的校准逻辑）
class GyroCalibration {
public:
    GyroCalibration();

    // 开始校准
    void startCalibration(float sample_rate_hz, uint32_t calibration_duration_ms, 
                          float movement_threshold, int16_t yaw_offset_centidegrees);
    
    // 更新校准（每帧调用，传入原始陀螺仪数据）
    bool updateCalibration(const float gyro_raw[3]);
    
    // 检查校准是否完成
    bool isCalibrationComplete() const { return calibration_complete_; }
    
    // 检查是否正在校准
    bool isCalibrating() const { return calibrating_; }
    
    // 获取零偏值
    void getGyroZero(float gyro_zero[3]) const;
    
    // 设置零偏值（用于从参数系统加载）
    void setGyroZero(const float gyro_zero[3]);
    
    // 应用零偏值到原始数据
    void applyZeroOffset(const float gyro_raw[3], float gyro_corrected[3]) const;

private:
    bool calibration_complete_;
    bool calibrating_;
    uint32_t calibration_cycles_remaining_;
    uint32_t calibration_cycles_total_;
    float calibration_sum_[3];  // 每个轴的累加值
    DeviationStats calibration_var_[3];  // 每个轴的标准差统计
    float gyro_zero_[3];  // 零偏值
    float movement_threshold_;
    int16_t yaw_offset_centidegrees_;
    
    bool isOnFirstCycle() const;
    bool isOnFinalCycle() const;
};

class RateCtrlAngularVelocity {
public:
    RateCtrlAngularVelocity();

    rt_err_t init();

private:
    static void workHandler(struct rt_work* work, void* parameter);
    static void asyncCallback(const void* data, void* user_data);

    void handleWork();

    struct rt_work work_;
    McnNode_t imu_node_;
    imu_raw_msg_t latest_imu_;
    
    // Betaflight 风格 LPF 滤波器（LPF1 和 LPF2）
    BfGyroLpfFilter lpf1_filter_;
    BfGyroLpfFilter lpf2_filter_;
    
    // Gyro calibration
    GyroCalibration gyro_calibration_;
    bool calibration_started_;
    
    // 初始化滤波器（从参数系统读取配置）
    void initFilters();
};

#endif /* RATE_CTRL_ANGULAR_VELOCITY_H__ */

