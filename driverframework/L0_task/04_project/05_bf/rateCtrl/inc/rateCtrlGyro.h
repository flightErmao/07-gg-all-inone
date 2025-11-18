#ifndef RATE_CTRL_GYRO__
#define RATE_CTRL_GYRO__

#include <rtthread.h>
#include <uMCN.h>
#include <ipc/workqueue.h>
#include <cstdint>
#include <cmath>
#include <cstring>

extern "C" {
#include "imu_raw_msg.h"
}

#include "bfGyroLpfFilter.hpp"
#include "bfSensorAlignment.hpp"
#include "bfNotchFilter.hpp"
#include "bfDynNotchFilter.hpp"
#include "../mlog/inc/mlog_gyro.hpp"

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

// RateCtrlAngularVelocity 是 gyro_t 的 C++ 版本
// 简单变量直接映射为成员变量，复杂成员变量对象化
class RateCtrlAngularVelocity {
public:
    RateCtrlAngularVelocity();

    rt_err_t init();

    // 获取滤波后的角速度数据（对应 gyro.gyroADCf）
    void getFilteredGyro(float gyro_filtered[3]) const {
        std::memcpy(gyro_filtered, gyro_adcf_, sizeof(gyro_adcf_));
    }
    
    // 获取对齐后的角速度数据（对应 gyro.gyroADC）
    void getGyroAdc(float gyro_adc[3]) const {
        std::memcpy(gyro_adc, gyro_adc_, sizeof(gyro_adc_));
    }

private:
    static void workHandler(struct rt_work* work, void* parameter);
    static void asyncCallback(const void* data, void* user_data);

    void handleWork();

    // ========== 从 gyro_t 映射的简单变量 ==========
    
    // 采样频率和时间（对应 gyro.sampleRateHz, targetLooptime, sampleLooptime）
    uint16_t sample_rate_hz_;           // gyro.sampleRateHz
    uint32_t target_looptime_us_;       // gyro.targetLooptime (微秒)
    uint32_t sample_looptime_us_;       // gyro.sampleLooptime (微秒)
    float scale_;                        // gyro.scale
    
    // 角速度数据（对应 gyro.gyroADC, gyro.gyroADCf）
    float gyro_adc_[3];                 // gyro.gyroADC[XYZ_AXIS_COUNT] - 对齐、校准、缩放但未滤波的数据
    float gyro_adcf_[3];                // gyro.gyroADCf[XYZ_AXIS_COUNT] - 滤波后的角速度数据
    
    // 降采样相关（对应 gyro.sampleCount, sampleSum, downsampleFilterEnabled）
    uint8_t sample_count_;              // gyro.sampleCount - 陀螺仪采样计数器
    float sample_sum_[3];               // gyro.sampleSum[XYZ_AXIS_COUNT] - 用于降采样的累加样本
    bool downsample_filter_enabled_;    // gyro.downsampleFilterEnabled - 是否使用 LPF2 降采样
    
    // 调试和状态相关
    uint8_t gyro_enabled_bitmask_;      // gyro.gyroEnabledBitmask
    uint8_t gyro_debug_mode_;           // gyro.gyroDebugMode
    bool gyro_has_overflow_protection_; // gyro.gyroHasOverflowProtection
    bool use_multi_gyro_debugging_;     // gyro.useMultiGyroDebugging
    uint8_t gyro_debug_axis_;           // gyro.gyroDebugAxis (flight_dynamics_index_t)
    
    // 加速度采样频率
    uint16_t acc_sample_rate_hz_;       // gyro.accSampleRateHz
    
#ifdef USE_DYN_LPF
    // 动态 LPF1 参数（对应 gyro.dynLpfFilter, dynLpfMin, dynLpfMax, dynLpfCurveExpo）
    uint8_t dyn_lpf_filter_;            // gyro.dynLpfFilter
    uint16_t dyn_lpf_min_;              // gyro.dynLpfMin
    uint16_t dyn_lpf_max_;              // gyro.dynLpfMax
    uint8_t dyn_lpf_curve_expo_;        // gyro.dynLpfCurveExpo
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
    uint8_t overflow_axis_mask_;        // gyro.overflowAxisMask
#endif

    // ========== 复杂成员变量对象化 ==========
    
    // 低通滤波器（对应 gyro.lowpassFilter, lowpass2Filter）
    // 原本：gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT] + filterApplyFnPtr
    BfGyroLpfFilter lpf1_filter_;       // LPF1 滤波器对象
    BfGyroLpfFilter lpf2_filter_;       // LPF2 滤波器对象
    
    // Notch 滤波器（对应 gyro.notchFilter1, notchFilter2）
    // 原本：biquadFilter_t notchFilter1[XYZ_AXIS_COUNT] + filterApplyFnPtr
    BfNotchFilter3Axis notch1_filter_;  // Notch1 滤波器对象
    BfNotchFilter3Axis notch2_filter_;  // Notch2 滤波器对象
    
    // 动态 notch 滤波器（对应 dynNotchInit/dynNotchFilter 函数）
    BfDynNotchFilter dyn_notch_filter_;
    
    // IMU 降采样滤波器（对应 gyro.imuGyroFilter[XYZ_AXIS_COUNT]）
    // 需要创建一个 PT1 滤波器对象，或者使用现有的 BfGyroLpfFilter
    BfGyroLpfFilter imu_gyro_filter_[3]; // gyro.imuGyroFilter[XYZ_AXIS_COUNT]
    
    // ========== 其他辅助成员 ==========
    
    // 传感器对齐
    BfSensorAlignment gyro_align_;
    float rotation_matrix_[3][3];       // 自定义旋转矩阵（如果使用）
    bool use_custom_matrix_;
    
    // Gyro calibration（对应 gyro.gyroSensor[].calibration）
    GyroCalibration gyro_calibration_;
    bool calibration_started_;
    
    // MCN 订阅相关
    struct rt_work work_;
    McnNode_t imu_node_;
    imu_raw_msg_t latest_imu_;
    
    // 工作队列相关
    struct rt_workqueue* workqueue_;  // 通过名称找到的工作队列
    // 工作队列名称从 Kconfig 获取：CONFIG_PROJECT_BF_WORKQUEUE_NAME
    
    // Mlog 数据记录（参考 aMlogStabilze.c）
    bf_mlog::MlogGyro mlog_gyro_;
    
    // ========== 内部方法 ==========
    
    // 初始化滤波器（从参数系统读取配置，参考 gyroInitFilters）
    void initFilters();
    
    // 应用完整的滤波链（参考 gyro_filter_impl.c）
    void applyFilterChain(const float input[3], float output[3]);
    
    // 初始化降采样滤波器（参考 gyroInitFilters 中的 imuGyroFilter 初始化）
    void initDownsampleFilter();
    
    // 设置目标循环时间（参考 gyro.c:750-759 中的 gyroSetTargetLooptime）
    void setTargetLooptime(uint8_t pid_denom);
};

#endif /* RATE_CTRL_ANGULAR_VELOCITY_H__ */

