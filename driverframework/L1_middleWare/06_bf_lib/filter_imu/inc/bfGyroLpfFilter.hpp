/**
 * @file bfGyroLpfFilter.hpp
 * 
 * Betaflight 风格陀螺仪低通滤波器封装类
 * 从 gyro_init.c 抽取的 LPF 滤波器逻辑
 */

#pragma once

#include <cstdint>
#include <cstring>

// 滤波器类型枚举（简化版，只支持常用类型）
enum class BfLpfFilterType : uint8_t {
    NONE = 0,
    PT1 = 1,      // 一阶低通滤波器
    BIQUAD = 2,   // 双二阶滤波器
    PT2 = 3,      // 二阶低通滤波器
    PT3 = 4       // 三阶低通滤波器
};

// 简化的 PT1 滤波器状态（单轴）
class Pt1FilterState {
public:
    Pt1FilterState() : state_(0.0f), k_(0.0f) {}
    
    void init(float k) {
        k_ = k;
        state_ = 0.0f;
    }
    
    float apply(float input) {
        state_ = state_ + k_ * (input - state_);
        return state_;
    }
    
    void reset() {
        state_ = 0.0f;
    }
    
private:
    float state_;
    float k_;
};

// Betaflight 风格陀螺仪低通滤波器类
class BfGyroLpfFilter {
public:
    BfGyroLpfFilter();
    ~BfGyroLpfFilter();
    
    /**
     * @brief 初始化滤波器
     * @param filter_type 滤波器类型（PT1, BIQUAD, PT2, PT3）
     * @param cutoff_hz 截止频率（Hz）
     * @param sample_rate_hz 采样频率（Hz）
     * @return true 初始化成功，false 失败
     */
    bool init(BfLpfFilterType filter_type, float cutoff_hz, float sample_rate_hz);
    
    /**
     * @brief 使能/禁用滤波器
     */
    void setEnabled(bool enabled) { enabled_ = enabled && initialized_; }
    
    /**
     * @brief 检查滤波器是否使能
     */
    bool isEnabled() const { return enabled_ && initialized_; }
    
    /**
     * @brief 应用滤波器到三轴陀螺仪数据
     * @param gyro_raw 原始陀螺仪数据 [x, y, z]
     * @param gyro_filtered 输出滤波后的数据 [x, y, z]
     */
    void apply(const float gyro_raw[3], float gyro_filtered[3]);
    
    /**
     * @brief 重置滤波器状态
     */
    void reset();
    
private:
    bool initialized_;
    bool enabled_;
    BfLpfFilterType filter_type_;
    float cutoff_hz_;
    float sample_rate_hz_;
    
    // PT1 滤波器状态（每个轴一个）
    Pt1FilterState pt1_filter_[3];
    
    // 计算 PT1 增益
    float calculatePt1Gain(float cutoff_hz, float dt) const;
    
    // 应用 PT1 滤波器
    void applyPt1(const float input[3], float output[3]);
};

