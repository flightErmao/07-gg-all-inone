/**
 * @file bfNotchFilter.hpp
 * 
 * Betaflight 风格 notch 滤波器封装类
 * 从 gyro_init.c 抽取的 notch filter 逻辑
 * 内联了 biquad 滤波器的实现，避免库间依赖
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f
#endif

// Biquad 滤波器结构体（从 biquad.h 内联）
typedef struct biquad_filter_s {
    float q;
    float b0, b1, b2;
    float a1, a2;
    float x1, x2, y1, y2;
    float weight;
} biquadFilter_t;

// Biquad 滤波器类型枚举（从 biquad.h 内联）
enum biquad_filter_type_e {
    FILTER_LPF = 0,
    FILTER_NOTCH,
    FILTER_BPF,
};

// 内联的 biquad 滤波器实现函数
namespace bf_biquad_internal {

// 更新 biquad 滤波器系数（从 biquad.c 内联）
static inline void updateBiquadFilter(biquadFilter_t *filter, float frequency, float time_interval, float Q,
                                      biquad_filter_type_e type, float weight) {
    // setup variables
    const float omega = 2.0f * M_PIf * frequency * time_interval;
    const float sn = std::sin(omega);
    const float cs = std::cos(omega);
    const float alpha = sn / (2.0f * Q);

    switch (type) {
        case FILTER_LPF:
            // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
            filter->b1 = 1.0f - cs;
            filter->b0 = filter->b1 * 0.5f;
            filter->b2 = filter->b0;
            filter->a1 = -2.0f * cs;
            filter->a2 = 1.0f - alpha;
            break;
        case FILTER_NOTCH:
            filter->b0 = 1.0f;
            filter->b1 = -2.0f * cs;
            filter->b2 = 1.0f;
            filter->a1 = filter->b1;
            filter->a2 = 1.0f - alpha;
            break;
        case FILTER_BPF:
            filter->b0 = alpha;
            filter->b1 = 0.0f;
            filter->b2 = -alpha;
            filter->a1 = -2.0f * cs;
            filter->a2 = 1.0f - alpha;
            break;
    }

    const float a0 = 1.0f + alpha;

    // precompute the coefficients
    filter->b0 /= a0;
    filter->b1 /= a0;
    filter->b2 /= a0;
    filter->a1 /= a0;
    filter->a2 /= a0;

    // update weight
    filter->weight = weight;
}

// 初始化 biquad 滤波器（从 biquad.c 内联）
static inline void initBiquadFilter(biquadFilter_t *filter, float frequency, float time_interval, float Q,
                                    biquad_filter_type_e type, float weight) {
    updateBiquadFilter(filter, frequency, time_interval, Q, type, weight);

    // zero initial samples
    filter->x1 = filter->x2 = 0.0f;
    filter->y1 = filter->y2 = 0.0f;
}

// 应用 DF1 biquad 滤波器（从 biquad.c 内联）
static inline float applyDF1BiquadFilter(biquadFilter_t *filter, float input) {
    /* compute result */
    const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 -
                         filter->a1 * filter->y1 - filter->a2 * filter->y2;

    /* shift x1 to x2, input to x1 */
    filter->x2 = filter->x1;
    filter->x1 = input;

    /* shift y1 to y2, result to y1 */
    filter->y2 = filter->y1;
    filter->y1 = result;

    return result;
}

}  // namespace bf_biquad_internal

// Notch 滤波器类（每个轴一个）
class BfNotchFilter {
public:
    BfNotchFilter() : initialized_(false), enabled_(false) {
        std::memset(&filter_, 0, sizeof(filter_));
    }
    
    /**
     * @brief 初始化 notch 滤波器
     * @param center_hz 中心频率（Hz）
     * @param cutoff_hz 截止频率（Hz）
     * @param sample_rate_hz 采样频率（Hz）
     * @return true 初始化成功，false 失败
     */
    bool init(float center_hz, float cutoff_hz, float sample_rate_hz) {
        if (center_hz <= 0.0f || cutoff_hz <= 0.0f || sample_rate_hz <= 0.0f) {
            initialized_ = false;
            enabled_ = false;
            return false;
        }
        
        // 计算奈奎斯特频率
        const float nyquist_freq = sample_rate_hz / 2.0f;
        
        // 调整 notch 频率到奈奎斯特范围内
        if (center_hz > nyquist_freq) {
            if (cutoff_hz < nyquist_freq) {
                center_hz = nyquist_freq;
            } else {
                initialized_ = false;
                enabled_ = false;
                return false;
            }
        }
        
        // 计算 Q 值（从 Betaflight filter.c 抽取）
        const float q = calculateNotchQ(center_hz, cutoff_hz);
        
        // 计算采样周期（秒）
        const float dt = 1.0f / sample_rate_hz;
        
        // 初始化 biquad 滤波器
        bf_biquad_internal::initBiquadFilter(&filter_, center_hz, dt, q, FILTER_NOTCH, 1.0f);
        
        initialized_ = true;
        enabled_ = true;
        return true;
    }
    
    /**
     * @brief 使能/禁用滤波器
     */
    void setEnabled(bool enabled) { 
        enabled_ = enabled && initialized_; 
    }
    
    /**
     * @brief 检查滤波器是否使能
     */
    bool isEnabled() const { 
        return enabled_ && initialized_; 
    }
    
    /**
     * @brief 应用滤波器
     * @param input 输入值
     * @return 滤波后的值
     */
    float apply(float input) const {
        if (!isEnabled()) {
            return input;
        }
        return bf_biquad_internal::applyDF1BiquadFilter(const_cast<biquadFilter_t*>(&filter_), input);
    }
    
    /**
     * @brief 重置滤波器状态
     */
    void reset() {
        filter_.x1 = 0.0f;
        filter_.x2 = 0.0f;
        filter_.y1 = 0.0f;
        filter_.y2 = 0.0f;
    }
    
private:
    /**
     * @brief 计算 notch 滤波器的 Q 值
     * @param center_hz 中心频率（Hz）
     * @param cutoff_hz 截止频率（Hz）
     * @return Q 值
     */
    static float calculateNotchQ(float center_hz, float cutoff_hz) {
        if (cutoff_hz <= 0.0f || center_hz <= 0.0f) {
            return 1.0f;
        }
        // Q = center_freq / bandwidth
        // bandwidth = 2 * (center_freq - cutoff_freq)
        const float bandwidth = 2.0f * (center_hz - cutoff_hz);
        if (bandwidth <= 0.0f) {
            return 1.0f;
        }
        return center_hz / bandwidth;
    }
    
    bool initialized_;
    bool enabled_;
    biquadFilter_t filter_;
};

// 三轴 Notch 滤波器封装类
class BfNotchFilter3Axis {
public:
    BfNotchFilter3Axis() {}
    
    /**
     * @brief 初始化三轴 notch 滤波器
     * @param center_hz 中心频率（Hz）
     * @param cutoff_hz 截止频率（Hz）
     * @param sample_rate_hz 采样频率（Hz）
     * @return true 初始化成功，false 失败
     */
    bool init(float center_hz, float cutoff_hz, float sample_rate_hz) {
        bool ret = true;
        for (int i = 0; i < 3; i++) {
            if (!filters_[i].init(center_hz, cutoff_hz, sample_rate_hz)) {
                ret = false;
            }
        }
        return ret;
    }
    
    /**
     * @brief 使能/禁用滤波器
     */
    void setEnabled(bool enabled) {
        for (int i = 0; i < 3; i++) {
            filters_[i].setEnabled(enabled);
        }
    }
    
    /**
     * @brief 检查滤波器是否使能
     */
    bool isEnabled() const {
        return filters_[0].isEnabled();  // 假设三个轴状态一致
    }
    
    /**
     * @brief 应用滤波器到三轴数据
     * @param input 输入数据 [x, y, z]
     * @param output 输出数据 [x, y, z]
     */
    void apply(const float input[3], float output[3]) const {
        for (int i = 0; i < 3; i++) {
            output[i] = filters_[i].apply(input[i]);
        }
    }
    
    /**
     * @brief 重置滤波器状态
     */
    void reset() {
        for (int i = 0; i < 3; i++) {
            filters_[i].reset();
        }
    }
    
private:
    BfNotchFilter filters_[3];
};

