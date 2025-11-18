/**
 * @file bfDynNotchFilter.hpp
 * 
 * Betaflight 风格动态 notch 滤波器封装类
 * 从 dyn_notch_filter.c 抽取的动态 notch filter 逻辑
 * 使用前向声明避免依赖 Betaflight 头文件
 */

#pragma once

#include <cstdint>
#include <cstring>

// 前向声明，避免包含 dyn_notch_filter.h（它依赖 Betaflight 头文件）
extern "C" {
    // 时间类型（微秒）
    typedef uint32_t timeUs_t;
    
    // 动态 notch 配置结构体（不透明类型）
    typedef struct dynNotchConfig_s dynNotchConfig_t;
    
    // 动态 notch 函数声明
    void dynNotchInit(const dynNotchConfig_t *config, timeUs_t targetLooptimeUs);
    void dynNotchPush(int axis, float sample);
    void dynNotchUpdate(void);
    float dynNotchFilter(int axis, float value);
    bool isDynNotchActive(void);
    int getMaxFFT(void);
    void resetMaxFFT(void);
}

// 动态 notch 滤波器包装类
class BfDynNotchFilter {
public:
    BfDynNotchFilter() : initialized_(false) {}
    
    /**
     * @brief 初始化动态 notch 滤波器
     * @param config 配置结构体指针（可为 nullptr，使用默认配置）
     * @param target_looptime_us 目标循环时间（微秒）
     * @return true 初始化成功，false 失败
     */
    bool init(const void* config, uint32_t target_looptime_us) {
        // 如果配置为 nullptr，使用默认配置
        if (config == nullptr) {
            // 使用默认参数（需要根据实际情况调整）
            // 这里我们直接调用 C 函数，它会在内部处理默认值
            return false;  // 需要有效的配置
        }
        
        dynNotchInit(static_cast<const dynNotchConfig_t*>(config), target_looptime_us);
        
        initialized_ = isDynNotchActive();
        return initialized_;
    }
    
    /**
     * @brief 检查滤波器是否激活
     */
    bool isActive() const {
        return initialized_ && isDynNotchActive();
    }
    
    /**
     * @brief 推送样本数据（用于频率分析）
     * @param axis 轴索引 (0=X, 1=Y, 2=Z)
     * @param sample 样本值
     */
    void push(int axis, float sample) {
        if (isActive()) {
            dynNotchPush(axis, sample);
        }
    }
    
    /**
     * @brief 更新动态 notch 滤波器（需要定期调用）
     */
    void update() {
        if (isActive()) {
            dynNotchUpdate();
        }
    }
    
    /**
     * @brief 应用动态 notch 滤波器
     * @param axis 轴索引 (0=X, 1=Y, 2=Z)
     * @param value 输入值
     * @return 滤波后的值
     */
    float apply(int axis, float value) const {
        if (!isActive()) {
            return value;
        }
        return dynNotchFilter(axis, value);
    }
    
    /**
     * @brief 应用动态 notch 滤波器到三轴数据
     * @param input 输入数据 [x, y, z]
     * @param output 输出数据 [x, y, z]
     */
    void apply3Axis(const float input[3], float output[3]) const {
        if (!isActive()) {
            std::memcpy(output, input, sizeof(float) * 3);
            return;
        }
        
        for (int i = 0; i < 3; i++) {
            output[i] = apply(i, input[i]);
        }
    }
    
private:
    bool initialized_;
};

