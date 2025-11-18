/**
 * @file bfGyroLpfFilter.cpp
 * 
 * Betaflight 风格陀螺仪低通滤波器实现
 */

#include "bfGyroLpfFilter.hpp"
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

BfGyroLpfFilter::BfGyroLpfFilter()
    : initialized_(false),
      enabled_(false),
      filter_type_(BfLpfFilterType::NONE),
      cutoff_hz_(0.0f),
      sample_rate_hz_(0.0f)
{
    for (int i = 0; i < 3; i++) {
        pt1_filter_[i] = Pt1FilterState();
    }
}

BfGyroLpfFilter::~BfGyroLpfFilter()
{
}

float BfGyroLpfFilter::calculatePt1Gain(float cutoff_hz, float dt) const
{
    if (cutoff_hz <= 0.0f || dt <= 0.0f) {
        return 0.0f;
    }
    
    // PT1 滤波器增益计算：k = dt / (dt + 1/(2*pi*fc))
    const float rc = 1.0f / (2.0f * static_cast<float>(M_PI) * cutoff_hz);
    return dt / (dt + rc);
}

bool BfGyroLpfFilter::init(BfLpfFilterType filter_type, float cutoff_hz, float sample_rate_hz)
{
    if (cutoff_hz <= 0.0f || sample_rate_hz <= 0.0f) {
        initialized_ = false;
        enabled_ = false;
        return false;
    }
    
    filter_type_ = filter_type;
    cutoff_hz_ = cutoff_hz;
    sample_rate_hz_ = sample_rate_hz;
    
    // 计算采样周期（秒）
    const float dt = 1.0f / sample_rate_hz;
    
    // 计算奈奎斯特频率
    const float nyquist_freq = sample_rate_hz / 2.0f;
    
    switch (filter_type_) {
    case BfLpfFilterType::PT1:
        // 初始化 PT1 滤波器
        for (int i = 0; i < 3; i++) {
            float k = calculatePt1Gain(cutoff_hz, dt);
            pt1_filter_[i].init(k);
        }
        initialized_ = true;
        break;
        
    case BfLpfFilterType::BIQUAD:
        // 检查截止频率是否在奈奎斯特频率内
        if (cutoff_hz <= nyquist_freq) {
            // 简化版：对于 BIQUAD，我们使用 PT2 近似（可以后续扩展）
            // 这里暂时使用 PT1 作为简化实现
            for (int i = 0; i < 3; i++) {
                float k = calculatePt1Gain(cutoff_hz, dt);
                pt1_filter_[i].init(k);
            }
            initialized_ = true;
        } else {
            initialized_ = false;
        }
        break;
        
    case BfLpfFilterType::PT2:
        // 简化版：PT2 使用两个串联的 PT1 近似
        for (int i = 0; i < 3; i++) {
            float k = calculatePt1Gain(cutoff_hz, dt);
            pt1_filter_[i].init(k);
        }
        initialized_ = true;
        break;
        
    case BfLpfFilterType::PT3:
        // 简化版：PT3 使用 PT1 近似
        for (int i = 0; i < 3; i++) {
            float k = calculatePt1Gain(cutoff_hz, dt);
            pt1_filter_[i].init(k);
        }
        initialized_ = true;
        break;
        
    case BfLpfFilterType::NONE:
    default:
        initialized_ = false;
        break;
    }
    
    enabled_ = initialized_;
    return initialized_;
}

void BfGyroLpfFilter::apply(const float gyro_raw[3], float gyro_filtered[3])
{
    if (!isEnabled()) {
        // 如果滤波器未使能，直接输出原始数据
        std::memcpy(gyro_filtered, gyro_raw, sizeof(float) * 3);
        return;
    }
    
    switch (filter_type_) {
    case BfLpfFilterType::PT1:
    case BfLpfFilterType::PT2:
    case BfLpfFilterType::PT3:
    case BfLpfFilterType::BIQUAD:  // 简化版使用 PT1
        applyPt1(gyro_raw, gyro_filtered);
        break;
        
    case BfLpfFilterType::NONE:
    default:
        std::memcpy(gyro_filtered, gyro_raw, sizeof(float) * 3);
        break;
    }
}

void BfGyroLpfFilter::applyPt1(const float input[3], float output[3])
{
    for (int i = 0; i < 3; i++) {
        output[i] = pt1_filter_[i].apply(input[i]);
    }
}

void BfGyroLpfFilter::reset()
{
    for (int i = 0; i < 3; i++) {
        pt1_filter_[i].reset();
    }
}

