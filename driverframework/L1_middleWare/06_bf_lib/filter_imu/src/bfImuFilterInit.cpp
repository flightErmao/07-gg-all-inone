/**
 * @file bfImuFilterInit.cpp
 * 
 * Betaflight 风格 IMU 滤波器初始化函数
 * 从 gyro_init.c 的 gyroInitFilters 函数抽取
 */

#include "bfImuFilterInit.h"
#include "bfGyroLpfFilter.hpp"
#include <cstring>

// 使用配置参数初始化 LPF 滤波器
void bfImuFilterInitLpf(BfGyroLpfFilter* lpf1, BfGyroLpfFilter* lpf2,
                        const BfLpfFilterConfig* lpf1_config,
                        const BfLpfFilterConfig* lpf2_config,
                        float sample_rate_hz)
{
    if (lpf1 == nullptr && lpf2 == nullptr) {
        return;
    }
    
    // 初始化 LPF1 滤波器
    if (lpf1 != nullptr && lpf1_config != nullptr) {
        if (lpf1_config->enabled && lpf1_config->cutoff_hz > 0.0f) {
            BfLpfFilterType filter_type = static_cast<BfLpfFilterType>(lpf1_config->type);
            if (lpf1->init(filter_type, lpf1_config->cutoff_hz, sample_rate_hz)) {
                lpf1->setEnabled(true);
            }
        }
    }
    
    // 初始化 LPF2 滤波器
    if (lpf2 != nullptr && lpf2_config != nullptr) {
        if (lpf2_config->enabled && lpf2_config->cutoff_hz > 0.0f) {
            BfLpfFilterType filter_type = static_cast<BfLpfFilterType>(lpf2_config->type);
            if (lpf2->init(filter_type, lpf2_config->cutoff_hz, sample_rate_hz)) {
                lpf2->setEnabled(true);
            }
        }
    }
}

