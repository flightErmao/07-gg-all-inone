/**
 * @file bfImuFilterInit.cpp
 * 
 * Betaflight 风格 IMU 滤波器初始化函数
 * 从 gyro_init.c 的 gyroInitFilters 函数抽取
 */

#include "bfImuFilterInit.h"
#include "bfGyroLpfFilter.hpp"
#include <cstring>

extern "C" {
#include "param.h"
#include "bfImuFilterParam.h"
}

// 从参数系统读取并初始化 LPF 滤波器
void bfImuFilterInitLpf(BfGyroLpfFilter* lpf1, BfGyroLpfFilter* lpf2, float sample_rate_hz)
{
    if (lpf1 == nullptr && lpf2 == nullptr) {
        return;
    }
    
    // 读取 LPF1 参数
    if (lpf1 != nullptr) {
        uint8_t lpf1_enabled = 0;
        uint8_t lpf1_type = 0;
        float lpf1_cutoff_hz = 0.0f;
        
        if (getParam("bf_imu_filter_lpf1_enabled", &lpf1_enabled, sizeof(lpf1_enabled)) == RT_EOK &&
            getParam("bf_imu_filter_lpf1_type", &lpf1_type, sizeof(lpf1_type)) == RT_EOK &&
            getParam("bf_imu_filter_lpf1_cutoff_hz", &lpf1_cutoff_hz, sizeof(lpf1_cutoff_hz)) == RT_EOK) {
            
            if (lpf1_enabled && lpf1_cutoff_hz > 0.0f) {
                BfLpfFilterType filter_type = static_cast<BfLpfFilterType>(lpf1_type);
                if (lpf1->init(filter_type, lpf1_cutoff_hz, sample_rate_hz)) {
                    lpf1->setEnabled(true);
                }
            }
        }
    }
    
    // 读取 LPF2 参数
    if (lpf2 != nullptr) {
        uint8_t lpf2_enabled = 0;
        uint8_t lpf2_type = 0;
        float lpf2_cutoff_hz = 0.0f;
        
        if (getParam("bf_imu_filter_lpf2_enabled", &lpf2_enabled, sizeof(lpf2_enabled)) == RT_EOK &&
            getParam("bf_imu_filter_lpf2_type", &lpf2_type, sizeof(lpf2_type)) == RT_EOK &&
            getParam("bf_imu_filter_lpf2_cutoff_hz", &lpf2_cutoff_hz, sizeof(lpf2_cutoff_hz)) == RT_EOK) {
            
            if (lpf2_enabled && lpf2_cutoff_hz > 0.0f) {
                BfLpfFilterType filter_type = static_cast<BfLpfFilterType>(lpf2_type);
                if (lpf2->init(filter_type, lpf2_cutoff_hz, sample_rate_hz)) {
                    lpf2->setEnabled(true);
                }
            }
        }
    }
}

