/**
 * @file bfImuFilterInit.h
 * 
 * Betaflight 风格 IMU 滤波器初始化接口
 */

#ifndef BF_IMU_FILTER_INIT_H__
#define BF_IMU_FILTER_INIT_H__

#include "bfGyroLpfFilter.hpp"
#include <cstdint>

/**
 * @brief LPF 滤波器配置结构
 */
struct BfLpfFilterConfig {
    bool enabled;
    uint8_t type;
    float cutoff_hz;
};

/**
 * @brief 使用配置参数初始化 LPF 滤波器
 * @param lpf1 LPF1 滤波器指针（可为 nullptr）
 * @param lpf2 LPF2 滤波器指针（可为 nullptr）
 * @param lpf1_config LPF1 配置（可为 nullptr）
 * @param lpf2_config LPF2 配置（可为 nullptr）
 * @param sample_rate_hz 采样频率（Hz）
 */
void bfImuFilterInitLpf(BfGyroLpfFilter* lpf1, BfGyroLpfFilter* lpf2,
                        const BfLpfFilterConfig* lpf1_config,
                        const BfLpfFilterConfig* lpf2_config,
                        float sample_rate_hz);

#endif /* BF_IMU_FILTER_INIT_H__ */

