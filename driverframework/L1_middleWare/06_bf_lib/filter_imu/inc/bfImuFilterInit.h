/**
 * @file bfImuFilterInit.h
 * 
 * Betaflight 风格 IMU 滤波器初始化接口
 */

#ifndef BF_IMU_FILTER_INIT_H__
#define BF_IMU_FILTER_INIT_H__

#include "bfGyroLpfFilter.hpp"

/**
 * @brief 从参数系统读取配置并初始化 LPF 滤波器
 * @param lpf1 LPF1 滤波器指针（可为 nullptr）
 * @param lpf2 LPF2 滤波器指针（可为 nullptr）
 * @param sample_rate_hz 采样频率（Hz）
 */
void bfImuFilterInitLpf(BfGyroLpfFilter* lpf1, BfGyroLpfFilter* lpf2, float sample_rate_hz);

#endif /* BF_IMU_FILTER_INIT_H__ */

