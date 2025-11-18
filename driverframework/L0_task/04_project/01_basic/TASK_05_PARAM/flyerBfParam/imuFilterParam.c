#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bfImuFilterParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void bf_imu_filter_param_default(void *address, uint8_t size);

/* Betaflight 风格 IMU 滤波器参数 - 完全仿照 gyroConfig 的参数名称 */

/* LPF1 滤波器参数 */
static uint8_t gyro_lpf1_type;  // 0=NONE, 1=PT1, 2=BIQUAD, 3=PT2, 4=PT3
static uint16_t gyro_lpf1_static_hz;  // LPF1 静态截止频率

/* LPF2 滤波器参数 */
static uint8_t gyro_lpf2_type;  // 0=NONE, 1=PT1, 2=BIQUAD, 3=PT2, 4=PT3
static uint16_t gyro_lpf2_static_hz;  // LPF2 静态截止频率

/* Notch 滤波器参数 */
static uint16_t gyro_soft_notch_hz_1;  // Notch1 中心频率
static uint16_t gyro_soft_notch_cutoff_1;  // Notch1 截止频率
static uint16_t gyro_soft_notch_hz_2;  // Notch2 中心频率
static uint16_t gyro_soft_notch_cutoff_2;  // Notch2 截止频率

/* 动态 LPF1 滤波器参数 */
static uint16_t gyro_lpf1_dyn_min_hz;  // 动态 LPF1 最小频率
static uint16_t gyro_lpf1_dyn_max_hz;  // 动态 LPF1 最大频率
static uint8_t gyro_lpf1_dyn_expo;  // 动态 LPF1 指数

/* 默认值 - 完全仿照 pgResetFn_gyroConfig 的默认值 */
#define GYRO_LPF1_DYN_MIN_HZ_DEFAULT 80
#define GYRO_LPF1_DYN_MAX_HZ_DEFAULT 250
#define GYRO_LPF2_HZ_DEFAULT 0
#define FILTER_PT1 1

static const uint8_t gyro_lpf1_type_default = FILTER_PT1;  // PT1
static const uint16_t gyro_lpf1_static_hz_default = GYRO_LPF1_DYN_MIN_HZ_DEFAULT;  // 默认使用动态最小值

static const uint8_t gyro_lpf2_type_default = FILTER_PT1;  // PT1
static const uint16_t gyro_lpf2_static_hz_default = GYRO_LPF2_HZ_DEFAULT;  // 默认禁用

static const uint16_t gyro_soft_notch_hz_1_default = 0;
static const uint16_t gyro_soft_notch_cutoff_1_default = 0;
static const uint16_t gyro_soft_notch_hz_2_default = 0;
static const uint16_t gyro_soft_notch_cutoff_2_default = 0;

static const uint16_t gyro_lpf1_dyn_min_hz_default = GYRO_LPF1_DYN_MIN_HZ_DEFAULT;
static const uint16_t gyro_lpf1_dyn_max_hz_default = GYRO_LPF1_DYN_MAX_HZ_DEFAULT;
static const uint8_t gyro_lpf1_dyn_expo_default = 5;

static const param_default_t bf_imu_filter_defaults[] = {
    {&gyro_lpf1_type, &gyro_lpf1_type_default},
    {&gyro_lpf1_static_hz, &gyro_lpf1_static_hz_default},
    {&gyro_lpf2_type, &gyro_lpf2_type_default},
    {&gyro_lpf2_static_hz, &gyro_lpf2_static_hz_default},
    {&gyro_soft_notch_hz_1, &gyro_soft_notch_hz_1_default},
    {&gyro_soft_notch_cutoff_1, &gyro_soft_notch_cutoff_1_default},
    {&gyro_soft_notch_hz_2, &gyro_soft_notch_hz_2_default},
    {&gyro_soft_notch_cutoff_2, &gyro_soft_notch_cutoff_2_default},
    {&gyro_lpf1_dyn_min_hz, &gyro_lpf1_dyn_min_hz_default},
    {&gyro_lpf1_dyn_max_hz, &gyro_lpf1_dyn_max_hz_default},
    {&gyro_lpf1_dyn_expo, &gyro_lpf1_dyn_expo_default},
};

static void bf_imu_filter_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_imu_filter_defaults) / sizeof(bf_imu_filter_defaults[0])); i++) {
        if (address == bf_imu_filter_defaults[i].param) {
            memcpy(address, bf_imu_filter_defaults[i].default_value, size);
            break;
        }
    }
}

static param_list bf_imu_filter_params[] = {
    {(void*)&gyro_lpf1_type, sizeof(gyro_lpf1_type), "filter_gyro_lpf1_type", "u8",
     bf_imu_filter_param_default},
    {(void*)&gyro_lpf1_static_hz, sizeof(gyro_lpf1_static_hz), "filter_gyro_lpf1_static_hz", "u16",
     bf_imu_filter_param_default},
    {(void*)&gyro_lpf2_type, sizeof(gyro_lpf2_type), "filter_gyro_lpf2_type", "u8",
     bf_imu_filter_param_default},
    {(void*)&gyro_lpf2_static_hz, sizeof(gyro_lpf2_static_hz), "filter_gyro_lpf2_static_hz", "u16",
     bf_imu_filter_param_default},
    {(void*)&gyro_soft_notch_hz_1, sizeof(gyro_soft_notch_hz_1), "filter_gyro_soft_notch_hz_1", "u16",
     bf_imu_filter_param_default},
    {(void*)&gyro_soft_notch_cutoff_1, sizeof(gyro_soft_notch_cutoff_1), "filter_gyro_soft_notch_cutoff_1", "u16",
     bf_imu_filter_param_default},
    {(void*)&gyro_soft_notch_hz_2, sizeof(gyro_soft_notch_hz_2), "filter_gyro_soft_notch_hz_2", "u16",
     bf_imu_filter_param_default},
    {(void*)&gyro_soft_notch_cutoff_2, sizeof(gyro_soft_notch_cutoff_2), "filter_gyro_soft_notch_cutoff_2", "u16",
     bf_imu_filter_param_default},
    {(void*)&gyro_lpf1_dyn_min_hz, sizeof(gyro_lpf1_dyn_min_hz), "filter_gyro_lpf1_dyn_min_hz", "u16",
     bf_imu_filter_param_default},
    {(void*)&gyro_lpf1_dyn_max_hz, sizeof(gyro_lpf1_dyn_max_hz), "filter_gyro_lpf1_dyn_max_hz", "u16",
     bf_imu_filter_param_default},
    {(void*)&gyro_lpf1_dyn_expo, sizeof(gyro_lpf1_dyn_expo), "filter_gyro_lpf1_dyn_expo", "u8",
     bf_imu_filter_param_default},
};

param_list *bfImuFilterParam_list(void) { return bf_imu_filter_params; }

size_t bfImuFilterParam_count(void) { return sizeof(bf_imu_filter_params) / sizeof(bf_imu_filter_params[0]); }

