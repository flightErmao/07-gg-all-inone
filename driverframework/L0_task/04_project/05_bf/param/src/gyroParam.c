#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "gyroParam.h"
#include "filter.h"  // For lowpassFilterType_e (FILTER_PT1, FILTER_BIQUAD, FILTER_PT2, FILTER_PT3)

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

/* 动态陷波滤波器参数 */
static uint16_t filter_dyn_notch_q;       // 动态陷波 Q 因子 (x100)
static uint16_t filter_dyn_notch_min_hz;  // 动态陷波最小频率
static uint16_t filter_dyn_notch_max_hz;  // 动态陷波最大频率
static uint8_t filter_dyn_notch_count;    // 动态陷波数量

static const uint8_t gyro_lpf1_type_default = FILTER_PT1;  // PT1 (from filter.h: lowpassFilterType_e)
static const uint16_t gyro_lpf1_static_hz_default = 80;    // 默认使用动态最小值

static const uint8_t gyro_lpf2_type_default = FILTER_PT1;  // PT1 (from filter.h: lowpassFilterType_e)
static const uint16_t gyro_lpf2_static_hz_default = 500;   // 默认禁用

static const uint16_t gyro_soft_notch_hz_1_default = 0;
static const uint16_t gyro_soft_notch_cutoff_1_default = 0;
static const uint16_t gyro_soft_notch_hz_2_default = 0;
static const uint16_t gyro_soft_notch_cutoff_2_default = 0;

static const uint16_t gyro_lpf1_dyn_min_hz_default = 80;
static const uint16_t gyro_lpf1_dyn_max_hz_default = 250;
static const uint8_t gyro_lpf1_dyn_expo_default = 5;

static const uint16_t filter_dyn_notch_q_default = 120;  // Q = 1.2
static const uint16_t filter_dyn_notch_min_hz_default = 250;
static const uint16_t filter_dyn_notch_max_hz_default = 550;
static const uint8_t filter_dyn_notch_count_default = 1;

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
    {&filter_dyn_notch_q, &filter_dyn_notch_q_default},
    {&filter_dyn_notch_min_hz, &filter_dyn_notch_min_hz_default},
    {&filter_dyn_notch_max_hz, &filter_dyn_notch_max_hz_default},
    {&filter_dyn_notch_count, &filter_dyn_notch_count_default},
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
    {(void*)&gyro_lpf1_type, sizeof(gyro_lpf1_type), "filter_gyro_lpf1_type", "u8", bf_imu_filter_param_default},
    {(void*)&gyro_lpf1_static_hz, sizeof(gyro_lpf1_static_hz), "filter_gyro_lpf1_static_hz", "u16",
     bf_imu_filter_param_default},
    {(void*)&gyro_lpf2_type, sizeof(gyro_lpf2_type), "filter_gyro_lpf2_type", "u8", bf_imu_filter_param_default},
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
    {(void*)&filter_dyn_notch_q, sizeof(filter_dyn_notch_q), "filter_dyn_notch_q", "u16", bf_imu_filter_param_default},
    {(void*)&filter_dyn_notch_min_hz, sizeof(filter_dyn_notch_min_hz), "filter_dyn_notch_min_hz", "u16",
     bf_imu_filter_param_default},
    {(void*)&filter_dyn_notch_max_hz, sizeof(filter_dyn_notch_max_hz), "filter_dyn_notch_max_hz", "u16",
     bf_imu_filter_param_default},
    {(void*)&filter_dyn_notch_count, sizeof(filter_dyn_notch_count), "filter_dyn_notch_count", "u8",
     bf_imu_filter_param_default},
};

param_list *bfImuFilterParam_list(void) { return bf_imu_filter_params; }

size_t bfImuFilterParam_count(void) { return sizeof(bf_imu_filter_params) / sizeof(bf_imu_filter_params[0]); }

