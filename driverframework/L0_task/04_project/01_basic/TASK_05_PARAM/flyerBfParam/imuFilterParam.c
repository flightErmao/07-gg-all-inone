#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bfImuFilterParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void bf_imu_filter_param_default(void *address, uint8_t size);

/* Betaflight 风格 IMU 滤波器参数 */
static float bf_imu_filter_sample_rate_hz;
static float bf_imu_filter_lpf_gyro_cutoff_hz;
static float bf_imu_filter_lpf_acc_cutoff_hz;
static float bf_imu_filter_notch0_freq_hz;
static float bf_imu_filter_notch0_bw_hz;
static float bf_imu_filter_notch1_freq_hz;
static float bf_imu_filter_notch1_bw_hz;
static float bf_imu_filter_lpf1_dyn_min_hz;
static float bf_imu_filter_lpf1_dyn_max_hz;
static float bf_imu_filter_lpf1_dyn_expo;

/* LPF1 滤波器参数 */
static uint8_t bf_imu_filter_lpf1_enabled;
static uint8_t bf_imu_filter_lpf1_type;  // 0=NONE, 1=PT1, 2=BIQUAD, 3=PT2, 4=PT3
static float bf_imu_filter_lpf1_cutoff_hz;

/* LPF2 滤波器参数 */
static uint8_t bf_imu_filter_lpf2_enabled;
static uint8_t bf_imu_filter_lpf2_type;  // 0=NONE, 1=PT1, 2=BIQUAD, 3=PT2, 4=PT3
static float bf_imu_filter_lpf2_cutoff_hz;

/* 默认值 - Betaflight 典型配置 */
static const float bf_imu_filter_sample_rate_hz_default = 8000.0f;  // Betaflight 默认 8kHz
static const float bf_imu_filter_lpf_gyro_cutoff_hz_default = 100.0f;
static const float bf_imu_filter_lpf_acc_cutoff_hz_default = 50.0f;
static const float bf_imu_filter_notch0_freq_hz_default = 0.0f;
static const float bf_imu_filter_notch0_bw_hz_default = 0.0f;
static const float bf_imu_filter_notch1_freq_hz_default = 0.0f;
static const float bf_imu_filter_notch1_bw_hz_default = 0.0f;
static const float bf_imu_filter_lpf1_dyn_min_hz_default = 80.0f;
static const float bf_imu_filter_lpf1_dyn_max_hz_default = 250.0f;
static const float bf_imu_filter_lpf1_dyn_expo_default = 5.0f;

/* LPF1 默认值 */
static const uint8_t bf_imu_filter_lpf1_enabled_default = 1;  // 默认使能
static const uint8_t bf_imu_filter_lpf1_type_default = 1;     // PT1
static const float bf_imu_filter_lpf1_cutoff_hz_default = 100.0f;

/* LPF2 默认值 */
static const uint8_t bf_imu_filter_lpf2_enabled_default = 0;  // 默认禁用
static const uint8_t bf_imu_filter_lpf2_type_default = 0;     // NONE
static const float bf_imu_filter_lpf2_cutoff_hz_default = 0.0f;

static const param_default_t bf_imu_filter_defaults[] = {
    {&bf_imu_filter_sample_rate_hz, &bf_imu_filter_sample_rate_hz_default},
    {&bf_imu_filter_lpf_gyro_cutoff_hz, &bf_imu_filter_lpf_gyro_cutoff_hz_default},
    {&bf_imu_filter_lpf_acc_cutoff_hz, &bf_imu_filter_lpf_acc_cutoff_hz_default},
    {&bf_imu_filter_notch0_freq_hz, &bf_imu_filter_notch0_freq_hz_default},
    {&bf_imu_filter_notch0_bw_hz, &bf_imu_filter_notch0_bw_hz_default},
    {&bf_imu_filter_notch1_freq_hz, &bf_imu_filter_notch1_freq_hz_default},
    {&bf_imu_filter_notch1_bw_hz, &bf_imu_filter_notch1_bw_hz_default},
    {&bf_imu_filter_lpf1_dyn_min_hz, &bf_imu_filter_lpf1_dyn_min_hz_default},
    {&bf_imu_filter_lpf1_dyn_max_hz, &bf_imu_filter_lpf1_dyn_max_hz_default},
    {&bf_imu_filter_lpf1_dyn_expo, &bf_imu_filter_lpf1_dyn_expo_default},
    {&bf_imu_filter_lpf1_enabled, &bf_imu_filter_lpf1_enabled_default},
    {&bf_imu_filter_lpf1_type, &bf_imu_filter_lpf1_type_default},
    {&bf_imu_filter_lpf1_cutoff_hz, &bf_imu_filter_lpf1_cutoff_hz_default},
    {&bf_imu_filter_lpf2_enabled, &bf_imu_filter_lpf2_enabled_default},
    {&bf_imu_filter_lpf2_type, &bf_imu_filter_lpf2_type_default},
    {&bf_imu_filter_lpf2_cutoff_hz, &bf_imu_filter_lpf2_cutoff_hz_default},
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
    {(void*)&bf_imu_filter_sample_rate_hz, sizeof(bf_imu_filter_sample_rate_hz), "bf_imu_filter_sample_rate_hz", "f",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_lpf_gyro_cutoff_hz, sizeof(bf_imu_filter_lpf_gyro_cutoff_hz), "bf_imu_filter_lpf_gyro_cutoff_hz", "f",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_lpf_acc_cutoff_hz, sizeof(bf_imu_filter_lpf_acc_cutoff_hz), "bf_imu_filter_lpf_acc_cutoff_hz", "f",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_notch0_freq_hz, sizeof(bf_imu_filter_notch0_freq_hz), "bf_imu_filter_notch0_freq_hz", "f",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_notch0_bw_hz, sizeof(bf_imu_filter_notch0_bw_hz), "bf_imu_filter_notch0_bw_hz", "f",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_notch1_freq_hz, sizeof(bf_imu_filter_notch1_freq_hz), "bf_imu_filter_notch1_freq_hz", "f",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_notch1_bw_hz, sizeof(bf_imu_filter_notch1_bw_hz), "bf_imu_filter_notch1_bw_hz", "f",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_lpf1_dyn_min_hz, sizeof(bf_imu_filter_lpf1_dyn_min_hz), "bf_imu_filter_lpf1_dyn_min_hz", "f",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_lpf1_dyn_max_hz, sizeof(bf_imu_filter_lpf1_dyn_max_hz), "bf_imu_filter_lpf1_dyn_max_hz", "f",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_lpf1_dyn_expo, sizeof(bf_imu_filter_lpf1_dyn_expo), "bf_imu_filter_lpf1_dyn_expo", "f",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_lpf1_enabled, sizeof(bf_imu_filter_lpf1_enabled), "bf_imu_filter_lpf1_enabled", "u8",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_lpf1_type, sizeof(bf_imu_filter_lpf1_type), "bf_imu_filter_lpf1_type", "u8",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_lpf1_cutoff_hz, sizeof(bf_imu_filter_lpf1_cutoff_hz), "bf_imu_filter_lpf1_cutoff_hz", "f",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_lpf2_enabled, sizeof(bf_imu_filter_lpf2_enabled), "bf_imu_filter_lpf2_enabled", "u8",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_lpf2_type, sizeof(bf_imu_filter_lpf2_type), "bf_imu_filter_lpf2_type", "u8",
     bf_imu_filter_param_default},
    {(void*)&bf_imu_filter_lpf2_cutoff_hz, sizeof(bf_imu_filter_lpf2_cutoff_hz), "bf_imu_filter_lpf2_cutoff_hz", "f",
     bf_imu_filter_param_default},
};

param_list *bfImuFilterParam_list(void) { return bf_imu_filter_params; }

size_t bfImuFilterParam_count(void) { return sizeof(bf_imu_filter_params) / sizeof(bf_imu_filter_params[0]); }

