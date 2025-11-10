#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "imuFilterParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void imu_filter_param_default(void *address, uint8_t size);

static float imu_filter_sample_rate_hz;
static float imu_filter_lpf_gyro_cutoff_hz;
static float imu_filter_lpf_acc_cutoff_hz;
static float imu_filter_lpf_angular_acc_cutoff_hz;
static float imu_filter_notch0_freq_hz;
static float imu_filter_notch0_bw_hz;
static float imu_filter_notch1_freq_hz;
static float imu_filter_notch1_bw_hz;

static const float imu_filter_sample_rate_hz_default = 1000.0f;
static const float imu_filter_gyro_lpf_cutoff_hz_default = 70.0f;
static const float imu_filter_acc_lpf_cutoff_hz_default = 30.0f;
static const float imu_filter_angular_acc_cutoff_hz_default = 80.0f;
static const float imu_filter_notch0_freq_hz_default = 0.0f;
static const float imu_filter_notch0_bw_hz_default = 0.0f;
static const float imu_filter_notch1_freq_hz_default = 0.0f;
static const float imu_filter_notch1_bw_hz_default = 0.0f;

static const param_default_t imu_filter_defaults[] = {
    {&imu_filter_sample_rate_hz, &imu_filter_sample_rate_hz_default},
    {&imu_filter_lpf_gyro_cutoff_hz, &imu_filter_gyro_lpf_cutoff_hz_default},
    {&imu_filter_lpf_acc_cutoff_hz, &imu_filter_acc_lpf_cutoff_hz_default},
    {&imu_filter_lpf_angular_acc_cutoff_hz, &imu_filter_angular_acc_cutoff_hz_default},
    {&imu_filter_notch0_freq_hz, &imu_filter_notch0_freq_hz_default},
    {&imu_filter_notch0_bw_hz, &imu_filter_notch0_bw_hz_default},
    {&imu_filter_notch1_freq_hz, &imu_filter_notch1_freq_hz_default},
    {&imu_filter_notch1_bw_hz, &imu_filter_notch1_bw_hz_default},
};

static void imu_filter_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(imu_filter_defaults) / sizeof(imu_filter_defaults[0])); i++) {
        if (address == imu_filter_defaults[i].param) {
            memcpy(address, imu_filter_defaults[i].default_value, size);
            break;
        }
    }
}

static param_list imu_filter_params[] = {
    {(void*)&imu_filter_sample_rate_hz, sizeof(imu_filter_sample_rate_hz), "imu_filter_sample_rate_hz", "f",
     imu_filter_param_default},
    {(void*)&imu_filter_lpf_gyro_cutoff_hz, sizeof(imu_filter_lpf_gyro_cutoff_hz), "imu_filter_lpf_gyro_cutoff_hz", "f",
     imu_filter_param_default},
    {(void*)&imu_filter_lpf_acc_cutoff_hz, sizeof(imu_filter_lpf_acc_cutoff_hz), "imu_filter_lpf_acc_cutoff_hz", "f",
     imu_filter_param_default},
    {(void*)&imu_filter_lpf_angular_acc_cutoff_hz, sizeof(imu_filter_lpf_angular_acc_cutoff_hz),
     "imu_filter_lpf_angular_acc_cutoff_hz", "f", imu_filter_param_default},
    {(void*)&imu_filter_notch0_freq_hz, sizeof(imu_filter_notch0_freq_hz), "imu_filter_notch0_freq_hz", "f",
     imu_filter_param_default},
    {(void*)&imu_filter_notch0_bw_hz, sizeof(imu_filter_notch0_bw_hz), "imu_filter_notch0_bw_hz", "f",
     imu_filter_param_default},
    {(void*)&imu_filter_notch1_freq_hz, sizeof(imu_filter_notch1_freq_hz), "imu_filter_notch1_freq_hz", "f",
     imu_filter_param_default},
    {(void*)&imu_filter_notch1_bw_hz, sizeof(imu_filter_notch1_bw_hz), "imu_filter_notch1_bw_hz", "f",
     imu_filter_param_default},
};

param_list *imuFilterParam_list(void) { return imu_filter_params; }

size_t imuFilterParam_count(void) { return sizeof(imu_filter_params) / sizeof(imu_filter_params[0]); }


