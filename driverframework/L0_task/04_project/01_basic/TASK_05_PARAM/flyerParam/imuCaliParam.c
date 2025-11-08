#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "imuCaliParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void imu_cali_param_default(void *address, uint8_t size);

static float imu_cali_gyro_bias[3];
static float imu_cali_acc_bias[3];
static uint32_t imu_cali_timestamp_ms;
static uint32_t imu_cali_status;

static const float imu_cali_gyro_bias_default[3] = {0.0f, 0.0f, 0.0f};
static const float imu_cali_acc_bias_default[3] = {0.0f, 0.0f, 0.0f};
static const uint32_t imu_cali_timestamp_default = 0;
static const uint32_t imu_cali_status_default = 0;

static const param_default_t imu_cali_defaults[] = {
    {imu_cali_gyro_bias, imu_cali_gyro_bias_default},
    {imu_cali_acc_bias, imu_cali_acc_bias_default},
    {&imu_cali_timestamp_ms, &imu_cali_timestamp_default},
    {&imu_cali_status, &imu_cali_status_default},
};

static void imu_cali_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(imu_cali_defaults) / sizeof(imu_cali_defaults[0])); i++) {
        if (address == imu_cali_defaults[i].param) {
            memcpy(address, imu_cali_defaults[i].default_value, size);
            break;
        }
    }
}

static param_list imu_cali_params[] = {
    {(void *)imu_cali_gyro_bias, sizeof(imu_cali_gyro_bias), IMU_CALI_PARAM_GYRO_BIAS, "vf", imu_cali_param_default},
    {(void *)imu_cali_acc_bias, sizeof(imu_cali_acc_bias), IMU_CALI_PARAM_ACC_BIAS, "vf", imu_cali_param_default},
    {(void *)&imu_cali_timestamp_ms, sizeof(imu_cali_timestamp_ms), IMU_CALI_PARAM_TIMESTAMP, "u", imu_cali_param_default},
    {(void *)&imu_cali_status, sizeof(imu_cali_status), IMU_CALI_PARAM_STATUS, "u", imu_cali_param_default},
};

param_list *imuCaliParam_list(void) { return imu_cali_params; }

size_t imuCaliParam_count(void) { return sizeof(imu_cali_params) / sizeof(imu_cali_params[0]); }


