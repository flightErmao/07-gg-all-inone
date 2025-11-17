#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bfImuCaliOffsetParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void bf_imu_cali_offset_param_default(void *address, uint8_t size);

/* Betaflight 风格 IMU 校准偏移参数 */
/* 注意：gyro 零偏值在运行时计算，不保存到参数系统 */
static int16_t bf_imu_cali_gyro_offset_yaw;  // 手动 yaw 轴偏移（centidegrees）

/* 默认值 */
static const int16_t bf_imu_cali_gyro_offset_yaw_default = 0;  // 0 centidegrees

static const param_default_t bf_imu_cali_offset_defaults[] = {
    {&bf_imu_cali_gyro_offset_yaw, &bf_imu_cali_gyro_offset_yaw_default},
};

static void bf_imu_cali_offset_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_imu_cali_offset_defaults) / sizeof(bf_imu_cali_offset_defaults[0])); i++) {
        if (address == bf_imu_cali_offset_defaults[i].param) {
            memcpy(address, bf_imu_cali_offset_defaults[i].default_value, size);
            break;
        }
    }
}

static param_list bf_imu_cali_offset_params[] = {
    {(void *)&bf_imu_cali_gyro_offset_yaw, sizeof(bf_imu_cali_gyro_offset_yaw), BF_IMU_CALI_OFFSET_PARAM_GYRO_OFFSET_YAW, "i16",
     bf_imu_cali_offset_param_default},
};

param_list *bfImuCaliOffsetParam_list(void) { return bf_imu_cali_offset_params; }

size_t bfImuCaliOffsetParam_count(void) {
    return sizeof(bf_imu_cali_offset_params) / sizeof(bf_imu_cali_offset_params[0]);
}

