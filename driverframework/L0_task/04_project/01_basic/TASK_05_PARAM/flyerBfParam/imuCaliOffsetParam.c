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

/* 加速度计校准偏移参数（3个float，对应 x, y, z 轴） */
static float bf_imu_cali_acc_offset[3];  // 加速度计偏移值（m/s^2）

/* 默认值 */
static const int16_t bf_imu_cali_gyro_offset_yaw_default = 0;  // 0 centidegrees

static const float bf_imu_cali_acc_offset_default[3] = {0.0f, 0.0f, 0.0f};  // 默认无偏移

static const param_default_t bf_imu_cali_offset_defaults[] = {
    {&bf_imu_cali_gyro_offset_yaw, &bf_imu_cali_gyro_offset_yaw_default},
    {bf_imu_cali_acc_offset, bf_imu_cali_acc_offset_default},
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
    {(void *)&bf_imu_cali_gyro_offset_yaw, sizeof(bf_imu_cali_gyro_offset_yaw), BF_IMU_CALI_OFFSET_PARAM_GYRO_OFFSET_YAW, "d",
     bf_imu_cali_offset_param_default},
    {(void *)bf_imu_cali_acc_offset, sizeof(bf_imu_cali_acc_offset), "cali_imu_acc_offset", "vf",
     bf_imu_cali_offset_param_default},
};

param_list *bfImuCaliOffsetParam_list(void) { return bf_imu_cali_offset_params; }

size_t bfImuCaliOffsetParam_count(void) {
    return sizeof(bf_imu_cali_offset_params) / sizeof(bf_imu_cali_offset_params[0]);
}
