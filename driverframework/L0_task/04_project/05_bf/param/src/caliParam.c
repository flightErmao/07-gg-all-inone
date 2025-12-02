#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <rtconfig.h>  // For PROJECT_BF_ACC_EN, PROJECT_BF_GYRO_FILTER_EN

#include "caliParam.h"
#include "boardalignment.h"  // For boardAlignment_t

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void bf_imu_cali_offset_param_default(void *address, uint8_t size);
static void bf_imu_cali_align_param_default(void *address, uint8_t size);

/* Betaflight 风格 IMU 校准偏移参数 */
/* 注意：gyro 零偏值在运行时计算，不保存到参数系统 */
static int16_t bf_imu_cali_gyro_offset_yaw;  // 手动 yaw 轴偏移（centidegrees）

/* 加速度计校准偏移参数（3个float，对应 x, y, z 轴） */
static float bf_imu_cali_acc_offset[3];  // 加速度计偏移值（m/s^2）

/* 加速度计 Trim 参数（用于姿态估计时的角度修正） */
static float cali_acc_trim_roll;   // Roll Trim（deg）
static float cali_acc_trim_pitch;  // Pitch Trim（deg）

/* IMU 对齐参数（acc 和 gyro 共用） */
static uint8_t imu_align_method;  // IMU 对齐方式 (sensor_align_e)
static int16_t imu_custom_align[3];  // IMU 自定义对齐角度 [roll, pitch, yaw] (decidegrees)

/* 板级对齐参数（Betaflight 风格） */
static boardAlignment_t board_alignment;  // 板级对齐角度 [roll, pitch, yaw] (degrees)

/* 默认值 */
static const int16_t bf_imu_cali_gyro_offset_yaw_default = 0;  // 0 centidegrees

static const float bf_imu_cali_acc_offset_default[3] = {0.0f, 0.0f, 0.0f};  // 默认无偏移

/* 加速度计 Trim 参数默认值 */
static const float cali_acc_trim_roll_default = 0.0f;
static const float cali_acc_trim_pitch_default = 0.0f;

/* IMU 对齐参数默认值 */
static const uint8_t imu_align_method_default = 7;             // ALIGN_DEFAULT
static const int16_t imu_custom_align_default[3] = {0, 0, 0};  // 默认无自定义对齐

/* 板级对齐参数默认值 */
static const boardAlignment_t board_alignment_default = {180, 0, 0};  // 默认无板级对齐

static const param_default_t bf_imu_cali_offset_defaults[] = {
    {&bf_imu_cali_gyro_offset_yaw, &bf_imu_cali_gyro_offset_yaw_default},
    {bf_imu_cali_acc_offset, bf_imu_cali_acc_offset_default},
    {&cali_acc_trim_roll, &cali_acc_trim_roll_default},
    {&cali_acc_trim_pitch, &cali_acc_trim_pitch_default},
};

static const param_default_t bf_imu_cali_align_defaults[] = {
    {&imu_align_method, &imu_align_method_default},
    {imu_custom_align, imu_custom_align_default},
    {&board_alignment, &board_alignment_default},
};

static void bf_imu_cali_offset_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_imu_cali_offset_defaults) / sizeof(bf_imu_cali_offset_defaults[0])); i++) {
        if (address == bf_imu_cali_offset_defaults[i].param) {
            memcpy(address, bf_imu_cali_offset_defaults[i].default_value, size);
            break;
        }
    }
}

static void bf_imu_cali_align_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_imu_cali_align_defaults) / sizeof(bf_imu_cali_align_defaults[0])); i++) {
        if (address == bf_imu_cali_align_defaults[i].param) {
            memcpy(address, bf_imu_cali_align_defaults[i].default_value, size);
            break;
        }
    }
}

static param_list bf_imu_cali_offset_params[] = {
    {(void *)&bf_imu_cali_gyro_offset_yaw, sizeof(bf_imu_cali_gyro_offset_yaw), BF_IMU_CALI_OFFSET_PARAM_GYRO_OFFSET_YAW, "d",
     bf_imu_cali_offset_param_default},
    {(void *)bf_imu_cali_acc_offset, sizeof(bf_imu_cali_acc_offset), BF_IMU_CALI_OFFSET_PARAM_ACC_OFFSET, "vf",
     bf_imu_cali_offset_param_default},
    {(void *)&cali_acc_trim_roll, sizeof(cali_acc_trim_roll), "cali_acc_trim_roll", "f",
     bf_imu_cali_offset_param_default},
    {(void *)&cali_acc_trim_pitch, sizeof(cali_acc_trim_pitch), "cali_acc_trim_pitch", "f",
     bf_imu_cali_offset_param_default},
    /* IMU 对齐参数 */
    {(void*)&imu_align_method, sizeof(imu_align_method), BF_IMU_CALI_PARAM_ALIGN_METHOD, "u8", bf_imu_cali_align_param_default},
    {(void*)imu_custom_align, sizeof(imu_custom_align), BF_IMU_CALI_PARAM_CUSTOM_ALIGN, "vw", bf_imu_cali_align_param_default},
    /* 板级对齐参数 */
    {(void*)&board_alignment, sizeof(board_alignment), BF_IMU_CALI_PARAM_BOARD_ALIGNMENT, "vw", bf_imu_cali_align_param_default},
};

param_list *bfImuCaliOffsetParam_list(void) { return bf_imu_cali_offset_params; }

size_t bfImuCaliOffsetParam_count(void) {
    return sizeof(bf_imu_cali_offset_params) / sizeof(bf_imu_cali_offset_params[0]);
}

