#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <rtconfig.h>  // For PROJECT_BF_ACC_EN, PROJECT_BF_GYRO_FILTER_EN

#include "imuParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

/* IMU 对齐参数 */
#ifdef PROJECT_BF_ACC_EN
static uint8_t imu_align_acc;  // 加速度计对齐方式 (sensor_align_e)
static int16_t imu_custom_align_acc[3];  // 加速度计自定义对齐角度 [roll, pitch, yaw] (decidegrees)
#endif

#ifdef PROJECT_BF_GYRO_FILTER_EN
static uint8_t imu_align_gyro;  // 陀螺仪对齐方式 (sensor_align_e)
static int16_t imu_custom_align_gyro[3];  // 陀螺仪自定义对齐角度 [roll, pitch, yaw] (decidegrees)
#endif

/* 默认值 - 与 Betaflight 一致 */
#ifdef PROJECT_BF_ACC_EN
static const uint8_t imu_align_acc_default = 0;  // ALIGN_DEFAULT
static const int16_t imu_custom_align_acc_default[3] = {0, 0, 0};  // 默认无自定义对齐
#endif

#ifdef PROJECT_BF_GYRO_FILTER_EN
static const uint8_t imu_align_gyro_default = 0;  // ALIGN_DEFAULT
static const int16_t imu_custom_align_gyro_default[3] = {0, 0, 0};  // 默认无自定义对齐
#endif

#ifdef PROJECT_BF_ACC_EN
static const param_default_t bf_imu_acc_defaults[] = {
    {&imu_align_acc, &imu_align_acc_default},
    {imu_custom_align_acc, imu_custom_align_acc_default},
};

static void bf_imu_acc_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_imu_acc_defaults) / sizeof(bf_imu_acc_defaults[0])); i++) {
        if (address == bf_imu_acc_defaults[i].param) {
            memcpy(address, bf_imu_acc_defaults[i].default_value, size);
            break;
        }
    }
}
#endif

#ifdef PROJECT_BF_GYRO_FILTER_EN
static const param_default_t bf_imu_gyro_defaults[] = {
    {&imu_align_gyro, &imu_align_gyro_default},
    {imu_custom_align_gyro, imu_custom_align_gyro_default},
};

static void bf_imu_gyro_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_imu_gyro_defaults) / sizeof(bf_imu_gyro_defaults[0])); i++) {
        if (address == bf_imu_gyro_defaults[i].param) {
            memcpy(address, bf_imu_gyro_defaults[i].default_value, size);
            break;
        }
    }
}
#endif

/* 参数列表 */
static param_list bf_imu_params[] = {
#ifdef PROJECT_BF_ACC_EN
    {(void*)&imu_align_acc, sizeof(imu_align_acc), BF_IMU_PARAM_ALIGN_ACC, "u8", bf_imu_acc_param_default},
    {(void*)imu_custom_align_acc, sizeof(imu_custom_align_acc), BF_IMU_PARAM_CUSTOM_ALIGN_ACC, "vd", bf_imu_acc_param_default},
#endif
#ifdef PROJECT_BF_GYRO_FILTER_EN
    {(void*)&imu_align_gyro, sizeof(imu_align_gyro), BF_IMU_PARAM_ALIGN_GYRO, "u8", bf_imu_gyro_param_default},
    {(void*)imu_custom_align_gyro, sizeof(imu_custom_align_gyro), BF_IMU_PARAM_CUSTOM_ALIGN_GYRO, "vd", bf_imu_gyro_param_default},
#endif
};

param_list *bfImuParam_list(void) { 
    return bf_imu_params; 
}

size_t bfImuParam_count(void) { 
    return sizeof(bf_imu_params) / sizeof(bf_imu_params[0]); 
}

