#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "mlogParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void bf_mlog_param_default(void *address, uint8_t size);

/* Betaflight 风格 Mlog 参数 */

/* Mlog 陀螺仪数据记录使能 */
static uint8_t mlog_gyro_en;  // 0=禁用, 1=使能
/* Mlog RC 数据记录使能 */
static uint8_t mlog_rc_en;  // 0=禁用, 1=使能
/* Mlog Motor 数据记录使能 */
static uint8_t mlog_motor_en;  // 0=禁用, 1=使能
/* Mlog PID 数据记录使能 */
static uint8_t mlog_pid_en;  // 0=禁用, 1=使能
/* Mlog RC 上锁控制使能 */
static uint8_t mlog_rc_arm_control;  // 0=禁用, 1=使能（上锁时启动log，解锁时停止log）

/* 默认值 */
static const uint8_t mlog_gyro_en_default = 1;  // 默认使用
static const uint8_t mlog_rc_en_default = 1;  // 默认使用
static const uint8_t mlog_motor_en_default = 1;  // 默认使用
static const uint8_t mlog_pid_en_default = 1;  // 默认使用
static const uint8_t mlog_rc_arm_control_default = 1;  // 默认使用

static const param_default_t bf_mlog_defaults[] = {
    {&mlog_gyro_en, &mlog_gyro_en_default},
    {&mlog_rc_en, &mlog_rc_en_default},
    {&mlog_motor_en, &mlog_motor_en_default},
    {&mlog_pid_en, &mlog_pid_en_default},
    {&mlog_rc_arm_control, &mlog_rc_arm_control_default},
};

static void bf_mlog_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_mlog_defaults) / sizeof(bf_mlog_defaults[0])); i++) {
        if (address == bf_mlog_defaults[i].param) {
            memcpy(address, bf_mlog_defaults[i].default_value, size);
            break;
        }
    }
}

static param_list bf_mlog_params[] = {
    {(void*)&mlog_gyro_en, sizeof(mlog_gyro_en), "mlog_gyro_en", "u8",
     bf_mlog_param_default},
    {(void*)&mlog_rc_en, sizeof(mlog_rc_en), "mlog_rc_en", "u8",
     bf_mlog_param_default},
    {(void*)&mlog_motor_en, sizeof(mlog_motor_en), "mlog_motor_en", "u8",
     bf_mlog_param_default},
    {(void*)&mlog_pid_en, sizeof(mlog_pid_en), "mlog_pid_en", "u8",
     bf_mlog_param_default},
    {(void*)&mlog_rc_arm_control, sizeof(mlog_rc_arm_control), "mlog_rc_arm_control", "u8",
     bf_mlog_param_default},
};

param_list *bfMlogParam_list(void) { return bf_mlog_params; }

size_t bfMlogParam_count(void) { return sizeof(bf_mlog_params) / sizeof(bf_mlog_params[0]); }

