#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bfPidParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void bf_pid_param_default(void *address, uint8_t size);

/* Betaflight 风格速率环 PID 参数 */
static float bf_rate_pid_roll[3];   // [P, I, D]
static float bf_rate_pid_pitch[3];  // [P, I, D]
static float bf_rate_pid_yaw[3];    // [P, I, D]

/* Betaflight 风格角度环 PID 参数（可选） */
static float bf_angle_pid_roll[3];   // [P, I, D]
static float bf_angle_pid_pitch[3];  // [P, I, D]
static float bf_angle_pid_yaw[3];    // [P, I, D]

/* PID 总和限制（PID Sum Limit）- 限制 P+I+D+F 的总和 */
/* 注意：这不是I项的单独限制，而是整个PID输出的限制 */
static float bf_pid_sum_limit;      // Roll/Pitch轴的PID sum限制
static float bf_pid_sum_limit_yaw;  // Yaw轴的PID sum限制

/* PID 处理分母（控制 PID 循环频率） */
static uint8_t pid_process_denom;  // 对应 activePidLoopDenom，用于计算 targetLooptime

/* 默认值 - Betaflight 典型配置 */
static const float bf_rate_pid_roll_default[3] = {42.0f, 65.0f, 29.0f};    // 典型 Betaflight 默认值
static const float bf_rate_pid_pitch_default[3] = {45.0f, 62.0f, 31.0f};
static const float bf_rate_pid_yaw_default[3] = {30.0f, 50.0f, 0.0f};

static const float bf_angle_pid_roll_default[3] = {0.0f, 0.0f, 0.0f};
static const float bf_angle_pid_pitch_default[3] = {0.0f, 0.0f, 0.0f};
static const float bf_angle_pid_yaw_default[3] = {0.0f, 0.0f, 0.0f};

/* PID sum限制的默认值（deg/s）- 与Betaflight一致 */
static const float bf_pid_sum_limit_default = 500.0f;      // Roll/Pitch轴默认500 deg/s
static const float bf_pid_sum_limit_yaw_default = 400.0f;   // Yaw轴默认400 deg/s

static const uint8_t pid_process_denom_default = 1;  // 默认值为 1，对应 activePidLoopDenom

static const param_default_t bf_pid_defaults[] = {
    {bf_rate_pid_roll, bf_rate_pid_roll_default},
    {bf_rate_pid_pitch, bf_rate_pid_pitch_default},
    {bf_rate_pid_yaw, bf_rate_pid_yaw_default},
    {bf_angle_pid_roll, bf_angle_pid_roll_default},
    {bf_angle_pid_pitch, bf_angle_pid_pitch_default},
    {bf_angle_pid_yaw, bf_angle_pid_yaw_default},
    {&bf_pid_sum_limit, &bf_pid_sum_limit_default},
    {&bf_pid_sum_limit_yaw, &bf_pid_sum_limit_yaw_default},
    {&pid_process_denom, &pid_process_denom_default},
};

static void bf_pid_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_pid_defaults) / sizeof(bf_pid_defaults[0])); i++) {
        if (address == bf_pid_defaults[i].param) {
            memcpy(address, bf_pid_defaults[i].default_value, size);
            break;
        }
    }
}

static param_list bf_pid_params[] = {
    {(void *)bf_rate_pid_roll, sizeof(bf_rate_pid_roll), "pid_rate_roll", "vf", bf_pid_param_default},
    {(void *)bf_rate_pid_pitch, sizeof(bf_rate_pid_pitch), "pid_rate_pitch", "vf", bf_pid_param_default},
    {(void *)bf_rate_pid_yaw, sizeof(bf_rate_pid_yaw), "pid_rate_yaw", "vf", bf_pid_param_default},
    {(void *)bf_angle_pid_roll, sizeof(bf_angle_pid_roll), "pid_angle_roll", "vf", bf_pid_param_default},
    {(void *)bf_angle_pid_pitch, sizeof(bf_angle_pid_pitch), "pid_angle_pitch", "vf", bf_pid_param_default},
    {(void *)bf_angle_pid_yaw, sizeof(bf_angle_pid_yaw), "pid_angle_yaw", "vf", bf_pid_param_default},
    {(void *)&bf_pid_sum_limit, sizeof(bf_pid_sum_limit), "pid_sum_limit", "f", bf_pid_param_default},
    {(void *)&bf_pid_sum_limit_yaw, sizeof(bf_pid_sum_limit_yaw), "pid_sum_limit_yaw", "f", bf_pid_param_default},
    {(void *)&pid_process_denom, sizeof(pid_process_denom), "pid_process_denom", "u8", bf_pid_param_default},
};

param_list *bfPidParam_list(void) { return bf_pid_params; }

size_t bfPidParam_count(void) { return sizeof(bf_pid_params) / sizeof(bf_pid_params[0]); }

