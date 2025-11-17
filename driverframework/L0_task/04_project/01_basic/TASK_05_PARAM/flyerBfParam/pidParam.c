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

/* PID 积分限制 */
static float bf_rate_pid_roll_i_limit;
static float bf_rate_pid_pitch_i_limit;
static float bf_rate_pid_yaw_i_limit;

/* 默认值 - Betaflight 典型配置 */
static const float bf_rate_pid_roll_default[3] = {42.0f, 65.0f, 29.0f};    // 典型 Betaflight 默认值
static const float bf_rate_pid_pitch_default[3] = {45.0f, 62.0f, 31.0f};
static const float bf_rate_pid_yaw_default[3] = {30.0f, 50.0f, 0.0f};

static const float bf_angle_pid_roll_default[3] = {0.0f, 0.0f, 0.0f};
static const float bf_angle_pid_pitch_default[3] = {0.0f, 0.0f, 0.0f};
static const float bf_angle_pid_yaw_default[3] = {0.0f, 0.0f, 0.0f};

static const float bf_rate_pid_roll_i_limit_default = 100.0f;
static const float bf_rate_pid_pitch_i_limit_default = 100.0f;
static const float bf_rate_pid_yaw_i_limit_default = 100.0f;

static const param_default_t bf_pid_defaults[] = {
    {bf_rate_pid_roll, bf_rate_pid_roll_default},
    {bf_rate_pid_pitch, bf_rate_pid_pitch_default},
    {bf_rate_pid_yaw, bf_rate_pid_yaw_default},
    {bf_angle_pid_roll, bf_angle_pid_roll_default},
    {bf_angle_pid_pitch, bf_angle_pid_pitch_default},
    {bf_angle_pid_yaw, bf_angle_pid_yaw_default},
    {&bf_rate_pid_roll_i_limit, &bf_rate_pid_roll_i_limit_default},
    {&bf_rate_pid_pitch_i_limit, &bf_rate_pid_pitch_i_limit_default},
    {&bf_rate_pid_yaw_i_limit, &bf_rate_pid_yaw_i_limit_default},
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
    {(void *)bf_rate_pid_roll, sizeof(bf_rate_pid_roll), "bf_rate_pid_roll", "vf", bf_pid_param_default},
    {(void *)bf_rate_pid_pitch, sizeof(bf_rate_pid_pitch), "bf_rate_pid_pitch", "vf", bf_pid_param_default},
    {(void *)bf_rate_pid_yaw, sizeof(bf_rate_pid_yaw), "bf_rate_pid_yaw", "vf", bf_pid_param_default},
    {(void *)bf_angle_pid_roll, sizeof(bf_angle_pid_roll), "bf_angle_pid_roll", "vf", bf_pid_param_default},
    {(void *)bf_angle_pid_pitch, sizeof(bf_angle_pid_pitch), "bf_angle_pid_pitch", "vf", bf_pid_param_default},
    {(void *)bf_angle_pid_yaw, sizeof(bf_angle_pid_yaw), "bf_angle_pid_yaw", "vf", bf_pid_param_default},
    {(void *)&bf_rate_pid_roll_i_limit, sizeof(bf_rate_pid_roll_i_limit), "bf_rate_pid_roll_i_limit", "f", bf_pid_param_default},
    {(void *)&bf_rate_pid_pitch_i_limit, sizeof(bf_rate_pid_pitch_i_limit), "bf_rate_pid_pitch_i_limit", "f", bf_pid_param_default},
    {(void *)&bf_rate_pid_yaw_i_limit, sizeof(bf_rate_pid_yaw_i_limit), "bf_rate_pid_yaw_i_limit", "f", bf_pid_param_default},
};

param_list *bfPidParam_list(void) { return bf_pid_params; }

size_t bfPidParam_count(void) { return sizeof(bf_pid_params) / sizeof(bf_pid_params[0]); }

