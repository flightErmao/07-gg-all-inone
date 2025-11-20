#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bfRcParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void bf_rc_param_default(void *address, uint8_t size);

/* Betaflight 风格 RC 速率参数 */

/* RC Rates - 控制基础灵敏度 (0-200) [roll, pitch, yaw] */
static float rc_rate[3];

/* RC Expo - 控制曲线的弯曲程度 (0-100) [roll, pitch, yaw] */
static float rc_expo[3];

/* Super Rates - 控制最大速率时的额外灵敏度 (0-100) [roll, pitch, yaw] */
static float rc_super_rate[3];

/* Rate Limits - 限制最大角速度 (deg/s) [roll, pitch, yaw] */
static float rc_rate_limit[3];

/* RC Deadband - 死区设置 */
static float rc_deadband;      // Roll/Pitch 死区
static float rc_yaw_deadband;  // Yaw 死区

/* 默认值 - Betaflight 典型配置 */
static const float rc_rate_default[3] = {100.0f, 100.0f, 100.0f};     // Default RC rate [roll, pitch, yaw]
static const float rc_expo_default[3] = {0.0f, 0.0f, 0.0f};          // No expo by default
static const float rc_super_rate_default[3] = {0.0f, 0.0f, 0.0f};    // No super rate by default
static const float rc_rate_limit_default[3] = {720.0f, 720.0f, 720.0f};  // Default 720 deg/s limit

static const float rc_deadband_default = 0.0f;       // No deadband by default
static const float rc_yaw_deadband_default = 0.0f;   // No yaw deadband by default

static const param_default_t bf_rc_defaults[] = {
    {rc_rate, rc_rate_default},
    {rc_expo, rc_expo_default},
    {rc_super_rate, rc_super_rate_default},
    {rc_rate_limit, rc_rate_limit_default},
    {&rc_deadband, &rc_deadband_default},
    {&rc_yaw_deadband, &rc_yaw_deadband_default},
};

static void bf_rc_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_rc_defaults) / sizeof(bf_rc_defaults[0])); i++) {
        if (address == bf_rc_defaults[i].param) {
            memcpy(address, bf_rc_defaults[i].default_value, size);
            break;
        }
    }
}

static param_list bf_rc_params[] = {
    {(void*)rc_rate, sizeof(rc_rate), "rc_rate", "vf", bf_rc_param_default},           // [roll, pitch, yaw]
    {(void*)rc_expo, sizeof(rc_expo), "rc_expo", "vf", bf_rc_param_default},           // [roll, pitch, yaw]
    {(void*)rc_super_rate, sizeof(rc_super_rate), "rc_super_rate", "vf", bf_rc_param_default},  // [roll, pitch, yaw]
    {(void*)rc_rate_limit, sizeof(rc_rate_limit), "rc_rate_limit", "vf", bf_rc_param_default},  // [roll, pitch, yaw]
    {(void*)&rc_deadband, sizeof(rc_deadband), "rc_deadband", "f", bf_rc_param_default},
    {(void*)&rc_yaw_deadband, sizeof(rc_yaw_deadband), "rc_yaw_deadband", "f", bf_rc_param_default},
};

param_list *bfRcParam_list(void) { return bf_rc_params; }

size_t bfRcParam_count(void) { return sizeof(bf_rc_params) / sizeof(bf_rc_params[0]); }

