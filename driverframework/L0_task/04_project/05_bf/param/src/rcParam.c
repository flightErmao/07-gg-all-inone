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

/* Channel Range - 每个通道的范围配置 (min, max) */
/* 使用与 rc_bf.h 中相同的结构体定义 */
typedef struct {
    uint16_t min;
    uint16_t max;
} channel_range_t;

static channel_range_t rc_channel_range_roll;    // Roll 通道范围 {min, max}
static channel_range_t rc_channel_range_pitch;   // Pitch 通道范围 {min, max}
static channel_range_t rc_channel_range_yaw;     // Yaw 通道范围 {min, max}
static channel_range_t rc_channel_range_throttle; // Throttle 通道范围 {min, max}

/* Channel Mapping - 通道映射字符串 (如 "AETR1234" 或 "TAER1234") */
/* A = Aileron (Roll), E = Elevator (Pitch), T = Throttle, R = Rudder (Yaw) */
/* 1,2,3,4... = Aux channels */
#define RC_CHANNEL_MAP_STRING_MAX_LEN 18
static char rc_channel_map_string[RC_CHANNEL_MAP_STRING_MAX_LEN + 1];  // +1 for null terminator

/* RC Smoothing - RC 平滑滤波参数 */
static float rc_smoothing_setpoint_cutoff;       // Setpoint 截止频率 (Hz, 0=自动)
static float rc_smoothing_throttle_cutoff;       // Throttle 截止频率 (Hz, 0=自动)
static float rc_smoothing_auto_factor_rpy;       // Roll/Pitch/Yaw 自动平滑因子 (0-250, 用于计算 autoSmoothnessFactor)
static float rc_smoothing_auto_factor_throttle;  // Throttle 自动平滑因子 (0-250, 用于计算 autoSmoothnessFactor)
static uint8_t rc_smoothing_enabled;             // 是否启用 RC smoothing (0=禁用, 1=启用)

/* 默认值 - Betaflight 典型配置 */
static const float rc_rate_default[3] = {100.0f, 100.0f, 100.0f};     // Default RC rate [roll, pitch, yaw]
static const float rc_expo_default[3] = {0.0f, 0.0f, 0.0f};          // No expo by default
static const float rc_super_rate_default[3] = {0.0f, 0.0f, 0.0f};    // No super rate by default
static const float rc_rate_limit_default[3] = {720.0f, 720.0f, 720.0f};  // Default 720 deg/s limit

static const float rc_deadband_default = 10.0f;     // No deadband by default
static const float rc_yaw_deadband_default = 2.0f;  // No yaw deadband by default

/* 默认通道范围 - 1000-2000 (PWM_RANGE_MIN to PWM_RANGE_MAX) */
static const channel_range_t rc_channel_range_roll_default = {1068, 1932};
static const channel_range_t rc_channel_range_pitch_default = {1066, 1932};
static const channel_range_t rc_channel_range_yaw_default = {1067, 1932};
static const channel_range_t rc_channel_range_throttle_default = {1000, 2000};

/* 默认通道映射 - AETR1234 (Aileron, Elevator, Throttle, Rudder, Aux1, Aux2, Aux3, Aux4) */
static const char rc_channel_map_string_default[] = "AETR1234";

/* 默认 RC Smoothing 参数 */
static const float rc_smoothing_setpoint_cutoff_default = 0.0f;       // 0 = 自动
static const float rc_smoothing_throttle_cutoff_default = 0.0f;       // 0 = 自动
static const float rc_smoothing_auto_factor_rpy_default = 0.0f;       // 默认值，对应 autoSmoothnessFactor = 1.5
static const float rc_smoothing_auto_factor_throttle_default = 0.0f;  // 默认值，对应 autoSmoothnessFactor = 1.5
static const uint8_t rc_smoothing_enabled_default = 1;                // 默认启用

static const param_default_t bf_rc_defaults[] = {
    {rc_rate, rc_rate_default},
    {rc_expo, rc_expo_default},
    {rc_super_rate, rc_super_rate_default},
    {rc_rate_limit, rc_rate_limit_default},
    {&rc_deadband, &rc_deadband_default},
    {&rc_yaw_deadband, &rc_yaw_deadband_default},
    {&rc_channel_range_roll, &rc_channel_range_roll_default},
    {&rc_channel_range_pitch, &rc_channel_range_pitch_default},
    {&rc_channel_range_yaw, &rc_channel_range_yaw_default},
    {&rc_channel_range_throttle, &rc_channel_range_throttle_default},
    {rc_channel_map_string, rc_channel_map_string_default},
    {&rc_smoothing_setpoint_cutoff, &rc_smoothing_setpoint_cutoff_default},
    {&rc_smoothing_throttle_cutoff, &rc_smoothing_throttle_cutoff_default},
    {&rc_smoothing_auto_factor_rpy, &rc_smoothing_auto_factor_rpy_default},
    {&rc_smoothing_auto_factor_throttle, &rc_smoothing_auto_factor_throttle_default},
    {&rc_smoothing_enabled, &rc_smoothing_enabled_default},
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
    {(void *)rc_rate, sizeof(rc_rate), "rc_rate", "vf", bf_rc_param_default},                    // [roll, pitch, yaw]
    {(void *)rc_expo, sizeof(rc_expo), "rc_expo", "vf", bf_rc_param_default},                    // [roll, pitch, yaw]
    {(void *)rc_super_rate, sizeof(rc_super_rate), "rc_super_rate", "vf", bf_rc_param_default},  // [roll, pitch, yaw]
    {(void *)rc_rate_limit, sizeof(rc_rate_limit), "rc_rate_limit", "vf", bf_rc_param_default},  // [roll, pitch, yaw]
    {(void *)&rc_deadband, sizeof(rc_deadband), "rc_deadband", "f", bf_rc_param_default},
    {(void *)&rc_yaw_deadband, sizeof(rc_yaw_deadband), "rc_yaw_deadband", "f", bf_rc_param_default},
    {(void *)&rc_channel_range_roll, sizeof(rc_channel_range_roll), "rc_channel_range_roll", "vw",
     bf_rc_param_default},  // {min, max} - word vector (2 uint16_t)
    {(void *)&rc_channel_range_pitch, sizeof(rc_channel_range_pitch), "rc_channel_range_pitch", "vw",
     bf_rc_param_default},  // {min, max} - word vector (2 uint16_t)
    {(void *)&rc_channel_range_yaw, sizeof(rc_channel_range_yaw), "rc_channel_range_yaw", "vw",
     bf_rc_param_default},  // {min, max} - word vector (2 uint16_t)
    {(void *)&rc_channel_range_throttle, sizeof(rc_channel_range_throttle), "rc_channel_range_throttle", "vw",
     bf_rc_param_default},  // {min, max} - word vector (2 uint16_t)
    {(void *)rc_channel_map_string, sizeof(rc_channel_map_string), "rc_channel_map", "s",
     bf_rc_param_default},  // Channel mapping string (e.g., "AETR1234")
    {(void *)&rc_smoothing_setpoint_cutoff, sizeof(rc_smoothing_setpoint_cutoff), "rc_smoothing_setpoint_cutoff", "f",
     bf_rc_param_default},  // Setpoint cutoff frequency (Hz, 0=auto)
    {(void *)&rc_smoothing_throttle_cutoff, sizeof(rc_smoothing_throttle_cutoff), "rc_smoothing_throttle_cutoff", "f",
     bf_rc_param_default},  // Throttle cutoff frequency (Hz, 0=auto)
    {(void *)&rc_smoothing_auto_factor_rpy, sizeof(rc_smoothing_auto_factor_rpy), "rc_smoothing_auto_factor_rpy", "f",
     bf_rc_param_default},  // Auto smoothness factor for RPY (0-250)
    {(void *)&rc_smoothing_auto_factor_throttle, sizeof(rc_smoothing_auto_factor_throttle),
     "rc_smoothing_auto_factor_throttle", "f", bf_rc_param_default},  // Auto smoothness factor for throttle (0-250)
    {(void *)&rc_smoothing_enabled, sizeof(rc_smoothing_enabled), "rc_smoothing_enabled", "u8",
     bf_rc_param_default},  // Enable RC smoothing (0=disabled, 1=enabled)
};

param_list *bfRcParam_list(void) { return bf_rc_params; }

size_t bfRcParam_count(void) { return sizeof(bf_rc_params) / sizeof(bf_rc_params[0]); }

