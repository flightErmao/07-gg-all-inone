#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "rcParam.h"

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

/* Channel Range - 每个通道的范围配置 (min, max)，初始化为默认值 */
static channel_range_t rc_channel_range_roll = {1068, 1932};   // Roll 通道范围 {min, max}
static channel_range_t rc_channel_range_pitch = {1066, 1932};  // Pitch 通道范围 {min, max} - 修复：直接初始化为默认值
static channel_range_t rc_channel_range_yaw = {1067, 1932};    // Yaw 通道范围 {min, max}
static channel_range_t rc_channel_range_throttle = {1000, 2000};  // Throttle 通道范围 {min, max}

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

/* RC Controls - RC 指令处理参数 */
/* 摇杆位置检测阈值 */
static uint16_t rc_mincheck;     // 最小位置阈值 (用于判断摇杆是否在最小位置，默认1050)
static uint16_t rc_maxcheck;     // 最大位置阈值 (用于判断摇杆是否在最大位置，默认1900)
static uint16_t rc_midrc;        // 中间值 (默认1500)
static uint16_t rc_rx_min_usec;  // RX最小值 (默认1000)
static uint16_t rc_rx_max_usec;  // RX最大值 (默认2000)

/* 解锁/锁定参数 */
static uint16_t rc_arm_delay_ms;          // 解锁延迟 (ms, 默认500)
static uint16_t rc_stick_delay_ms;        // 摇杆延迟 (ms, 默认50)
static uint16_t rc_stick_autorepeat_ms;   // 摇杆自动重复延迟 (ms, 默认250)
static uint8_t rc_use_stick_arming;       // 是否使用摇杆解锁 (0=禁用, 1=启用, 默认1)
static uint8_t rc_auto_disarm_delay;      // 自动解锁延迟 (秒, 0=禁用, 默认5)
static uint8_t rc_gyro_cal_on_first_arm;  // 首次解锁时校准陀螺仪 (0=禁用, 1=启用, 默认0)

/* AUX通道配置 - 解锁 */
static uint8_t rc_arm_aux_channel;          // 解锁AUX通道号 (0=禁用, 默认6, 通道索引从1开始，实际为AUX2)
static uint16_t rc_arm_aux_threshold_low;   // 解锁阈值低 (默认1400)
static uint16_t rc_arm_aux_threshold_high;  // 解锁阈值高 (默认1600)

/* AUX通道配置 - 飞行模式 */
static uint8_t rc_mode_aux_channel;          // 飞行模式AUX通道号 (0=禁用, 默认5, 通道索引从1开始，实际为AUX1)
static uint16_t rc_mode_aux_threshold_low;   // 飞行模式阈值低 (默认1400)
static uint16_t rc_mode_aux_threshold_high;  // 飞行模式阈值高 (默认1600)

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

/* 默认 RC Controls 参数 */
static const uint16_t rc_mincheck_default = 1050;                 // 最小位置阈值
static const uint16_t rc_maxcheck_default = 1900;                 // 最大位置阈值
static const uint16_t rc_midrc_default = 1500;                    // 中间值
static const uint16_t rc_rx_min_usec_default = 1000;              // RX最小值
static const uint16_t rc_rx_max_usec_default = 2000;              // RX最大值
static const uint16_t rc_arm_delay_ms_default = 500;              // 解锁延迟 500ms
static const uint16_t rc_stick_delay_ms_default = 50;             // 摇杆延迟 50ms
static const uint16_t rc_stick_autorepeat_ms_default = 250;       // 摇杆自动重复延迟 250ms
static const uint8_t rc_use_stick_arming_default = 0;             // 默认使用摇杆解锁
static const uint8_t rc_auto_disarm_delay_default = 5;            // 默认自动解锁延迟 5秒
static const uint8_t rc_gyro_cal_on_first_arm_default = 0;        // 默认不校准
static const uint8_t rc_arm_aux_channel_default = 6;              // 默认通道6 (AUX2)用于解锁
static const uint16_t rc_arm_aux_threshold_low_default = 1400;    // 解锁阈值低
static const uint16_t rc_arm_aux_threshold_high_default = 1600;   // 解锁阈值高
static const uint8_t rc_mode_aux_channel_default = 5;             // 默认通道5 (AUX1)用于飞行模式
static const uint16_t rc_mode_aux_threshold_low_default = 1400;   // 飞行模式阈值低
static const uint16_t rc_mode_aux_threshold_high_default = 1600;  // 飞行模式阈值高

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
    {&rc_mincheck, &rc_mincheck_default},
    {&rc_maxcheck, &rc_maxcheck_default},
    {&rc_midrc, &rc_midrc_default},
    {&rc_rx_min_usec, &rc_rx_min_usec_default},
    {&rc_rx_max_usec, &rc_rx_max_usec_default},
    {&rc_arm_delay_ms, &rc_arm_delay_ms_default},
    {&rc_stick_delay_ms, &rc_stick_delay_ms_default},
    {&rc_stick_autorepeat_ms, &rc_stick_autorepeat_ms_default},
    {&rc_use_stick_arming, &rc_use_stick_arming_default},
    {&rc_auto_disarm_delay, &rc_auto_disarm_delay_default},
    {&rc_gyro_cal_on_first_arm, &rc_gyro_cal_on_first_arm_default},
    {&rc_arm_aux_channel, &rc_arm_aux_channel_default},
    {&rc_arm_aux_threshold_low, &rc_arm_aux_threshold_low_default},
    {&rc_arm_aux_threshold_high, &rc_arm_aux_threshold_high_default},
    {&rc_mode_aux_channel, &rc_mode_aux_channel_default},
    {&rc_mode_aux_threshold_low, &rc_mode_aux_threshold_low_default},
    {&rc_mode_aux_threshold_high, &rc_mode_aux_threshold_high_default},
};

static void bf_rc_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_rc_defaults) / sizeof(bf_rc_defaults[0])); i++) {
        if (address == bf_rc_defaults[i].param) {
            memcpy(address, bf_rc_defaults[i].default_value, size);
            break;
        }
    }
    /* 额外检查：如果地址匹配失败，对于 rc_channel_range_pitch 进行特殊处理 */
    /* 这确保即使地址比较失败，也能正确恢复默认值 */
    if (address == &rc_channel_range_pitch && size == sizeof(rc_channel_range_pitch)) {
      /* 检查是否需要恢复默认值（如果当前值为 0 0） */
      if (rc_channel_range_pitch.min == 0 && rc_channel_range_pitch.max == 0) {
        rc_channel_range_pitch = rc_channel_range_pitch_default;
      }
    }
}

static param_list bf_rc_params[] = {
    {(void*)rc_rate, sizeof(rc_rate), "rc_rate", "vf", bf_rc_param_default},                    // [roll, pitch, yaw]
    {(void*)rc_expo, sizeof(rc_expo), "rc_expo", "vf", bf_rc_param_default},                    // [roll, pitch, yaw]
    {(void*)rc_super_rate, sizeof(rc_super_rate), "rc_super_rate", "vf", bf_rc_param_default},  // [roll, pitch, yaw]
    {(void*)rc_rate_limit, sizeof(rc_rate_limit), "rc_rate_limit", "vf", bf_rc_param_default},  // [roll, pitch, yaw]
    {(void*)&rc_deadband, sizeof(rc_deadband), "rc_deadband", "f", bf_rc_param_default},
    {(void*)&rc_yaw_deadband, sizeof(rc_yaw_deadband), "rc_yaw_deadband", "f", bf_rc_param_default},
    {(void*)&rc_channel_range_roll, sizeof(rc_channel_range_roll), "rc_channel_range_roll", "vw",
     bf_rc_param_default},  // {min, max} - word vector (2 uint16_t)
    {(void*)&rc_channel_range_pitch, sizeof(rc_channel_range_pitch), "rc_channel_range_pitch", "vw",
     bf_rc_param_default},  // {min, max} - word vector (2 uint16_t)
    {(void*)&rc_channel_range_yaw, sizeof(rc_channel_range_yaw), "rc_channel_range_yaw", "vw",
     bf_rc_param_default},  // {min, max} - word vector (2 uint16_t)
    {(void*)&rc_channel_range_throttle, sizeof(rc_channel_range_throttle), "rc_channel_range_throttle", "vw",
     bf_rc_param_default},  // {min, max} - word vector (2 uint16_t)
    {(void*)rc_channel_map_string, sizeof(rc_channel_map_string), "rc_channel_map", "s",
     bf_rc_param_default},  // Channel mapping string (e.g., "AETR1234")
    {(void*)&rc_smoothing_setpoint_cutoff, sizeof(rc_smoothing_setpoint_cutoff), "rc_smoothing_setpoint_cutoff", "f",
     bf_rc_param_default},  // Setpoint cutoff frequency (Hz, 0=auto)
    {(void*)&rc_smoothing_throttle_cutoff, sizeof(rc_smoothing_throttle_cutoff), "rc_smoothing_throttle_cutoff", "f",
     bf_rc_param_default},  // Throttle cutoff frequency (Hz, 0=auto)
    {(void*)&rc_smoothing_auto_factor_rpy, sizeof(rc_smoothing_auto_factor_rpy), "rc_smoothing_auto_factor_rpy", "f",
     bf_rc_param_default},  // Auto smoothness factor for RPY (0-250)
    {(void*)&rc_smoothing_auto_factor_throttle, sizeof(rc_smoothing_auto_factor_throttle),
     "rc_smoothing_auto_factor_throttle", "f", bf_rc_param_default},  // Auto smoothness factor for throttle (0-250)
    {(void*)&rc_smoothing_enabled, sizeof(rc_smoothing_enabled), "rc_smoothing_enabled", "u8",
     bf_rc_param_default},  // Enable RC smoothing (0=disabled, 1=enabled)
    {(void*)&rc_mincheck, sizeof(rc_mincheck), "rc_mincheck", "u16", bf_rc_param_default},  // Min position threshold
    {(void*)&rc_maxcheck, sizeof(rc_maxcheck), "rc_maxcheck", "u16", bf_rc_param_default},  // Max position threshold
    {(void*)&rc_midrc, sizeof(rc_midrc), "rc_midrc", "u16", bf_rc_param_default},           // Middle position value
    {(void*)&rc_rx_min_usec, sizeof(rc_rx_min_usec), "rc_rx_min_usec", "u16",
     bf_rc_param_default},  // RX minimum value (usec)
    {(void*)&rc_rx_max_usec, sizeof(rc_rx_max_usec), "rc_rx_max_usec", "u16",
     bf_rc_param_default},  // RX maximum value (usec)
    {(void*)&rc_arm_delay_ms, sizeof(rc_arm_delay_ms), "rc_arm_delay_ms", "u16",
     bf_rc_param_default},  // Arm delay (ms)
    {(void*)&rc_stick_delay_ms, sizeof(rc_stick_delay_ms), "rc_stick_delay_ms", "u16",
     bf_rc_param_default},  // Stick delay (ms)
    {(void*)&rc_stick_autorepeat_ms, sizeof(rc_stick_autorepeat_ms), "rc_stick_autorepeat_ms", "u16",
     bf_rc_param_default},  // Stick autorepeat delay (ms)
    {(void*)&rc_use_stick_arming, sizeof(rc_use_stick_arming), "rc_use_stick_arming", "u8",
     bf_rc_param_default},  // Use stick arming (0=disabled, 1=enabled)
    {(void*)&rc_auto_disarm_delay, sizeof(rc_auto_disarm_delay), "rc_auto_disarm_delay", "u8",
     bf_rc_param_default},  // Auto disarm delay (seconds, 0=disabled)
    {(void*)&rc_gyro_cal_on_first_arm, sizeof(rc_gyro_cal_on_first_arm), "rc_gyro_cal_on_first_arm", "u8",
     bf_rc_param_default},  // Calibrate gyro on first arm (0=disabled, 1=enabled)
    {(void*)&rc_arm_aux_channel, sizeof(rc_arm_aux_channel), "rc_arm_aux_channel", "u8",
     bf_rc_param_default},  // Arm AUX channel (0=disabled, 5=AUX1, 6=AUX2, ...)
    {(void*)&rc_arm_aux_threshold_low, sizeof(rc_arm_aux_threshold_low), "rc_arm_aux_threshold_low", "u16",
     bf_rc_param_default},  // Arm AUX threshold low
    {(void*)&rc_arm_aux_threshold_high, sizeof(rc_arm_aux_threshold_high), "rc_arm_aux_threshold_high", "u16",
     bf_rc_param_default},  // Arm AUX threshold high
    {(void*)&rc_mode_aux_channel, sizeof(rc_mode_aux_channel), "rc_mode_aux_channel", "u8",
     bf_rc_param_default},  // Mode AUX channel (0=disabled, 5=AUX1, 6=AUX2, ...)
    {(void*)&rc_mode_aux_threshold_low, sizeof(rc_mode_aux_threshold_low), "rc_mode_aux_threshold_low", "u16",
     bf_rc_param_default},  // Mode AUX threshold low
    {(void*)&rc_mode_aux_threshold_high, sizeof(rc_mode_aux_threshold_high), "rc_mode_aux_threshold_high", "u16",
     bf_rc_param_default},  // Mode AUX threshold high
};

param_list *bfRcParam_list(void) { return bf_rc_params; }

size_t bfRcParam_count(void) { return sizeof(bf_rc_params) / sizeof(bf_rc_params[0]); }

