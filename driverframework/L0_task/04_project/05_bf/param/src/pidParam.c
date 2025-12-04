#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <rtconfig.h>  // For PROJECT_BF_PID_D_MAX_EN

#include "pidParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void bf_pid_param_default(void *address, uint8_t size);

/* Betaflight 风格速率环 PID 参数 */
static float bf_rate_pid_roll[3];   // [P, I, D]
static float bf_rate_pid_pitch[3];  // [P, I, D]
static float bf_rate_pid_yaw[3];    // [P, I, D]

/* Rate PID 总和限制（Rate PID Sum Limit）- 限制 rate PID 的 P+I+D+F 总和 */
/* 注意：这不是I项的单独限制，而是整个 rate PID 输出的限制（单位：deg/s） */
static float bf_pid_rate_sum_limit;      // Roll/Pitch轴的 rate PID sum限制
static float bf_pid_rate_sum_limit_yaw;  // Yaw轴的 rate PID sum限制

/* PID 滤波器参数 */
static float bf_pid_iterm_windup;   // I项windup百分比（默认80%）

/* Dterm 滤波器参数（与Betaflight一致） */
static uint16_t bf_pid_dterm_notch_hz;        // Dterm陷波滤波器中心频率（Hz）
static uint16_t bf_pid_dterm_notch_cutoff;    // Dterm陷波滤波器截止频率（Hz）
static uint16_t bf_pid_dterm_lpf1_static_hz;  // Dterm低通滤波器1静态截止频率（Hz）
static uint8_t bf_pid_dterm_lpf1_type;        // Dterm低通滤波器1类型（0=PT1, 1=BIQUAD, 2=PT2, 3=PT3）
static uint16_t bf_pid_dterm_lpf2_static_hz;  // Dterm低通滤波器2静态截止频率（Hz）
static uint8_t bf_pid_dterm_lpf2_type;        // Dterm低通滤波器2类型（0=PT1, 1=BIQUAD, 2=PT2, 3=PT3）

/* Dynamic LPF 参数（与Betaflight一致） */
static uint16_t bf_pid_dterm_lpf1_dyn_min_hz;  // Dterm低通滤波器1动态模式最小频率（Hz）
static uint16_t bf_pid_dterm_lpf1_dyn_max_hz;  // Dterm低通滤波器1动态模式最大频率（Hz）
static uint8_t bf_pid_dterm_lpf1_dyn_expo;     // Dterm低通滤波器1动态模式曲线指数

#ifdef PROJECT_BF_PID_D_MAX_EN
/* D_MAX 参数（与Betaflight一致） */
static uint8_t bf_pid_d_max[3];       // 每个轴的最大D值 [Roll, Pitch, Yaw]
static uint8_t bf_pid_d_max_gain;     // D_MAX增益因子（用于陀螺仪/setpoint活动量）
static uint8_t bf_pid_d_max_advance;  // D_MAX setpoint输入的百分比乘数
#endif

/* Yaw P项滤波器参数 */
static uint16_t bf_pid_yaw_lowpass_hz;  // Yaw P项低通滤波器截止频率（Hz）
static float bf_pid_yaw_lpf_hz;         // Legacy: 保持向后兼容，映射到yaw_lowpass_hz

/* 角度模式参数（与Betaflight一致） */
#ifdef PROJECT_BF_ATTITUDE_EN
static uint8_t bf_pid_angle_limit;                    // 角度模式最大角度限制（度）
static uint8_t bf_pid_angle_earth_ref;                // 地球参考补偿增益（0-100）
static float bf_pid_angle_p_gain;                      // 角度模式P增益
static float bf_pid_angle_feedforward;                 // 角度模式前馈增益
static uint8_t bf_pid_angle_feedforward_smoothing_ms;  // 角度前馈平滑时间常数（毫秒）
#endif

/* PID 处理分母（控制 PID 循环频率） */
static uint8_t pid_process_denom;  // 对应 activePidLoopDenom，用于计算 targetLooptime

/* 默认值 - Betaflight 典型配置 */
static const float bf_rate_pid_roll_default[3] = {0.0f, 0.0f, 0.0f};  // 典型 Betaflight 默认值
static const float bf_rate_pid_pitch_default[3] = {45.0f, 62.0f, 31.0f};
static const float bf_rate_pid_yaw_default[3] = {0.0f, 0.0f, 0.0f};

/* Rate PID sum限制的默认值（deg/s）- 与Betaflight一致 */
static const float bf_pid_rate_sum_limit_default = 500.0f;      // Roll/Pitch轴默认500 deg/s
static const float bf_pid_rate_sum_limit_yaw_default = 400.0f;   // Yaw轴默认400 deg/s

/* PID 滤波器参数默认值 - 与Betaflight一致 */
static const float bf_pid_iterm_windup_default = 80.0f;     // I项windup百分比默认80%

/* Dterm 滤波器参数默认值 - 与Betaflight一致 */
static const uint16_t bf_pid_dterm_notch_hz_default = 0;          // Dterm陷波滤波器默认关闭
static const uint16_t bf_pid_dterm_notch_cutoff_default = 0;      // Dterm陷波滤波器截止频率默认0
static const uint16_t bf_pid_dterm_lpf1_static_hz_default = 100;  // Dterm LPF1默认100Hz（Betaflight默认值）
static const uint8_t bf_pid_dterm_lpf1_type_default = 0;          // Dterm LPF1类型默认PT1
static const uint16_t bf_pid_dterm_lpf2_static_hz_default = 150;  // Dterm LPF2默认150Hz
static const uint8_t bf_pid_dterm_lpf2_type_default = 0;          // Dterm LPF2类型默认PT1

/* Dynamic LPF 参数默认值 - 与Betaflight一致 */
static const uint16_t bf_pid_dterm_lpf1_dyn_min_hz_default = 75;   // Dynamic LPF默认75Hz
static const uint16_t bf_pid_dterm_lpf1_dyn_max_hz_default = 150;  // Dynamic LPF默认150Hz
static const uint8_t bf_pid_dterm_lpf1_dyn_expo_default = 5;       // Dynamic LPF曲线指数默认5（0.5）

#ifdef PROJECT_BF_PID_D_MAX_EN
/* D_MAX 参数默认值 - 与Betaflight一致 */
static const uint8_t bf_pid_d_max_default[3] = {40, 46, 0};  // Roll=40, Pitch=46, Yaw=0（默认关闭）
static const uint8_t bf_pid_d_max_gain_default = 37;         // D_MAX增益默认37
static const uint8_t bf_pid_d_max_advance_default = 20;      // D_MAX advance默认20
#endif

/* Yaw P项滤波器参数默认值 */
static const uint16_t bf_pid_yaw_lowpass_hz_default = 90;  // Yaw P项LPF默认90Hz
static const float bf_pid_yaw_lpf_hz_default = 90.0f;      // Legacy: 保持向后兼容

#ifdef PROJECT_BF_ATTITUDE_EN
/* 角度模式参数默认值 - 与Betaflight一致 */
static const uint8_t bf_pid_angle_limit_default = 60;                    // 角度模式最大角度默认60度
static const uint8_t bf_pid_angle_earth_ref_default = 100;                // 地球参考补偿增益默认100（0-100）
static const float bf_pid_angle_p_gain_default = 10.0f;                   // 角度模式P增益默认10.0
static const float bf_pid_angle_feedforward_default = 40.0f;              // 角度模式前馈增益默认40.0
static const uint8_t bf_pid_angle_feedforward_smoothing_ms_default = 80;  // 角度前馈平滑时间常数默认80ms
#endif

static const uint8_t pid_process_denom_default = 1;  // 默认值为 1，对应 activePidLoopDenom

static const param_default_t bf_pid_defaults[] = {
    {bf_rate_pid_roll, bf_rate_pid_roll_default},
    {bf_rate_pid_pitch, bf_rate_pid_pitch_default},
    {bf_rate_pid_yaw, bf_rate_pid_yaw_default},
    {&bf_pid_rate_sum_limit, &bf_pid_rate_sum_limit_default},
    {&bf_pid_rate_sum_limit_yaw, &bf_pid_rate_sum_limit_yaw_default},
    {&bf_pid_iterm_windup, &bf_pid_iterm_windup_default},
    {&bf_pid_dterm_notch_hz, &bf_pid_dterm_notch_hz_default},
    {&bf_pid_dterm_notch_cutoff, &bf_pid_dterm_notch_cutoff_default},
    {&bf_pid_dterm_lpf1_static_hz, &bf_pid_dterm_lpf1_static_hz_default},
    {&bf_pid_dterm_lpf1_type, &bf_pid_dterm_lpf1_type_default},
    {&bf_pid_dterm_lpf2_static_hz, &bf_pid_dterm_lpf2_static_hz_default},
    {&bf_pid_dterm_lpf2_type, &bf_pid_dterm_lpf2_type_default},
    {&bf_pid_dterm_lpf1_dyn_min_hz, &bf_pid_dterm_lpf1_dyn_min_hz_default},
    {&bf_pid_dterm_lpf1_dyn_max_hz, &bf_pid_dterm_lpf1_dyn_max_hz_default},
    {&bf_pid_dterm_lpf1_dyn_expo, &bf_pid_dterm_lpf1_dyn_expo_default},
#ifdef PROJECT_BF_PID_D_MAX_EN
    {bf_pid_d_max, bf_pid_d_max_default},
    {&bf_pid_d_max_gain, &bf_pid_d_max_gain_default},
    {&bf_pid_d_max_advance, &bf_pid_d_max_advance_default},
#endif
    {&bf_pid_yaw_lowpass_hz, &bf_pid_yaw_lowpass_hz_default},
    {&bf_pid_yaw_lpf_hz, &bf_pid_yaw_lpf_hz_default},
#ifdef PROJECT_BF_ATTITUDE_EN
    {&bf_pid_angle_limit, &bf_pid_angle_limit_default},
    {&bf_pid_angle_earth_ref, &bf_pid_angle_earth_ref_default},
    {&bf_pid_angle_p_gain, &bf_pid_angle_p_gain_default},
    {&bf_pid_angle_feedforward, &bf_pid_angle_feedforward_default},
    {&bf_pid_angle_feedforward_smoothing_ms, &bf_pid_angle_feedforward_smoothing_ms_default},
#endif
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
    {(void*)bf_rate_pid_roll, sizeof(bf_rate_pid_roll), "pid_rate_roll", "vf", bf_pid_param_default},
    {(void*)bf_rate_pid_pitch, sizeof(bf_rate_pid_pitch), "pid_rate_pitch", "vf", bf_pid_param_default},
    {(void*)bf_rate_pid_yaw, sizeof(bf_rate_pid_yaw), "pid_rate_yaw", "vf", bf_pid_param_default},
    {(void*)&bf_pid_rate_sum_limit, sizeof(bf_pid_rate_sum_limit), "pid_rate_sum_limit", "f", bf_pid_param_default},
    {(void*)&bf_pid_rate_sum_limit_yaw, sizeof(bf_pid_rate_sum_limit_yaw), "pid_rate_sum_limit_yaw", "f", bf_pid_param_default},
    {(void*)&bf_pid_iterm_windup, sizeof(bf_pid_iterm_windup), "pid_iterm_windup", "f", bf_pid_param_default},
    {(void*)&bf_pid_dterm_notch_hz, sizeof(bf_pid_dterm_notch_hz), "pid_dterm_notch_hz", "u16", bf_pid_param_default},
    {(void*)&bf_pid_dterm_notch_cutoff, sizeof(bf_pid_dterm_notch_cutoff), "pid_dterm_notch_cutoff", "u16",
     bf_pid_param_default},
    {(void*)&bf_pid_dterm_lpf1_static_hz, sizeof(bf_pid_dterm_lpf1_static_hz), "pid_dterm_lpf1_static_hz", "u16",
     bf_pid_param_default},
    {(void*)&bf_pid_dterm_lpf1_type, sizeof(bf_pid_dterm_lpf1_type), "pid_dterm_lpf1_type", "u8", bf_pid_param_default},
    {(void*)&bf_pid_dterm_lpf2_static_hz, sizeof(bf_pid_dterm_lpf2_static_hz), "pid_dterm_lpf2_static_hz", "u16",
     bf_pid_param_default},
    {(void*)&bf_pid_dterm_lpf2_type, sizeof(bf_pid_dterm_lpf2_type), "pid_dterm_lpf2_type", "u8", bf_pid_param_default},
    {(void*)&bf_pid_dterm_lpf1_dyn_min_hz, sizeof(bf_pid_dterm_lpf1_dyn_min_hz), "pid_dterm_lpf1_dyn_min_hz", "u16",
     bf_pid_param_default},
    {(void*)&bf_pid_dterm_lpf1_dyn_max_hz, sizeof(bf_pid_dterm_lpf1_dyn_max_hz), "pid_dterm_lpf1_dyn_max_hz", "u16",
     bf_pid_param_default},
    {(void*)&bf_pid_dterm_lpf1_dyn_expo, sizeof(bf_pid_dterm_lpf1_dyn_expo), "pid_dterm_lpf1_dyn_expo", "u8",
     bf_pid_param_default},
#ifdef PROJECT_BF_PID_D_MAX_EN
    {(void*)bf_pid_d_max, sizeof(bf_pid_d_max), "pid_d_max", "vu8", bf_pid_param_default},
    {(void*)&bf_pid_d_max_gain, sizeof(bf_pid_d_max_gain), "pid_d_max_gain", "u8", bf_pid_param_default},
    {(void*)&bf_pid_d_max_advance, sizeof(bf_pid_d_max_advance), "pid_d_max_advance", "u8", bf_pid_param_default},
#endif
    {(void*)&bf_pid_yaw_lowpass_hz, sizeof(bf_pid_yaw_lowpass_hz), "pid_yaw_lowpass_hz", "u16", bf_pid_param_default},
    {(void*)&bf_pid_yaw_lpf_hz, sizeof(bf_pid_yaw_lpf_hz), "pid_yaw_lpf_hz", "f", bf_pid_param_default},
#ifdef PROJECT_BF_ATTITUDE_EN
    {(void*)&bf_pid_angle_limit, sizeof(bf_pid_angle_limit), "pid_angle_limit", "u8", bf_pid_param_default},
    {(void*)&bf_pid_angle_earth_ref, sizeof(bf_pid_angle_earth_ref), "pid_angle_earth_ref", "u8", bf_pid_param_default},
    {(void*)&bf_pid_angle_p_gain, sizeof(bf_pid_angle_p_gain), "pid_angle_p_gain", "f", bf_pid_param_default},
    {(void*)&bf_pid_angle_feedforward, sizeof(bf_pid_angle_feedforward), "pid_angle_feedforward", "f", bf_pid_param_default},
    {(void*)&bf_pid_angle_feedforward_smoothing_ms, sizeof(bf_pid_angle_feedforward_smoothing_ms), "pid_angle_feedforward_smoothing_ms", "u8", bf_pid_param_default},
#endif
    {(void*)&pid_process_denom, sizeof(pid_process_denom), "pid_process_denom", "u8", bf_pid_param_default},
};

param_list *bfPidParam_list(void) { return bf_pid_params; }

size_t bfPidParam_count(void) { return sizeof(bf_pid_params) / sizeof(bf_pid_params[0]); }

