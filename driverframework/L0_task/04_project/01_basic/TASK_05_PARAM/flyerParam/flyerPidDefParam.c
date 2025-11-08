#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "flyerPidDefParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void flyer_param_default(void *address, uint8_t size);

/* flight/angle_pid - 角度环 PID 参数 */
static float angle_pid_roll[3];
static float angle_pid_pitch[3];
static float angle_pid_yaw[3];
static float angle_pid_roll_i_limit;
static float angle_pid_pitch_i_limit;
static float angle_pid_yaw_i_limit;

/* flight/rate_pid - 速率环 PID 参数 */
static float rate_pid_roll[3];
static float rate_pid_pitch[3];
static float rate_pid_yaw[3];

/* PX4 风格速率控制参数 */
static float mc_rollrate_p;
static float mc_rollrate_i;
static float mc_rr_int_lim;
static float mc_rollrate_d;
static float mc_rollrate_ff;
static float mc_rollrate_k;

static float mc_pitchrate_p;
static float mc_pitchrate_i;
static float mc_pr_int_lim;
static float mc_pitchrate_d;
static float mc_pitchrate_ff;
static float mc_pitchrate_k;

static float mc_yawrate_p;
static float mc_yawrate_i;
static float mc_yr_int_lim;
static float mc_yawrate_d;
static float mc_yawrate_ff;
static float mc_yawrate_k;
static float mc_yaw_tq_cutoff;

static float mc_acro_r_max;
static float mc_acro_p_max;
static float mc_acro_y_max;
static float mc_acro_expo;
static float mc_acro_expo_y;
static float mc_acro_supexpo;
static float mc_acro_supexpo_y;

/* 默认值 */
static const float angle_pid_roll_default[3] = {0.0f, 0.0f, 0.0f};
static const float angle_pid_pitch_default[3] = {0.0f, 0.0f, 0.0f};
static const float angle_pid_yaw_default[3] = {0.0f, 0.0f, 0.0f};
static const float angle_pid_roll_i_limit_default = 30.0f;
static const float angle_pid_pitch_i_limit_default = 30.0f;
static const float angle_pid_yaw_i_limit_default = 180.0f;

static const float rate_pid_roll_default[3] = {42.0f, 65.0f, 29.0f};
static const float rate_pid_pitch_default[3] = {45.0f, 62.0f, 31.0f};
static const float rate_pid_yaw_default[3] = {30.0f, 50.0f, 0.0f};

static const float mc_rollrate_p_default = 0.0f;
static const float mc_rollrate_i_default = 0.0f;
static const float mc_rr_int_lim_default = 500.0f;
static const float mc_rollrate_d_default = 0.0f;
static const float mc_rollrate_ff_default = 0.0f;
static const float mc_rollrate_k_default = 0.0f;

static const float mc_pitchrate_p_default = 45.0f;
static const float mc_pitchrate_i_default = 62.0f;
static const float mc_pr_int_lim_default = 500.0f;
static const float mc_pitchrate_d_default = 0.0f;
static const float mc_pitchrate_ff_default = 0.0f;
static const float mc_pitchrate_k_default = 0.0f;

static const float mc_yawrate_p_default = 0.0f;
static const float mc_yawrate_i_default = 0.0f;
static const float mc_yr_int_lim_default = 50.0f;
static const float mc_yawrate_d_default = 0.0f;
static const float mc_yawrate_ff_default = 0.0f;
static const float mc_yawrate_k_default = 0.0f;
static const float mc_yaw_tq_cutoff_default = 0.0f;

static const float mc_acro_r_max_default = 100.0f;
static const float mc_acro_p_max_default = 100.0f;
static const float mc_acro_y_max_default = 100.0f;
static const float mc_acro_expo_default = 0.0f;
static const float mc_acro_expo_y_default = 0.0f;
static const float mc_acro_supexpo_default = 0.0f;
static const float mc_acro_supexpo_y_default = 0.0f;

static const param_default_t flyer_default_params[] = {
    {&angle_pid_roll, angle_pid_roll_default},
    {&angle_pid_pitch, angle_pid_pitch_default},
    {&angle_pid_yaw, angle_pid_yaw_default},
    {&angle_pid_roll_i_limit, &angle_pid_roll_i_limit_default},
    {&angle_pid_pitch_i_limit, &angle_pid_pitch_i_limit_default},
    {&angle_pid_yaw_i_limit, &angle_pid_yaw_i_limit_default},
    {&rate_pid_roll, rate_pid_roll_default},
    {&rate_pid_pitch, rate_pid_pitch_default},
    {&rate_pid_yaw, rate_pid_yaw_default},

    {&mc_rollrate_p, &mc_rollrate_p_default},
    {&mc_rollrate_i, &mc_rollrate_i_default},
    {&mc_rr_int_lim, &mc_rr_int_lim_default},
    {&mc_rollrate_d, &mc_rollrate_d_default},
    {&mc_rollrate_ff, &mc_rollrate_ff_default},
    {&mc_rollrate_k, &mc_rollrate_k_default},

    {&mc_pitchrate_p, &mc_pitchrate_p_default},
    {&mc_pitchrate_i, &mc_pitchrate_i_default},
    {&mc_pr_int_lim, &mc_pr_int_lim_default},
    {&mc_pitchrate_d, &mc_pitchrate_d_default},
    {&mc_pitchrate_ff, &mc_pitchrate_ff_default},
    {&mc_pitchrate_k, &mc_pitchrate_k_default},

    {&mc_yawrate_p, &mc_yawrate_p_default},
    {&mc_yawrate_i, &mc_yawrate_i_default},
    {&mc_yr_int_lim, &mc_yr_int_lim_default},
    {&mc_yawrate_d, &mc_yawrate_d_default},
    {&mc_yawrate_ff, &mc_yawrate_ff_default},
    {&mc_yawrate_k, &mc_yawrate_k_default},
    {&mc_yaw_tq_cutoff, &mc_yaw_tq_cutoff_default},

    {&mc_acro_r_max, &mc_acro_r_max_default},
    {&mc_acro_p_max, &mc_acro_p_max_default},
    {&mc_acro_y_max, &mc_acro_y_max_default},
    {&mc_acro_expo, &mc_acro_expo_default},
    {&mc_acro_expo_y, &mc_acro_expo_y_default},
    {&mc_acro_supexpo, &mc_acro_supexpo_default},
    {&mc_acro_supexpo_y, &mc_acro_supexpo_y_default},
};

static void flyer_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(flyer_default_params) / sizeof(flyer_default_params[0])); i++) {
        if (address == flyer_default_params[i].param) {
            memcpy(address, flyer_default_params[i].default_value, size);
            break;
        }
    }
}

static param_list flyer_params[] = {
    {(void *)&angle_pid_roll, sizeof(angle_pid_roll), "angle_pid_roll", "vf", flyer_param_default},
    {(void *)&angle_pid_pitch, sizeof(angle_pid_pitch), "angle_pid_pitch", "vf", flyer_param_default},
    {(void *)&angle_pid_yaw, sizeof(angle_pid_yaw), "angle_pid_yaw", "vf", flyer_param_default},
    {(void *)&angle_pid_roll_i_limit, sizeof(angle_pid_roll_i_limit), "angle_pid_roll_i_limit", "f", flyer_param_default},
    {(void *)&angle_pid_pitch_i_limit, sizeof(angle_pid_pitch_i_limit), "angle_pid_pitch_i_limit", "f", flyer_param_default},
    {(void *)&angle_pid_yaw_i_limit, sizeof(angle_pid_yaw_i_limit), "angle_pid_yaw_i_limit", "f", flyer_param_default},
    {(void *)&rate_pid_roll, sizeof(rate_pid_roll), "rate_pid_roll", "vf", flyer_param_default},
    {(void *)&rate_pid_pitch, sizeof(rate_pid_pitch), "rate_pid_pitch", "vf", flyer_param_default},
    {(void *)&rate_pid_yaw, sizeof(rate_pid_yaw), "rate_pid_yaw", "vf", flyer_param_default},

    {(void *)&mc_rollrate_p, sizeof(mc_rollrate_p), "mc_rollrate_p", "f", flyer_param_default},
    {(void *)&mc_rollrate_i, sizeof(mc_rollrate_i), "mc_rollrate_i", "f", flyer_param_default},
    {(void *)&mc_rr_int_lim, sizeof(mc_rr_int_lim), "mc_rr_int_lim", "f", flyer_param_default},
    {(void *)&mc_rollrate_d, sizeof(mc_rollrate_d), "mc_rollrate_d", "f", flyer_param_default},
    {(void *)&mc_rollrate_ff, sizeof(mc_rollrate_ff), "mc_rollrate_ff", "f", flyer_param_default},
    {(void *)&mc_rollrate_k, sizeof(mc_rollrate_k), "mc_rollrate_k", "f", flyer_param_default},

    {(void *)&mc_pitchrate_p, sizeof(mc_pitchrate_p), "mc_pitchrate_p", "f", flyer_param_default},
    {(void *)&mc_pitchrate_i, sizeof(mc_pitchrate_i), "mc_pitchrate_i", "f", flyer_param_default},
    {(void *)&mc_pr_int_lim, sizeof(mc_pr_int_lim), "mc_pr_int_lim", "f", flyer_param_default},
    {(void *)&mc_pitchrate_d, sizeof(mc_pitchrate_d), "mc_pitchrate_d", "f", flyer_param_default},
    {(void *)&mc_pitchrate_ff, sizeof(mc_pitchrate_ff), "mc_pitchrate_ff", "f", flyer_param_default},
    {(void *)&mc_pitchrate_k, sizeof(mc_pitchrate_k), "mc_pitchrate_k", "f", flyer_param_default},

    {(void *)&mc_yawrate_p, sizeof(mc_yawrate_p), "mc_yawrate_p", "f", flyer_param_default},
    {(void *)&mc_yawrate_i, sizeof(mc_yawrate_i), "mc_yawrate_i", "f", flyer_param_default},
    {(void *)&mc_yr_int_lim, sizeof(mc_yr_int_lim), "mc_yr_int_lim", "f", flyer_param_default},
    {(void *)&mc_yawrate_d, sizeof(mc_yawrate_d), "mc_yawrate_d", "f", flyer_param_default},
    {(void *)&mc_yawrate_ff, sizeof(mc_yawrate_ff), "mc_yawrate_ff", "f", flyer_param_default},
    {(void *)&mc_yawrate_k, sizeof(mc_yawrate_k), "mc_yawrate_k", "f", flyer_param_default},
    {(void *)&mc_yaw_tq_cutoff, sizeof(mc_yaw_tq_cutoff), "mc_yaw_tq_cutoff", "f", flyer_param_default},

    {(void *)&mc_acro_r_max, sizeof(mc_acro_r_max), "mc_acro_r_max", "f", flyer_param_default},
    {(void *)&mc_acro_p_max, sizeof(mc_acro_p_max), "mc_acro_p_max", "f", flyer_param_default},
    {(void *)&mc_acro_y_max, sizeof(mc_acro_y_max), "mc_acro_y_max", "f", flyer_param_default},
    {(void *)&mc_acro_expo, sizeof(mc_acro_expo), "mc_acro_expo", "f", flyer_param_default},
    {(void *)&mc_acro_expo_y, sizeof(mc_acro_expo_y), "mc_acro_expo_y", "f", flyer_param_default},
    {(void *)&mc_acro_supexpo, sizeof(mc_acro_supexpo), "mc_acro_supexpo", "f", flyer_param_default},
    {(void *)&mc_acro_supexpo_y, sizeof(mc_acro_supexpo_y), "mc_acro_supexpo_y", "f", flyer_param_default},
};

param_list *flyerPidDefParam_list(void) { return flyer_params; }

size_t flyerPidDefParam_count(void) { return sizeof(flyer_params) / sizeof(flyer_params[0]); }

