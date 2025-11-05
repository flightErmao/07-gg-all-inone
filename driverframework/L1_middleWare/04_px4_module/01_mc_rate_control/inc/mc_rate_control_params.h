/****************************************************************************
 *
 * Multicopter Rate Control Parameters
 * Localized parameters for rate control module
 *
 ****************************************************************************/

#ifndef __MC_RATE_CONTROL_PARAMS_H__
#define __MC_RATE_CONTROL_PARAMS_H__

#ifdef __cplusplus
extern "C" {
#endif

// Rate Control Parameters (from mc_rate_control_params.c default values)
#define MC_ROLLRATE_P     0.15f
#define MC_ROLLRATE_I     0.2f
#define MC_RR_INT_LIM     0.30f
#define MC_ROLLRATE_D     0.003f
#define MC_ROLLRATE_FF    0.0f
#define MC_ROLLRATE_K     1.0f

#define MC_PITCHRATE_P    0.15f
#define MC_PITCHRATE_I     0.2f
#define MC_PR_INT_LIM      0.30f
#define MC_PITCHRATE_D     0.003f
#define MC_PITCHRATE_FF   0.0f
#define MC_PITCHRATE_K     1.0f

#define MC_YAWRATE_P       0.2f
#define MC_YAWRATE_I       0.1f
#define MC_YR_INT_LIM      0.30f
#define MC_YAWRATE_D       0.0f
#define MC_YAWRATE_FF      0.0f
#define MC_YAWRATE_K       1.0f

#define MC_YAW_TQ_CUTOFF   2.0f
#define MC_BAT_SCALE_EN    0

// Acro Mode Parameters (from mc_acro_params.c default values)
#define MC_ACRO_R_MAX      100.0f    // deg/s
#define MC_ACRO_P_MAX      100.0f    // deg/s
#define MC_ACRO_Y_MAX      100.0f    // deg/s
#define MC_ACRO_EXPO       0.0f
#define MC_ACRO_EXPO_Y     0.0f
#define MC_ACRO_SUPEXPO    0.0f
#define MC_ACRO_SUPEXPOY   0.0f

#ifdef __cplusplus
}
#endif

#endif /* __MC_RATE_CONTROL_PARAMS_H__ */

