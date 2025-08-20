/*
 * File: LED_blink.h
 *
 * Code generated for Simulink model 'LED_blink'.
 *
 * Model version                  : 1.2
 * Simulink Coder version         : 9.6 (R2021b) 14-May-2021
 * C/C++ source code generated on : Sun Aug  3 16:13:22 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_LED_blink_h_
#define RTW_HEADER_LED_blink_h_
#include <stddef.h>
#include <string.h>
#ifndef LED_blink_COMMON_INCLUDES_
#define LED_blink_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* LED_blink_COMMON_INCLUDES_ */

#include "LED_blink_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  boolean_T Delay_DSTATE[2];           /* '<Root>/Delay' */
} DW_LED_blink_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  boolean_T LED_gg;                    /* '<Root>/LED' */
} ExtY_LED_blink_T;

/* Parameters (default storage) */
struct P_LED_blink_T_ {
  boolean_T Delay_InitialCondition;/* Computed Parameter: Delay_InitialCondition
                                    * Referenced by: '<Root>/Delay'
                                    */
};

/* Real-time Model Data Structure */
struct tag_RTM_LED_blink_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (default storage) */
extern P_LED_blink_T LED_blink_P;

/* Block states (default storage) */
extern DW_LED_blink_T LED_blink_DW;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_LED_blink_T LED_blink_Y;

/* Model entry point functions */
extern void LED_blink_initialize(void);
extern void LED_blink_step(void);

/* Real-time Model object */
extern RT_MODEL_LED_blink_T *const LED_blink_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LED_blink'
 */
#endif                                 /* RTW_HEADER_LED_blink_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
