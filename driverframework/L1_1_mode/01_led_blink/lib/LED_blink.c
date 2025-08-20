/*
 * File: LED_blink.c
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

#include "LED_blink.h"
#include "LED_blink_private.h"

/* Block states (default storage) */
DW_LED_blink_T LED_blink_DW;

/* External outputs (root outports fed by signals with default storage) */
ExtY_LED_blink_T LED_blink_Y;

/* Real-time model */
static RT_MODEL_LED_blink_T LED_blink_M_;
RT_MODEL_LED_blink_T *const LED_blink_M = &LED_blink_M_;

/* Model step function */
void LED_blink_step(void)
{
  boolean_T rtb_Delay;

  /* Delay: '<Root>/Delay' */
  rtb_Delay = LED_blink_DW.Delay_DSTATE[0];

  /* Outport: '<Root>/LED' incorporates:
   *  Delay: '<Root>/Delay'
   */
  LED_blink_Y.LED_gg = LED_blink_DW.Delay_DSTATE[0];

  /* Update for Delay: '<Root>/Delay' incorporates:
   *  Logic: '<Root>/Logical Operator'
   */
  LED_blink_DW.Delay_DSTATE[0] = LED_blink_DW.Delay_DSTATE[1];
  LED_blink_DW.Delay_DSTATE[1] = !rtb_Delay;
}

/* Model initialize function */
void LED_blink_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(LED_blink_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&LED_blink_DW, 0,
                sizeof(DW_LED_blink_T));

  /* external outputs */
  LED_blink_Y.LED_gg = false;

  /* InitializeConditions for Delay: '<Root>/Delay' */
  LED_blink_DW.Delay_DSTATE[0] = LED_blink_P.Delay_InitialCondition;
  LED_blink_DW.Delay_DSTATE[1] = LED_blink_P.Delay_InitialCondition;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
