/*
 * File: foc_simulink0_data.c
 *
 * Code generated for Simulink model 'foc_simulink0'.
 *
 * Model version                  : 1.319
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Tue Oct 24 23:01:26 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "foc_simulink0.h"

/* Block parameters (default storage) */
P rtP = {
  /* Variable: D_PI_I
   * Referenced by: '<S42>/Integral Gain'
   */
  600.0F,

  /* Variable: D_PI_KB
   * Referenced by: '<S38>/Kb'
   */
  15.0F,

  /* Variable: D_PI_LOW_LIMIT
   * Referenced by: '<S52>/Saturation'
   */
  -24.0F,

  /* Variable: D_PI_P
   * Referenced by: '<S50>/Proportional Gain'
   */
  1.4F,

  /* Variable: D_PI_UP_LIMIT
   * Referenced by: '<S52>/Saturation'
   */
  24.0F,

  /* Variable: Q_PI_I
   * Referenced by: '<S92>/Integral Gain'
   */
  600.0F,

  /* Variable: Q_PI_KB
   * Referenced by: '<S88>/Kb'
   */
  15.0F,

  /* Variable: Q_PI_LOW_LIMIT
   * Referenced by: '<S102>/Saturation'
   */
  -24.0F,

  /* Variable: Q_PI_P
   * Referenced by: '<S100>/Proportional Gain'
   */
  1.4F,

  /* Variable: Q_PI_UP_LIMIT
   * Referenced by: '<S102>/Saturation'
   */
  24.0F
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
