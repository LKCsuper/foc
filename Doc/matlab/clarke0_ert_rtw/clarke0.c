/*
 * File: clarke0.c
 *
 * Code generated for Simulink model 'clarke0'.
 *
 * Model version                  : 1.68
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Sun Apr  9 11:04:24 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "clarke0.h"

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void clarke0_step(void)
{
  /* Outport: '<Root>/Ialpha' incorporates:
   *  Inport: '<Root>/IA'
   */
  rtY.Ialpha = rtU.IA;

  /* Outport: '<Root>/Ibeta' incorporates:
   *  Inport: '<Root>/IB'
   *  Inport: '<Root>/IC'
   *  MATLAB Function: '<S1>/toIbeta'
   */
  rtY.Ibeta = (rtU.IB - rtU.IC) * 0.66666666666666663 * 0.86602540378443871;
}

/* Model initialize function */
void clarke0_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
