/*
 * File: MATLAB0.c
 *
 * Code generated for Simulink model 'MATLAB0'.
 *
 * Model version                  : 1.67
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Sun Apr  9 11:01:31 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "MATLAB0.h"

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void MATLAB0_step(void)
{
  /* Outport: '<Root>/Ibeta' incorporates:
   *  Inport: '<Root>/IB'
   *  Inport: '<Root>/IC'
   *  MATLAB Function: '<Root>/MATLAB Function1'
   */
  rtY.Ibeta = (rtU.IB - rtU.IC) * 0.66666666666666663 * 0.86602540378443871;
}

/* Model initialize function */
void MATLAB0_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
