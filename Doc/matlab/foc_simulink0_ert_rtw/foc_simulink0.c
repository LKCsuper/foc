/*
 * File: foc_simulink0.c
 *
 * Code generated for Simulink model 'foc_simulink0'.
 *
 * Model version                  : 1.209
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Tue Oct  3 20:36:08 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "foc_simulink0.h"
#include <math.h>
#include "rtwtypes.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void foc_simulink0_step(void)
{
  int32_T rtb_N;
  real32_T rtb_Gain;
  real32_T rtb_Gain1;
  real32_T rtb_Integrator_j;
  real32_T rtb_Kb;
  real32_T rtb_Saturation;
  real32_T rtb_Sum2;
  real32_T rtb_T2;
  real32_T rtb_Valpha;
  real32_T rtb_X;
  real32_T rtb_Y;

  /* Gain: '<S4>/Gain' incorporates:
   *  Inport: '<Root>/Ia'
   *  MATLAB Function: '<S4>/MATLAB Function'
   */
  rtb_Gain = 1.5F * rtU.Ia * 0.666666687F;

  /* Gain: '<S4>/Gain1' incorporates:
   *  Inport: '<Root>/Ia'
   *  Inport: '<Root>/Ib'
   *  MATLAB Function: '<S4>/MATLAB Function1'
   */
  rtb_Gain1 = (2.0F * rtU.Ib + rtU.Ia) * 0.866025388F * 0.666666687F;

  /* MATLAB Function: '<S6>/MATLAB Function' incorporates:
   *  Inport: '<Root>/Theta'
   *  MATLAB Function: '<S3>/MATLAB Function'
   *  MATLAB Function: '<S3>/MATLAB Function1'
   *  MATLAB Function: '<S6>/MATLAB Function1'
   */
  rtb_X = sinf(rtU.Theta);
  rtb_Y = cosf(rtU.Theta);

  /* Sum: '<S1>/Sum1' incorporates:
   *  Inport: '<Root>/Id_ref'
   *  MATLAB Function: '<S6>/MATLAB Function'
   */
  rtb_Integrator_j = rtU.Id_ref - (rtb_Gain * rtb_Y + rtb_Gain1 * rtb_X);

  /* Sum: '<S56>/Sum' incorporates:
   *  DiscreteIntegrator: '<S47>/Integrator'
   *  Gain: '<S52>/Proportional Gain'
   */
  rtb_Sum2 = 2.51327419F * rtb_Integrator_j + rtDW.di;

  /* Saturate: '<S54>/Saturation' */
  if (rtb_Sum2 > 24.0F) {
    rtb_Saturation = 24.0F;
  } else if (rtb_Sum2 < -24.0F) {
    rtb_Saturation = -24.0F;
  } else {
    rtb_Saturation = rtb_Sum2;
  }

  /* End of Saturate: '<S54>/Saturation' */

  /* Gain: '<S40>/Kb' incorporates:
   *  Sum: '<S40>/SumI2'
   */
  rtb_Kb = (rtb_Saturation - rtb_Sum2) * 15.0F;

  /* Sum: '<S1>/Sum2' incorporates:
   *  Inport: '<Root>/Iq_ref'
   *  MATLAB Function: '<S6>/MATLAB Function1'
   */
  rtb_Sum2 = rtU.Iq_ref - (-rtb_Gain * rtb_X + rtb_Gain1 * rtb_Y);

  /* Sum: '<S106>/Sum' incorporates:
   *  DiscreteIntegrator: '<S97>/Integrator'
   *  Gain: '<S102>/Proportional Gain'
   */
  rtb_Gain = 2.51327419F * rtb_Sum2 + rtDW.qi;

  /* Saturate: '<S104>/Saturation' */
  if (rtb_Gain > 24.0F) {
    rtb_Gain1 = 24.0F;
  } else if (rtb_Gain < -24.0F) {
    rtb_Gain1 = -24.0F;
  } else {
    rtb_Gain1 = rtb_Gain;
  }

  /* End of Saturate: '<S104>/Saturation' */

  /* MATLAB Function: '<S3>/MATLAB Function' */
  rtb_Valpha = rtb_Saturation * rtb_Y - rtb_Gain1 * rtb_X;

  /* MATLAB Function: '<S3>/MATLAB Function1' */
  rtb_Saturation = rtb_Saturation * rtb_X + rtb_Gain1 * rtb_Y;

  /* MATLAB Function: '<S2>/MATLAB Function' */
  rtb_N = (((1.73205078F * rtb_Valpha - rtb_Saturation > 0.0F) << 1) +
           (rtb_Saturation > 0.0F)) + ((-1.73205078F * rtb_Valpha -
    rtb_Saturation > 0.0F) << 2);

  /* MATLAB Function: '<S2>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/Tpwm'
   *  Inport: '<Root>/Udc'
   */
  rtb_X = 1.73205078F * rtU.Tpwm * rtb_Saturation / rtU.Udc;
  rtb_T2 = 1.73205078F * rtU.Tpwm / rtU.Udc;
  rtb_Y = (0.866025388F * rtb_Valpha + 0.5F * rtb_Saturation) * rtb_T2;
  rtb_Valpha = (-0.866025388F * rtb_Valpha + 0.5F * rtb_Saturation) * rtb_T2;

  /* MATLAB Function: '<S2>/MATLAB Function2' */
  rtb_Saturation = 0.0F;
  rtb_T2 = 0.0F;

  /* Outport: '<Root>/Tcm1' incorporates:
   *  MATLAB Function: '<S2>/MATLAB Function3'
   */
  rtY.Tcm1 = 0.0F;

  /* Outport: '<Root>/Tcm2' incorporates:
   *  MATLAB Function: '<S2>/MATLAB Function3'
   */
  rtY.Tcm2 = 0.0F;

  /* Outport: '<Root>/Tcm3' incorporates:
   *  MATLAB Function: '<S2>/MATLAB Function3'
   */
  rtY.Tcm3 = 0.0F;

  /* MATLAB Function: '<S2>/MATLAB Function2' incorporates:
   *  MATLAB Function: '<S2>/MATLAB Function3'
   */
  if (rtb_N == 1) {
    /* Outport: '<Root>/Tcm1' incorporates:
     *  Inport: '<Root>/Tpwm'
     */
    rtY.Tcm1 = ((rtU.Tpwm + rtb_Valpha) - rtb_Y) / 4.0F;

    /* Outport: '<Root>/Tcm2' incorporates:
     *  Inport: '<Root>/Tpwm'
     */
    rtY.Tcm2 = ((rtU.Tpwm - rtb_Valpha) - rtb_Y) / 4.0F;

    /* Outport: '<Root>/Tcm3' incorporates:
     *  Inport: '<Root>/Tpwm'
     */
    rtY.Tcm3 = ((rtU.Tpwm + rtb_Valpha) + rtb_Y) / 4.0F;
  } else {
    if (rtb_N == 2) {
      rtb_Saturation = rtb_Y;
      rtb_T2 = -rtb_X;
    } else if (rtb_N == 3) {
      rtb_Saturation = -rtb_Valpha;
      rtb_T2 = rtb_X;
    } else if (rtb_N == 4) {
      rtb_Saturation = -rtb_X;
      rtb_T2 = rtb_Valpha;
    } else if (rtb_N == 5) {
      rtb_Saturation = rtb_X;
      rtb_T2 = -rtb_Y;
    } else if (rtb_N == 6) {
      rtb_Saturation = -rtb_Y;
      rtb_T2 = -rtb_Valpha;
    }

    if (rtb_N == 2) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm - rtb_Saturation) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm + rtb_Saturation) + rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm + rtb_Saturation) - rtb_T2) / 4.0F;
    } else if (rtb_N == 3) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm - rtb_Saturation) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm + rtb_Saturation) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm + rtb_Saturation) + rtb_T2) / 4.0F;
    } else if (rtb_N == 4) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm + rtb_Saturation) + rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm + rtb_Saturation) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm - rtb_Saturation) - rtb_T2) / 4.0F;
    } else if (rtb_N == 5) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm + rtb_Saturation) + rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm - rtb_Saturation) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm + rtb_Saturation) - rtb_T2) / 4.0F;
    } else if (rtb_N == 6) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm + rtb_Saturation) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm + rtb_Saturation) + rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm - rtb_Saturation) - rtb_T2) / 4.0F;
    }
  }

  /* Update for DiscreteIntegrator: '<S47>/Integrator' incorporates:
   *  Gain: '<S44>/Integral Gain'
   *  Sum: '<S40>/SumI4'
   */
  rtDW.di += (1256.63708F * rtb_Integrator_j + rtb_Kb) * 1.0E-7F;

  /* Update for DiscreteIntegrator: '<S97>/Integrator' incorporates:
   *  Gain: '<S90>/Kb'
   *  Gain: '<S94>/Integral Gain'
   *  Sum: '<S90>/SumI2'
   *  Sum: '<S90>/SumI4'
   */
  rtDW.qi += ((rtb_Gain1 - rtb_Gain) * 15.0F + 1256.63708F * rtb_Sum2) * 1.0E-7F;
}

/* Model initialize function */
void foc_simulink0_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
