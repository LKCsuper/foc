/*
 * File: foc_simulink0.c
 *
 * Code generated for Simulink model 'foc_simulink0'.
 *
 * Model version                  : 1.276
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Fri Oct  6 14:23:30 2023
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
  real32_T rtb_Id;
  real32_T rtb_Kb;
  real32_T rtb_Saturation;
  real32_T rtb_Saturation_j;
  real32_T rtb_Sum;
  real32_T rtb_Sum1;
  real32_T rtb_T2;
  real32_T rtb_Valpha;
  real32_T rtb_Vbeta;
  real32_T rtb_X;
  real32_T rtb_Y;

  /* Gain: '<S5>/Gain' incorporates:
   *  Inport: '<Root>/Ia'
   *  MATLAB Function: '<S5>/MATLAB Function'
   */
  rtb_Gain = 1.5F * rtU.Ia * 0.666666687F;

  /* Gain: '<S5>/Gain1' incorporates:
   *  Inport: '<Root>/Ia'
   *  Inport: '<Root>/Ib'
   *  MATLAB Function: '<S5>/MATLAB Function1'
   */
  rtb_Gain1 = (2.0F * rtU.Ib + rtU.Ia) * 0.866025388F * 0.666666687F;

  /* MATLAB Function: '<S7>/park_id_fun' incorporates:
   *  Inport: '<Root>/Theta'
   *  MATLAB Function: '<S4>/antipark_valpha_fun'
   *  MATLAB Function: '<S4>/antipark_vbeta_fun'
   *  MATLAB Function: '<S7>/park_iq_fun'
   */
  rtb_X = sinf(rtU.Theta);
  rtb_Y = cosf(rtU.Theta);
  rtb_Id = rtb_Gain * rtb_Y + rtb_Gain1 * rtb_X;

  /* MATLAB Function: '<S7>/park_iq_fun' */
  rtb_Gain = -rtb_Gain * rtb_X + rtb_Gain1 * rtb_Y;

  /* Sum: '<S1>/Sum2' incorporates:
   *  Inport: '<Root>/Iq_ref'
   */
  rtb_Gain1 = rtU.Iq_ref + rtb_Gain;

  /* Sum: '<S104>/Sum' incorporates:
   *  DiscreteIntegrator: '<S95>/Integrator'
   *  Gain: '<S100>/Proportional Gain'
   */
  rtb_Sum1 = 0.54162F * rtb_Gain1 + rtDW.qi;

  /* Saturate: '<S102>/Saturation' */
  if (rtb_Sum1 > 24.0F) {
    rtb_Saturation = 24.0F;
  } else if (rtb_Sum1 < -24.0F) {
    rtb_Saturation = -24.0F;
  } else {
    rtb_Saturation = rtb_Sum1;
  }

  /* End of Saturate: '<S102>/Saturation' */

  /* Gain: '<S88>/Kb' incorporates:
   *  Sum: '<S88>/SumI2'
   */
  rtb_Kb = (rtb_Saturation - rtb_Sum1) * 15.0F;

  /* Sum: '<S1>/Sum1' incorporates:
   *  Inport: '<Root>/Id_ref'
   */
  rtb_Sum1 = rtU.Id_ref + rtb_Id;

  /* Sum: '<S54>/Sum' incorporates:
   *  DiscreteIntegrator: '<S45>/Integrator'
   *  Gain: '<S50>/Proportional Gain'
   */
  rtb_Sum = 0.64167F * rtb_Sum1 + rtDW.di;

  /* Saturate: '<S52>/Saturation' */
  if (rtb_Sum > 24.0F) {
    rtb_Saturation_j = 24.0F;
  } else if (rtb_Sum < -24.0F) {
    rtb_Saturation_j = -24.0F;
  } else {
    rtb_Saturation_j = rtb_Sum;
  }

  /* End of Saturate: '<S52>/Saturation' */

  /* MATLAB Function: '<S4>/antipark_valpha_fun' */
  rtb_Valpha = rtb_Saturation_j * rtb_Y - rtb_Saturation * rtb_X;

  /* MATLAB Function: '<S4>/antipark_vbeta_fun' */
  rtb_Vbeta = rtb_Saturation_j * rtb_X + rtb_Saturation * rtb_Y;

  /* MATLAB Function: '<S9>/svm1' */
  rtb_N = (((1.73205078F * rtb_Valpha - rtb_Vbeta > 0.0F) << 1) + (rtb_Vbeta >
            0.0F)) + ((-1.73205078F * rtb_Valpha - rtb_Vbeta > 0.0F) << 2);

  /* MATLAB Function: '<S9>/svm2' incorporates:
   *  Inport: '<Root>/Tpwm'
   *  Inport: '<Root>/Udc'
   */
  rtb_X = 1.73205078F * rtU.Tpwm * rtb_Vbeta / rtU.Udc;
  rtb_T2 = 1.73205078F * rtU.Tpwm / rtU.Udc;
  rtb_Y = (0.866025388F * rtb_Valpha + 0.5F * rtb_Vbeta) * rtb_T2;
  rtb_Valpha = (-0.866025388F * rtb_Valpha + 0.5F * rtb_Vbeta) * rtb_T2;

  /* MATLAB Function: '<S9>/svm3' */
  rtb_Vbeta = 0.0F;
  rtb_T2 = 0.0F;

  /* Outport: '<Root>/Tcm1' incorporates:
   *  MATLAB Function: '<S9>/svm4'
   */
  rtY.Tcm1 = 0.0F;

  /* Outport: '<Root>/Tcm2' incorporates:
   *  MATLAB Function: '<S9>/svm4'
   */
  rtY.Tcm2 = 0.0F;

  /* Outport: '<Root>/Tcm3' incorporates:
   *  MATLAB Function: '<S9>/svm4'
   */
  rtY.Tcm3 = 0.0F;

  /* MATLAB Function: '<S9>/svm3' incorporates:
   *  MATLAB Function: '<S9>/svm4'
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
      rtb_Vbeta = rtb_Y;
      rtb_T2 = -rtb_X;
    } else if (rtb_N == 3) {
      rtb_Vbeta = -rtb_Valpha;
      rtb_T2 = rtb_X;
    } else if (rtb_N == 4) {
      rtb_Vbeta = -rtb_X;
      rtb_T2 = rtb_Valpha;
    } else if (rtb_N == 5) {
      rtb_Vbeta = rtb_X;
      rtb_T2 = -rtb_Y;
    } else if (rtb_N == 6) {
      rtb_Vbeta = -rtb_Y;
      rtb_T2 = -rtb_Valpha;
    }

    if (rtb_N == 2) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm - rtb_Vbeta) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm + rtb_Vbeta) + rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm + rtb_Vbeta) - rtb_T2) / 4.0F;
    } else if (rtb_N == 3) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm - rtb_Vbeta) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm + rtb_Vbeta) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm + rtb_Vbeta) + rtb_T2) / 4.0F;
    } else if (rtb_N == 4) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm + rtb_Vbeta) + rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm + rtb_Vbeta) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm - rtb_Vbeta) - rtb_T2) / 4.0F;
    } else if (rtb_N == 5) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm + rtb_Vbeta) + rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm - rtb_Vbeta) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm + rtb_Vbeta) - rtb_T2) / 4.0F;
    } else if (rtb_N == 6) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm + rtb_Vbeta) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm + rtb_Vbeta) + rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm - rtb_Vbeta) - rtb_T2) / 4.0F;
    }
  }

  /* Outport: '<Root>/Vd' */
  rtY.Vd = rtb_Saturation_j;

  /* Outport: '<Root>/Vq' */
  rtY.Vq = rtb_Saturation;

  /* Outport: '<Root>/Iq' */
  rtY.Iq = rtb_Gain;

  /* Outport: '<Root>/Id' */
  rtY.Id = rtb_Id;

  /* Update for DiscreteIntegrator: '<S95>/Integrator' incorporates:
   *  Gain: '<S92>/Integral Gain'
   *  Sum: '<S88>/SumI4'
   */
  rtDW.qi += (450.0F * rtb_Gain1 + rtb_Kb) * 1.0E-7F;

  /* Update for DiscreteIntegrator: '<S45>/Integrator' incorporates:
   *  Gain: '<S38>/Kb'
   *  Gain: '<S42>/Integral Gain'
   *  Sum: '<S38>/SumI2'
   *  Sum: '<S38>/SumI4'
   */
  rtDW.di += ((rtb_Saturation_j - rtb_Sum) * 15.0F + 450.0F * rtb_Sum1) *
    1.0E-7F;
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
