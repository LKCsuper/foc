/*
 * File: foc_simulink0.c
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

#ifdef __cplusplus
#define SFB_EXTERN_C                   extern "C"
#else
#define SFB_EXTERN_C                   extern
#endif

SFB_EXTERN_C void EKF_CAL_Start_wrapper(real_T *xD);
SFB_EXTERN_C void EKF_CAL_Outputs_wrapper(const real32_T *Ualpha,
  const real32_T *Ubeta,
  const real32_T *Ialpha,
  const real32_T *Ibeta,
  const real32_T *Rs_in,
  const real32_T *Flux_in,
  const real32_T *Ls_in,
  real_T *y0,
  real_T *y1,
  real_T *Speed,
  real_T *Angle,
  const real_T *xD);
SFB_EXTERN_C void EKF_CAL_Update_wrapper(const real32_T *Ualpha,
  const real32_T *Ubeta,
  const real32_T *Ialpha,
  const real32_T *Ibeta,
  const real32_T *Rs_in,
  const real32_T *Flux_in,
  const real32_T *Ls_in,
  real_T *y0,
  real_T *y1,
  real_T *Speed,
  real_T *Angle,
  real_T *xD);
SFB_EXTERN_C void EKF_CAL_Terminate_wrapper(real_T *xD);

#undef SFB_EXTERN_C

/* Model step function */
void foc_simulink0_step(void)
{
  real_T rtb_iq_back;
  real32_T rtb_Id;
  real32_T rtb_IntegralGain;
  real32_T rtb_Integrator_j;
  real32_T rtb_Iq;
  real32_T rtb_Saturation;
  real32_T rtb_Sum;
  real32_T rtb_T1;
  real32_T rtb_T2;
  real32_T rtb_X;
  real32_T rtb_Y;
  real32_T rtb_id_back;
  real32_T rtb_iq_sum;

  /* Gain: '<S5>/Gain' incorporates:
   *  Inport: '<Root>/Ia'
   *  MATLAB Function: '<S5>/MATLAB Function'
   */
  rtDW.Gain = 1.5F * rtU.Ia * 0.666666687F;

  /* Gain: '<S5>/Gain1' incorporates:
   *  Inport: '<Root>/Ia'
   *  Inport: '<Root>/Ib'
   *  MATLAB Function: '<S5>/MATLAB Function1'
   */
  rtDW.Gain1 = (2.0F * rtU.Ib + rtU.Ia) * 0.866025388F * 0.666666687F;

  /* MATLAB Function: '<S7>/park_id_fun' incorporates:
   *  Inport: '<Root>/Theta'
   *  MATLAB Function: '<S4>/antipark_valpha_fun'
   *  MATLAB Function: '<S4>/antipark_vbeta_fun'
   *  MATLAB Function: '<S7>/park_iq_fun'
   */
  rtb_X = sinf(rtU.Theta);
  rtb_Y = cosf(rtU.Theta);
  rtb_Id = -(rtDW.Gain * rtb_Y + rtDW.Gain1 * rtb_X);

  /* Sum: '<S1>/id_sum' incorporates:
   *  Inport: '<Root>/Id_ref'
   */
  rtb_Sum = rtU.Id_ref - rtb_Id;

  /* Sum: '<S54>/Sum' incorporates:
   *  DiscreteIntegrator: '<S45>/Integrator'
   *  Gain: '<S50>/Proportional Gain'
   */
  rtb_Integrator_j = rtP.D_PI_P * rtb_Sum + rtDW.di;

  /* Saturate: '<S52>/Saturation' */
  if (rtb_Integrator_j > rtP.D_PI_UP_LIMIT) {
    rtb_iq_sum = rtP.D_PI_UP_LIMIT;
  } else if (rtb_Integrator_j < rtP.D_PI_LOW_LIMIT) {
    rtb_iq_sum = rtP.D_PI_LOW_LIMIT;
  } else {
    rtb_iq_sum = rtb_Integrator_j;
  }

  /* End of Saturate: '<S52>/Saturation' */

  /* Gain: '<S38>/Kb' incorporates:
   *  Sum: '<S38>/SumI2'
   */
  rtb_Integrator_j = (rtb_iq_sum - rtb_Integrator_j) * rtP.D_PI_KB;

  /* Gain: '<S42>/Integral Gain' */
  rtb_IntegralGain = rtP.D_PI_I * rtb_Sum;

  /* Sum: '<S1>/id_back' */
  rtb_id_back = rtb_iq_sum;

  /* MATLAB Function: '<S7>/park_iq_fun' */
  rtb_Iq = -(-rtDW.Gain * rtb_X + rtDW.Gain1 * rtb_Y);

  /* Sum: '<S1>/iq_sum' incorporates:
   *  Inport: '<Root>/Iq_ref'
   */
  rtb_iq_sum = rtU.Iq_ref - rtb_Iq;

  /* Sum: '<S104>/Sum' incorporates:
   *  DiscreteIntegrator: '<S95>/Integrator'
   *  Gain: '<S100>/Proportional Gain'
   */
  rtb_Sum = rtP.Q_PI_P * rtb_iq_sum + rtDW.qi;

  /* Saturate: '<S102>/Saturation' */
  if (rtb_Sum > rtP.Q_PI_UP_LIMIT) {
    rtb_Saturation = rtP.Q_PI_UP_LIMIT;
  } else if (rtb_Sum < rtP.Q_PI_LOW_LIMIT) {
    rtb_Saturation = rtP.Q_PI_LOW_LIMIT;
  } else {
    rtb_Saturation = rtb_Sum;
  }

  /* End of Saturate: '<S102>/Saturation' */

  /* MATLAB Function: '<S4>/antipark_valpha_fun' incorporates:
   *  Sum: '<S1>/iq_back'
   */
  rtDW.Valpha = rtb_id_back * rtb_Y - rtb_Saturation * rtb_X;

  /* MATLAB Function: '<S4>/antipark_vbeta_fun' incorporates:
   *  Sum: '<S1>/iq_back'
   */
  rtDW.Vbeta = rtb_id_back * rtb_X + rtb_Saturation * rtb_Y;

  /* MATLAB Function: '<S9>/svm1' */
  rtb_iq_back = (((1.73205078F * rtDW.Valpha - rtDW.Vbeta > 0.0F) << 1) +
                 (rtDW.Vbeta > 0.0F)) + ((-1.73205078F * rtDW.Valpha -
    rtDW.Vbeta > 0.0F) << 2);

  /* MATLAB Function: '<S9>/svm2' incorporates:
   *  Inport: '<Root>/Tpwm'
   *  Inport: '<Root>/Udc'
   */
  rtb_X = 1.73205078F * rtU.Tpwm * rtDW.Vbeta / rtU.Udc;
  rtb_id_back = 1.73205078F * rtU.Tpwm / rtU.Udc;
  rtb_Y = (0.866025388F * rtDW.Valpha + 0.5F * rtDW.Vbeta) * rtb_id_back;
  rtb_id_back *= -0.866025388F * rtDW.Valpha + 0.5F * rtDW.Vbeta;

  /* MATLAB Function: '<S9>/svm3' */
  rtb_T1 = 0.0F;
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
  if (rtb_iq_back == 1.0) {
    /* Outport: '<Root>/Tcm1' incorporates:
     *  Inport: '<Root>/Tpwm'
     */
    rtY.Tcm1 = ((rtU.Tpwm + rtb_id_back) - rtb_Y) / 4.0F;

    /* Outport: '<Root>/Tcm2' incorporates:
     *  Inport: '<Root>/Tpwm'
     */
    rtY.Tcm2 = ((rtU.Tpwm - rtb_id_back) - rtb_Y) / 4.0F;

    /* Outport: '<Root>/Tcm3' incorporates:
     *  Inport: '<Root>/Tpwm'
     */
    rtY.Tcm3 = ((rtU.Tpwm + rtb_id_back) + rtb_Y) / 4.0F;
  } else {
    if (rtb_iq_back == 2.0) {
      rtb_T1 = rtb_Y;
      rtb_T2 = -rtb_X;
    } else if (rtb_iq_back == 3.0) {
      rtb_T1 = -rtb_id_back;
      rtb_T2 = rtb_X;
    } else if (rtb_iq_back == 4.0) {
      rtb_T1 = -rtb_X;
      rtb_T2 = rtb_id_back;
    } else if (rtb_iq_back == 5.0) {
      rtb_T1 = rtb_X;
      rtb_T2 = -rtb_Y;
    } else if (rtb_iq_back == 6.0) {
      rtb_T1 = -rtb_Y;
      rtb_T2 = -rtb_id_back;
    }

    if (rtb_iq_back == 2.0) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm - rtb_T1) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm + rtb_T1) + rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm + rtb_T1) - rtb_T2) / 4.0F;
    } else if (rtb_iq_back == 3.0) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm - rtb_T1) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm + rtb_T1) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm + rtb_T1) + rtb_T2) / 4.0F;
    } else if (rtb_iq_back == 4.0) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm + rtb_T1) + rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm + rtb_T1) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm - rtb_T1) - rtb_T2) / 4.0F;
    } else if (rtb_iq_back == 5.0) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm + rtb_T1) + rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm - rtb_T1) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm + rtb_T1) - rtb_T2) / 4.0F;
    } else if (rtb_iq_back == 6.0) {
      /* Outport: '<Root>/Tcm1' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm1 = ((rtU.Tpwm + rtb_T1) - rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm2' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm2 = ((rtU.Tpwm + rtb_T1) + rtb_T2) / 4.0F;

      /* Outport: '<Root>/Tcm3' incorporates:
       *  Inport: '<Root>/Tpwm'
       */
      rtY.Tcm3 = ((rtU.Tpwm - rtb_T1) - rtb_T2) / 4.0F;
    }
  }

  /* S-Function (EKF_CAL): '<S1>/EKF' incorporates:
   *  Inport: '<Root>/Ls'
   *  Inport: '<Root>/Rs'
   *  Inport: '<Root>/flux'
   *  Outport: '<Root>/Angle'
   *  Outport: '<Root>/Out1'
   *  Outport: '<Root>/Out2'
   *  Outport: '<Root>/Speed'
   */
  EKF_CAL_Outputs_wrapper(&rtDW.Valpha, &rtDW.Vbeta, &rtDW.Gain, &rtDW.Gain1,
    &rtU.Rs, &rtU.flux, &rtU.Ls, &rtY.Out1, &rtY.Out2, &rtY.Speed, &rtY.Angle,
    &rtDW.EKF_DSTATE[0]);

  /* Outport: '<Root>/Iq' */
  rtY.Iq = rtb_Iq;

  /* Outport: '<Root>/Id' */
  rtY.Id = rtb_Id;

  /* Update for DiscreteIntegrator: '<S45>/Integrator' incorporates:
   *  Sum: '<S38>/SumI4'
   */
  rtDW.di += (rtb_Integrator_j + rtb_IntegralGain) * 1.0E-6F;

  /* Update for DiscreteIntegrator: '<S95>/Integrator' incorporates:
   *  Gain: '<S88>/Kb'
   *  Gain: '<S92>/Integral Gain'
   *  Sum: '<S88>/SumI2'
   *  Sum: '<S88>/SumI4'
   */
  rtDW.qi += ((rtb_Saturation - rtb_Sum) * rtP.Q_PI_KB + rtP.Q_PI_I * rtb_iq_sum)
    * 1.0E-6F;

  /* Update for S-Function (EKF_CAL): '<S1>/EKF' incorporates:
   *  Inport: '<Root>/Ls'
   *  Inport: '<Root>/Rs'
   *  Inport: '<Root>/flux'
   *  Outport: '<Root>/Angle'
   *  Outport: '<Root>/Out1'
   *  Outport: '<Root>/Out2'
   *  Outport: '<Root>/Speed'
   */

  /* S-Function "EKF_CAL_wrapper" Block: <S1>/EKF */
  EKF_CAL_Update_wrapper(&rtDW.Valpha, &rtDW.Vbeta, &rtDW.Gain, &rtDW.Gain1,
    &rtU.Rs, &rtU.flux, &rtU.Ls, &rtY.Out1, &rtY.Out2, &rtY.Speed, &rtY.Angle,
    &rtDW.EKF_DSTATE[0]);
}

/* Model initialize function */
void foc_simulink0_initialize(void)
{
  /* Start for S-Function (EKF_CAL): '<S1>/EKF' incorporates:
   *  Inport: '<Root>/Ls'
   *  Inport: '<Root>/Rs'
   *  Inport: '<Root>/flux'
   *  Outport: '<Root>/Angle'
   *  Outport: '<Root>/Out1'
   *  Outport: '<Root>/Out2'
   *  Outport: '<Root>/Speed'
   */

  /* S-Function Block: <S1>/EKF */
  EKF_CAL_Start_wrapper(&rtDW.EKF_DSTATE[0]);

  /* InitializeConditions for S-Function (EKF_CAL): '<S1>/EKF' incorporates:
   *  Inport: '<Root>/Ls'
   *  Inport: '<Root>/Rs'
   *  Inport: '<Root>/flux'
   *  Outport: '<Root>/Angle'
   *  Outport: '<Root>/Out1'
   *  Outport: '<Root>/Out2'
   *  Outport: '<Root>/Speed'
   */

  /* S-Function Block: <S1>/EKF */
  {
    real_T initVector[4] = { 0, 0, 0, 0 };

    {
      int_T i1;
      real_T *dw_DSTATE = &rtDW.EKF_DSTATE[0];
      for (i1=0; i1 < 4; i1++) {
        dw_DSTATE[i1] = initVector[i1];
      }
    }
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
