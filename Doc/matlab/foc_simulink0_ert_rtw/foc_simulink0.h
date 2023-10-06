/*
 * File: foc_simulink0.h
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

#ifndef RTW_HEADER_foc_simulink0_h_
#define RTW_HEADER_foc_simulink0_h_
#ifndef foc_simulink0_COMMON_INCLUDES_
#define foc_simulink0_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* foc_simulink0_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T qi;                         /* '<S95>/Integrator' */
  real32_T di;                         /* '<S45>/Integrator' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T Id_ref;                     /* '<Root>/Id_ref' */
  real32_T Iq_ref;                     /* '<Root>/Iq_ref' */
  real32_T Theta;                      /* '<Root>/Theta' */
  real32_T Tpwm;                       /* '<Root>/Tpwm' */
  real32_T Udc;                        /* '<Root>/Udc' */
  real32_T Ia;                         /* '<Root>/Ia' */
  real32_T Ib;                         /* '<Root>/Ib' */
  real32_T Ic;                         /* '<Root>/Ic' */
  real_T we[2];                        /* '<Root>/we' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T Tcm1;                       /* '<Root>/Tcm1' */
  real32_T Tcm2;                       /* '<Root>/Tcm2' */
  real32_T Tcm3;                       /* '<Root>/Tcm3' */
  real32_T Vd;                         /* '<Root>/Vd' */
  real32_T Vq;                         /* '<Root>/Vq' */
  real32_T Id;                         /* '<Root>/Id' */
  real32_T Iq;                         /* '<Root>/Iq' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void foc_simulink0_initialize(void);
extern void foc_simulink0_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Scope' : Unused code path elimination
 * Block '<S1>/Scope1' : Unused code path elimination
 * Block '<S1>/Scope2' : Unused code path elimination
 * Block '<S1>/Scope3' : Unused code path elimination
 * Block '<S1>/Scope4' : Unused code path elimination
 * Block '<S1>/Scope5' : Unused code path elimination
 * Block '<S1>/Scope6' : Unused code path elimination
 * Block '<S7>/Scope' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('foc_simulink/foc_simulink')    - opens subsystem foc_simulink/foc_simulink
 * hilite_system('foc_simulink/foc_simulink/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'foc_simulink'
 * '<S1>'   : 'foc_simulink/foc_simulink'
 * '<S2>'   : 'foc_simulink/foc_simulink/MATLAB Function'
 * '<S3>'   : 'foc_simulink/foc_simulink/MATLAB Function1'
 * '<S4>'   : 'foc_simulink/foc_simulink/anti_park'
 * '<S5>'   : 'foc_simulink/foc_simulink/clarke'
 * '<S6>'   : 'foc_simulink/foc_simulink/d Controller'
 * '<S7>'   : 'foc_simulink/foc_simulink/park'
 * '<S8>'   : 'foc_simulink/foc_simulink/q Controller'
 * '<S9>'   : 'foc_simulink/foc_simulink/svpwm'
 * '<S10>'  : 'foc_simulink/foc_simulink/anti_park/antipark_valpha_fun'
 * '<S11>'  : 'foc_simulink/foc_simulink/anti_park/antipark_vbeta_fun'
 * '<S12>'  : 'foc_simulink/foc_simulink/clarke/MATLAB Function'
 * '<S13>'  : 'foc_simulink/foc_simulink/clarke/MATLAB Function1'
 * '<S14>'  : 'foc_simulink/foc_simulink/d Controller/Anti-windup'
 * '<S15>'  : 'foc_simulink/foc_simulink/d Controller/D Gain'
 * '<S16>'  : 'foc_simulink/foc_simulink/d Controller/Filter'
 * '<S17>'  : 'foc_simulink/foc_simulink/d Controller/Filter ICs'
 * '<S18>'  : 'foc_simulink/foc_simulink/d Controller/I Gain'
 * '<S19>'  : 'foc_simulink/foc_simulink/d Controller/Ideal P Gain'
 * '<S20>'  : 'foc_simulink/foc_simulink/d Controller/Ideal P Gain Fdbk'
 * '<S21>'  : 'foc_simulink/foc_simulink/d Controller/Integrator'
 * '<S22>'  : 'foc_simulink/foc_simulink/d Controller/Integrator ICs'
 * '<S23>'  : 'foc_simulink/foc_simulink/d Controller/N Copy'
 * '<S24>'  : 'foc_simulink/foc_simulink/d Controller/N Gain'
 * '<S25>'  : 'foc_simulink/foc_simulink/d Controller/P Copy'
 * '<S26>'  : 'foc_simulink/foc_simulink/d Controller/Parallel P Gain'
 * '<S27>'  : 'foc_simulink/foc_simulink/d Controller/Reset Signal'
 * '<S28>'  : 'foc_simulink/foc_simulink/d Controller/Saturation'
 * '<S29>'  : 'foc_simulink/foc_simulink/d Controller/Saturation Fdbk'
 * '<S30>'  : 'foc_simulink/foc_simulink/d Controller/Sum'
 * '<S31>'  : 'foc_simulink/foc_simulink/d Controller/Sum Fdbk'
 * '<S32>'  : 'foc_simulink/foc_simulink/d Controller/Tracking Mode'
 * '<S33>'  : 'foc_simulink/foc_simulink/d Controller/Tracking Mode Sum'
 * '<S34>'  : 'foc_simulink/foc_simulink/d Controller/Tsamp - Integral'
 * '<S35>'  : 'foc_simulink/foc_simulink/d Controller/Tsamp - Ngain'
 * '<S36>'  : 'foc_simulink/foc_simulink/d Controller/postSat Signal'
 * '<S37>'  : 'foc_simulink/foc_simulink/d Controller/preSat Signal'
 * '<S38>'  : 'foc_simulink/foc_simulink/d Controller/Anti-windup/Back Calculation'
 * '<S39>'  : 'foc_simulink/foc_simulink/d Controller/D Gain/Disabled'
 * '<S40>'  : 'foc_simulink/foc_simulink/d Controller/Filter/Disabled'
 * '<S41>'  : 'foc_simulink/foc_simulink/d Controller/Filter ICs/Disabled'
 * '<S42>'  : 'foc_simulink/foc_simulink/d Controller/I Gain/Internal Parameters'
 * '<S43>'  : 'foc_simulink/foc_simulink/d Controller/Ideal P Gain/Passthrough'
 * '<S44>'  : 'foc_simulink/foc_simulink/d Controller/Ideal P Gain Fdbk/Disabled'
 * '<S45>'  : 'foc_simulink/foc_simulink/d Controller/Integrator/Discrete'
 * '<S46>'  : 'foc_simulink/foc_simulink/d Controller/Integrator ICs/Internal IC'
 * '<S47>'  : 'foc_simulink/foc_simulink/d Controller/N Copy/Disabled wSignal Specification'
 * '<S48>'  : 'foc_simulink/foc_simulink/d Controller/N Gain/Disabled'
 * '<S49>'  : 'foc_simulink/foc_simulink/d Controller/P Copy/Disabled'
 * '<S50>'  : 'foc_simulink/foc_simulink/d Controller/Parallel P Gain/Internal Parameters'
 * '<S51>'  : 'foc_simulink/foc_simulink/d Controller/Reset Signal/Disabled'
 * '<S52>'  : 'foc_simulink/foc_simulink/d Controller/Saturation/Enabled'
 * '<S53>'  : 'foc_simulink/foc_simulink/d Controller/Saturation Fdbk/Disabled'
 * '<S54>'  : 'foc_simulink/foc_simulink/d Controller/Sum/Sum_PI'
 * '<S55>'  : 'foc_simulink/foc_simulink/d Controller/Sum Fdbk/Disabled'
 * '<S56>'  : 'foc_simulink/foc_simulink/d Controller/Tracking Mode/Disabled'
 * '<S57>'  : 'foc_simulink/foc_simulink/d Controller/Tracking Mode Sum/Passthrough'
 * '<S58>'  : 'foc_simulink/foc_simulink/d Controller/Tsamp - Integral/Passthrough'
 * '<S59>'  : 'foc_simulink/foc_simulink/d Controller/Tsamp - Ngain/Passthrough'
 * '<S60>'  : 'foc_simulink/foc_simulink/d Controller/postSat Signal/Forward_Path'
 * '<S61>'  : 'foc_simulink/foc_simulink/d Controller/preSat Signal/Forward_Path'
 * '<S62>'  : 'foc_simulink/foc_simulink/park/park_id_fun'
 * '<S63>'  : 'foc_simulink/foc_simulink/park/park_iq_fun'
 * '<S64>'  : 'foc_simulink/foc_simulink/q Controller/Anti-windup'
 * '<S65>'  : 'foc_simulink/foc_simulink/q Controller/D Gain'
 * '<S66>'  : 'foc_simulink/foc_simulink/q Controller/Filter'
 * '<S67>'  : 'foc_simulink/foc_simulink/q Controller/Filter ICs'
 * '<S68>'  : 'foc_simulink/foc_simulink/q Controller/I Gain'
 * '<S69>'  : 'foc_simulink/foc_simulink/q Controller/Ideal P Gain'
 * '<S70>'  : 'foc_simulink/foc_simulink/q Controller/Ideal P Gain Fdbk'
 * '<S71>'  : 'foc_simulink/foc_simulink/q Controller/Integrator'
 * '<S72>'  : 'foc_simulink/foc_simulink/q Controller/Integrator ICs'
 * '<S73>'  : 'foc_simulink/foc_simulink/q Controller/N Copy'
 * '<S74>'  : 'foc_simulink/foc_simulink/q Controller/N Gain'
 * '<S75>'  : 'foc_simulink/foc_simulink/q Controller/P Copy'
 * '<S76>'  : 'foc_simulink/foc_simulink/q Controller/Parallel P Gain'
 * '<S77>'  : 'foc_simulink/foc_simulink/q Controller/Reset Signal'
 * '<S78>'  : 'foc_simulink/foc_simulink/q Controller/Saturation'
 * '<S79>'  : 'foc_simulink/foc_simulink/q Controller/Saturation Fdbk'
 * '<S80>'  : 'foc_simulink/foc_simulink/q Controller/Sum'
 * '<S81>'  : 'foc_simulink/foc_simulink/q Controller/Sum Fdbk'
 * '<S82>'  : 'foc_simulink/foc_simulink/q Controller/Tracking Mode'
 * '<S83>'  : 'foc_simulink/foc_simulink/q Controller/Tracking Mode Sum'
 * '<S84>'  : 'foc_simulink/foc_simulink/q Controller/Tsamp - Integral'
 * '<S85>'  : 'foc_simulink/foc_simulink/q Controller/Tsamp - Ngain'
 * '<S86>'  : 'foc_simulink/foc_simulink/q Controller/postSat Signal'
 * '<S87>'  : 'foc_simulink/foc_simulink/q Controller/preSat Signal'
 * '<S88>'  : 'foc_simulink/foc_simulink/q Controller/Anti-windup/Back Calculation'
 * '<S89>'  : 'foc_simulink/foc_simulink/q Controller/D Gain/Disabled'
 * '<S90>'  : 'foc_simulink/foc_simulink/q Controller/Filter/Disabled'
 * '<S91>'  : 'foc_simulink/foc_simulink/q Controller/Filter ICs/Disabled'
 * '<S92>'  : 'foc_simulink/foc_simulink/q Controller/I Gain/Internal Parameters'
 * '<S93>'  : 'foc_simulink/foc_simulink/q Controller/Ideal P Gain/Passthrough'
 * '<S94>'  : 'foc_simulink/foc_simulink/q Controller/Ideal P Gain Fdbk/Disabled'
 * '<S95>'  : 'foc_simulink/foc_simulink/q Controller/Integrator/Discrete'
 * '<S96>'  : 'foc_simulink/foc_simulink/q Controller/Integrator ICs/Internal IC'
 * '<S97>'  : 'foc_simulink/foc_simulink/q Controller/N Copy/Disabled wSignal Specification'
 * '<S98>'  : 'foc_simulink/foc_simulink/q Controller/N Gain/Disabled'
 * '<S99>'  : 'foc_simulink/foc_simulink/q Controller/P Copy/Disabled'
 * '<S100>' : 'foc_simulink/foc_simulink/q Controller/Parallel P Gain/Internal Parameters'
 * '<S101>' : 'foc_simulink/foc_simulink/q Controller/Reset Signal/Disabled'
 * '<S102>' : 'foc_simulink/foc_simulink/q Controller/Saturation/Enabled'
 * '<S103>' : 'foc_simulink/foc_simulink/q Controller/Saturation Fdbk/Disabled'
 * '<S104>' : 'foc_simulink/foc_simulink/q Controller/Sum/Sum_PI'
 * '<S105>' : 'foc_simulink/foc_simulink/q Controller/Sum Fdbk/Disabled'
 * '<S106>' : 'foc_simulink/foc_simulink/q Controller/Tracking Mode/Disabled'
 * '<S107>' : 'foc_simulink/foc_simulink/q Controller/Tracking Mode Sum/Passthrough'
 * '<S108>' : 'foc_simulink/foc_simulink/q Controller/Tsamp - Integral/Passthrough'
 * '<S109>' : 'foc_simulink/foc_simulink/q Controller/Tsamp - Ngain/Passthrough'
 * '<S110>' : 'foc_simulink/foc_simulink/q Controller/postSat Signal/Forward_Path'
 * '<S111>' : 'foc_simulink/foc_simulink/q Controller/preSat Signal/Forward_Path'
 * '<S112>' : 'foc_simulink/foc_simulink/svpwm/svm1'
 * '<S113>' : 'foc_simulink/foc_simulink/svpwm/svm2'
 * '<S114>' : 'foc_simulink/foc_simulink/svpwm/svm3'
 * '<S115>' : 'foc_simulink/foc_simulink/svpwm/svm4'
 */
#endif                                 /* RTW_HEADER_foc_simulink0_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
