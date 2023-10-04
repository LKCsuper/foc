/*
 * File: foc_simulink0.h
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
  real32_T di;                         /* '<S47>/Integrator' */
  real32_T qi;                         /* '<S97>/Integrator' */
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
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T Tcm1;                       /* '<Root>/Tcm1' */
  real32_T Tcm2;                       /* '<Root>/Tcm2' */
  real32_T Tcm3;                       /* '<Root>/Tcm3' */
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
 * Block '<S6>/Scope' : Unused code path elimination
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
 * '<S2>'   : 'foc_simulink/foc_simulink/Subsystem'
 * '<S3>'   : 'foc_simulink/foc_simulink/anti_park'
 * '<S4>'   : 'foc_simulink/foc_simulink/clarke'
 * '<S5>'   : 'foc_simulink/foc_simulink/d Controller'
 * '<S6>'   : 'foc_simulink/foc_simulink/park'
 * '<S7>'   : 'foc_simulink/foc_simulink/q Controller'
 * '<S8>'   : 'foc_simulink/foc_simulink/Subsystem/MATLAB Function'
 * '<S9>'   : 'foc_simulink/foc_simulink/Subsystem/MATLAB Function1'
 * '<S10>'  : 'foc_simulink/foc_simulink/Subsystem/MATLAB Function2'
 * '<S11>'  : 'foc_simulink/foc_simulink/Subsystem/MATLAB Function3'
 * '<S12>'  : 'foc_simulink/foc_simulink/anti_park/MATLAB Function'
 * '<S13>'  : 'foc_simulink/foc_simulink/anti_park/MATLAB Function1'
 * '<S14>'  : 'foc_simulink/foc_simulink/clarke/MATLAB Function'
 * '<S15>'  : 'foc_simulink/foc_simulink/clarke/MATLAB Function1'
 * '<S16>'  : 'foc_simulink/foc_simulink/d Controller/Anti-windup'
 * '<S17>'  : 'foc_simulink/foc_simulink/d Controller/D Gain'
 * '<S18>'  : 'foc_simulink/foc_simulink/d Controller/Filter'
 * '<S19>'  : 'foc_simulink/foc_simulink/d Controller/Filter ICs'
 * '<S20>'  : 'foc_simulink/foc_simulink/d Controller/I Gain'
 * '<S21>'  : 'foc_simulink/foc_simulink/d Controller/Ideal P Gain'
 * '<S22>'  : 'foc_simulink/foc_simulink/d Controller/Ideal P Gain Fdbk'
 * '<S23>'  : 'foc_simulink/foc_simulink/d Controller/Integrator'
 * '<S24>'  : 'foc_simulink/foc_simulink/d Controller/Integrator ICs'
 * '<S25>'  : 'foc_simulink/foc_simulink/d Controller/N Copy'
 * '<S26>'  : 'foc_simulink/foc_simulink/d Controller/N Gain'
 * '<S27>'  : 'foc_simulink/foc_simulink/d Controller/P Copy'
 * '<S28>'  : 'foc_simulink/foc_simulink/d Controller/Parallel P Gain'
 * '<S29>'  : 'foc_simulink/foc_simulink/d Controller/Reset Signal'
 * '<S30>'  : 'foc_simulink/foc_simulink/d Controller/Saturation'
 * '<S31>'  : 'foc_simulink/foc_simulink/d Controller/Saturation Fdbk'
 * '<S32>'  : 'foc_simulink/foc_simulink/d Controller/Sum'
 * '<S33>'  : 'foc_simulink/foc_simulink/d Controller/Sum Fdbk'
 * '<S34>'  : 'foc_simulink/foc_simulink/d Controller/Tracking Mode'
 * '<S35>'  : 'foc_simulink/foc_simulink/d Controller/Tracking Mode Sum'
 * '<S36>'  : 'foc_simulink/foc_simulink/d Controller/Tsamp - Integral'
 * '<S37>'  : 'foc_simulink/foc_simulink/d Controller/Tsamp - Ngain'
 * '<S38>'  : 'foc_simulink/foc_simulink/d Controller/postSat Signal'
 * '<S39>'  : 'foc_simulink/foc_simulink/d Controller/preSat Signal'
 * '<S40>'  : 'foc_simulink/foc_simulink/d Controller/Anti-windup/Back Calculation'
 * '<S41>'  : 'foc_simulink/foc_simulink/d Controller/D Gain/Disabled'
 * '<S42>'  : 'foc_simulink/foc_simulink/d Controller/Filter/Disabled'
 * '<S43>'  : 'foc_simulink/foc_simulink/d Controller/Filter ICs/Disabled'
 * '<S44>'  : 'foc_simulink/foc_simulink/d Controller/I Gain/Internal Parameters'
 * '<S45>'  : 'foc_simulink/foc_simulink/d Controller/Ideal P Gain/Passthrough'
 * '<S46>'  : 'foc_simulink/foc_simulink/d Controller/Ideal P Gain Fdbk/Disabled'
 * '<S47>'  : 'foc_simulink/foc_simulink/d Controller/Integrator/Discrete'
 * '<S48>'  : 'foc_simulink/foc_simulink/d Controller/Integrator ICs/Internal IC'
 * '<S49>'  : 'foc_simulink/foc_simulink/d Controller/N Copy/Disabled wSignal Specification'
 * '<S50>'  : 'foc_simulink/foc_simulink/d Controller/N Gain/Disabled'
 * '<S51>'  : 'foc_simulink/foc_simulink/d Controller/P Copy/Disabled'
 * '<S52>'  : 'foc_simulink/foc_simulink/d Controller/Parallel P Gain/Internal Parameters'
 * '<S53>'  : 'foc_simulink/foc_simulink/d Controller/Reset Signal/Disabled'
 * '<S54>'  : 'foc_simulink/foc_simulink/d Controller/Saturation/Enabled'
 * '<S55>'  : 'foc_simulink/foc_simulink/d Controller/Saturation Fdbk/Disabled'
 * '<S56>'  : 'foc_simulink/foc_simulink/d Controller/Sum/Sum_PI'
 * '<S57>'  : 'foc_simulink/foc_simulink/d Controller/Sum Fdbk/Disabled'
 * '<S58>'  : 'foc_simulink/foc_simulink/d Controller/Tracking Mode/Disabled'
 * '<S59>'  : 'foc_simulink/foc_simulink/d Controller/Tracking Mode Sum/Passthrough'
 * '<S60>'  : 'foc_simulink/foc_simulink/d Controller/Tsamp - Integral/Passthrough'
 * '<S61>'  : 'foc_simulink/foc_simulink/d Controller/Tsamp - Ngain/Passthrough'
 * '<S62>'  : 'foc_simulink/foc_simulink/d Controller/postSat Signal/Forward_Path'
 * '<S63>'  : 'foc_simulink/foc_simulink/d Controller/preSat Signal/Forward_Path'
 * '<S64>'  : 'foc_simulink/foc_simulink/park/MATLAB Function'
 * '<S65>'  : 'foc_simulink/foc_simulink/park/MATLAB Function1'
 * '<S66>'  : 'foc_simulink/foc_simulink/q Controller/Anti-windup'
 * '<S67>'  : 'foc_simulink/foc_simulink/q Controller/D Gain'
 * '<S68>'  : 'foc_simulink/foc_simulink/q Controller/Filter'
 * '<S69>'  : 'foc_simulink/foc_simulink/q Controller/Filter ICs'
 * '<S70>'  : 'foc_simulink/foc_simulink/q Controller/I Gain'
 * '<S71>'  : 'foc_simulink/foc_simulink/q Controller/Ideal P Gain'
 * '<S72>'  : 'foc_simulink/foc_simulink/q Controller/Ideal P Gain Fdbk'
 * '<S73>'  : 'foc_simulink/foc_simulink/q Controller/Integrator'
 * '<S74>'  : 'foc_simulink/foc_simulink/q Controller/Integrator ICs'
 * '<S75>'  : 'foc_simulink/foc_simulink/q Controller/N Copy'
 * '<S76>'  : 'foc_simulink/foc_simulink/q Controller/N Gain'
 * '<S77>'  : 'foc_simulink/foc_simulink/q Controller/P Copy'
 * '<S78>'  : 'foc_simulink/foc_simulink/q Controller/Parallel P Gain'
 * '<S79>'  : 'foc_simulink/foc_simulink/q Controller/Reset Signal'
 * '<S80>'  : 'foc_simulink/foc_simulink/q Controller/Saturation'
 * '<S81>'  : 'foc_simulink/foc_simulink/q Controller/Saturation Fdbk'
 * '<S82>'  : 'foc_simulink/foc_simulink/q Controller/Sum'
 * '<S83>'  : 'foc_simulink/foc_simulink/q Controller/Sum Fdbk'
 * '<S84>'  : 'foc_simulink/foc_simulink/q Controller/Tracking Mode'
 * '<S85>'  : 'foc_simulink/foc_simulink/q Controller/Tracking Mode Sum'
 * '<S86>'  : 'foc_simulink/foc_simulink/q Controller/Tsamp - Integral'
 * '<S87>'  : 'foc_simulink/foc_simulink/q Controller/Tsamp - Ngain'
 * '<S88>'  : 'foc_simulink/foc_simulink/q Controller/postSat Signal'
 * '<S89>'  : 'foc_simulink/foc_simulink/q Controller/preSat Signal'
 * '<S90>'  : 'foc_simulink/foc_simulink/q Controller/Anti-windup/Back Calculation'
 * '<S91>'  : 'foc_simulink/foc_simulink/q Controller/D Gain/Disabled'
 * '<S92>'  : 'foc_simulink/foc_simulink/q Controller/Filter/Disabled'
 * '<S93>'  : 'foc_simulink/foc_simulink/q Controller/Filter ICs/Disabled'
 * '<S94>'  : 'foc_simulink/foc_simulink/q Controller/I Gain/Internal Parameters'
 * '<S95>'  : 'foc_simulink/foc_simulink/q Controller/Ideal P Gain/Passthrough'
 * '<S96>'  : 'foc_simulink/foc_simulink/q Controller/Ideal P Gain Fdbk/Disabled'
 * '<S97>'  : 'foc_simulink/foc_simulink/q Controller/Integrator/Discrete'
 * '<S98>'  : 'foc_simulink/foc_simulink/q Controller/Integrator ICs/Internal IC'
 * '<S99>'  : 'foc_simulink/foc_simulink/q Controller/N Copy/Disabled wSignal Specification'
 * '<S100>' : 'foc_simulink/foc_simulink/q Controller/N Gain/Disabled'
 * '<S101>' : 'foc_simulink/foc_simulink/q Controller/P Copy/Disabled'
 * '<S102>' : 'foc_simulink/foc_simulink/q Controller/Parallel P Gain/Internal Parameters'
 * '<S103>' : 'foc_simulink/foc_simulink/q Controller/Reset Signal/Disabled'
 * '<S104>' : 'foc_simulink/foc_simulink/q Controller/Saturation/Enabled'
 * '<S105>' : 'foc_simulink/foc_simulink/q Controller/Saturation Fdbk/Disabled'
 * '<S106>' : 'foc_simulink/foc_simulink/q Controller/Sum/Sum_PI'
 * '<S107>' : 'foc_simulink/foc_simulink/q Controller/Sum Fdbk/Disabled'
 * '<S108>' : 'foc_simulink/foc_simulink/q Controller/Tracking Mode/Disabled'
 * '<S109>' : 'foc_simulink/foc_simulink/q Controller/Tracking Mode Sum/Passthrough'
 * '<S110>' : 'foc_simulink/foc_simulink/q Controller/Tsamp - Integral/Passthrough'
 * '<S111>' : 'foc_simulink/foc_simulink/q Controller/Tsamp - Ngain/Passthrough'
 * '<S112>' : 'foc_simulink/foc_simulink/q Controller/postSat Signal/Forward_Path'
 * '<S113>' : 'foc_simulink/foc_simulink/q Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_foc_simulink0_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
