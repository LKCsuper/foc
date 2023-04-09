/*
 * File: MATLAB0.h
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

#ifndef RTW_HEADER_MATLAB0_h_
#define RTW_HEADER_MATLAB0_h_
#ifndef MATLAB0_COMMON_INCLUDES_
#define MATLAB0_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* MATLAB0_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T IA;                           /* '<Root>/IA' */
  real_T IB;                           /* '<Root>/IB' */
  real_T IC;                           /* '<Root>/IC' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Ibeta;                        /* '<Root>/Ibeta' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void MATLAB0_initialize(void);
extern void MATLAB0_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

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
 * hilite_system('clarke/clarke/MATLAB Function1')    - opens subsystem clarke/clarke/MATLAB Function1
 * hilite_system('clarke/clarke/MATLAB Function1/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'clarke/clarke'
 * '<S1>'   : 'clarke/clarke/MATLAB Function1'
 */
#endif                                 /* RTW_HEADER_MATLAB0_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
