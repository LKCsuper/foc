/*
 * Subsystem.h
 *
 * Code generation for model "Subsystem".
 *
 * Model version              : 1.46
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C source code generated on : Sun Apr  9 10:33:16 2023
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Subsystem_h_
#define RTW_HEADER_Subsystem_h_
#ifndef Subsystem_COMMON_INCLUDES_
#define Subsystem_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* Subsystem_COMMON_INCLUDES_ */

#include "Subsystem_types.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include <float.h>
#include <string.h>
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T Vd;                         /* '<Root>/Vd' */
  real32_T Vq;                         /* '<Root>/Vq' */
  real32_T Thea;                       /* '<Root>/Thea' */
} ExtU_Subsystem_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T Iq;                         /* '<Root>/Iq' */
  real32_T Id;                         /* '<Root>/Id' */
} ExtY_Subsystem_T;

/* Real-time Model Data Structure */
struct tag_RTM_Subsystem_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* External inputs (root inport signals with default storage) */
extern ExtU_Subsystem_T Subsystem_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Subsystem_T Subsystem_Y;

/* Model entry point functions */
extern void Subsystem_initialize(void);
extern void Subsystem_step(void);
extern void Subsystem_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Subsystem_T *const Subsystem_M;

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
 * hilite_system('foc/Subsystem')    - opens subsystem foc/Subsystem
 * hilite_system('foc/Subsystem/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'foc'
 * '<S1>'   : 'foc/Subsystem'
 * '<S2>'   : 'foc/Subsystem/clarke'
 * '<S3>'   : 'foc/Subsystem/park'
 * '<S4>'   : 'foc/Subsystem/rePark'
 * '<S5>'   : 'foc/Subsystem/reclarke'
 * '<S6>'   : 'foc/Subsystem/clarke/MATLAB Function'
 * '<S7>'   : 'foc/Subsystem/clarke/MATLAB Function1'
 * '<S8>'   : 'foc/Subsystem/park/MATLAB Function'
 * '<S9>'   : 'foc/Subsystem/park/MATLAB Function1'
 * '<S10>'  : 'foc/Subsystem/rePark/MATLAB Function'
 * '<S11>'  : 'foc/Subsystem/rePark/MATLAB Function1'
 * '<S12>'  : 'foc/Subsystem/reclarke/MATLAB Function'
 * '<S13>'  : 'foc/Subsystem/reclarke/MATLAB Function1'
 * '<S14>'  : 'foc/Subsystem/reclarke/MATLAB Function2'
 */
#endif                                 /* RTW_HEADER_Subsystem_h_ */
