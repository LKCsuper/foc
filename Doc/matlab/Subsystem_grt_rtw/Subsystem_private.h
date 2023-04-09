/*
 * Subsystem_private.h
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

#ifndef RTW_HEADER_Subsystem_private_h_
#define RTW_HEADER_Subsystem_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "Subsystem_types.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetTFinal
#define rtmSetTFinal(rtm, val)         ((rtm)->Timing.tFinal = (val))
#endif

extern real32_T rt_remf_snf(real32_T u0, real32_T u1);

#endif                                 /* RTW_HEADER_Subsystem_private_h_ */
