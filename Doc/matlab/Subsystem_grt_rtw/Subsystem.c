/*
 * Subsystem.c
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

#include "Subsystem.h"
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "Subsystem_private.h"
#include <math.h>
#include <string.h>
#include <float.h>

/* External inputs (root inport signals with default storage) */
ExtU_Subsystem_T Subsystem_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Subsystem_T Subsystem_Y;

/* Real-time model */
static RT_MODEL_Subsystem_T Subsystem_M_;
RT_MODEL_Subsystem_T *const Subsystem_M = &Subsystem_M_;
real32_T rt_remf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1) || rtIsInfF(u0)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u1)) {
    y = u0;
  } else if ((u1 != 0.0F) && (u1 != truncf(u1))) {
    real32_T q;
    q = fabsf(u0 / u1);
    if (!(fabsf(q - floorf(q + 0.5F)) > FLT_EPSILON * q)) {
      y = 0.0F * u0;
    } else {
      y = fmodf(u0, u1);
    }
  } else {
    y = fmodf(u0, u1);
  }

  return y;
}

/* Model step function */
void Subsystem_step(void)
{
  real32_T absx;
  real32_T b_absx;
  real32_T rtb_Ialpha;
  real32_T rtb_Ibeta;
  real32_T rtb_Id;
  real32_T rtb_Id_tmp;
  real32_T rtb_Iq;
  real32_T rtb_Va;
  real32_T rtb_Valpha;
  real32_T rtb_Vb;
  real32_T rtb_Vbeta;
  real32_T rtb_Vc;
  int8_T n;
  boolean_T guard1 = false;
  boolean_T tmp;
  boolean_T tmp_0;

  /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
   *  Inport: '<Root>/Thea'
   *  Inport: '<Root>/Vd'
   *  Inport: '<Root>/Vq'
   *  MATLAB Function: '<S3>/MATLAB Function'
   *  MATLAB Function: '<S3>/MATLAB Function1'
   *  MATLAB Function: '<S4>/MATLAB Function1'
   */
  tmp = rtIsInfF(Subsystem_U.Thea);
  tmp_0 = rtIsNaNF(Subsystem_U.Thea);
  if (tmp || tmp_0) {
    rtb_Id = (rtNaNF);
    absx = (rtNaNF);
  } else {
    rtb_Id_tmp = rt_remf_snf(Subsystem_U.Thea, 360.0F);
    rtb_Id = rtb_Id_tmp;
    absx = fabsf(rtb_Id);
    if (absx > 180.0F) {
      if (rtb_Id > 0.0F) {
        rtb_Id -= 360.0F;
      } else {
        rtb_Id += 360.0F;
      }

      absx = fabsf(rtb_Id);
    }

    if (absx <= 45.0F) {
      rtb_Id = sinf(0.0174532924F * rtb_Id);
    } else {
      guard1 = false;
      if (absx <= 135.0F) {
        if (rtb_Id > 0.0F) {
          rtb_Id = cosf((rtb_Id - 90.0F) * 0.0174532924F);
        } else {
          rtb_Id = (rtb_Id + 90.0F) * 0.0174532924F;
          n = -1;
          guard1 = true;
        }
      } else {
        if (rtb_Id > 0.0F) {
          rtb_Id = (rtb_Id - 180.0F) * 0.0174532924F;
          n = 2;
        } else {
          rtb_Id = (rtb_Id + 180.0F) * 0.0174532924F;
          n = -2;
        }

        guard1 = true;
      }

      if (guard1) {
        if (n == -1) {
          rtb_Id = -cosf(rtb_Id);
        } else {
          rtb_Id = -sinf(rtb_Id);
        }
      }
    }

    absx = rtb_Id_tmp;
    b_absx = fabsf(rtb_Id_tmp);
    if (b_absx > 180.0F) {
      if (rtb_Id_tmp > 0.0F) {
        absx = rtb_Id_tmp - 360.0F;
      } else {
        absx = rtb_Id_tmp + 360.0F;
      }

      b_absx = fabsf(absx);
    }

    if (b_absx <= 45.0F) {
      absx = cosf(0.0174532924F * absx);
    } else {
      guard1 = false;
      if (b_absx <= 135.0F) {
        if (absx > 0.0F) {
          absx = -sinf((absx - 90.0F) * 0.0174532924F);
        } else {
          absx = (absx + 90.0F) * 0.0174532924F;
          n = -1;
          guard1 = true;
        }
      } else {
        if (absx > 0.0F) {
          absx = (absx - 180.0F) * 0.0174532924F;
          n = 2;
        } else {
          absx = (absx + 180.0F) * 0.0174532924F;
          n = -2;
        }

        guard1 = true;
      }

      if (guard1) {
        if (n == -1) {
          absx = sinf(absx);
        } else {
          absx = -cosf(absx);
        }
      }
    }
  }

  rtb_Valpha = -Subsystem_U.Vq * rtb_Id + Subsystem_U.Vd * absx;

  /* End of MATLAB Function: '<S4>/MATLAB Function' */

  /* MATLAB Function: '<S4>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/Thea'
   *  Inport: '<Root>/Vd'
   *  Inport: '<Root>/Vq'
   */
  if (tmp || tmp_0) {
    rtb_Id = (rtNaNF);
    absx = (rtNaNF);
  } else {
    rtb_Id_tmp = rt_remf_snf(Subsystem_U.Thea, 360.0F);
    rtb_Id = rtb_Id_tmp;
    absx = fabsf(rtb_Id);
    if (absx > 180.0F) {
      if (rtb_Id > 0.0F) {
        rtb_Id -= 360.0F;
      } else {
        rtb_Id += 360.0F;
      }

      absx = fabsf(rtb_Id);
    }

    if (absx <= 45.0F) {
      rtb_Id = cosf(0.0174532924F * rtb_Id);
    } else {
      guard1 = false;
      if (absx <= 135.0F) {
        if (rtb_Id > 0.0F) {
          rtb_Id = -sinf((rtb_Id - 90.0F) * 0.0174532924F);
        } else {
          rtb_Id = (rtb_Id + 90.0F) * 0.0174532924F;
          n = -1;
          guard1 = true;
        }
      } else {
        if (rtb_Id > 0.0F) {
          rtb_Id = (rtb_Id - 180.0F) * 0.0174532924F;
          n = 2;
        } else {
          rtb_Id = (rtb_Id + 180.0F) * 0.0174532924F;
          n = -2;
        }

        guard1 = true;
      }

      if (guard1) {
        if (n == -1) {
          rtb_Id = sinf(rtb_Id);
        } else {
          rtb_Id = -cosf(rtb_Id);
        }
      }
    }

    absx = rtb_Id_tmp;
    b_absx = fabsf(rtb_Id_tmp);
    if (b_absx > 180.0F) {
      if (rtb_Id_tmp > 0.0F) {
        absx = rtb_Id_tmp - 360.0F;
      } else {
        absx = rtb_Id_tmp + 360.0F;
      }

      b_absx = fabsf(absx);
    }

    if (b_absx <= 45.0F) {
      absx = sinf(0.0174532924F * absx);
    } else {
      guard1 = false;
      if (b_absx <= 135.0F) {
        if (absx > 0.0F) {
          absx = cosf((absx - 90.0F) * 0.0174532924F);
        } else {
          absx = (absx + 90.0F) * 0.0174532924F;
          n = -1;
          guard1 = true;
        }
      } else {
        if (absx > 0.0F) {
          absx = (absx - 180.0F) * 0.0174532924F;
          n = 2;
        } else {
          absx = (absx + 180.0F) * 0.0174532924F;
          n = -2;
        }

        guard1 = true;
      }

      if (guard1) {
        if (n == -1) {
          absx = -cosf(absx);
        } else {
          absx = -sinf(absx);
        }
      }
    }
  }

  rtb_Vbeta = Subsystem_U.Vq * rtb_Id + Subsystem_U.Vd * absx;

  /* MATLAB Function: '<S5>/MATLAB Function' */
  rtb_Va = rtb_Valpha;

  /* MATLAB Function: '<S5>/MATLAB Function1' incorporates:
   *  MATLAB Function: '<S5>/MATLAB Function2'
   */
  rtb_Id_tmp = -rtb_Valpha * 0.5F;
  rtb_Vb = rtb_Vbeta * 0.866025388F + rtb_Id_tmp;

  /* MATLAB Function: '<S5>/MATLAB Function2' */
  rtb_Vc = rtb_Id_tmp - rtb_Vbeta * 0.866025388F;

  /* MATLAB Function: '<S2>/MATLAB Function' */
  rtb_Ialpha = rtb_Va;

  /* MATLAB Function: '<S2>/MATLAB Function1' */
  rtb_Ibeta = (rtb_Vb - rtb_Vc) * 0.666666687F * 0.866025388F;

  /* MATLAB Function: '<S3>/MATLAB Function' incorporates:
   *  Inport: '<Root>/Thea'
   */
  if (tmp || tmp_0) {
    rtb_Id = (rtNaNF);
    absx = (rtNaNF);
  } else {
    rtb_Id_tmp = rt_remf_snf(Subsystem_U.Thea, 360.0F);
    rtb_Id = rtb_Id_tmp;
    absx = fabsf(rtb_Id);
    if (absx > 180.0F) {
      if (rtb_Id > 0.0F) {
        rtb_Id -= 360.0F;
      } else {
        rtb_Id += 360.0F;
      }

      absx = fabsf(rtb_Id);
    }

    if (absx <= 45.0F) {
      rtb_Id = sinf(0.0174532924F * rtb_Id);
    } else {
      guard1 = false;
      if (absx <= 135.0F) {
        if (rtb_Id > 0.0F) {
          rtb_Id = cosf((rtb_Id - 90.0F) * 0.0174532924F);
        } else {
          rtb_Id = (rtb_Id + 90.0F) * 0.0174532924F;
          n = -1;
          guard1 = true;
        }
      } else {
        if (rtb_Id > 0.0F) {
          rtb_Id = (rtb_Id - 180.0F) * 0.0174532924F;
          n = 2;
        } else {
          rtb_Id = (rtb_Id + 180.0F) * 0.0174532924F;
          n = -2;
        }

        guard1 = true;
      }

      if (guard1) {
        if (n == -1) {
          rtb_Id = -cosf(rtb_Id);
        } else {
          rtb_Id = -sinf(rtb_Id);
        }
      }
    }

    absx = rtb_Id_tmp;
    b_absx = fabsf(rtb_Id_tmp);
    if (b_absx > 180.0F) {
      if (rtb_Id_tmp > 0.0F) {
        absx = rtb_Id_tmp - 360.0F;
      } else {
        absx = rtb_Id_tmp + 360.0F;
      }

      b_absx = fabsf(absx);
    }

    if (b_absx <= 45.0F) {
      absx = cosf(0.0174532924F * absx);
    } else {
      guard1 = false;
      if (b_absx <= 135.0F) {
        if (absx > 0.0F) {
          absx = -sinf((absx - 90.0F) * 0.0174532924F);
        } else {
          absx = (absx + 90.0F) * 0.0174532924F;
          n = -1;
          guard1 = true;
        }
      } else {
        if (absx > 0.0F) {
          absx = (absx - 180.0F) * 0.0174532924F;
          n = 2;
        } else {
          absx = (absx + 180.0F) * 0.0174532924F;
          n = -2;
        }

        guard1 = true;
      }

      if (guard1) {
        if (n == -1) {
          absx = sinf(absx);
        } else {
          absx = -cosf(absx);
        }
      }
    }
  }

  rtb_Iq = -rtb_Ialpha * rtb_Id + rtb_Ibeta * absx;

  /* Outport: '<Root>/Iq' */
  Subsystem_Y.Iq = rtb_Iq;

  /* MATLAB Function: '<S3>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/Thea'
   */
  if (tmp || tmp_0) {
    rtb_Id = (rtNaNF);
    absx = (rtNaNF);
  } else {
    rtb_Id_tmp = rt_remf_snf(Subsystem_U.Thea, 360.0F);
    rtb_Id = rtb_Id_tmp;
    absx = fabsf(rtb_Id);
    if (absx > 180.0F) {
      if (rtb_Id > 0.0F) {
        rtb_Id -= 360.0F;
      } else {
        rtb_Id += 360.0F;
      }

      absx = fabsf(rtb_Id);
    }

    if (absx <= 45.0F) {
      rtb_Id = cosf(0.0174532924F * rtb_Id);
    } else {
      guard1 = false;
      if (absx <= 135.0F) {
        if (rtb_Id > 0.0F) {
          rtb_Id = -sinf((rtb_Id - 90.0F) * 0.0174532924F);
        } else {
          rtb_Id = (rtb_Id + 90.0F) * 0.0174532924F;
          n = -1;
          guard1 = true;
        }
      } else {
        if (rtb_Id > 0.0F) {
          rtb_Id = (rtb_Id - 180.0F) * 0.0174532924F;
          n = 2;
        } else {
          rtb_Id = (rtb_Id + 180.0F) * 0.0174532924F;
          n = -2;
        }

        guard1 = true;
      }

      if (guard1) {
        if (n == -1) {
          rtb_Id = sinf(rtb_Id);
        } else {
          rtb_Id = -cosf(rtb_Id);
        }
      }
    }

    absx = rtb_Id_tmp;
    b_absx = fabsf(rtb_Id_tmp);
    if (b_absx > 180.0F) {
      if (rtb_Id_tmp > 0.0F) {
        absx = rtb_Id_tmp - 360.0F;
      } else {
        absx = rtb_Id_tmp + 360.0F;
      }

      b_absx = fabsf(absx);
    }

    if (b_absx <= 45.0F) {
      absx = sinf(0.0174532924F * absx);
    } else {
      guard1 = false;
      if (b_absx <= 135.0F) {
        if (absx > 0.0F) {
          absx = cosf((absx - 90.0F) * 0.0174532924F);
        } else {
          absx = (absx + 90.0F) * 0.0174532924F;
          n = -1;
          guard1 = true;
        }
      } else {
        if (absx > 0.0F) {
          absx = (absx - 180.0F) * 0.0174532924F;
          n = 2;
        } else {
          absx = (absx + 180.0F) * 0.0174532924F;
          n = -2;
        }

        guard1 = true;
      }

      if (guard1) {
        if (n == -1) {
          absx = -cosf(absx);
        } else {
          absx = -sinf(absx);
        }
      }
    }
  }

  rtb_Id = rtb_Ialpha * rtb_Id + rtb_Ibeta * absx;

  /* Outport: '<Root>/Id' */
  Subsystem_Y.Id = rtb_Id;

  /* Matfile logging */
  rt_UpdateTXYLogVars(Subsystem_M->rtwLogInfo, (&Subsystem_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [1.0E-6s, 0.0s] */
    if ((rtmGetTFinal(Subsystem_M)!=-1) &&
        !((rtmGetTFinal(Subsystem_M)-Subsystem_M->Timing.taskTime0) >
          Subsystem_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(Subsystem_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++Subsystem_M->Timing.clockTick0)) {
    ++Subsystem_M->Timing.clockTickH0;
  }

  Subsystem_M->Timing.taskTime0 = Subsystem_M->Timing.clockTick0 *
    Subsystem_M->Timing.stepSize0 + Subsystem_M->Timing.clockTickH0 *
    Subsystem_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void Subsystem_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)Subsystem_M, 0,
                sizeof(RT_MODEL_Subsystem_T));
  rtmSetTFinal(Subsystem_M, 1.0);
  Subsystem_M->Timing.stepSize0 = 1.0E-6;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    Subsystem_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(Subsystem_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(Subsystem_M->rtwLogInfo, (NULL));
    rtliSetLogT(Subsystem_M->rtwLogInfo, "tout");
    rtliSetLogX(Subsystem_M->rtwLogInfo, "");
    rtliSetLogXFinal(Subsystem_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(Subsystem_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(Subsystem_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(Subsystem_M->rtwLogInfo, 0);
    rtliSetLogDecimation(Subsystem_M->rtwLogInfo, 1);
    rtliSetLogY(Subsystem_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(Subsystem_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(Subsystem_M->rtwLogInfo, (NULL));
  }

  /* external inputs */
  (void)memset(&Subsystem_U, 0, sizeof(ExtU_Subsystem_T));

  /* external outputs */
  (void)memset(&Subsystem_Y, 0, sizeof(ExtY_Subsystem_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(Subsystem_M->rtwLogInfo, 0.0, rtmGetTFinal
    (Subsystem_M), Subsystem_M->Timing.stepSize0, (&rtmGetErrorStatus
    (Subsystem_M)));
}

/* Model terminate function */
void Subsystem_terminate(void)
{
  /* (no terminate code required) */
}
