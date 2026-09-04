/*
 * @Description: 应用层头文件（motor / param / sensor / cmd）
 * @Author: lkc
 * @detail: 当前仍依赖 includes_platform.h（阶段 4 渐进拆分）；业务 .c 可只
 *          #include "includes_app.h"，或继续用总入口 includes.h。
 */
#ifndef _INCLUDES_APP_H
#define _INCLUDES_APP_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes_platform.h"

/* types */
#include "motor_types.h"

/* simple foc（学习分支，未接入主 ISR） */
#include "motor_simple_utils.h"

/* motor */
#include "motor_common.h"
#include "motor_param.h"
#include "motor_cal.h"
#include "motor_math.h"
#include "motor_hall.h"
#include "motor_encoder.h"
#include "motor_foc.h"
#include "motor_sense.h"
#include "motor_task.h"
#include "motor_it.h"
#include "motor_pid.h"
#include "motor_protect.h"
#include "motor_speed.h"
#include "motor_sensorless.h"
#include "motor_tim.h"
#include "motor_ctrl.h"
#include "motor_detect.h"
#include "flash_storage.h"
#include "motor.h"

#include "motor_simple_includes.h"

/* cmd */
#include "command.h"
#include "cmd_serial_plot.h"

#ifdef __cplusplus
}
#endif
#endif
