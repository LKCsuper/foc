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

/* param */
#include "motor_common.h"
#include "motor_nameplate.h"
#include "motor_param.h"

/* 算法 */
#include "motor_math.h"

/* sensor */
#include "motor_hall.h"
#include "motor_encoder.h"
#include "motor_mt6701.h"
#include "motor_phase_order.h"
#include "motor_feedback.h"
#include "motor_hall_angle.h"
#include "motor_hall_diag.h"

/* 主控 / 环路 */
#include "motor_foc.h"
#include "motor_current.h"
#include "motor_anticogging.h"
#include "motor_sense.h"
#include "motor_task.h"
#include "motor_it.h"
#include "motor_foc_isr.h"
#include "motor_timing.h"
#include "motor_bldc.h"
#include "motor_pid.h"
#include "motor_fw.h"

/* 观测器（.h 在 motor/inc，.c 在 motor/observer） */
#include "motor_hfi.h"
#include "motor_flux.h"
#include "motor_luenberger.h"
#include "motor_ekf.h"
#include "motor_smo.h"
#include "motor_sensorless.h"

/* 环路 */
#include "motor_protect.h"
#include "motor_speed.h"
#include "motor_position.h"
#include "motor_joint.h"
#include "motor_joint_protocol.h"

/* 校准 / 存储 */
#include "motor_detect.h"
/* 可选：非阻塞偏移核心，裸机/主机逐步喂样本 */
#include "motor_detect_offset.h"
#include "flash_storage.h"
#include "motor.h"

#include "motor_simple_includes.h"

/* cmd */
#include "textbuf.h"
#include "command.h"
#include "cmd_serial_plot.h"

#ifdef __cplusplus
}
#endif
#endif
