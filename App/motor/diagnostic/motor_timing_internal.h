/*
 * @Description: 计时诊断模块内部共享状态，仅 motor_timing / motor_diag_* 源文件包含
 * @Author: lkc
 */
#ifndef MOTOR_TIMING_INTERNAL_H
#define MOTOR_TIMING_INTERNAL_H

/* Includes ------------------------------------------------------------------*/
#include "motor_timing.h"

/* 原为 motor_timing.c 文件级静态量；拆分后跨文件共享，不对模块外公开 */
extern bool s_pair_mode;
extern bool s_pair_half;
extern bool s_handoff_active;
extern bool s_profile_active;
extern bool s_performance_enabled;
extern volatile uint32_t s_profile_sum[MOTOR_TIMING_STAGE_COUNT];
extern volatile uint32_t s_profile_count[MOTOR_TIMING_STAGE_COUNT];
extern volatile uint32_t s_profile_max[MOTOR_TIMING_STAGE_COUNT];
extern bool s_trace_active;
extern bool s_trace_compact;
extern bool s_trace_fault;
extern volatile uint32_t s_speed_complete_us;
extern bool s_cadence_active;
extern bool s_task_only;
extern bool s_trace_entry_seed;
extern uint16_t s_trace_gap_max;
extern uint16_t s_trace_full_max;
extern uint16_t s_trace_pending;
extern uint16_t s_trace_up;

#endif
