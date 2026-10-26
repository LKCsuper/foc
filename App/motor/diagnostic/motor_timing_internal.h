/*
 * @Description: 计时诊断模块内部共享状态，仅 motor_timing / motor_diag_* 源文件包含
 * @Author: lkc
 */
#ifndef MOTOR_TIMING_INTERNAL_H
#define MOTOR_TIMING_INTERNAL_H

/* Includes ------------------------------------------------------------------*/
#include "motor_timing.h"

/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 诊断模块（motor_timing / motor_diag_*）跨文件共享的模式开关与统计。
 * @detail: 原为 19 个分散的 extern 全局量（命名 s_* 但并非 static）。按写入方分组集中到一个结构体，
 *          字段语义、类型和 volatile 属性与拆分前逐一对应，不对模块外公开。
 */
typedef struct {
    /* motor_diag_handoff.c：交接短窗口与半周期配对 */
    bool pair_mode;
    bool pair_half;
    bool handoff_active;
    /* motor_timing.c：分段计时 */
    bool profile_active;
    bool performance_enabled; /* 上电默认 true */
    /* motor_diag_trace.c：运行/故障轨迹 */
    bool trace_active;
    bool trace_compact;
    bool trace_fault;
    /* motor_diag_schedule.c：调度节拍与回调间隔统计 */
    bool cadence_active;
    bool task_only;
    bool trace_entry_seed;
    volatile uint32_t speed_complete_us;
    uint16_t trace_gap_max;
    uint16_t trace_full_max;
    uint16_t trace_pending;
    uint16_t trace_up;
} motor_diag_shared_t;

/* Private variables ---------------------------------------------------------*/
extern motor_diag_shared_t g_motor_diag;

/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 清零全部分段计时累计（仅诊断模块内部使用）。
 * @return {void} 无
 */
void Motor_Timing_ProfileClearAll(void);

#endif
