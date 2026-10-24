/*
 * @Description: 电机诊断命令：调度/分段计时、交接与轨迹采样、快照、电流测试及停机基准
 * @Author: lkc
 * @detail: 自 cmd_openloop.c 拆出，命令仍由 command.c 命令表注册。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include "common_assert.h"
#include "motor_control.h"
#include "motor_foc_isr.h"
#include "motor_timing.h"
#include "motor_diag_phase.h"
#include "motor_speed_ff_detect.h"
#include "motor_crit.h"
#include "motor_output.h"
#include "motor_foc.h"
#include "motor_current.h"
#include "motor_pid.h"
#include "motor_reference_trig.h"
#include "motor_feedback.h"
#include "motor_mt6701.h"
#include "hal_motor.h"
#include <math.h>
#include <stdarg.h>
#if MOTOR_CFG_CMD_DIAG
/* Private functions ---------------------------------------------------------*/
static MOTOR_PARAM_S test_motor;

/**
 * @description: 停机比较空框架、查表及泰勒三角后端，不操作真实电机。
 * @detail: 每组四次调用隔离中断，输入构造和结果累加均在计时区外。
 * @return {int} 0 成功，-1 停机条件不满足
 */
static int ol_trig_bench(void)
{
    motor_command_t command;
    uint32_t totals[3][8] = {{0U}}, maxima[3][8] = {{0U}};
    const int terms[5] = {3, 5, 7, 9, 12};
    volatile float checksum = 0.0f;
    unsigned family, mode, batch, index;
    if (Motor_GetCommand(&motor, &command) != MOTOR_OK || command.enable || motor.ol_enable ||
        command.target_rpm != 0.0f || Motor_Position_IsEnabled() || Motor_Joint_IsEnabled() ||
        Motor_Bldc_IsSelected() || Motor_Detect_IsActive() || Motor_Protect_IsFaultLatched()) {
        PRINTF("Invalid: trigbench requires stopped FOC, zero target, position/joint off, no detection/fault\n");
        return -1;
    }
    for (family = 0U; family < 3U; ++family) {
        for (mode = 0U; mode < 8U; ++mode) {
            for (batch = 0U; batch < 128U; ++batch) {
                float sine = 0.0f, cosine = 0.0f;
                float angle = family == 0U ? -3.141592654f + 6.283185307f * (float)batch / 127.0f :
                              family == 1U ? -64.0f + 128.0f * (float)batch / 127.0f :
                              ((float)(batch % 9U) - 3.5f) * 1.570796327f +
                              ((batch & 1U) ? 0.000001f : -0.000001f);
                uint32_t key = Motor_Crit_Enter();
                uint32_t begin = Hal_Motor_TimeUs();
                uint32_t elapsed;
                for (index = 0U; index < 4U; ++index) {
                    if (mode == 1U) sincos_f32(angle, &sine, &cosine);
                    else if (mode == 2U) sincos_taylor_f32(angle, &sine, &cosine);
                    else if (mode >= 3U) Motor_ReferenceTrigRaw(angle, terms[mode - 3U], &sine, &cosine);
                }
                elapsed = Hal_Motor_TimeUs() - begin;
                Motor_Crit_Exit(key);
                totals[family][mode] += elapsed;
                if (elapsed > maxima[family][mode]) maxima[family][mode] = elapsed;
                checksum += sine + cosine;
            }
        }
    }
    for (family = 0U; family < 3U; ++family) {
        for (mode = 0U; mode < 8U; ++mode) {
            PRINTF("Trig bench family=%u mode=%u n=512 total=%lu avg=%.3f max_batch=%lu us\n",
                   family, mode, (unsigned long)totals[family][mode], (float)totals[family][mode] / 512.0f,
                   (unsigned long)maxima[family][mode]);
        }
    }
    PRINTF("Families: 0=single_turn 1=64rad 2=quadrant_edges; modes 0=empty 1=table 2=Taylor 3=ref_n3 4=ref_n5 5=ref_n7 6=ref_n9 7=ref_n12; checksum=%.6f; PWM untouched\n",
           (double)checksum);
    return 0;
}

/**
 * @description: 在停机独立实例上逐层比较电流计算耗时，不调用输出路径。
 * @detail: 四次调用为一组隔离中断；停用计时与交接采集，PI 共享统计会包含合成样本。
 * @return {int} 0 成功，-1 不满足停机条件
 */
static int ol_current_bench(void)
{
    motor_current_target_ctx_t ctx;
    motor_control_t control;
    motor_command_t command;
    uint32_t totals[2][23] = {{0U}}, maxima[2][23] = {{0U}};
    volatile float checksum = 0.0f;
    unsigned load, mode, batch, index;
    if (Motor_GetCommand(&motor, &command) != MOTOR_OK || command.enable || motor.ol_enable ||
        command.target_rpm != 0.0f || Motor_Position_IsEnabled() || Motor_Joint_IsEnabled() ||
        Motor_Bldc_IsSelected() || Motor_Detect_IsActive() || Motor_Protect_IsFaultLatched() ||
        Motor_Feedback_GetSourceDefault() != MOTOR_FEEDBACK_SRC_SENSORLESS) {
        PRINTF("Invalid: currentbench requires stopped FOC, zero target, position/joint off, no detection/fault\n");
        return -1;
    }
    Motor_Timing_CadenceStop();
    Motor_Timing_HandoffStop();
    memset(&ctx, 0, sizeof(ctx));
    ctx.if_iq_scale = 1.0f;
    ctx.soft_current_scale = 1.0f;
    Motor_Control_Init(&control);
    control.startup.phase = MOTOR_STARTUP_DRAG_RAMP;
    control.startup.soft_current_scale = 1.0f;
    control.startup.drag_current_scale = 1.0f;
    test_motor = motor;
    test_motor.ol_current_kp = 0.12f;
    test_motor.ol_current_ki = 18.0f;
    test_motor.ol_id_set = 0.0f;
    test_motor.ol_iq_set = 1.0f;
    test_motor.ol_align_current = 0.0f;
    test_motor.ol_align_enable = false;
    test_motor.ol_theta = 0.6f;
    test_motor.ol_mode = MOTOR_OL_MODE_IF;
    test_motor.ol_vq_limit = 4.0f;
    test_motor.sensorless_enable = true;
    test_motor.speed_enable = false;
    test_motor.joint_current_hold = false;
    test_motor.joint_voltage_hold = false;
    test_motor.position_voltage_hold = false;
    test_motor.fw_current_max = 0.0f;
    for (load = 0U; load < 2U; ++load) {
        test_motor.ia = load ? 80.0f : 0.3f;
        test_motor.ib = load ? -40.0f : -0.1f;
        test_motor.ic = -test_motor.ia - test_motor.ib;
        Motor_Current_FeedbackUpdateDq(&test_motor, 0.6f);
        for (mode = 0U; mode < 23U; ++mode) {
            if (mode == 9U) {
                Motor_Timing_CadenceReset();
                Motor_Timing_CallbackStop();
                Motor_Timing_ProfileSetDetailed(true);
            }
            for (batch = 0U; batch < 64U; ++batch) {
                motor_dq_voltage_t voltage = {load ? 3.0f : 0.3f, load ? 4.0f : 0.4f};
                float id_target = 0.0f, iq_target = 1.0f;
                bool probe = false, capture = false;
                uint32_t key, begin, elapsed;
                test_motor.ol_vd_int = test_motor.ol_vq_int = 0.0f;
                control.fw.id = control.fw.last_vd = control.fw.last_vq = 0.0f;
                key = Motor_Crit_Enter();
                if (mode == 9U) {
                    unsigned arm;
                    bool armed = false;
                    for (arm = 0U; arm < 256U; ++arm) {
                        if (Motor_Timing_ProfileBegin()) {
                            armed = true;
                            break;
                        }
                    }
                    if (!armed) {
                        Motor_Crit_Exit(key);
                        Motor_Timing_CadenceStop();
                        Motor_Timing_ProfileSetDetailed(false);
                        PRINTF("Invalid: currentbench profiling could not be armed\n");
                        return -1;
                    }
                }
                begin = Hal_Motor_TimeUs();
                for (index = 0U; index < 4U; ++index) {
                    switch (mode) {
                    case 1U:
                        Motor_Current_FeedbackUpdateDq(&test_motor, 0.6f);
                        break;
                    case 2U:
                        Motor_Current_GetTargets(&test_motor, &ctx, &id_target, &iq_target);
                        break;
                    case 3U:
                    case 5U:
                        voltage.vd = Motor_Current_PI_Run(id_target, test_motor.Id, &test_motor.ol_vd_int,
                                                          0.12f, 18.0f, 1.0f / 15000.0f, 4.0f);
                        voltage.vq = Motor_Current_PI_Run(iq_target, test_motor.Iq, &test_motor.ol_vq_int,
                                                          0.12f, 18.0f, 1.0f / 15000.0f, 4.0f);
                        if (mode == 5U) Motor_Current_LimitVoltage(&test_motor, &voltage,
                                                   id_target - test_motor.Id, iq_target - test_motor.Iq, 4.0f);
                        break;
                    case 4U:
                        voltage.vd = load ? 3.0f : 0.3f;
                        voltage.vq = load ? 4.0f : 0.4f;
                        Motor_Current_LimitVoltage(&test_motor, &voltage,
                                                   id_target - test_motor.Id, iq_target - test_motor.Iq, 4.0f);
                        break;
                    case 6U:
                        voltage = Motor_Current_RunPi(&test_motor, id_target, iq_target, 1.0f / 15000.0f, 4.0f);
                        break;
                    case 7U:
                    case 9U:
                    case 13U:
                        voltage = Motor_Current_Step(&test_motor, 0.6f, &ctx, 1.0f / 15000.0f, 4.0f,
                                                     mode == 13U ? NULL : &id_target,
                                                     mode == 13U ? NULL : &iq_target);
                        break;
                    case 8U:
                    case 10U:
                    case 11U:
                    case 12U:
                        if (mode == 10U || mode == 12U) probe = Motor_Timing_ProfileDetailedIsActive();
                        Motor_Current_FeedbackUpdateDq(&test_motor, 0.6f);
                        Motor_Current_GetTargets(&test_motor, &ctx, &id_target, &iq_target);
                        if (mode == 11U || mode == 12U) capture = Motor_Timing_HandoffDue();
                        voltage = Motor_Current_RunPi(&test_motor, id_target, iq_target, 1.0f / 15000.0f, 4.0f);
                        break;
                    default:
                        if (mode >= 14U) {
                            motor_control_result_t result = Motor_Control_IfDiagnosticStep(&control, &test_motor,
                                                                                           &ctx, mode - 14U);
                            voltage.vd = result.vd;
                            voltage.vq = result.vq;
                        }
                        break;
                    }
                }
                elapsed = Hal_Motor_TimeUs() - begin;
                if (mode == 9U) Motor_Timing_ProfileEnd();
                Motor_Crit_Exit(key);
                if (probe || capture) {
                    PRINTF("Invalid: currentbench expected unarmed diagnostic checks\n");
                    return -1;
                }
                totals[load][mode] += elapsed;
                if (elapsed > maxima[load][mode]) maxima[load][mode] = elapsed;
                checksum += voltage.vd + voltage.vq + id_target + iq_target + test_motor.Id;
            }
            if (mode == 9U) {
                Motor_Timing_CadenceStop();
                Motor_Timing_ProfileSetDetailed(false);
            }
        }
    }
    for (load = 0U; load < 2U; ++load) {
        for (mode = 0U; mode < 23U; ++mode) {
            PRINTF("Current bench load=%u mode=%u n=256 total=%lu avg=%.3f max_batch=%lu us\n",
                   load, mode, (unsigned long)totals[load][mode], (float)totals[load][mode] / 256.0f,
                   (unsigned long)maxima[load][mode]);
        }
    }
    PRINTF("Modes: 0=empty 1=Park 2=targets 3=PI 4=limit 5=PI+limit 6=RunPi 7=Step 8=chain 9=Step_profile 10=chain_probe 11=chain_capture 12=chain_both 13=Step_null; checksum=%.6f\n",
           (double)checksum);
    PRINTF("Synthetic stopped instance, PWM untouched; IRQ isolated batch4; mode9 arms profiling before timing; profiling/handoff stopped at end; PI/stage statistics include synthetic calls\n");
    PRINTF("Outer modes: 14=empty 15=angle 16=given_angle 17=context 18=FW_off 19=vlimit 20=RunIf 21=RunIf_given 22=Step_same_entry\n");
    return 0;
}

/**
 * @description: 控制停机基准、控制节拍和任务调度诊断。
 * @param {int} argc 命令参数个数
 * @param {char **} argv 命令参数
 * @return {int} 0 成功，-1 参数或停机条件不满足
 */
int Cmd_SensorlessCadence(int argc, char *argv[])
{
    motor_cadence_t c;
    motor_callback_t cb;
    if (argc > 1 && strcmp(argv[1], "trigbench") == 0) return ol_trig_bench();
    if (argc > 1 && strcmp(argv[1], "currentbench") == 0) return ol_current_bench();
    if (argc > 1 && strcmp(argv[1], "irqtest") == 0) {
        motor_command_t command;
        bool masked, valid, restored;
        uint32_t start_us, duration_us;
        osStatus_t status;
        if (argc != 3 || (strcmp(argv[2], "on") != 0 && strcmp(argv[2], "off") != 0)) {
            PRINTF("Invalid: slcadence irqtest on|off (5 seconds, stopped only)\n");
            return -1;
        }
        if (Motor_GetCommand(&motor, &command) != MOTOR_OK || command.enable || motor.ol_enable ||
            command.target_rpm != 0.0f || Motor_Position_IsEnabled() || Motor_Joint_IsEnabled() ||
            Motor_Bldc_IsSelected() || Motor_Detect_IsActive() || Motor_Protect_IsFaultLatched()) {
            PRINTF("Invalid: irqtest requires stopped FOC, zero target, position/joint off, no detection/fault\n");
            return -1;
        }
        masked = strcmp(argv[2], "off") == 0;
        if (Motor_RequestStop(&motor) != MOTOR_OK || !Hal_Motor_StoppedIrqTestBegin(masked)) {
            PRINTF("Invalid: irqtest could not confirm gate disabled\n");
            return -1;
        }
        Motor_Timing_SpeedTaskReset();
        start_us = Hal_Motor_TimeUs();
        status = osDelay(pdMS_TO_TICKS(5000U));
        duration_us = Hal_Motor_TimeUs() - start_us;
        Motor_Timing_ScheduleDebugFreeze();
        valid = status == osOK && Hal_Motor_StoppedIrqTestCheck(masked) &&
                Motor_GetCommand(&motor, &command) == MOTOR_OK && !command.enable && !motor.ol_enable &&
                command.target_rpm == 0.0f && !Motor_Position_IsEnabled() && !Motor_Joint_IsEnabled() &&
                !Motor_Bldc_IsSelected() && !Motor_Detect_IsActive() && !Motor_Protect_IsFaultLatched();
        if (Motor_RequestStop(&motor) != MOTOR_OK) valid = false;
        restored = Hal_Motor_StoppedIrqTestEnd(valid);
        PRINTF("IRQ test %s: masked=%u gate_low=%u restored=%u duration=%luus; motor stays stopped\n",
               valid && restored ? "PASS" : "FAIL", masked ? 1U : 0U, valid ? 1U : 0U,
               restored ? 1U : 0U, (unsigned long)duration_us);
        return valid && restored ? 0 : -1;
    }
    if (argc > 1 && strcmp(argv[1], "debug") == 0) {
        if (motor.ol_enable) {
            PRINTF("Invalid: slcadence debug requires stopped motor\n");
            return 0;
        }
        Motor_Timing_ScheduleDebugFreeze();
        PRINTF("Schedule RAM frozen: sequence=%lu bytes=%u\n",
               (unsigned long)g_motor_schedule_debug.sequence, (unsigned)sizeof(g_motor_schedule_debug));
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "blendbench") == 0) {
        static motor_sensorless_runtime_t test_runtime;
        motor_sensorless_effects_t effects;
        unsigned mode, i;
        if (motor.ol_enable) {
            PRINTF("Invalid: slcadence blendbench requires stopped motor\n");
            return -1;
        }
        /* 仅调用独立实例核心，effects不落地，绝不启动真实电机或修改默认观测器。 */
        test_motor = motor;
        test_motor.ol_enable = true;
        test_motor.ol_mode = MOTOR_OL_MODE_IF;
        test_motor.sensorless_enable = true;
        test_motor.ol_pole_pairs = 7.0f;
        test_motor.ol_current_rpm = test_motor.ol_target_rpm = 600.0f;
        test_motor.sensorless_motor_r = 0.646111f;
        test_motor.sensorless_motor_l = 0.000133819f;
        test_motor.sensorless_flux_linkage = 0.0023358f;
        test_motor.sensorless_observer_gain = 12000000.0f;
        test_motor.sensorless_pll_kp = 2000.0f;
        test_motor.sensorless_pll_ki = 30000.0f;
        test_motor.sensorless_blend_time = 0.2f;
        test_motor.ia = test_motor.ib = test_motor.ic = 0.0f;
        for (mode = 0; mode < 3U; ++mode) {
            uint32_t total = 0U, maximum = 0U;
            unsigned closed = 0U;
            for (i = 0; i < 128U; ++i) {
                float phase = (float)(i % 4U) * 1.5f - 2.8f;
                float s, co;
                uint32_t key, begin, elapsed;
                sincos_f32(phase, &s, &co);
                memset(&test_runtime, 0, sizeof(test_runtime));
                test_runtime.initialized = true;
                test_runtime.x_alpha = 0.0023358f * co;
                test_runtime.x_beta = 0.0023358f * s;
                test_runtime.applied_v_alpha = -0.0023358f * 439.82297f * s;
                test_runtime.applied_v_beta = 0.0023358f * 439.82297f * co;
                test_runtime.pll_angle = phase;
                test_runtime.pll_speed_electrical = 439.82297f;
                test_runtime.filtered_rpm = 600.0f;
                test_runtime.state = mode == 0U ? MOTOR_SENSORLESS_OPEN_LOOP : MOTOR_SENSORLESS_CLOSED;
                test_runtime.blend = mode == 1U ? ((i & 1U) ? 0.01f : 0.99f) : 1.0f;
                test_runtime.stats.samples = i % 8U;
                test_motor.sensorless_takeover = mode != 0U;
                test_motor.ol_theta = phase - 1.2f;
                utils_norm_angle_rad(&test_motor.ol_theta);
                key = Motor_Crit_Enter();
                begin = Hal_Motor_TimeUs();
                Motor_Sensorless_UpdateState(&test_runtime, &test_motor, 1.0f / 15000.0f, &effects);
                elapsed = Hal_Motor_TimeUs() - begin;
                Motor_Crit_Exit(key);
                total += elapsed;
                if (elapsed > maximum) maximum = elapsed;
                if (test_runtime.state == MOTOR_SENSORLESS_CLOSED) closed++;
            }
            PRINTF("Stopped observer bench mode=%u (0=open 1=blend 2=closed) n=128 avg=%.3f max=%lu us closed=%u\n",
                   mode, (float)total / 128.0f, (unsigned long)maximum, closed);
        }
        PRINTF("Synthetic independent instance, 600RPM 7pp; IRQ isolated per update; PWM unchanged\n");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "faultbench") == 0) {
        motor_fault_trace_sample_t sample;
        unsigned mode, batch, index;
        if (motor.ol_enable) {
            PRINTF("Invalid: faultbench requires stopped motor\n");
            return -1;
        }
        for (mode = 0U; mode < 2U; mode++) {
            uint32_t total = 0U, maximum = 0U;
            unsigned writes = 0U;
            for (batch = 0U; batch < 256U; batch += 4U) {
                uint32_t key = Motor_Crit_Enter();
                uint32_t begin, elapsed;
                Motor_Timing_FaultTraceArm();
                begin = Hal_Motor_TimeUs();
                for (index = 0U; index < 4U; index++) {
                    uint32_t stamp = batch + index;
                    if (Motor_Timing_FaultTraceDue(stamp, true, mode == 0U)) {
                        Motor_FocIsr_CaptureFaultTraceSample(&motor, stamp, 1.0f / 15000.0f, &sample);
                        Motor_Timing_FaultTracePush(&sample);
                        writes++;
                    }
                }
                elapsed = Hal_Motor_TimeUs() - begin;
                Motor_Timing_FaultTraceFreeze(0U);
                Motor_Crit_Exit(key);
                total += elapsed;
                if (elapsed > maximum) maximum = elapsed;
            }
            PRINTF("Fault bench mode=%u n=256 writes=%u total=%lu us avg=%.3f us max_batch=%lu us\n",
                   mode, writes, (unsigned long)total, (float)total / 256.0f, (unsigned long)maximum);
        }
        PRINTF("Stopped only; mode0=force each call mode1=stride4; IRQ isolated batch4; PWM unchanged; synthetic trace replaces RAM; performance counters paused\n");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "bench") == 0) {
        static MOTOR_PARAM_S bench_motor;
        motor_trace_sample_t sample = {0};
        uint32_t begin, transport_us = 0U, write_us = 0U, capture_us = 0U, maximum_batch_us = 0U;
        unsigned i, batch;
        bool isolated = argc > 2 && strcmp(argv[2], "isolated") == 0;
        if (motor.ol_enable) {
            PRINTF("Invalid: slcadence bench requires stopped motor\n");
            return -1;
        }
        memset(&bench_motor, 0, sizeof(bench_motor));
        bench_motor.ol_current_kp = 0.12f;
        bench_motor.ol_vd_int = 0.5f;
        bench_motor.ol_vq_int = 1.0f;
        for (batch = 0; batch < 256U; batch += 4U) {
            uint32_t key = isolated ? Motor_Crit_Enter() : 0U;
            uint32_t elapsed;
            begin = Hal_Motor_TimeUs();
            for (i = batch; i < batch + 4U; ++i) {
                Motor_Current_TransportPi(&bench_motor, 0.001f + (float)(i % 16U) * 0.001f,
                                          0.0f, 0.5f, 0.0f, 0.5f);
            }
            elapsed = Hal_Motor_TimeUs() - begin;
            if (isolated) Motor_Crit_Exit(key);
            transport_us += elapsed;
            if (elapsed > maximum_batch_us) maximum_batch_us = elapsed;
        }
        for (batch = 0; batch < 256U; batch += 4U) {
            uint32_t key = isolated ? Motor_Crit_Enter() : 0U;
            uint32_t elapsed;
            begin = Hal_Motor_TimeUs();
            for (i = batch; i < batch + 4U; ++i) {
                Motor_FocIsr_CaptureTraceSample(&motor, 0.0f, 0.0f, &sample);
            }
            elapsed = Hal_Motor_TimeUs() - begin;
            if (isolated) Motor_Crit_Exit(key);
            capture_us += elapsed;
            if (elapsed > maximum_batch_us) maximum_batch_us = elapsed;
        }
        Motor_Timing_TraceArmEntryFast();
        for (batch = 0; batch < 384U; batch += 4U) {
            uint32_t key = isolated ? Motor_Crit_Enter() : 0U;
            uint32_t elapsed;
            begin = Hal_Motor_TimeUs();
            for (i = batch; i < batch + 4U; ++i) {
                sample.time_us = i;
                sample.theta = (float)i * 0.001f;
                if (Motor_Timing_TraceDue(2U)) Motor_Timing_TracePush(&sample);
            }
            elapsed = Hal_Motor_TimeUs() - begin;
            if (isolated) Motor_Crit_Exit(key);
            write_us += elapsed;
            if (elapsed > maximum_batch_us) maximum_batch_us = elapsed;
        }
        Motor_Timing_TraceStop();
        PRINTF("Stopped bench: transport n=256 total=%lu us avg=%.3f us checksum=%.6f/%.6f\n",
               (unsigned long)transport_us, (float)transport_us / 256.0f,
               bench_motor.ol_vd_int, bench_motor.ol_vq_int);
        PRINTF("Stopped bench: RAM Due+Push n=384 total=%lu us avg=%.3f us; excludes diagnostic capture\n",
               (unsigned long)write_us, (float)write_us / 384.0f);
        PRINTF("Stopped bench: diagnostic capture n=256 total=%lu us avg=%.3f us\n",
               (unsigned long)capture_us, (float)capture_us / 256.0f);
        PRINTF("Bench isolated=%u batch=4 max_batch=%lu us; RAM trace replaced by synthetic samples; PWM unchanged\n",
               isolated ? 1U : 0U, (unsigned long)maximum_batch_us);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "reset") == 0) {
        Motor_Timing_ProfileSetDetailed(false);
        Motor_Timing_CadenceReset();
    }
    if (argc > 1 && strcmp(argv[1], "detail") == 0) {
        Motor_Timing_ProfileSetDetailed(true);
        Motor_Timing_CadenceReset();
    }
    if (argc > 1 && strcmp(argv[1], "lean") == 0) {
        Motor_Timing_ProfileSetDetailed(argc > 2 && strcmp(argv[2], "detail") == 0);
        Motor_Timing_CadenceReset();
        Motor_Timing_CallbackStop();
        PRINTF("Lean cadence: callback statistics disabled; sampled IRQ body and task statistics enabled\n");
    }
    if (argc > 1 && strcmp(argv[1], "task") == 0) {
        Motor_Timing_SpeedTaskReset();
        PRINTF("Task-only statistics: IRQ/cadence/runtime/stage profiling disabled\n");
    }
    if (argc > 1 && strcmp(argv[1], "probe") == 0) Motor_Timing_WakeProbeReset();
    if (argc > 1 && strcmp(argv[1], "split") == 0) {
        Motor_Timing_SplitReset();
        PRINTF("Split timing: full/skip/publish and wake overlap; stage profiling off\n");
    }
    if (argc > 1 && strcmp(argv[1], "stop") == 0) Motor_Timing_CadenceStop();
    Motor_Timing_CadenceGet(&c);
    Motor_Timing_CallbackGet(&cb);
    {
        motor_profile_snapshot_t snapshot;
        unsigned i;
        Motor_Timing_ProfileSnapshot(&snapshot);
        for (i = 0U; i < MOTOR_TIMING_STAGE_COUNT; ++i) {
            PRINTF("stage %u n=%lu avg=%.2f max=%lu us\n", i, (unsigned long)snapshot.count[i],
                   snapshot.count[i] ? (float)snapshot.sum_us[i] / (float)snapshot.count[i] : 0.0f,
                   (unsigned long)snapshot.max_us[i]);
        }
    }
    PRINTF("cadence n=%lu sum=%lu min=%lu max=%lu up=%lu down=%lu gaps=%lu\n",
           (unsigned long)c.count, (unsigned long)c.sum_us, (unsigned long)c.min_us,
           (unsigned long)c.max_us, (unsigned long)c.up, (unsigned long)c.down, (unsigned long)c.gaps);
    PRINTF("cadence Hz=%.1f mean=%.3fus nominal_dt=%.3fus bins=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\n",
           c.sum_us ? 1000000.0f * (float)c.count / (float)c.sum_us : 0.0f,
           c.count ? (float)c.sum_us / (float)c.count : 0.0f, Motor_Timing_GetControlDt(&motor) * 1000000.0f,
           (unsigned long)c.bins[0], (unsigned long)c.bins[1], (unsigned long)c.bins[2],
           (unsigned long)c.bins[3], (unsigned long)c.bins[4], (unsigned long)c.bins[5],
           (unsigned long)c.bins[6], (unsigned long)c.bins[7]);
    PRINTF("callback n=%lu Hz=%.1f max_gap=%lu gaps=%lu bins=%lu/%lu/%lu/%lu/%lu/%lu\n",
           (unsigned long)cb.count, cb.interval_sum ? 1000000.0f * (float)cb.interval_count / (float)cb.interval_sum : 0.0f,
           (unsigned long)cb.interval_max, (unsigned long)cb.gaps,
           (unsigned long)cb.bins[0], (unsigned long)cb.bins[1], (unsigned long)cb.bins[2],
           (unsigned long)cb.bins[3], (unsigned long)cb.bins[4], (unsigned long)cb.bins[5]);
    PRINTF("callback full=%lu avg=%.2f max=%lu skip_avg=%.2f skip_max=%lu over33=%lu over66=%lu pending=%lu\n",
           (unsigned long)cb.full, cb.full ? (float)cb.full_sum / (float)cb.full : 0.0f, (unsigned long)cb.full_max,
           cb.count > cb.full ? (float)cb.skip_sum / (float)(cb.count - cb.full) : 0.0f, (unsigned long)cb.skip_max,
           (unsigned long)cb.over33, (unsigned long)cb.over66, (unsigned long)cb.pending);
    {
        motor_split_diag_t split;
        Motor_Timing_SplitGet(&split);
        PRINTF("split publish n=%lu avg=%.2f max=%lu on_full=%lu on_skip=%lu\n",
               (unsigned long)split.publish_count,
               split.publish_count ? (float)split.publish_sum_us / (float)split.publish_count : 0.0f,
               (unsigned long)split.publish_max_us, (unsigned long)split.publish_on_full,
               (unsigned long)(split.publish_count - split.publish_on_full));
        PRINTF("wake overlap n=%lu delay_avg=%.1f dma_avg=%.1f full_n=%lu full_us=%lu skip_n=%lu skip_us=%lu publish_n=%lu publish_us=%lu\n",
               (unsigned long)split.wake_count,
               split.wake_count ? (float)split.wake_delay_sum_us / (float)split.wake_count : 0.0f,
               split.wake_count ? (float)split.wake_dma_sum_us / (float)split.wake_count : 0.0f,
               (unsigned long)split.wake_full_count, (unsigned long)split.wake_full_sum_us,
               (unsigned long)split.wake_skip_count, (unsigned long)split.wake_skip_sum_us,
               (unsigned long)split.wake_publish_count, (unsigned long)split.wake_publish_sum_us);
        PRINTF("worst wake delay=%lu dma=%lu full_n=%lu full_us=%lu skip_n=%lu skip_us=%lu publish_n=%lu publish_us=%lu\n",
               (unsigned long)split.worst_delay_us, (unsigned long)split.worst_dma_us,
               (unsigned long)split.worst_full_count, (unsigned long)split.worst_full_us,
               (unsigned long)split.worst_skip_count, (unsigned long)split.worst_skip_us,
               (unsigned long)split.worst_publish_count, (unsigned long)split.worst_publish_us);
    }
    {
        motor_speed_task_diag_t task;
        Motor_Timing_SpeedTaskGet(&task);
        PRINTF("wake probe worst_PI valid=%lu delay=%luus DMA_n=%lu DMA_body=%luus\n",
               (unsigned long)task.worst_pi_wake.valid, (unsigned long)task.worst_pi_wake.delay_us,
               (unsigned long)task.worst_pi_wake.dma_count, (unsigned long)task.worst_pi_wake.dma_us);
        PRINTF("wake probe worst_wake valid=%lu delay=%luus DMA_n=%lu DMA_body=%luus\n",
               (unsigned long)task.worst_wake.valid, (unsigned long)task.worst_wake.delay_us,
               (unsigned long)task.worst_wake.dma_count, (unsigned long)task.worst_wake.dma_us);
        PRINTF("speed PI gaps <=1000/1100/1250/1500/2000/>2000us=%lu/%lu/%lu/%lu/%lu/%lu\n",
               (unsigned long)task.pi_gap_bins[0], (unsigned long)task.pi_gap_bins[1],
               (unsigned long)task.pi_gap_bins[2], (unsigned long)task.pi_gap_bins[3],
               (unsigned long)task.pi_gap_bins[4], (unsigned long)task.pi_gap_bins[5]);
        PRINTF("speed PI over1000us=%lu ratio=%.2f%% longest_streak=%lu\n",
               (unsigned long)task.pi_over_1ms,
               task.pi_count > 1U ? 100.0 * (double)task.pi_over_1ms / (double)(task.pi_count - 1U) : 0.0,
               (unsigned long)task.pi_over_streak_max);
        PRINTF("speed PI updates=%lu no_update=%lu max_gap=%luus mean_gap=%.1fus\n",
               (unsigned long)task.pi_count, (unsigned long)task.pi_no_update,
               (unsigned long)task.pi_max_gap_us,
               task.pi_count > 1U ? (double)task.pi_sum_gap_us / (double)(task.pi_count - 1U) : 0.0);
        PRINTF("speed task n=%lu max_gap=%luus last=%lu gate=%u target=%.1f feedback=%.1f iq=%.4f\n",
               (unsigned long)task.count, (unsigned long)task.max_gap_us, (unsigned long)task.last_us,
               task.allowed ? 1U : 0U, task.target, task.feedback, task.iq);
        PRINTF("speed task worst_gap_ticks=%lu tick_hz=%lu phase=%lu tick=%lu\n",
               (unsigned long)task.max_gap_ticks, (unsigned long)osKernelGetTickFreq(),
               (unsigned long)task.phase, (unsigned long)task.tick);
        PRINTF("speed task phase_max_us wait=%lu position=%lu speed=%lu publish=%lu wake_late_max_ticks=%lu\n",
               (unsigned long)task.phase_max_us[0], (unsigned long)task.phase_max_us[1],
               (unsigned long)task.phase_max_us[2], (unsigned long)task.phase_max_us[3],
               (unsigned long)task.wake_late_max_ticks);
        PRINTF("speed task phase_ticks_at_max wait=%lu position=%lu speed=%lu publish=%lu\n",
               (unsigned long)task.phase_max_ticks[0], (unsigned long)task.phase_max_ticks[1],
               (unsigned long)task.phase_max_ticks[2], (unsigned long)task.phase_max_ticks[3]);
    }
    {
        unsigned kind;
        for (kind = 0U; kind < 2U; kind++) {
            motor_irq_diag_t irq;
            Motor_Timing_IrqGet(kind, &irq);
            PRINTF("irq sampled body kind=%u n=%lu avg=%.2fus max=%lu span=%lu sum=%lu tick_pending=%lu switch_pending=%lu\n",
                   kind, (unsigned long)irq.count, irq.count ? (float)irq.sum_us / irq.count : 0.0f,
                   (unsigned long)irq.max_us, (unsigned long)(irq.last_us - irq.first_us), (unsigned long)irq.sum_us,
                   (unsigned long)irq.tick_pending, (unsigned long)irq.switch_pending);
            PRINTF("irq sampling kind=%u stride=%u seen=%lu; sum/max/pending are sampled, not exhaustive\n",
                   kind, (unsigned)MOTOR_TIMING_IRQ_STRIDE, (unsigned long)irq.seen);
        }
    }
    return 0;
}

/**
 * @description: sltrace 命令 — 武装或导出无感 RAM 环形诊断。
 * @detail: ol_enable 为真时直接拒绝。arm 使用失锁触发，离开 CLOSED 后再收 16 条；
 *          entry 在首次 CLOSED 当拍触发后再收 384 条，不停止电机。
 *          参数不是 arm 或 entry 时先停止写入，再按时间顺序打印。打印发生在命令里，不在控制中断里。
 * @param {int} argc       参数个数
 * @param {char *} argv[]  arm、entry，或无额外参数表示导出
 * @return {int} 0 成功，-1 电机仍在运行
 * @author: lkc
 */
int Cmd_SensorlessHandoff(int argc, char *argv[])
{
    unsigned index;
    motor_handoff_sample_t sample;
    char row[512];
    if (argc > 1 && (strcmp(argv[1], "pwm") == 0 || strcmp(argv[1], "pwmlong") == 0 ||
                     strcmp(argv[1], "phaseedge") == 0 || strcmp(argv[1], "phase3000") == 0)) {
        motor_command_t command;
        if (argc != 2 || Motor_GetCommand(&motor, &command) != MOTOR_OK || command.enable || motor.ol_enable ||
            command.target_rpm != 0.0f || Motor_Position_IsEnabled() || Motor_Joint_IsEnabled() ||
            Motor_Bldc_IsSelected() || Motor_Detect_IsActive() || Motor_Protect_IsFaultLatched()) {
            PRINTF("Invalid: slhandoff pwm requires stopped FOC, zero target, no position/joint/detection/fault\n");
            return -1;
        }
        if (strcmp(argv[1], "phase3000") == 0) {
            Motor_Timing_PhaseEdgeReserve3000();
            PRINTF("FOC phase reserved: target 3000, ramp within 1%% and observer within 5%% for 1s; not encoder speed\n");
            return 0;
        }
        if (strcmp(argv[1], "phaseedge") == 0) {
            Motor_Timing_PhaseEdgeArm();
            PRINTF("FOC callback phase armed: 64 events, freeze 16 callbacks after full-phase change or after 10s\n");
            return 0;
        }
        if (strcmp(argv[1], "pwmlong") == 0) Hal_Motor_PwmWindowReserveLong();
        else Hal_Motor_PwmWindowReserve();
        PRINTF("PWM window reserved: 32 rows stride=%lu after ramp within 5%% of target for 1s; not shaft speed\n",
               (unsigned long)Hal_Motor_PwmWindowStride());
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "phaseoff") == 0) {
        if (motor.ol_enable) {
            PRINTF("Invalid: phaseoff requires stopped motor\n");
            return -1;
        }
        Motor_Timing_PhaseEdgeStop();
        PRINTF("FOC callback phase count=%lu seen=%lu trigger=%lu reason=%lu\n",
               (unsigned long)g_motor_phase_debug.count, (unsigned long)g_motor_phase_debug.seen,
               (unsigned long)g_motor_phase_debug.trigger_sequence, (unsigned long)g_motor_phase_debug.reason);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "pwmoff") == 0) {
        if (motor.ol_enable) {
            PRINTF("Invalid: slhandoff pwmoff requires stopped motor\n");
            return -1;
        }
        Hal_Motor_PwmWindowEnable(false);
        PRINTF("PWM window state=%lu count=%lu/32 stride=%lu; read via JTAG only after stop\n",
               (unsigned long)g_hal_motor_pwm_window.enabled, (unsigned long)g_hal_motor_pwm_window.count,
               (unsigned long)Hal_Motor_PwmWindowStride());
        return 0;
    }
    if (motor.ol_enable) {
        PRINTF("Invalid: slhandoff requires stopped motor\n");
        return -1;
    }
    Hal_Motor_PwmDiagnosticEnable(false);
    if (argc > 1 && (strcmp(argv[1], "pair") == 0 || strcmp(argv[1], "half") == 0)) {
        bool half = strcmp(argv[1], "half") == 0;
        if (half) Motor_Timing_PairHalfArm();
        else Motor_Timing_PairArm();
        Hal_Motor_PwmDiagnosticEnable(true);
        PRINTF("ADC pair armed: stride=%u, software PWM sequence only; callback phase is not trigger phase\n",
               half ? 901U : 450U);
        if (half) PRINTF("Half mode is STOPPED ONLY; starting motor cancels capture\n");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "arm") == 0) {
        Motor_Timing_HandoffArm();
        PRINTF("Handoff armed: 32 rows, stride 15, trigger blend completion\n");
        return 0;
    }
    if (argc > 1 && (strcmp(argv[1], "phase") == 0 || strcmp(argv[1], "phaselong") == 0)) {
        bool extended = strcmp(argv[1], "phaselong") == 0;
        if (extended) Motor_Timing_PhaseArmLong();
        else Motor_Timing_PhaseArm();
        PRINTF("PLL phase capture armed: shared 32-row buffer, stride=%u, post=%u\n",
               extended ? 450U : 15U, extended ? 27U : 15U);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "step") == 0) {
        Motor_Timing_ObserverStepArm();
        PRINTF("Observer step armed: shared 32 rows, stride=450, trigger follows blend completion\n");
        return 0;
    }
    Motor_Timing_HandoffStop();
    if (Motor_Timing_PairMode()) {
        motor_pair_sample_t pair;
        PRINTF("%s samples=%u\n", Motor_Timing_PairHalfMode() ? "half" : "pair", Motor_Timing_HandoffCount());
        if (Motor_Timing_PairHalfMode()) {
            PRINTF("seq,time_us,flags,prev_sample,prev_pwm,prev_entry_us,prev_ia,prev_ib,prev_ic,prev_adc0,prev_adc1,prev_adc2,prev_ccr0,prev_ccr1,prev_ccr2,prev_count,prev_cr1,prev_capture_flags,prev_period,sample,pwm,entry_us,ia,ib,ic,adc0,adc1,adc2,ccr0,ccr1,ccr2,count,cr1,capture_flags,period,v_alpha,v_beta,crc32\n");
        } else {
            PRINTF("seq,time_us,flags,prev_sample,prev_pwm,prev_entry_us,prev_ia,prev_ib,prev_ic,prev_adc0,prev_adc1,prev_adc2,prev_ccr0,prev_ccr1,prev_ccr2,prev_count,prev_cr1,prev_recon,prev_period,sample,pwm,entry_us,ia,ib,ic,adc0,adc1,adc2,ccr0,ccr1,ccr2,count,cr1,recon,period,v_alpha,v_beta,crc32\n");
        }
        for (index = 0U; index < Motor_Timing_HandoffCount(); index++) {
            unsigned frame_index;
            int length;
            if (!Motor_Timing_PairGet(index, &pair)) break;
            length = snprintf(row, sizeof(row), "%u,%lu,%lu", index, (unsigned long)pair.time_us, (unsigned long)pair.flags);
            for (frame_index = 0U; frame_index < 2U; frame_index++) {
                const motor_pair_frame_t *frame = frame_index ? &pair.current : &pair.previous;
                int added;
                if (length < 0 || (unsigned)length >= sizeof(row)) return -1;
                added = snprintf(row + length, sizeof(row) - (unsigned)length,
                    ",%lu,%lu,%lu,%.9g,%.9g,%.9g,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
                    (unsigned long)frame->sample_sequence, (unsigned long)frame->pwm_sequence,
                    (unsigned long)frame->entry_us, frame->ia, frame->ib, frame->ic,
                    frame->adc[0], frame->adc[1], frame->adc[2], frame->ccr[0], frame->ccr[1], frame->ccr[2],
                    frame->callback_count, frame->callback_cr1, frame->reconstructed, frame->period);
                if (added < 0 || (unsigned)added >= sizeof(row) - (unsigned)length) return -1;
                length += added;
            }
            {
                int added = snprintf(row + length, sizeof(row) - (unsigned)length, ",%.9g,%.9g", pair.v_alpha, pair.v_beta);
                if (added < 0 || (unsigned)added >= sizeof(row) - (unsigned)length) return -1;
                length += added;
            }
            PRINTF("%s,%08lx\n", row, (unsigned long)crc32((uint8_t *)row, (uint32_t)length));
            Bsp_Usart_Flush();
        }
        return 0;
    }
    if (Motor_Timing_ObserverStepMode()) {
        motor_observer_step_sample_t step;
        PRINTF("step samples=%u\n", Motor_Timing_HandoffCount());
        PRINTF("seq,time_us,flags,x_before_alpha,x_before_beta,x_after_alpha,x_after_beta,i_before_alpha,i_before_beta,i_alpha,i_beta,v_alpha,v_beta,correction_alpha,correction_beta,dt,resistance,inductance,flux_nominal,gain,pll_before,observer,phase_error,blend,control_angle,previous_dt,phase_offset,crc32\n");
        for (index = 0U; index < Motor_Timing_HandoffCount(); index++) {
            int length;
            if (!Motor_Timing_ObserverStepGet(index, &step)) break;
            length = snprintf(row, sizeof(row), "%u,%lu,%lu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
                index, (unsigned long)step.time_us, (unsigned long)step.flags,
                step.x_before_alpha, step.x_before_beta, step.x_after_alpha, step.x_after_beta,
                step.i_before_alpha, step.i_before_beta, step.i_alpha, step.i_beta,
                step.v_alpha, step.v_beta, step.correction_alpha, step.correction_beta,
                step.dt, step.resistance, step.inductance, step.flux_nominal, step.gain,
                step.pll_before, step.observer, step.phase_error, step.blend, step.control_angle,
                step.previous_dt, step.phase_offset);
            if (length < 0 || (unsigned)length >= sizeof(row)) return -1;
            PRINTF("%s,%08lx\n", row, (unsigned long)crc32((uint8_t *)row, (uint32_t)length));
            Bsp_Usart_Flush();
        }
        return 0;
    }
    if (Motor_Timing_PhaseMode()) {
        motor_phase_sample_t phase;
        PRINTF("phase samples=%u\n", Motor_Timing_HandoffCount());
        PRINTF("seq,time_us,flags,observer,pll_before,pll_after,phase_error,speed_before,speed_after,rpm,flux,x_alpha,x_beta,v_alpha,v_beta,i_alpha,i_beta,dt,kp,ki,resistance,inductance,flux_nominal,phase_offset,blend,control_angle,crc32\n");
        for (index = 0U; index < Motor_Timing_HandoffCount(); index++) {
            int length;
            if (!Motor_Timing_PhaseGet(index, &phase)) break;
            length = snprintf(row, sizeof(row), "%u,%lu,%lu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
                index, (unsigned long)phase.time_us, (unsigned long)phase.flags,
                phase.observer, phase.pll_before, phase.pll_after, phase.phase_error,
                phase.speed_before, phase.speed_after, phase.rpm, phase.flux,
                phase.x_alpha, phase.x_beta, phase.v_alpha, phase.v_beta, phase.i_alpha, phase.i_beta,
                phase.dt, phase.kp, phase.ki, phase.resistance, phase.inductance,
                phase.flux_nominal, phase.phase_offset, phase.blend, phase.control_angle);
            if (length < 0 || (unsigned)length >= sizeof(row)) return -1;
            PRINTF("%s,%08lx\n", row, (unsigned long)crc32((uint8_t *)row, (uint32_t)length));
            Bsp_Usart_Flush();
        }
        return 0;
    }
    PRINTF("handoff samples=%u\n", Motor_Timing_HandoffCount());
    PRINTF("seq,time_us,flags,theta,delta,id,iq,old_id,old_iq,id_cmd,iq_cmd,vd_before,vq_before,vd_transport,vq_transport,vd_after,vq_after,vd,vq,ia,ib,ic,vbus,kp,dt,adc0,adc1,adc2,crc32\n");
    for (index = 0U; index < Motor_Timing_HandoffCount(); index++) {
        int length;
        if (!Motor_Timing_HandoffGet(index, &sample)) break;
        length = snprintf(row, sizeof(row), "%u,%lu,%lu,%.7f,%.7f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.3f,%.6f,%.7f,%u,%u,%u",
            index, (unsigned long)sample.time_us, (unsigned long)sample.flags, sample.theta, sample.delta,
            sample.id, sample.iq, sample.old_id, sample.old_iq, sample.id_cmd, sample.iq_cmd,
            sample.vd_before, sample.vq_before, sample.vd_transport, sample.vq_transport,
            sample.vd_after, sample.vq_after, sample.vd, sample.vq, sample.ia, sample.ib, sample.ic,
            sample.vbus, sample.kp, sample.dt, (unsigned)sample.adc[0], (unsigned)sample.adc[1], (unsigned)sample.adc[2]);
        if (length < 0 || (unsigned)length >= sizeof(row)) return -1;
        PRINTF("%s,%08lx\n", row, (unsigned long)crc32((uint8_t *)row, (uint32_t)length));
        Bsp_Usart_Flush();
    }
    return 0;
}

int Cmd_SensorlessTrace(int argc, char *argv[])
{
    unsigned i, count;
    motor_trace_sample_t t;
    if (motor.ol_enable) {
        PRINTF("sltrace requires stopped motor\n");
        return -1;
    }
    if (argc > 1 && (strcmp(argv[1], "tracking") == 0 || strcmp(argv[1], "tracking-clear") == 0 ||
                     strcmp(argv[1], "tracking-arm") == 0 || strcmp(argv[1], "tracking-off") == 0)) {
        motor_current_tracking_diag_t diag;
        bool clear = strcmp(argv[1], "tracking-clear") == 0;
        if (strcmp(argv[1], "tracking-arm") == 0 && !Motor_FocIsr_EnableTracking(true)) return -1;
        if (strcmp(argv[1], "tracking-off") == 0 && !Motor_FocIsr_EnableTracking(false)) return -1;
        if (!Motor_FocIsr_ReadTracking(&diag, clear)) return -1;
        PRINTF("Tracking observe-only: threshold=2A hold=2ms; no stop, no fault action; enabled=%u clear=%u\n",
               Motor_FocIsr_TrackingIsEnabled() ? 1U : 0U, clear ? 1U : 0U);
        PRINTF("Tracking samples=%lu events=%lu latched=%u max_error=%.3fA first_entry_us=%lu\n",
               (unsigned long)diag.eligible_samples, (unsigned long)diag.events, diag.event_latched ? 1U : 0U,
               sqrtf(diag.max_error_squared), (unsigned long)diag.first_entry_us);
        PRINTF("Tracking first id/iq=%.3f/%.3f target=%.3f/%.3f A; stopped read, retained across stop/fault\n",
               diag.first_id, diag.first_iq, diag.first_id_cmd, diag.first_iq_cmd);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "debug") == 0) {
        if (!Motor_Timing_TraceIsFault()) {
            PRINTF("Invalid: debug export requires fault RAM trace\n");
            return -1;
        }
        Motor_Timing_TraceStop();
        PRINTF("Fault RAM frozen: descriptor=0x%08lx bytes=%u samples=%lu; read via J-Link, no UART rows\n",
               (unsigned long)(uintptr_t)&g_motor_fault_debug, (unsigned)sizeof(g_motor_fault_debug),
               (unsigned long)g_motor_fault_debug.count);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "fault") == 0) {
        if (Motor_Protect_IsFaultLatched()) {
            PRINTF("Fault trace refused: preserve latched fault; no automatic clear\n");
            return -1;
        }
        Motor_Timing_FaultTraceArm();
        PRINTF("Fault RAM trace armed: stride=%u, capacity=%u; force fault entry; previous-control fields\n",
               (unsigned)MOTOR_FAULT_TRACE_STRIDE, (unsigned)MOTOR_TRACE_CAPACITY);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "arm") == 0) {
        Motor_Timing_TraceArm();
        PRINTF("RAM trace armed; 2ms samples, freeze after CLOSED exit\n");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "compact") == 0) {
        Motor_Timing_TraceArmCompact();
        PRINTF("Compact RAM trace armed: %u bytes, capacity=%u, stride=%u; performance counters paused\n",
               (unsigned)sizeof(motor_trace_compact_sample_t), (unsigned)MOTOR_TRACE_COMPACT_CAPACITY,
               (unsigned)MOTOR_TRACE_COMPACT_STRIDE);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "entry") == 0) {
        Motor_Timing_TraceArmEntry();
        PRINTF("RAM trace armed; freeze 384 samples after first CLOSED; motor keeps running\n");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "entryfast") == 0) {
        Motor_Timing_TraceArmEntryFast();
        PRINTF("RAM trace armed; 384 consecutive control samples after first CLOSED; motor keeps running\n");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "flight") == 0) {
        Motor_Timing_TraceArmFlight();
        PRINTF("Light RAM flight trace armed; entry192 + rolling320; freeze on CLOSED exit or test stop\n");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "lite") == 0) {
        Motor_Timing_TraceArmLight();
        PRINTF("Light RAM trace armed; 384 samples at 8 control steps after first CLOSED; motor keeps running\n");
        return 0;
    }
    Motor_Timing_TraceStop();
    count = Motor_Timing_TraceCount();
    PRINTF("trace samples=%u\n", count);
    if (Motor_Timing_TraceIsFault()) {
        motor_fault_trace_sample_t sample;
        PRINTF("trace format=fault stride=%u; ADC/abc/vbus/dt=current entry, other fields=previous control; no trigger-time latch\n",
               (unsigned)MOTOR_FAULT_TRACE_STRIDE);
        PRINTF("seq,time_us,flags,adc0,adc1,adc2,ia,ib,ic,prev_theta,prev_pll,prev_flux,prev_rpm,prev_v_alpha,prev_v_beta,prev_vd_int,prev_vq_int,prev_id,prev_iq,prev_id_cmd,prev_iq_cmd,vbus,dt,crc32\n");
        for (i = 0U; i < count; i++) {
            char row[512];
            int length;
            if (!Motor_Timing_FaultTraceGet(i, &sample)) return -1;
            length = snprintf(row, sizeof(row),
                "%u,%lu,%lu,%u,%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
                i, (unsigned long)sample.time_us, (unsigned long)sample.flags,
                (unsigned)sample.adc[0], (unsigned)sample.adc[1], (unsigned)sample.adc[2],
                sample.ia, sample.ib, sample.ic, sample.theta, sample.pll, sample.flux, sample.rpm,
                sample.v_alpha, sample.v_beta, sample.vd_int, sample.vq_int, sample.id, sample.iq,
                sample.id_cmd, sample.iq_cmd, sample.v_bus, sample.dt);
            if (length < 0 || (unsigned)length >= sizeof(row)) return -1;
            PRINTF("%s,%08lx\n", row, (unsigned long)crc32((const uint8_t *)row, (uint32_t)length));
            Bsp_Usart_Flush();
        }
        return 0;
    }
    if (Motor_Timing_TraceIsCompact()) {
        motor_trace_compact_sample_t compact;
        PRINTF("trace format=compact sample_bytes=%u stride=%u capacity=%u\n",
               (unsigned)sizeof(compact), (unsigned)MOTOR_TRACE_COMPACT_STRIDE,
               (unsigned)MOTOR_TRACE_COMPACT_CAPACITY);
        PRINTF("seq,time_us,flags,theta,rpm,id,iq,id_cmd,iq_cmd,crc32\n");
        for (i = 0U; i < count; i++) {
            char row[384];
            int length;
            if (!Motor_Timing_TraceGetCompact(i, &compact)) break;
            length = snprintf(row, sizeof(row), "%u,%lu,%lu,%.5f,%.1f,%.4f,%.4f,%.4f,%.4f",
                              i, (unsigned long)compact.time_us, (unsigned long)compact.flags,
                              compact.theta, compact.rpm, compact.id, compact.iq, compact.id_cmd, compact.iq_cmd);
            if (length < 0 || (unsigned)length >= sizeof(row)) {
                PRINTF("Compact export failed: row formatting\n");
                return -1;
            }
            PRINTF("%s,%08lx\n", row, (unsigned long)crc32((const uint8_t *)row, (uint32_t)length));
            Bsp_Usart_Flush();
        }
        return 0;
    }
    if (Motor_Timing_TraceIsLight()) {
        motor_trace_light_sample_t light;
        PRINTF("trace format=lite sample_bytes=%u post_stride=8\n", (unsigned)sizeof(light));
        if (Motor_Timing_TraceIsFlight()) PRINTF("trace flight entry_rows=%u rolling_rows=%u rolling_stride=128; may contain a time gap\n",
                Motor_Timing_TraceFlightEntryCount(), count - Motor_Timing_TraceFlightEntryCount());
        PRINTF("time_us,flags,theta,pll,flux,rpm,id,iq,id_cmd,iq_cmd,blend,loss,va,vb,gap_max_us,full_max_us,task_count,task_age_us,task_gate,speed_target,speed_feedback,speed_iq,task_tick,task_phase,task_phase_age_us\n");
        for (i = 0U; i < count; ++i) {
            if (!Motor_Timing_TraceGetLight(i, &light)) break;
            PRINTF("%lu,%lu,%.5f,%.5f,%.6f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.4f,%.4f,%u,%u,%lu,%lu,%lu,%.1f,%.1f,%.4f,%lu,%lu,%lu\n",
                   (unsigned long)light.time_us, (unsigned long)light.flags,
                   light.theta, light.pll, light.flux, light.rpm, light.id, light.iq,
                   light.id_cmd, light.iq_cmd, light.blend, light.loss, light.v_alpha, light.v_beta,
                   (unsigned)light.gap_max_us, (unsigned)light.full_max_us,
                   (unsigned long)light.task_count, (unsigned long)light.task_age_us, (unsigned long)light.task_gate,
                   light.speed_target, light.speed_feedback, light.speed_iq,
                   (unsigned long)light.task_tick, (unsigned long)light.task_phase, (unsigned long)light.task_phase_age_us);
            Bsp_Usart_Flush();
        }
        return 0;
    }
    PRINTF("time_us,flags,theta,pll,flux,rpm,id,iq,iq_PI,va,vb,enc,enc_rpm,age,blend,loss,id_cmd,iq_cmd,ia,ib,vd,vq,vbus,vlimit,gap_max_us,full_max_us,pending,up\n");
    for (i = 0U; i < count; ++i) {
        if (!Motor_Timing_TraceGet(i, &t)) break;
        PRINTF("%lu,%lu,%.5f,%.5f,%.6f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.1f,%.6f,%.4f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%u,%u,%u,%u\n",
               (unsigned long)t.time_us, (unsigned long)t.flags, t.theta, t.pll, t.flux, t.rpm,
               t.id, t.iq, t.iq_target, t.v_alpha, t.v_beta, t.enc_angle, t.enc_rpm, t.enc_age,
               t.blend, t.loss_time, t.id_cmd, t.iq_cmd, t.i_alpha, t.i_beta, t.vd, t.vq,
               t.v_bus, t.v_limit, (unsigned)t.gap_max_us, (unsigned)t.full_max_us,
               (unsigned)t.pending_count, (unsigned)t.up_count);
        Bsp_Usart_Flush();
    }
    return 0;
}

/**
 * @description: olsnap 命令 — 运行中短快照（Enable/转速/故障/IdIq），避免 olstatus 大页卡死 UART。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 无额外参数
 * @return {int} 0 成功
 */
int Cmd_OlSnap(int argc, char *argv[])
{
    motor_status_t status;
    mc_fault_code fault;
    float vd_out;
    float vq_out;
    float v_limit;
    float pwm_alpha;
    float pwm_beta;
    float pwm_period;
    uint32_t sample_up_count;
    uint32_t sample_down_count;
    char ts[20];

    (void)argc;
    (void)argv;

    if (Motor_GetStatus(&motor, &status) != MOTOR_OK) {
        PRINTF("Snap failed\n");
        return -1;
    }
    fault = status.fault;
    Cmd_FormatUptime(ts, sizeof(ts));
    PRINTF("\n===== Open-Loop Snap %s =====\n", ts);
    PRINTF("  Enable     : %s\n", status.enable ? "ON" : "OFF");
    PRINTF("  Target RPM : %.0f\n", status.target_rpm);
    PRINTF("  Ramp RPM   : %.0f\n", status.current_rpm);
    PRINTF("  Meas RPM   : %.1f (%s)\n", status.measured_rpm,
           status.feedback.speed_valid ? "valid" : "invalid");
    if (status.feedback.used_source == MOTOR_FEEDBACK_SRC_HALL) {
        PRINTF("  Hall RPM   : raw=%.1f filtered=%.1f fast=%.1f age=%.1fms state=%d\n",
               status.feedback.mech_rpm_raw, status.feedback.mech_rpm, status.feedback.mech_rpm_fast,
               status.feedback.sample_age_s * 1000.0f, status.feedback.sensor_state);
        PRINTF("  Hall table : %u %u %u %u %u %u %u %u (state 0..7, 255=invalid)\n",
               (unsigned int)(uint8_t)motor.hall_table[0], (unsigned int)(uint8_t)motor.hall_table[1],
               (unsigned int)(uint8_t)motor.hall_table[2], (unsigned int)(uint8_t)motor.hall_table[3],
               (unsigned int)(uint8_t)motor.hall_table[4], (unsigned int)(uint8_t)motor.hall_table[5],
               (unsigned int)(uint8_t)motor.hall_table[6], (unsigned int)(uint8_t)motor.hall_table[7]);
    }
    Motor_DmaInt_GetSampleDirectionCounts(&sample_up_count, &sample_down_count);
    PRINTF("  FOC sample : up=%lu down=%lu\n", (unsigned long)sample_up_count,
           (unsigned long)sample_down_count);
    PRINTF("  V_bus      : %.1f V\n", status.v_bus);
    PRINTF("  Theta/CCR  : %.4f rad / %lu %lu %lu\n", status.theta, (unsigned long)status.duty1,
           (unsigned long)status.duty2, (unsigned long)status.duty3);
    if (status.feedback.used_source == MOTOR_FEEDBACK_SRC_HALL) {
        float center = 0.0f;
        float ddeg;

        (void)Motor_Hall_DecodeAngle(motor.hall_table, status.feedback.sensor_state, &center);
        ddeg = (status.theta - center) * 180.0f / (float)M_PI;
        while (ddeg > 180.0f) {
            ddeg -= 360.0f;
        }
        while (ddeg < -180.0f) {
            ddeg += 360.0f;
        }
        PRINTF("  Hall ddeg  : %.1f deg (center %.3f rad)\n", (double)ddeg, (double)center);
    }
    PRINTF("  Fault      : %d %s (%s)\n", fault, Motor_Protect_GetFaultName(fault),
           status.fault_latched ? "LATCHED" : "CLEAR");
    PRINTF("  Ia,Ib,Ic   : %.3f %.3f %.3f A\n", status.ia, status.ib, status.ic);
    PRINTF("  Iabc sum   : %.3f A\n", status.ia + status.ib + status.ic);
    PRINTF("  Iadc raw   : %.0f %.0f %.0f (CURR1/2/3)\n", status.current_adc[0], status.current_adc[1],
           status.current_adc[2]);
    PRINTF("  Iadc zero  : %.1f %.1f %.1f (CURR1/2/3)\n", status.current_offset[0],
           status.current_offset[1], status.current_offset[2]);
    PRINTF("  Id, Iq     : %.3f %.3f A (target %.3f %.3f)\n", status.id, status.iq,
           status.id_target, status.iq_target);
    PRINTF("  FW         : max=%.2fA duty=%.2f\n", motor.fw_current_max, motor.fw_duty_start);
    PRINTF("  Speed Iq   : %.3f A\n", status.speed_iq);
    PRINTF("  Vd, Vq int : %.3f %.3f\n", status.vd_int, status.vq_int);
    {
        motor_current_pi_diag_t pi_diag;

        Motor_Current_TakePiDiag(&pi_diag);
        PRINTF("  PI trace   : n=%lu clamp=%lu ki=%.2f dt=%.1fus Ierr=%.4f |Ierr|=%.4f dVint=%.3f\n",
               (unsigned long)pi_diag.samples, (unsigned long)pi_diag.clamp_samples, pi_diag.ki, pi_diag.dt * 1.0e6f,
               pi_diag.iq_err_avg, pi_diag.iq_err_abs_avg, pi_diag.vq_int_delta);
    }
    /* 电流 PI 的限幅/抗饱和会回写积分项，故同一快照可还原本拍限幅后的 DQ 请求。 */
    vd_out = status.vd_int + (status.id_target - status.id) * motor.ol_current_kp;
    vq_out = status.vq_int + (status.iq_target - status.iq) * motor.ol_current_kp;
    v_limit = Motor_Control_GetVoltageLimit(&motor, status.v_bus);
    PRINTF("  Vd, Vq out : %.3f %.3f V (|V|=%.3f / %.3f V)\n", vd_out, vq_out,
           sqrtf(vd_out * vd_out + vq_out * vq_out), v_limit);
    /* 由逻辑 A/B/C 比较值估算理想平均相电压；不代表电机端实测电压。 */
    pwm_period = (float)Hal_Motor_PwmPeriodCounts();
    if (status.enable && pwm_period > 0.0f) {
        pwm_alpha = status.v_bus * (2.0f * (float)status.duty1 - (float)status.duty2 -
                                    (float)status.duty3) / (3.0f * pwm_period);
        pwm_beta = status.v_bus * ((float)status.duty2 - (float)status.duty3) /
                   (1.7320508f * pwm_period);
        PRINTF("  PWM ideal  : |Valphabeta|=%.3f V (CCR estimate)\n",
               sqrtf(pwm_alpha * pwm_alpha + pwm_beta * pwm_beta));
    }
    {
        int recon_last = 0;
        uint32_t recon_samples = 0U;
        uint32_t recon_hits = 0U;
        uint32_t recon_a = 0U;
        uint32_t recon_b = 0U;
        uint32_t recon_c = 0U;
        const char *recon_name;

        Motor_Current_GetReconstructDiag(&recon_last, &recon_samples, &recon_hits, &recon_a, &recon_b, &recon_c);
        if (recon_last == 1) {
            recon_name = "A";
        } else if (recon_last == 2) {
            recon_name = "B";
        } else if (recon_last == 3) {
            recon_name = "C";
        } else {
            recon_name = "none";
        }
        PRINTF("  Reconstruct : last=%s hits=%lu/%lu (A=%lu B=%lu C=%lu) thr=%lu\n", recon_name,
               (unsigned long)recon_hits, (unsigned long)recon_samples, (unsigned long)recon_a, (unsigned long)recon_b,
               (unsigned long)recon_c, (unsigned long)MOTOR_CURRENT_SHUNT_PICK_COUNTS);
    }
    PRINTF("=============================\n");
    /* TX DMA 可抢占 FOC，Flush 能把本页发完，避免 olsnap 半截卡死上位机解析。 */
    Bsp_Usart_Flush();
    return 0;
}

/**
 * @description: 静态 D 轴电流环测试。
 * @detail: 固定电角度并保持 Id，跳过 olstart 的零速替代逻辑；预热后统计电流，最后自动停机并恢复配置。
 *          默认 0.30 A，避开 BM75 的 0.10~0.20 A 低电流采样盲区。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表：[Id_A] [duration_ms]
 * @return {int} 0=完成测试，-1=前置条件或运行故障
 */
int Cmd_CurrentTest(int argc, char *argv[])
{
    const uint32_t warmup_ms = 1000U;
    const uint32_t sample_period_ms = 50U;
    motor_config_t saved_cfg;
    motor_config_t test_cfg;
    motor_command_t saved_cmd;
    motor_command_t test_cmd;
    motor_status_t status;
    motor_feedback_source_t saved_feedback;
    float current_a = 0.30f;
    uint32_t duration_ms = 2500U;
    uint32_t elapsed_ms;
    uint32_t samples = 0U;
    float id_sum = 0.0f;
    float iq_sum = 0.0f;
    float id_abs_error_sum = 0.0f;
    float iq_abs_sum = 0.0f;
    float id_target_sum = 0.0f;
    float iq_target_sum = 0.0f;
    float vd_int_sum = 0.0f;
    float vq_int_sum = 0.0f;
    float adc_delta_abs_sum[3] = {0.0f, 0.0f, 0.0f};
    mc_fault_code fault = FAULT_CODE_NONE;
    bool restored = false;

    if (argc > 3) {
        PRINTF("Usage: curtest [Id_A] [duration_ms]\n");
        return -1;
    }
    if (argc >= 2) {
        current_a = fabsf(atof(argv[1]));
    }
    if (argc >= 3) {
        duration_ms = (uint32_t)atoi(argv[2]);
    }
    if (current_a < 0.25f || current_a > 0.50f || duration_ms < 1800U || duration_ms > 5000U) {
        PRINTF("Invalid curtest: Id 0.25..0.50 A, duration 1800..5000 ms\n");
        PRINTF("Recommended: curtest 0.30 2500\n");
        return -1;
    }
    if (Motor_GetCommand(&motor, &saved_cmd) != MOTOR_OK || saved_cmd.enable) {
        PRINTF("Stop motor first: olstop\n");
        return -1;
    }
    if (Motor_Position_IsEnabled() || Motor_Joint_IsEnabled()) {
        PRINTF("Disable position/joint first: pos off, joint off\n");
        return -1;
    }
    if (Motor_GetStatus(&motor, &status) != MOTOR_OK || status.fault_latched) {
        PRINTF("Clear motor fault first: olfault clear\n");
        return -1;
    }
    if (!Cmd_Ol_CaptureConfig(&saved_cfg)) {
        PRINTF("Cannot capture motor config\n");
        return -1;
    }

    saved_feedback = Motor_GetFeedbackSource();
    test_cfg = saved_cfg;
    test_cfg.ol_align_enable = true;
    test_cfg.ol_align_current = current_a;
    test_cfg.ol_align_time = (float)duration_ms * 0.001f + 1.0f;
    test_cfg.ol_align_angle = 0.0f;
    test_cfg.hfi_start_enable = false;
    if (Motor_SetConfig(&motor, &test_cfg) != MOTOR_OK ||
        Motor_SetFeedbackSource(MOTOR_FEEDBACK_SRC_OPENLOOP) != MOTOR_OK) {
        PRINTF("Cannot prepare current test\n");
        goto cleanup;
    }

    test_cmd = saved_cmd;
    test_cmd.enable = true;
    test_cmd.mode = MOTOR_OL_MODE_IF;
    test_cmd.target_rpm = 0.0f;
    test_cmd.iq_set = 0.0f;
    test_cmd.id_set = 0.0f;
    /* 零速 speed_enable 仅用于满足运行门槛；整个测试在对齐阶段结束前自动停机。 */
    test_cmd.speed_enable = true;

    PRINTF("Current test: fixed theta=0, Id=%.2f A, Iq=0, %lu ms\n", current_a,
           (unsigned long)duration_ms);
    PRINTF("Keep shaft free; it may move once into electrical alignment. Auto-stop enabled.\n");
    Bsp_Usart_Flush();
    if (Motor_SetCommand(&motor, &test_cmd) != MOTOR_OK) {
        PRINTF("Cannot start current test\n");
        goto cleanup;
    }

    for (elapsed_ms = 0U; elapsed_ms < duration_ms; elapsed_ms += sample_period_ms) {
        osDelay(pdMS_TO_TICKS(sample_period_ms));
        if (Motor_GetStatus(&motor, &status) != MOTOR_OK) {
            continue;
        }
        if (status.fault_latched) {
            fault = status.fault;
            break;
        }
        if (elapsed_ms + sample_period_ms < warmup_ms || !status.aligning) {
            continue;
        }
        id_sum += status.id;
        iq_sum += status.iq;
        id_abs_error_sum += fabsf(current_a - status.id);
        iq_abs_sum += fabsf(status.iq);
        id_target_sum += status.id_target;
        iq_target_sum += status.iq_target;
        vd_int_sum += status.vd_int;
        vq_int_sum += status.vq_int;
        adc_delta_abs_sum[0] += fabsf(status.current_adc[0] - status.current_offset[0]);
        adc_delta_abs_sum[1] += fabsf(status.current_adc[1] - status.current_offset[1]);
        adc_delta_abs_sum[2] += fabsf(status.current_adc[2] - status.current_offset[2]);
        samples++;
    }

cleanup:
    (void)Motor_RequestStop(&motor);
    if (Motor_SetConfig(&motor, &saved_cfg) == MOTOR_OK &&
        Motor_SetFeedbackSource(saved_feedback) == MOTOR_OK &&
        Motor_SetCommand(&motor, &saved_cmd) == MOTOR_OK) {
        restored = true;
    }

    if (fault != FAULT_CODE_NONE) {
        PRINTF("Current test: ABORT fault=%d %s\n", fault, Motor_Protect_GetFaultName(fault));
        return -1;
    }
    if (samples == 0U) {
        PRINTF("Current test: INVALID, no aligned samples; motor stopped, config restored=%s\n",
               restored ? "yes" : "no");
        return -1;
    }

    PRINTF("Current test result: n=%lu Id_avg=%.3f A Iq_avg=%.3f A\n", (unsigned long)samples,
           id_sum / (float)samples, iq_sum / (float)samples);
    PRINTF("  mean |Id error|=%.3f A, mean |Iq|=%.3f A\n", id_abs_error_sum / (float)samples,
           iq_abs_sum / (float)samples);
    PRINTF("  applied targets Id=%.3f A Iq=%.3f A, integrators Vd=%.3f V Vq=%.3f V\n",
           id_target_sum / (float)samples, iq_target_sum / (float)samples,
           vd_int_sum / (float)samples, vq_int_sum / (float)samples);
    PRINTF("  mean |ADC-zero|=%.2f %.2f %.2f counts\n", adc_delta_abs_sum[0] / (float)samples,
           adc_delta_abs_sum[1] / (float)samples, adc_delta_abs_sum[2] / (float)samples);
    PRINTF("  fault=NONE, auto-stopped=yes, config restored=%s\n", restored ? "yes" : "no");
    PRINTF("Note: BM75 0.10..0.20 A is only about 1..2.5 ADC counts; do not validate PI there.\n");
    {
        float tolerance_a = fmaxf(0.10f, 0.30f * current_a);
        bool tracking = samples >= 10U && fabsf(id_sum / (float)samples - current_a) <= tolerance_a &&
                        id_abs_error_sum / (float)samples <= tolerance_a &&
                        iq_abs_sum / (float)samples <= tolerance_a;
        PRINTF("  tracking=%s tolerance=%.3f A\n", tracking ? "PASS" : "FAIL", tolerance_a);
        return restored && tracking ? 0 : -1;
    }
}

/**
 * @description: 打印霍尔扇区电流诊断。
 * @detail: 停机后读上一轮统计；运行中也可只读查询。不修改控制状态。
 * @param {int} argc       参数个数，未使用
 * @param {char *[]} argv  参数表，未使用
 * @return {int} 0=已打印
 * @author: lkc
 */
int Cmd_OlCurrentDiag(int argc, char *argv[])
{
    motor_hall_current_diag_sector_t sectors[8];
    uint32_t total = 0U;
    int state;

    (void)argc;
    (void)argv;
    Motor_Hall_DiagGetSectors(sectors);
    PRINTF("Hall current diag (1/16 control samples; |sum|>0.5A, |Id err|>1.0A)\n");
    PRINTF("state samples sum_over sum_max id_over id_max\n");
    for (state = 1; state <= 6; state++) {
        total += sectors[state].samples;
        PRINTF("%d %lu %lu %.3f %lu %.3f\n", state,
               (unsigned long)sectors[state].samples,
               (unsigned long)sectors[state].phase_sum_over,
               sectors[state].max_abs_phase_sum_a,
               (unsigned long)sectors[state].id_error_over,
               sectors[state].max_abs_id_error_a);
    }
    PRINTF("phase state first_samples first_id_over second_samples second_id_over\n");
    for (state = 1; state <= 6; state++) {
        PRINTF("phase %d %lu %lu %lu %lu\n", state,
               (unsigned long)sectors[state].first_half_samples,
               (unsigned long)sectors[state].first_half_id_over,
               (unsigned long)sectors[state].second_half_samples,
               (unsigned long)sectors[state].second_half_id_over);
    }
    PRINTF("bias state mean_sum_a positive_over negative_over\n");
    for (state = 1; state <= 6; state++) {
        float mean = sectors[state].samples > 0U
                         ? (float)sectors[state].signed_phase_sum_milliamps /
                               (1000.0f * (float)sectors[state].samples)
                         : 0.0f;
        PRINTF("bias %d %.3f %lu %lu\n", state, mean,
               (unsigned long)sectors[state].phase_sum_positive_over,
               (unsigned long)sectors[state].phase_sum_negative_over);
    }
    PRINTF("raw state curr1_mean curr2_mean curr3_mean\n");
    for (state = 1; state <= 6; state++) {
        float count = (float)sectors[state].samples;
        PRINTF("raw %d %.3f %.3f %.3f\n", state,
               count > 0.0f ? (float)sectors[state].current_adc_sum[0] / count : 0.0f,
               count > 0.0f ? (float)sectors[state].current_adc_sum[1] / count : 0.0f,
               count > 0.0f ? (float)sectors[state].current_adc_sum[2] / count : 0.0f);
    }
    PRINTF("total samples: %lu\n", (unsigned long)total);
    {
        motor_hall_current_worst_t worst;

        Motor_Hall_DiagGetWorst(&worst);
        PRINTF("worst beats=%lu abs=%.3f signed=%.3f ia=%.3f ib=%.3f ic=%.3f id=%.3f iq=%.3f\n",
               (unsigned long)worst.beats, worst.abs_sum_a, worst.signed_sum_a, worst.ia, worst.ib, worst.ic,
               worst.id, worst.iq);
        PRINTF("worst adc=%u %u %u zero=%.1f %.1f %.1f\n", worst.current_adc[0], worst.current_adc[1],
               worst.current_adc[2], motor.currentOffset[0], motor.currentOffset[1], motor.currentOffset[2]);
        PRINTF("worst ccr=%lu %lu %lu arr=%u cnt=%u hall=%u age_ms=%.1f\n", (unsigned long)worst.duty[0],
               (unsigned long)worst.duty[1], (unsigned long)worst.duty[2], worst.tim1_arr, worst.tim1_cnt,
               (unsigned)worst.hall_state, (double)worst.edge_age_ms);
    }
    {
        motor_hall_current_worst_t worstid;
        const char *recon_name;
        float delta_deg;

        Motor_Hall_DiagGetWorstId(&worstid);
        if (worstid.recon_phase == 1U) {
            recon_name = "A";
        } else if (worstid.recon_phase == 2U) {
            recon_name = "B";
        } else if (worstid.recon_phase == 3U) {
            recon_name = "C";
        } else {
            recon_name = "none";
        }
        delta_deg = (worstid.theta_rad - worstid.sector_center_rad) * 180.0f / (float)M_PI;
        while (delta_deg > 180.0f) {
            delta_deg -= 360.0f;
        }
        while (delta_deg < -180.0f) {
            delta_deg += 360.0f;
        }
        PRINTF("worstid beats=%lu id=%.3f target=%.3f err=%.3f iq=%.3f ia=%.3f ib=%.3f ic=%.3f sum=%.3f\n",
               (unsigned long)worstid.beats, worstid.id, worstid.id_target, worstid.id_error, worstid.iq, worstid.ia,
               worstid.ib, worstid.ic, worstid.signed_sum_a);
        PRINTF("worstid theta=%.3f center=%.3f ddeg=%.1f hall=%u half=%s recon=%s age_ms=%.1f\n", worstid.theta_rad,
               worstid.sector_center_rad, (double)delta_deg, (unsigned)worstid.hall_state,
               worstid.first_half ? "first" : "second", recon_name, (double)worstid.edge_age_ms);
        PRINTF("worstid ccr=%lu %lu %lu arr=%u cnt=%u adc=%u %u %u\n", (unsigned long)worstid.duty[0],
               (unsigned long)worstid.duty[1], (unsigned long)worstid.duty[2], worstid.tim1_arr, worstid.tim1_cnt,
               worstid.current_adc[0], worstid.current_adc[1], worstid.current_adc[2]);
    }
    {
        motor_hall_current_edge_diag_t edge;
        const char *recon_name;
        float edge_pct;
        float body_pct;

        Motor_Hall_DiagGetEdge(&edge);
        edge_pct = edge.edge_samples > 0U ? 100.0f * (float)edge.edge_id_over / (float)edge.edge_samples : 0.0f;
        body_pct = edge.body_samples > 0U ? 100.0f * (float)edge.body_id_over / (float)edge.body_samples : 0.0f;
        PRINTF("edge samples=%lu id_over=%lu (%.1f%%) body samples=%lu id_over=%lu (%.1f%%) jumps=%lu\n",
               (unsigned long)edge.edge_samples, (unsigned long)edge.edge_id_over, (double)edge_pct,
               (unsigned long)edge.body_samples, (unsigned long)edge.body_id_over, (double)body_pct,
               (unsigned long)edge.jump_samples);
        {
            uint32_t n = edge.edge_samples + edge.body_samples;
            double nf = (double)n;
            float mean_id = 0.0f;
            float mean_iq = 0.0f;
            float mean_iqt = 0.0f;
            float mean_abs_id = 0.0f;
            float mean_abs_iq = 0.0f;
            float rms_id = 0.0f;
            float rms_iq = 0.0f;
            float rms_i = 0.0f;
            float mean_ia = 0.0f;
            float mean_ib = 0.0f;
            float mean_ic = 0.0f;

            if (n > 0U) {
                mean_id = (float)((double)edge.id_mA_sum / (1000.0 * nf));
                mean_iq = (float)((double)edge.iq_mA_sum / (1000.0 * nf));
                mean_iqt = (float)((double)edge.iq_target_mA_sum / (1000.0 * nf));
                mean_abs_id = (float)((double)edge.id_abs_mA_sum / (1000.0 * nf));
                mean_abs_iq = (float)((double)edge.iq_abs_mA_sum / (1000.0 * nf));
                rms_id = (float)(sqrt((double)edge.id2_mA2_sum / nf) / 1000.0);
                rms_iq = (float)(sqrt((double)edge.iq2_mA2_sum / nf) / 1000.0);
                rms_i = (float)(sqrt(((double)edge.id2_mA2_sum + (double)edge.iq2_mA2_sum) / nf) / 1000.0);
                mean_ia = (float)((double)edge.ia_mA_sum / (1000.0 * nf));
                mean_ib = (float)((double)edge.ib_mA_sum / (1000.0 * nf));
                mean_ic = (float)((double)edge.ic_mA_sum / (1000.0 * nf));
            }
            PRINTF("mean samples=%lu Id=%.3f Iq=%.3f |Id|=%.3f |Iq|=%.3f "
                   "rmsId=%.3f rmsIq=%.3f rmsI=%.3f IqT=%.3f\n",
                   (unsigned long)n, mean_id, mean_iq, mean_abs_id, mean_abs_iq, rms_id, rms_iq, rms_i, mean_iqt);
            PRINTF("mean ia=%.3f ib=%.3f ic=%.3f\n", mean_ia, mean_ib, mean_ic);
        }
        if (edge.jump.recon_phase == 1U) {
            recon_name = "A";
        } else if (edge.jump.recon_phase == 2U) {
            recon_name = "B";
        } else if (edge.jump.recon_phase == 3U) {
            recon_name = "C";
        } else {
            recon_name = "none";
        }
        PRINTF("jump prev_abc=%.3f %.3f %.3f abc=%.3f %.3f %.3f id=%.3f hall=%u recon=%s age_ms=%.1f\n",
               edge.jump.prev_ia, edge.jump.prev_ib, edge.jump.prev_ic, edge.jump.ia, edge.jump.ib, edge.jump.ic,
               edge.jump.id, (unsigned)edge.jump.hall_state, recon_name, (double)edge.jump.edge_age_ms);
    }
    Bsp_Usart_Flush();
    return 0;
}
#endif /* MOTOR_CFG_CMD_DIAG */
#ifdef __cplusplus
}
#endif
