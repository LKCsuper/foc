/*
 * @Description: 无感观测命令：sl* 配置、接管、统计与 sltest 有界测试序列
 * @Author: lkc
 * @detail: 自 cmd_openloop.c 拆出，命令仍由 command.c 命令表注册。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include "cmd_openloop.h"
#include "cmd_speed.h"
#include "common_assert.h"
#include "motor_control.h"
#include "motor_foc_isr.h"
#include "motor_timing.h"
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
#include <stdarg.h>/* Private functions ---------------------------------------------------------*/
#if MOTOR_CFG_CMD
/**
 * @description: slstats 命令 — 打印到达目标转速后的观测统计。
 * @detail: 停机后仍保留上次窗口。分箱和均值只用于诊断，不改变接管判定。
 * @param {int} argc       参数个数
 * @param {char *} argv[]  无额外参数
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_SensorlessStats(int argc, char *argv[])
{
    motor_sensorless_stats_t st;
    (void)argc;
    (void)argv;
    Motor_Sensorless_GetStats(&st);
    PRINTF("slstats n=%lu time=%.3fs fail F=%lu S=%lu D=%lu P=%lu\n", (unsigned long)st.samples,
           st.elapsed, (unsigned long)st.flux_fail, (unsigned long)st.speed_fail,
           (unsigned long)st.direction_fail, (unsigned long)st.phase_fail);
    PRINTF("range flux=%.6f..%.6fWb phase=%.4f..%.4frad longest=%.6fs\n",
           st.flux_min, st.flux_max, st.phase_min, st.phase_max, st.longest_candidate);
    if (st.samples != 0U) {
        PRINTF("mean rotor flux alpha=%.6f beta=%.6f Wb\n", st.flux_alpha_sum / st.samples,
               st.flux_beta_sum / st.samples);
    }
    PRINTF("PWM reconstruction error max=%.6f V (excludes deadtime/device drop)\n",
           sqrtf(st.modulation_error_square_max));
    {
        unsigned index;
        for (index = 0; index < MOTOR_SENSORLESS_STATS_BINS; ++index) {
            const motor_sensorless_angle_bin_t *b = &st.bins[index];
            float scale;
            if (b->samples == 0U) continue;
            scale = 1.0f / b->samples;
            PRINTF("bin%u n=%lu P=%lu ph=%.5f ph2=%.5f flux=%.6f ia=%.5f ib=%.5f va=%.5f vb=%.5f fa=%.6f fb=%.6f\n",
                   index, (unsigned long)b->samples, (unsigned long)b->phase_fail,
                   b->phase_sum * scale, b->phase_square_sum * scale, b->flux_sum * scale,
                   b->i_alpha_sum * scale, b->i_beta_sum * scale, b->v_alpha_sum * scale, b->v_beta_sum * scale,
                   b->flux_alpha_sum * scale, b->flux_beta_sum * scale);
        }
    }
    return 0;
}

/**
 * @description: sltakeover 命令 — 授权或禁止无感观测器接管电角度。
 * @detail: 停机时复位观测器和速度环。运行中保留 PLL；下一控制拍撤销 CLOSED，
 *          超过开环安全上限时请求失速故障，不退回高速 I/F。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 0 或 1
 * @return {int} 0 成功，-1 参数错误或前置条件不满足
 * @author: lkc
 */
int Cmd_SensorlessTakeover(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;
    int enable;

    if (argc < 2) {
        PRINTF("Sensorless takeover = %s  (usage: sltakeover <0|1>)\n",
               motor.sensorless_takeover ? "ARMED" : "BLOCKED");
        return 0;
    }
    enable = atoi(argv[1]);
    if (enable != 0 && enable != 1) {
        PRINTF("Invalid value, use 0 or 1\n");
        return -1;
    }
    if (enable && !motor.sensorless_enable) {
        PRINTF("Enable observer first: slenable 1\n");
        return -1;
    }

    if (!Cmd_Ol_CaptureConfig(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.sensorless_takeover = (enable != 0);
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        Cmd_Ol_PrintConfigResult(result);
        return -1;
    }
    if (!motor.ol_enable) {
        Motor_Sensorless_Reset();
        Motor_SpeedLoop_Reset();
    }
    PRINTF("Sensorless takeover %s\n", motor.sensorless_takeover ? "armed" : "blocked");
    return 0;
}

/* sltest: bounded sensorless sequence, task context only. */
/* 测试保护边界：独立于电机模型；调整需重新验证适用硬件范围。 */
typedef struct {
    float bus_min_v, bus_max_v, pole_pairs;
    float rpm_min, rpm_max, current_abort_a;
    float startup_speed_max_rpm, closed_speed_max_rpm, overspeed_ratio;
    float command_tolerance_rpm, encoder_age_max_s;
    uint32_t hold_min_s, hold_max_s;
} sltest_bounds_t;
static const sltest_bounds_t sltest_bounds_default = {
    .bus_min_v = 15.0f, .bus_max_v = 17.0f, .pole_pairs = 7.0f,
    .rpm_min = 500.0f, .rpm_max = 2600.0f, .current_abort_a = 1.0f,
    .startup_speed_max_rpm = 900.0f, .closed_speed_max_rpm = 3000.0f, .overspeed_ratio = 1.25f,
    .command_tolerance_rpm = 1.0f, .encoder_age_max_s = 0.005f,
    .hold_min_s = 2U, .hold_max_s = 20U
};
static sltest_bounds_t sltest_bounds;

/* 2804/16V空载测试配置，单位见字段后缀。仅测试期间覆盖，退出恢复原配置。
 * R/L/磁链和零偏仍来自当前电机配置，不在这里指定。 */
typedef struct {
    float default_rpms[3];
    uint32_t hold_s, startup_ms, takeover_ms, settle_ms, poll_ms, output_ms;
    float startup_rpm, startup_iq_a, startup_tolerance_rpm, ramp_rpm_s;
    float speed_kp, speed_ki, low_iq_limit_a, high_iq_limit_a, high_iq_above_rpm;
    float openloop_max_rpm, phase_tolerance_rad, blend_s;
    float high_voltage_above_rpm, high_voltage_limit_v;
    float tracking_absolute_rpm, tracking_relative;
    unsigned tracking_bad_samples;
} sltest_profile_t;
static const sltest_profile_t sltest_profile_default = {
    .default_rpms = {500.0f, 600.0f, 700.0f}, .hold_s = 5U,
    .startup_ms = 8000U, .takeover_ms = 5000U, .settle_ms = 2000U,
    .poll_ms = 50U, .output_ms = 1000U,
    .startup_rpm = 600.0f, .startup_iq_a = 0.30f, .startup_tolerance_rpm = 120.0f,
    .ramp_rpm_s = 100.0f, .speed_kp = 0.0003f, .speed_ki = 0.0005f,
    .low_iq_limit_a = 0.15f, .high_iq_limit_a = 0.30f, .high_iq_above_rpm = 1800.0f,
    .openloop_max_rpm = 800.0f, .phase_tolerance_rad = 1.4f, .blend_s = 0.05f,
    .high_voltage_above_rpm = 1200.0f, .high_voltage_limit_v = 8.0f,
    .tracking_absolute_rpm = 60.0f, .tracking_relative = 0.05f, .tracking_bad_samples = 5U
};
static sltest_profile_t sltest_profile;


static void sltest_print_output(const motor_status_t *status, float rpm)
{
    float period = (float)Hal_Motor_PwmPeriodCounts();
    float alpha = period > 0.0f ? status->v_bus *
        (2.0f * status->duty1 - status->duty2 - status->duty3) / (3.0f * period) : 0.0f;
    float beta = period > 0.0f ? status->v_bus *
        ((float)status->duty2 - status->duty3) / (1.732050808f * period) : 0.0f;
    PRINTF("sltest output: target=%.0f rpm=%.1f iq=%.4f/%.4f speedIq=%.4f limit=%.3f "
           "id=%.4f PWMV=%.3f/%.1f bus=%.2f\n", status->target_rpm, rpm,
           status->iq, status->iq_target, status->speed_iq, motor.speed_iq_limit,
           status->id, sqrtf(alpha * alpha + beta * beta), motor.ol_vq_limit, status->v_bus);
}
/* phase: 0=拖转，1=等待接管，2=闭环稳定等待，3=统计并检查跟踪。 */
static bool sltest_monitor(uint32_t duration_ms, float target, unsigned phase, float *average)
{
    uint32_t start = Hal_Motor_TimeUs();
    unsigned n = 0U, tracking_bad = 0U;
    float sum = 0.0f;
    uint32_t last_output = start;
    do {
        motor_status_t status = {0};
        motor_sensorless_state_t state;
        motor_result_t read_result;
        unsigned reason;
        bool protect;
        if (Cmd_Speed_StopKey()) { PRINTF("sltest: stopped by input\n"); return false; }
        read_result = Motor_GetStatus(&motor, &status);
        protect = Motor_Protect_IsFaultLatched();
        /* 位图允许同时保留多个原因；API失败时不解释未取得的状态。 */
        reason = read_result != MOTOR_OK ? 1U :
                 (!status.enable ? 2U : 0U) | (status.fault_latched ? 4U : 0U) |
                 (!isfinite(status.iq) || !isfinite(status.id) ? 16U : 0U) |
                 (fabsf(status.iq) > sltest_bounds.current_abort_a || fabsf(status.id) > sltest_bounds.current_abort_a ? 32U : 0U);
        if (protect) reason |= 8U;
        if (reason != 0U) {
            uint32_t observed_us = Hal_Motor_TimeUs();
            float rpm = Motor_Sensorless_GetMechanicalRpm();
            state = Motor_Sensorless_GetState();
            /* 先关输出再打印；这是50ms任务监测快照，非ADC硬件故障瞬间。 */
            (void)Motor_RequestStop(&motor);
            PRINTF("sltest: fault/current/state check failed\n");
            PRINTF("abort time=%lu phase=%u reason=0x%02x api=%d enable=%u fault=%u protect=%u "
                   "id=%.4f iq=%.4f target=%.1f rpm=%.1f sl=%u\n",
                   (unsigned long)observed_us, phase, reason, (int)read_result,
                   status.enable ? 1U : 0U, status.fault_latched ? 1U : 0U, protect ? 1U : 0U,
                   status.id, status.iq, status.target_rpm, rpm, (unsigned)state);
            return false;
        }
        state = Motor_Sensorless_GetState();
        {
            float rpm = Motor_Sensorless_GetMechanicalRpm();
            float speed_limit = phase < 2U ? sltest_bounds.startup_speed_max_rpm :
                fminf(sltest_bounds.closed_speed_max_rpm,
                      fmaxf(sltest_bounds.startup_speed_max_rpm, sltest_bounds.overspeed_ratio * target));
            if (!isfinite(rpm) || fabsf(rpm) > speed_limit || fabsf(status.target_rpm - target) > sltest_bounds.command_tolerance_rpm) {
                PRINTF("sltest: speed/target check failed phase=%u rpm=%.1f limit=%.1f command=%.1f expected=%.1f\n",
                       phase, rpm, speed_limit, status.target_rpm, target); return false;
            }
            if (phase == 1U && state == MOTOR_SENSORLESS_CLOSED && Motor_SpeedLoop_IsFeedbackValid()) return true;
            if (phase >= 2U && (state != MOTOR_SENSORLESS_CLOSED || !Motor_SpeedLoop_IsFeedbackValid())) {
                PRINTF("sltest: lost sensorless takeover\n"); return false;
            }
            if (phase == 3U) {
                if ((uint32_t)(Hal_Motor_TimeUs() - last_output) >= sltest_profile.output_ms * 1000U) {
                    sltest_print_output(&status, rpm);
                    last_output = Hal_Motor_TimeUs();
                }
                tracking_bad = fabsf(rpm - target) > fmaxf(sltest_profile.tracking_absolute_rpm, sltest_profile.tracking_relative * target) ? tracking_bad + 1U : 0U;
                if (tracking_bad >= sltest_profile.tracking_bad_samples) {
                    sltest_print_output(&status, rpm);
                    PRINTF("sltest: speed not tracking (limit %.1f%% or %.0fRPM)\n",
                           100.0f * sltest_profile.tracking_relative, sltest_profile.tracking_absolute_rpm); return false;
                }
                sum += rpm; n++;
            }
        }
        osDelay(pdMS_TO_TICKS(sltest_profile.poll_ms));
    } while ((uint32_t)(Hal_Motor_TimeUs() - start) / 1000U < duration_ms);
    if (phase == 1U) { PRINTF("sltest: takeover timeout\n"); return false; }
    if (average != NULL) *average = n ? sum / (float)n : 0.0f;
    if (phase == 3U && n && fabsf(sum / (float)n - target) > fmaxf(sltest_profile.tracking_absolute_rpm, sltest_profile.tracking_relative * target)) {
        PRINTF("sltest: average speed not tracking\n"); return false;
    }
    return true;
}

/**
 * @description: 无感多速度空载测试，结束/异常/新输入均停机并恢复配置。
 * @detail: sltest [hold_s [rpm1 rpm2 ...]]；默认5s、500/600/700RPM。
 *          沿用当前模型、零偏；600RPM拖转接管后开始分档。PASS仅指固件状态与观测跟踪。
 */
int Cmd_SensorlessTest(int argc, char *argv[])
{
    float rpms[6] = {0};
    unsigned count = 3U, i;
    float max_target = sltest_profile_default.startup_rpm;
    uint32_t hold_s = sltest_profile_default.hold_s;
    motor_config_t saved, cfg;
    motor_command_t initial, cmd;
    motor_feedback_source_t feedback;
    bool ok = false, restored;
    char *end;
    unsigned first_arg = 1U;
    unsigned trace_mode = 3U;
    bool motor3508 = false;
    sltest_profile = sltest_profile_default;
    sltest_bounds = sltest_bounds_default;
    while ((unsigned)argc > first_arg && strncmp(argv[first_arg], "--", 2U) == 0) {
        const char *option = argv[first_arg];
        if (strcmp(option, "--3508") == 0) motor3508 = true;
        else if (strncmp(option, "--trace=", 8U) == 0) {
            const char *value = option + 8;
            if (strcmp(value, "off") == 0) trace_mode = 0U;
            else if (strcmp(value, "entry") == 0) trace_mode = 1U;
            else if (strcmp(value, "entryfast") == 0) trace_mode = 2U;
            else if (strcmp(value, "lite") == 0) trace_mode = 3U;
            else if (strcmp(value, "flight") == 0) trace_mode = 4U;
            else { PRINTF("Invalid sltest trace: off|entry|entryfast|lite\n"); return -1; }
        } else { PRINTF("Invalid sltest option\n"); return -1; }
        first_arg++;
    }
    if (motor3508) {
        /* 已验证的3508/24V低速诊断配置；显式选择，禁止借此进入高速。 */
        sltest_bounds.bus_min_v = 22.0f; sltest_bounds.bus_max_v = 26.0f;
        sltest_bounds.rpm_max = 600.0f; sltest_bounds.current_abort_a = 1.6f;
        sltest_profile.default_rpms[2] = 600.0f;
        sltest_profile.startup_iq_a = 1.0f;
        sltest_profile.speed_kp = 0.003f; sltest_profile.speed_ki = 0.01f;
        sltest_profile.low_iq_limit_a = 1.0f;
        sltest_profile.blend_s = 0.2f;
    }
    for (i = 0U; i < 3U; i++) rpms[i] = sltest_profile.default_rpms[i];
    if ((unsigned)argc > first_arg + 7U) { PRINTF("Usage: sltest [--3508] [--trace=off|entry|entryfast|lite|flight] [hold_s [rpm1 ... rpm6]]\n"); return -1; }
    if ((unsigned)argc > first_arg) {
        long value = strtol(argv[first_arg], &end, 10);
        if (*end || value < (long)sltest_bounds.hold_min_s || value > (long)sltest_bounds.hold_max_s) { PRINTF("sltest hold: %lu..%lu seconds\n", (unsigned long)sltest_bounds.hold_min_s, (unsigned long)sltest_bounds.hold_max_s); return -1; }
        hold_s = (uint32_t)value;
    }
    if ((unsigned)argc > first_arg + 1U) {
        count = (unsigned)argc - first_arg - 1U;
        for (i = 0U; i < count; i++) {
            rpms[i] = strtof(argv[i + first_arg + 1U], &end);
            if (end == argv[i + first_arg + 1U] || *end || !isfinite(rpms[i]) || rpms[i] < sltest_bounds.rpm_min || rpms[i] > sltest_bounds.rpm_max) {
                PRINTF("sltest RPM: %.0f..%.0f, positive only\n", sltest_bounds.rpm_min, sltest_bounds.rpm_max); return -1;
            }
        }
    }
    if (Motor_GetCommand(&motor, &initial) != MOTOR_OK || initial.enable || Motor_Bldc_IsSelected() ||
        Motor_Detect_IsActive() || Motor_Protect_IsFaultLatched()) {
        PRINTF("sltest requires stopped FOC, no detection, clear fault\n"); return -1;
    }
    if (!Cmd_Ol_CaptureConfig(&saved)) { PRINTF("sltest: config unavailable\n"); return -1; }
    for (i = 0U; i < count; i++) max_target = fmaxf(max_target, rpms[i]);
    if (saved.ol_max_rpm < max_target || saved.sensorless_transition_rpm > sltest_profile.startup_rpm ||
        saved.sensorless_fallback_rpm >= sltest_bounds.rpm_min || !isfinite(motor.v_bus) || motor.v_bus < sltest_bounds.bus_min_v || motor.v_bus > sltest_bounds.bus_max_v ||
        motor.ol_pole_pairs != sltest_bounds.pole_pairs || !isfinite(saved.sensorless_motor_r) || saved.sensorless_motor_r <= 0.0f ||
        !isfinite(saved.sensorless_motor_l) || saved.sensorless_motor_l <= 0.0f ||
        !isfinite(saved.sensorless_flux_linkage) || saved.sensorless_flux_linkage <= 0.0f) {
        PRINTF("sltest: requires %.0f pole-pairs, %.1f..%.1fV, transition<=%.0f, fallback<%.0f\n",
               sltest_bounds.pole_pairs, sltest_bounds.bus_min_v, sltest_bounds.bus_max_v,
               sltest_profile.startup_rpm, sltest_bounds.rpm_min); return -1;
    }
    feedback = Motor_GetFeedbackSource();
    cfg = saved;
    cfg.sensorless_enable = true; cfg.sensorless_takeover = false;
    cfg.ol_align_enable = false; cfg.hfi_start_enable = false; cfg.ol_ramp_rpm_s = sltest_profile.ramp_rpm_s;
    cfg.speed_kp = sltest_profile.speed_kp; cfg.speed_ki = sltest_profile.speed_ki;
    cfg.speed_iq_limit = sltest_profile.low_iq_limit_a;
    cfg.sensorless_openloop_max_rpm = sltest_profile.openloop_max_rpm;
    cfg.sensorless_phase_tolerance = sltest_profile.phase_tolerance_rad;
    cfg.sensorless_blend_time = sltest_profile.blend_s;
    /* 高速需覆盖反电势电压，仍保持800RPM开环上限及原电流限幅。
     * 16V母线下8V低于线性调制范围；测试结束恢复原电压上限。 */
    if (max_target > sltest_profile.high_voltage_above_rpm) cfg.ol_vq_limit = sltest_profile.high_voltage_limit_v;
    (void)Motor_Position_Enable(false); (void)Motor_Joint_Enable(false);
    PRINTF("sltest: %.0fRPM startup -> %u tiers, %lus each; new input stops\n",
           sltest_profile.startup_rpm, count, (unsigned long)hold_s);
    PRINTF("model R=%.3f ohm L=%.3f mH flux=%.3f mWb; existing offsets, RAM only\n",
           cfg.sensorless_motor_r, cfg.sensorless_motor_l * 1000.0f, cfg.sensorless_flux_linkage * 1000.0f);
    PRINTF("sltest voltage limit=%.1f V; high speeds require CLOSED\n", cfg.ol_vq_limit);
    PRINTF("sltest limits: Iq=%.2f/%.2fA above %.0fRPM; tracking %.1f%% or %.0fRPM, %u samples @ %lums\n",
           sltest_profile.low_iq_limit_a, sltest_profile.high_iq_limit_a, sltest_profile.high_iq_above_rpm,
           sltest_profile.tracking_relative * 100.0f, sltest_profile.tracking_absolute_rpm,
           sltest_profile.tracking_bad_samples, (unsigned long)sltest_profile.poll_ms);
    if (Motor_SetConfig(&motor, &cfg) != MOTOR_OK || Motor_SetFeedbackSource(MOTOR_FEEDBACK_SRC_SENSORLESS) != MOTOR_OK)
        goto done;
    cmd = initial; cmd.enable = true; cmd.mode = MOTOR_OL_MODE_IF;
    cmd.target_rpm = sltest_profile.startup_rpm; cmd.iq_set = sltest_profile.startup_iq_a; cmd.id_set = 0.0f; cmd.speed_enable = false;
    if (trace_mode == 0U) {
        Motor_Timing_TraceArm();
        Motor_Timing_TraceStop();
    } else if (trace_mode == 2U) Motor_Timing_TraceArmEntryFast();
    else if (trace_mode == 3U) Motor_Timing_TraceArmLight();
    else if (trace_mode == 4U) Motor_Timing_TraceArmFlight();
    else Motor_Timing_TraceArmEntry();
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK || !sltest_monitor(sltest_profile.startup_ms, sltest_profile.startup_rpm, 0U, NULL)) goto done;
    if (fabsf(Motor_Sensorless_GetMechanicalRpm() - sltest_profile.startup_rpm) > sltest_profile.startup_tolerance_rpm) {
        PRINTF("sltest: startup not tracking\n"); goto done;
    }
    if (motor3508) {
        cmd.iq_set = 0.5f;
        if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK ||
            !sltest_monitor(sltest_profile.settle_ms, sltest_profile.startup_rpm, 0U, NULL)) goto done;
    }
    cmd.speed_enable = true;
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) goto done;
    cfg.speed_enable = true;
    cfg.sensorless_takeover = true;
    if (Motor_SetConfig(&motor, &cfg) != MOTOR_OK || !sltest_monitor(sltest_profile.takeover_ms, sltest_profile.startup_rpm, 1U, NULL) ||
        !sltest_monitor(sltest_profile.settle_ms, sltest_profile.startup_rpm, 2U, NULL)) goto done;
    for (i = 0U; i < count; i++) {
        float average;
        cmd.target_rpm = rpms[i];
        /* 仅高速档放宽转矩限流，600RPM接管继续使用原0.15A。 */
        cfg.speed_iq_limit = rpms[i] > sltest_profile.high_iq_above_rpm ?
            sltest_profile.high_iq_limit_a : sltest_profile.low_iq_limit_a;
        if (Motor_SetConfig(&motor, &cfg) != MOTOR_OK) goto done;
        PRINTF("sltest %u/%u target=%.0f RPM\n", i + 1U, count, rpms[i]);
        if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK || !sltest_monitor(sltest_profile.settle_ms, rpms[i], 2U, NULL) ||
            !sltest_monitor(hold_s * 1000U, rpms[i], 3U, &average)) goto done;
        PRINTF("  CLOSED avg=%.1f RPM", average);
        if (Motor_Encoder_IsCompareEnabled() && Motor_Encoder_IsSpeedValid() && Motor_Encoder_GetSampleAge() < sltest_bounds.encoder_age_max_s)
            PRINTF(" encoder=%.1f RPM(reference)", Motor_Encoder_GetMechRpm());
        PRINTF("\n");
    }
    ok = true;
done:
    /* 在关桥改变观测状态之前冻结故障前窗口；不发送串口、不等待。 */
    Motor_Timing_TraceStop();
    (void)Motor_RequestStop(&motor);
    Motor_Timing_CadenceStop();
    restored = Motor_SetConfig(&motor, &saved) == MOTOR_OK;
    restored = (Motor_SetFeedbackSource(feedback) == MOTOR_OK) && restored;
    initial.enable = false; initial.target_rpm = 0.0f; initial.speed_enable = false;
    restored = (Motor_SetCommand(&motor, &initial) == MOTOR_OK) && restored;
    PRINTF("sltest: %s; motor stopped; config %s; observer tracking only\n",
           ok && restored ? "PASS" : "FAIL/ABORT", restored ? "restored" : "restore failed");
    return ok && restored ? 0 : -1;
}
/* end sltest */

/**
 * @description: slenable 命令 — 启停无感磁链观测器。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 0 或 1
 * @return {int} 0 成功，-1 参数无效
 * @author: lkc
 */
int Cmd_SensorlessEnable(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;
    int enable;

    if (argc < 2) {
        PRINTF("Sensorless observer = %s  (usage: slenable <0|1>)\n", motor.sensorless_enable ? "ON" : "OFF");
        return 0;
    }
    enable = atoi(argv[1]);
    if (enable != 0 && enable != 1) {
        PRINTF("Invalid value, use 0 or 1\n");
        return -1;
    }

    if (!Cmd_Ol_CaptureConfig(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.sensorless_enable = (enable != 0);
    if (!cfg.sensorless_enable) {
        cfg.sensorless_takeover = false;
        cfg.speed_enable = false;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        Cmd_Ol_PrintConfigResult(result);
        return -1;
    }

    Motor_Sensorless_Reset();
    Motor_SpeedLoop_Reset();
    PRINTF("Sensorless observer %s\n", motor.sensorless_enable ? "enabled" : "disabled");
    return 0;
}

/**
 * @description: slrpm 命令 — 设置无感接管与失锁回退转速阈值。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表：<transition_rpm> <fallback_rpm>
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_SensorlessRpm(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 3) {
        PRINTF("Sensorless transition/fallback = %.0f/%.0f RPM\n", motor.sensorless_transition_rpm,
               motor.sensorless_fallback_rpm);
        PRINTF("Usage: slrpm <transition_rpm> <fallback_rpm>\n");
        return 0;
    }

    if (!Cmd_Ol_CaptureConfig(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.sensorless_transition_rpm = fabsf(atof(argv[1]));
    cfg.sensorless_fallback_rpm = fabsf(atof(argv[2]));
    if (cfg.sensorless_fallback_rpm > cfg.sensorless_transition_rpm) {
        cfg.sensorless_fallback_rpm = cfg.sensorless_transition_rpm;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        Cmd_Ol_PrintConfigResult(result);
        return -1;
    }
    Motor_Sensorless_Reset();
    PRINTF("Sensorless transition/fallback set to %.0f/%.0f RPM\n", motor.sensorless_transition_rpm,
           motor.sensorless_fallback_rpm);
    return 0;
}

/**
 * @description: slguard 命令 — 设置 I/F 到无感闭环过渡的安全约束。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表：<openloop_max_rpm> <phase_tol_rad> <blend_s>
 * @return {int} 0 成功，-1 运行中修改
 * @author: lkc
 */
int Cmd_SensorlessGuard(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;
    float open_loop_max_rpm;
    float phase_tolerance;
    float blend_time;

    if (argc < 4) {
        PRINTF("Sensorless guard: OLmax=%.0fRPM phase=%.2frad blend=%.2fs\n", motor.sensorless_openloop_max_rpm,
               motor.sensorless_phase_tolerance, motor.sensorless_blend_time);
        PRINTF("Usage: slguard <openloop_max_rpm> <phase_tolerance_rad> <blend_s>\n");
        return 0;
    }

    if (!Cmd_Ol_CaptureConfig(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }

    open_loop_max_rpm = fabsf((float)atof(argv[1]));
    phase_tolerance = fabsf((float)atof(argv[2]));
    blend_time = fabsf((float)atof(argv[3]));

    /* 非零 I/F 上限必须覆盖接管门槛，否则永远到不了允许接管的转速。 */
    if (open_loop_max_rpm > 0.0f && open_loop_max_rpm < cfg.sensorless_transition_rpm) {
        open_loop_max_rpm = cfg.sensorless_transition_rpm;
    }
    if (cfg.ol_max_rpm > 0.0f && open_loop_max_rpm > cfg.ol_max_rpm) {
        open_loop_max_rpm = cfg.ol_max_rpm;
    }

    /* 此门限作用于磁链和命令电流矢量夹角，不是PLL与I/F的实际切角。 */
    if (phase_tolerance < 0.10f) {
        phase_tolerance = 0.10f;
    }
    if (phase_tolerance > 1.50f) {
        phase_tolerance = 1.50f;
    }

    /* Shell 调试不允许瞬间切角，也不让过渡时间长到明显拖慢控制。 */
    if (blend_time < 0.05f) {
        blend_time = 0.05f;
    }
    if (blend_time > 1.00f) {
        blend_time = 1.00f;
    }

    cfg.sensorless_openloop_max_rpm = open_loop_max_rpm;
    cfg.sensorless_phase_tolerance = phase_tolerance;
    cfg.sensorless_blend_time = blend_time;
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        Cmd_Ol_PrintConfigResult(result);
        return -1;
    }
    Motor_Sensorless_Reset();
    Motor_SpeedLoop_Reset();

    PRINTF("Sensorless guard set: OLmax=%.0fRPM phase=%.2frad blend=%.2fs\n", motor.sensorless_openloop_max_rpm,
           motor.sensorless_phase_tolerance, motor.sensorless_blend_time);
    if (motor.sensorless_openloop_max_rpm == 0.0f) {
        PRINTF("WARNING: I/F high-speed guard is disabled\n");
    }
    return 0;
}

/**
 * @description: slmotor 命令 — 设置无感电机模型 R/L/磁链参数。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表：<R_ohm> <L_mH> <flux_mWb>
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_SensorlessMotor(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 4) {
        PRINTF("Sensorless motor: R=%.4f ohm L=%.4f mH Flux=%.4f mWb\n", motor.sensorless_motor_r,
               motor.sensorless_motor_l * 1000.0f, motor.sensorless_flux_linkage * 1000.0f);
        PRINTF("Usage: slmotor <R_ohm> <L_mH> <flux_mWb>\n");
        return 0;
    }

    if (!Cmd_Ol_CaptureConfig(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.sensorless_motor_r = fabsf(atof(argv[1]));
    cfg.sensorless_motor_l = fabsf(atof(argv[2])) * 0.001f;
    cfg.sensorless_flux_linkage = fabsf(atof(argv[3])) * 0.001f;
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        Cmd_Ol_PrintConfigResult(result);
        return -1;
    }
    Motor_Sensorless_Reset();
    PRINTF("Sensorless motor set: R=%.4f ohm L=%.4f mH Flux=%.4f mWb\n", motor.sensorless_motor_r,
           motor.sensorless_motor_l * 1000.0f, motor.sensorless_flux_linkage * 1000.0f);
    return 0;
}

/**
 * @description: sldt 命令 — 停机设置仅RAM的观测电压死区补偿比例。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表：<effective_seconds_times_PWM_Hz>
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_SensorlessDeadtime(int argc, char *argv[])
{
    char *end;
    float fraction;
    if (argc == 1) {
        PRINTF("Observer deadtime fraction=%.6f (RAM only; effective seconds * PWM Hz)\n",
               Motor_OutputGetObserverDeadtime());
        return 0;
    }
    if (argc != 2 || motor.ol_enable) {
        PRINTF("Invalid: sldt <fraction 0..0.02>, stopped only\n");
        return -1;
    }
    fraction = strtof(argv[1], &end);
    if (end == argv[1] || *end != '\0' || !Motor_OutputSetObserverDeadtime(fraction)) {
        PRINTF("Invalid observer deadtime fraction\n");
        return -1;
    }
    PRINTF("Observer deadtime fraction=%.6f (RAM only; PWM unchanged)\n", fraction);
    return 0;
}

/** slobs：设置观测器和PLL增益，沿用原配置更新流程。 */
int Cmd_SensorlessObserver(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 4) {
        PRINTF("Observer gain=%.1f PLL Kp=%.1f Ki=%.1f\n", motor.sensorless_observer_gain, motor.sensorless_pll_kp,
               motor.sensorless_pll_ki);
        PRINTF("Usage: slobs <observer_gain> <pll_kp> <pll_ki>\n");
        return 0;
    }

    if (!Cmd_Ol_CaptureConfig(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.sensorless_observer_gain = fabsf(atof(argv[1]));
    cfg.sensorless_pll_kp = fabsf(atof(argv[2]));
    cfg.sensorless_pll_ki = fabsf(atof(argv[3]));
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        Cmd_Ol_PrintConfigResult(result);
        return -1;
    }
    Motor_Sensorless_Reset();
    PRINTF("Observer gain=%.1f PLL Kp=%.1f Ki=%.1f\n", motor.sensorless_observer_gain, motor.sensorless_pll_kp,
           motor.sensorless_pll_ki);
    return 0;
}

/**
 * @description: slphase 命令 — 设置无感观测器固定电角度补偿。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为相位偏移 (rad)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_SensorlessPhase(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Sensorless phase offset = %.4f rad  (usage: slphase <rad>)\n", motor.sensorless_phase_offset);
        return 0;
    }
    if (!Cmd_Ol_CaptureConfig(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.sensorless_phase_offset = atof(argv[1]);
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        Cmd_Ol_PrintConfigResult(result);
        return -1;
    }
    Motor_Sensorless_Reset();
    PRINTF("Sensorless phase offset set to %.4f rad\n", motor.sensorless_phase_offset);
    return 0;
}
#endif /* MOTOR_CFG_CMD */
#ifdef __cplusplus
}
#endif
