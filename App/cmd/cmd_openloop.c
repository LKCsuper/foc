/*
 * @Description: 开环 V/F 控制命令 — 运行时调整转速和 V/F 参数
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-08-30 10:31:14
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-24 23:29:19
 * @detail:
 *   olspeed <rpm>     设置目标转速，负数为反转
 *   olspeed            查看当前开环状态
 *   olboost <V>        设置 V/F 启动补偿电压
 *   olkvf <V/Hz>       设置 V/F 斜率
 *   olramp <RPM/s>     设置软启动斜坡速率
 *   olvlimit <V>       设置 Vq 上限
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
/* Private typedef -----------------------------------------------------------*/
typedef enum {
    OPEN_LOOP_NONE,
    OPEN_LOOP_CREATE,
    OPEN_LOOP_DESTORY,
    OPEN_LOOP_SET,
    OPEN_LOOP_START,
    OPEN_LOOP_STOP,
    OPEN_LOOP_NUM,
} OPEN_LOOP_TYPE;
/* Private macro -------------------------------------------------------------*/
#define OL_STATUS_BUFFER_SIZE (3072U)
/* Private variables ---------------------------------------------------------*/
STATIC TaskHandle_t taskPlotHandle;
/* olstatus 先在 RAM 中完成格式化，再一次性交给 UART DMA，避免逐行 printf。 */
STATIC char sgcOlStatusBuffer[OL_STATUS_BUFFER_SIZE];
/* 防止用户开启后台 openloop 状态任务后，与手动 olstatus 同时改写静态缓冲区。 */
STATIC volatile bool sgbOlStatusBusy = false;

STATIC const osThreadAttr_t taskDutyAttr = {
    .name = "Open Loop",
    .priority = PRI_INIT,
};
/* Private function prototypes -----------------------------------------------*/
static bool ol_status_try_lock(void);
static void ol_status_unlock(void);
static bool ol_capture_config(motor_config_t *cfg);
static void ol_print_config_result(motor_result_t result);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 捕获当前配置快照供 Shell 修改。
 * @author: lkc
 */
static bool ol_capture_config(motor_config_t *cfg)
{
    motor_settings_t settings;

    if (cfg == NULL || !Motor_Settings_Capture(&motor, &settings)) return false;
    *cfg = settings.config;
    return true;
}

/**
 * @description: 打印配置提交失败原因。
 * @author: lkc
 */
static void ol_print_config_result(motor_result_t result)
{
    if (result == MOTOR_ERR_BUSY) {
        PRINTF("Stop motor before changing this config: olstop\n");
    } else if (result != MOTOR_OK) {
        PRINTF("Invalid config\n");
    }
}

/**
 * @description: 尝试获取 olstatus 静态缓冲区互斥锁。
 * @return {bool} true 表示加锁成功，false 表示已有输出在进行
 * @author: lkc
 */
static bool ol_status_try_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    bool locked;

    __disable_irq();
    locked = !sgbOlStatusBusy;
    if (locked) {
        sgbOlStatusBusy = true;
    }
    if (0U == primask) {
        __enable_irq();
    }

    return locked;
}

/**
 * @description: 释放 olstatus 静态缓冲区互斥锁。
 * @return {void} 无
 * @author: lkc
 */
static void ol_status_unlock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    sgbOlStatusBusy = false;
    if (0U == primask) {
        __enable_irq();
    }
}

/**
 * @description: 把状态页按块送入 DMA，并在换行前补 CR。
 * @detail: 不逐字节关中断。块与块之间让出 1 个 tick，避免 1800 RPM 时电流环被挤偏。
 * @param {const char *} data  已格式化的文本
 * @param {size_t} len  字节数
 * @return {void} 无
 */
static void ol_status_send(const char *data, size_t len)
{
    size_t index = 0U;

    while (index < len) {
        char chunk[160];
        size_t filled = 0U;

        while (index < len) {
            size_t need = (data[index] == '\n') ? 2U : 1U;

            if ((filled + need) > sizeof(chunk)) {
                break;
            }
            if (data[index] == '\n') {
                chunk[filled++] = '\r';
            }
            chunk[filled++] = data[index++];
        }
        (void)Bsp_Usart_Write((const UCHAR *)chunk, (ULONG)filled);
        if (osKernelGetState() == osKernelRunning) {
            (void)osDelay(1U);
        }
    }
}

/**
 * @description: 向状态页尾部追加一段格式化文字。
 * @param {size_t *} used 已用字节数，入参出参
 * @param {const char *} format printf 风格格式串
 * @return {void} 无
 * @author: lkc
 */

static void ol_status_append(size_t *used, const char *format, ...)
{
    size_t remaining;
    int written;
    va_list arguments;

    if (*used >= (OL_STATUS_BUFFER_SIZE - 1U)) return;

    remaining = OL_STATUS_BUFFER_SIZE - *used;
    va_start(arguments, format);
    written = vsnprintf(&sgcOlStatusBuffer[*used], remaining, format, arguments);
    va_end(arguments);

    if (written < 0) return;

    if ((size_t)written >= remaining) {
        /* 保留字符串结尾，避免异常参数导致后续发送越过缓冲区。 */
        *used = OL_STATUS_BUFFER_SIZE - 1U;
        sgcOlStatusBuffer[*used] = '\0';
    } else {
        *used += (size_t)written;
    }
}

/**
 * @description: 将无感状态枚举转换为可读字符串。
 * @param {motor_sensorless_state_t} state 无感状态
 * @return {const char *} 状态名称字符串
 * @author: lkc
 */
static const char *ol_sensorless_state_name(motor_sensorless_state_t state)
{
    switch (state) {
        case MOTOR_SENSORLESS_LOCKING:
            return "LOCKING";
        case MOTOR_SENSORLESS_CLOSED:
            return "CLOSED";
        case MOTOR_SENSORLESS_OPEN_LOOP:
        default:
            return "OPEN_LOOP";
    }
}

/**
 * @description: 当 I/F 目标转速超过安全上限时打印提示信息。
 * @param {float} target_rpm 用户请求的目标机械转速 (RPM)
 * @return {void} 无
 * @author: lkc
 */
static void ol_print_high_speed_guard_notice(float target_rpm)
{
    bool high_speed_request = motor.ol_mode == MOTOR_OL_MODE_IF && motor.sensorless_openloop_max_rpm > 0.0f &&
                              fabsf(target_rpm) > motor.sensorless_openloop_max_rpm &&
                              !Motor_Feedback_IsSensoredSource(Motor_Feedback_GetSourceDefault());

    if (!high_speed_request) return;

    PRINTF("I/F safety hold: %.0f RPM until sensorless takeover is 100%% complete\n",
           motor.sensorless_openloop_max_rpm);

    if (!motor.sensorless_enable || !motor.sensorless_takeover || !motor.speed_enable) {
        PRINTF("High-speed closed loop is not fully armed; run olstop first, then:\n");
        if (!motor.sensorless_enable) {
            PRINTF("  slenable 1\n");
        }
        if (!motor.sensorless_takeover) {
            PRINTF("  sltakeover 1\n");
        }
        if (!motor.speed_enable) {
            PRINTF("  spenable 1\n");
        }
    }
}

/**
 * @description: 格式化并输出开环、电流环、无感与保护状态到串口。
 * @return {void} 无
 * @author: lkc
 */
static void ol_print_status(void)
{
    const char *sensorless_state;
    motor_sensorless_diagnostics_t sensorless_diagnostics;
    motor_status_t status;
    size_t used = 0U;
    mc_fault_code fault;
    float adc_frequency_hz;
    float foc_frequency_hz;
    float foc_average_us;
    float foc_maximum_us;
    uint32_t format_start;
    uint32_t format_elapsed_us;

    if (!ol_status_try_lock()) {
        PRINTF("Status output is already running\n");
        return;
    }
    format_start = Hal_Motor_TimeUs();

    if (Motor_GetStatus(&motor, &status) != MOTOR_OK) {
        ol_status_unlock();
        PRINTF("Status snapshot failed\n");
        return;
    }

    sensorless_state = ol_sensorless_state_name(Motor_Sensorless_GetState());

    fault = status.fault;
    Motor_Sensorless_GetDiagnostics(&sensorless_diagnostics);
    adc_frequency_hz = Motor_DmaInt_GetAdcFrequencyHz();
    foc_frequency_hz = Motor_DmaInt_GetControlFrequencyHz();
    foc_average_us = Motor_DmaInt_GetAverageRuntimeUs();
    foc_maximum_us = Motor_DmaInt_GetMaximumRuntimeUs();

    {
        char ts[20];

        Cmd_FormatUptime(ts, sizeof(ts));
        ol_status_append(&used, "\n===== Open-Loop Status %s =====\n", ts);
        ol_status_append(&used, "  Uptime     : %s\n", ts);
    }
    ol_status_append(&used, "  Enable     : %s\n", status.enable ? "ON" : "OFF");
    ol_status_append(&used, "  Mode       : %d (%s)\n", status.mode,
                     status.mode == MOTOR_OL_MODE_IF ? "I/F current-loop" : "V/F voltage");
    ol_status_append(&used, "  Target RPM : %.0f\n", status.target_rpm);
    ol_status_append(&used, "  Max RPM    : %.0f\n", motor.ol_max_rpm);
    ol_status_append(&used, "  Ramp RPM   : %.0f\n", status.current_rpm);
    ol_status_append(&used, "  Start RPM  : %.0f\n", motor.ol_min_rpm);
    ol_status_append(&used, "  Ramp       : %.0f RPM/s\n", motor.ol_ramp_rpm_s);
    ol_status_append(&used, "  Pole pairs : %.0f\n", motor.ol_pole_pairs);
    ol_status_append(&used, "  PWM map    : ACB (B/C swapped)\n");
    ol_status_append(&used, "  V_bus      : %.1f V\n", status.v_bus);
    ol_status_append(&used, "  Theta/CCR  : %.4f rad / %lu %lu %lu\n", status.theta,
                     (unsigned long)status.duty1, (unsigned long)status.duty2,
                     (unsigned long)status.duty3);
    /*
     * 无示波器诊断：同时打印算法快照、ADC 反馈与 TIM1 的真实寄存器。
     * 这能区分“只算出了 CCR”和“高级定时器确实放行了六路 PWM”。
     */
    ol_status_append(&used, "  ADC current: %lu %lu %lu raw\n",
                     (unsigned long)motor.currentAdc[0], (unsigned long)motor.currentAdc[1],
                     (unsigned long)motor.currentAdc[2]);
    ol_status_append(&used, "  ADC phase  : %.3f %.3f %.3f V(raw)\n",
                     motor.voltAdc[0], motor.voltAdc[1], motor.voltAdc[2]);
    ol_status_append(&used, "  Phase Vabc : %.3f %.3f %.3f V\n", motor.va, motor.vb, motor.vc);
    ol_status_append(&used, "  Current abc: %.3f %.3f %.3f A\n", status.ia, status.ib, status.ic);
    ol_status_append(&used, "  TIM1 actual: ARR=%lu CCR=%lu/%lu/%lu CNT=%lu\n",
                     (unsigned long)TIM1->ARR, (unsigned long)TIM1->CCR1,
                     (unsigned long)TIM1->CCR2, (unsigned long)TIM1->CCR3,
                     (unsigned long)TIM1->CNT);
    ol_status_append(&used, "  TIM1 gate  : CEN=%d MOE=%d BKE=%d BIF=%d CCER=%04lX\n",
                     (TIM1->CR1 & TIM_CR1_CEN) != 0U, (TIM1->BDTR & TIM_BDTR_MOE) != 0U,
                     (TIM1->BDTR & TIM_BDTR_BKE) != 0U, (TIM1->SR & TIM_SR_BIF) != 0U,
                     (unsigned long)(TIM1->CCER & 0xFFFFU));
    ol_status_append(&used, "  TIM1 ch    : U=%d/%d V=%d/%d W=%d/%d mode=%lX/%lX\n",
                     (TIM1->CCER & TIM_CCER_CC1E) != 0U, (TIM1->CCER & TIM_CCER_CC1NE) != 0U,
                     (TIM1->CCER & TIM_CCER_CC2E) != 0U, (TIM1->CCER & TIM_CCER_CC2NE) != 0U,
                     (TIM1->CCER & TIM_CCER_CC3E) != 0U, (TIM1->CCER & TIM_CCER_CC3NE) != 0U,
                     (unsigned long)TIM1->CCMR1, (unsigned long)TIM1->CCMR2);
    ol_status_append(&used, "  Fault      : %d %s (%s)\n", fault, Motor_Protect_GetFaultName(fault),
                     status.fault_latched ? "LATCHED" : "CLEAR");
    ol_status_append(&used, "  FOC loop   : %.0f Hz adc=%.0f div=%d avg=%.1fus max=%.1fus load=n/a\n",
                     foc_frequency_hz, adc_frequency_hz, FOC_CONTROL_LOOP_FREQ_DIVIDER, foc_average_us, foc_maximum_us);
    ol_status_append(&used, "  UART TX    : %s @ %lu baud, bytes=%lu drop=%lu\n",
                     Bsp_Usart_TxDmaIsReady() ? "DMA" : "POLLING",
                     (unsigned long)DEBUG_USART_BAUD,
                     (unsigned long)Bsp_Usart_GetTxBytes(),
                     (unsigned long)Bsp_Usart_GetTxDrops());

    if (status.mode == MOTOR_OL_MODE_IF) {
        /* 电流模式 */
        ol_status_append(&used, "  Iq set     : %.2f A\n", motor.ol_iq_set);
        ol_status_append(&used, "  Id set     : %.2f A\n", motor.ol_id_set);
        ol_status_append(&used, "  FW         : max=%.2fA duty=%.2f ramp=%.1fA/s\n", motor.fw_current_max,
                         motor.fw_duty_start, motor.fw_ramp_a_s);
        ol_status_append(&used, "  Current Kp : %.4f\n", motor.ol_current_kp);
        ol_status_append(&used, "  Current Ki : %.2f\n", motor.ol_current_ki);
        ol_status_append(&used, "  Vq limit   : %.1f V\n", motor.ol_vq_limit);
        ol_status_append(&used, "  Start Phs  : %.2f rad\n", motor.ol_start_phase_boost);
        ol_status_append(&used, "  Align cfg  : %s Id=%.2fA time=%.2fs angle=%.2frad\n",
                         motor.ol_align_enable ? "ON" : "OFF", motor.ol_align_current, motor.ol_align_time,
                         motor.ol_align_angle);
        ol_status_append(&used, "  Align run  : %s elapsed=%.3fs\n", status.aligning ? "ALIGNING" : "RAMP/IDLE",
                         status.align_elapsed_s);
        ol_status_append(&used, "  HFI start   : %s V=%.2f cycles=%u min=%.2f run=%d ok=%d angle=%.3f conf=%.3f\n",
                         motor.hfi_start_enable ? "ON" : "OFF", motor.hfi_start_voltage, motor.hfi_start_cycles,
                         motor.hfi_start_min_confidence, Motor_HfiStart_IsRunning(), Motor_HfiStart_Succeeded(),
                         Motor_HfiStart_GetAngle(), Motor_HfiStart_GetConfidence());
        ol_status_append(&used, "  Ia,Ib,Ic   : %.3f %.3f %.3f A\n", status.ia, status.ib, status.ic);
        ol_status_append(&used, "  Id, Iq     : %.3f %.3f A (target %.3f %.3f)\n", status.id, status.iq,
                         status.id_target, status.iq_target);
        ol_status_append(&used, "  Vd, Vq int : %.3f %.3f\n", status.vd_int, status.vq_int);
        ol_status_append(&used, "  Speed loop : %s\n", status.speed_enable ? "ON" : "OFF");
        ol_status_append(&used, "  Sensorless : %s (%s)\n", sensorless_state,
                         motor.sensorless_enable ? "ENABLED" : "DISABLED");
        ol_status_append(&used, "  SL takeover: %s\n", motor.sensorless_takeover ? "ARMED" : "BLOCKED");
        ol_status_append(
            &used, "  SL checks  : F=%d S=%d D=%d P=%d lock=%.3fs blend=%.2f V=%.2fV\n", sensorless_diagnostics.flux_ok,
            sensorless_diagnostics.speed_ok, sensorless_diagnostics.direction_ok, sensorless_diagnostics.phase_ok,
            sensorless_diagnostics.lock_timer, sensorless_diagnostics.blend, sensorless_diagnostics.applied_voltage);
        ol_status_append(&used, "  SL limits  : trans=%.0f fallback=%.0f OLmax=%.0f phase=%.2frad blend=%.2fs\n",
                         motor.sensorless_transition_rpm, motor.sensorless_fallback_rpm,
                         motor.sensorless_openloop_max_rpm, motor.sensorless_phase_tolerance,
                         motor.sensorless_blend_time);
        ol_status_append(&used, "  Observer RPM: %.1f\n", Motor_Sensorless_GetMechanicalRpm());
        ol_status_append(&used, "  Flux       : %.6f Wb\n", Motor_Sensorless_GetFluxMagnitude());
        ol_status_append(&used, "  Phase err  : %.3f rad\n", Motor_Sensorless_GetPhaseError());
        ol_status_append(&used, "  Meas RPM   : %.1f\n", status.measured_rpm);
        ol_status_append(&used, "  Speed err  : %.1f RPM\n", status.speed_error);
        ol_status_append(&used, "  Speed Iq   : %.3f A\n", status.speed_iq);
        if (status.feedback.used_source == MOTOR_FEEDBACK_SRC_HALL) {
            ol_status_append(&used, "  Hall RPM   : raw=%.1f filtered=%.1f fast=%.1f age=%.1fms state=%d valid=%d\n",
                             status.feedback.mech_rpm_raw, status.feedback.mech_rpm,
                             status.feedback.mech_rpm_fast,
                             status.feedback.sample_age_s * 1000.0f, status.feedback.sensor_state,
                             status.feedback.speed_valid);
        }
        ol_status_append(&used, "  Speed PI   : Kp=%.5f Ki=%.5f Limit=%.2fA @ %.0fHz\n", motor.speed_kp, motor.speed_ki,
                         motor.speed_iq_limit, motor.speed_loop_hz);
    } else {
        /* V/F 模式 */
        ol_status_append(&used, "  V_boost    : %.2f V\n", motor.ol_v_boost);
        ol_status_append(&used, "  Kv_f       : %.4f V/Hz\n", motor.ol_kv_f);
        ol_status_append(&used, "  Vq limit   : %.1f V\n", motor.ol_vq_limit);

        float f_elec = fabsf(status.current_rpm) / 60.0f * motor.ol_pole_pairs;
        float vq_est = motor.ol_v_boost + motor.ol_kv_f * f_elec;
        if (vq_est > motor.ol_vq_limit)
            vq_est = motor.ol_vq_limit;
        float mod = 0.0f;
        if (status.v_bus > 1.0f)
            mod = vq_est / status.v_bus * 100.0f;
        ol_status_append(&used, "  Est Vq     : %.2f V (%.0f%% mod)\n", vq_est, mod);
    }
    /* 统计从状态快照到 RAM 格式化完成的墙钟时间，其中也包含 FOC 中断抢占时间。 */
    format_elapsed_us = Hal_Motor_TimeUs() - format_start;
    ol_status_append(&used, "  Status fmt : %lu us (RAM)\n", (unsigned long)format_elapsed_us);
    ol_status_append(&used, "=============================\n\n");

    ol_status_send(sgcOlStatusBuffer, used);
    Bsp_Usart_Flush();
    ol_status_unlock();
}

/**
 * @description: olspeed 命令 — 设置或查看开环目标转速。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为目标转速 (RPM)，负数为反转
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlSpeed(int argc, char *argv[])
{
    motor_command_t cmd;
    float rpm;

    if (argc < 2) {
        ol_print_status();
        PRINTF("Usage: olspeed <rpm>  — set target speed, negative for reverse\n");
        return 0;
    }

    rpm = atof(argv[1]);
    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return -1;
    /* 运行中禁止直接过零反转：零速附近 Iq 仍在，轴会顿一下。 */
    if (cmd.enable) {
        float ref_rpm = (fabsf(motor.ol_current_rpm) > 1.0f) ? motor.ol_current_rpm : cmd.target_rpm;

        if (rpm != 0.0f && ref_rpm != 0.0f && ((rpm > 0.0f) != (ref_rpm > 0.0f))) {
            PRINTF("Reverse while running: olstop first, then olspeed / olstart\n");
            return -1;
        }
    }
    (void)Motor_Position_Enable(false);
    (void)Motor_Joint_Enable(false);
    /* 只改目标转速，保留 enable；启动/停止必须用 olstart / olstop，避免 olspeed 误启动过流 */
    cmd.target_rpm = rpm;
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        PRINTF("Invalid speed command\n");
        return -1;
    }
    PRINTF("Target RPM set to %.0f (%s)\n", motor.ol_target_rpm, motor.ol_enable ? "running" : "stopped, use olstart");
    ol_print_high_speed_guard_notice(motor.ol_target_rpm);
    return 0;
}

/**
 * @description: olboost 命令 — 设置 V/F 启动补偿电压。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为补偿电压 (V)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlBoost(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("V_boost = %.2f V  (usage: olboost <volts>)\n", motor.ol_v_boost);
        return 0;
    }
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.ol_v_boost = atof(argv[1]);
    if (cfg.ol_v_boost < 0.0f) {
        cfg.ol_v_boost = 0.0f;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    PRINTF("V_boost set to %.2f V\n", motor.ol_v_boost);
    return 0;
}

/**
 * @description: olkvf 命令 — 设置 V/F 斜率。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为斜率 (V/Hz)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlKvf(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Kv_f = %.4f V/Hz  (usage: olkvf <V/Hz>)\n", motor.ol_kv_f);
        return 0;
    }
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.ol_kv_f = atof(argv[1]);
    if (cfg.ol_kv_f < 0.0f) {
        cfg.ol_kv_f = 0.0f;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    PRINTF("Kv_f set to %.4f V/Hz\n", motor.ol_kv_f);
    return 0;
}

/**
 * @description: olramp 命令 — 设置软启动斜坡速率。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为斜坡速率 (RPM/s)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlRamp(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Ramp = %.0f RPM/s (I/F V/F only; speed loop uses 25000 ERPM/s)\n", motor.ol_ramp_rpm_s);
        PRINTF("Usage: olramp <rpm/s>\n");
        return 0;
    }
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.ol_ramp_rpm_s = atof(argv[1]);
    if (cfg.ol_ramp_rpm_s < 0.0f) {
        cfg.ol_ramp_rpm_s = 0.0f;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    PRINTF("Ramp set to %.0f RPM/s (I/F V/F; speed loop ignores this)\n", motor.ol_ramp_rpm_s);
    return 0;
}

/**
 * @description: olvlimit 命令 — 设置 Vq 电压上限。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 Vq 上限 (V)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlVlimit(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Vq_limit = %.1f V  (usage: olvlimit <volts>)\n", motor.ol_vq_limit);
        return 0;
    }
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.ol_vq_limit = atof(argv[1]);
    if (cfg.ol_vq_limit < 0.0f) {
        cfg.ol_vq_limit = 0.0f;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    PRINTF("Vq_limit set to %.1f V\n", motor.ol_vq_limit);
    return 0;
}

/**
 * @description: olmax 命令 — 设置开环最大转速限制。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为最大转速 (RPM)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlMaxSpeed(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Max RPM = %.0f  (usage: olmax <rpm>)\n", motor.ol_max_rpm);
        return 0;
    }

    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.ol_max_rpm = atof(argv[1]);
    if (cfg.ol_max_rpm < 1.0f) {
        cfg.ol_max_rpm = 1.0f;
    }
    if (cfg.ol_min_rpm > cfg.ol_max_rpm) {
        cfg.ol_min_rpm = cfg.ol_max_rpm;
    }
    if (cfg.sensorless_openloop_max_rpm > cfg.ol_max_rpm) {
        cfg.sensorless_openloop_max_rpm = cfg.ol_max_rpm;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    PRINTF("Max RPM set to %.0f\n", motor.ol_max_rpm);
    return 0;
}

/**
 * @description: olmin 命令 — 设置目标为零时 olstart 使用的默认启动转速。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为最小转速 (RPM)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlMinSpeed(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Start RPM = %.0f  (usage: olmin <rpm>)\n", motor.ol_min_rpm);
        return 0;
    }

    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.ol_min_rpm = atof(argv[1]);
    if (cfg.ol_min_rpm < 0.0f) {
        cfg.ol_min_rpm = 0.0f;
    }
    if (cfg.ol_max_rpm > 0.0f && cfg.ol_min_rpm > cfg.ol_max_rpm) {
        cfg.ol_min_rpm = cfg.ol_max_rpm;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    PRINTF("Start RPM set to %.0f\n", motor.ol_min_rpm);
    return 0;
}

/**
 * @description: olstart 命令 — 启动开环运行。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表（无额外参数）
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlStart(int argc, char *argv[])
{
    motor_command_t cmd;

    (void)argc;
    (void)argv;

    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return -1;
    if (cmd.target_rpm == 0.0f && !Motor_Position_IsEnabled() && !Motor_Joint_IsEnabled()) {
        cmd.target_rpm = motor.ol_min_rpm;
    }
    /* 先打完串口再使能：FOC 一起转 UART DMA 完成中断会被饿死，连 “started” 都发不出去。 */
    PRINTF("Open-loop started (%s), target %.0f RPM",
           (motor.ol_mode == MOTOR_OL_MODE_VF) ? "V/F" : "I/F", cmd.target_rpm);
    if (motor.ol_mode == MOTOR_OL_MODE_VF) {
        PRINTF(", boost=%.1fV\n", motor.ol_v_boost);
    } else if (motor.speed_enable && Motor_Feedback_GetSourceDefault() == MOTOR_FEEDBACK_SRC_ENCODER) {
        PRINTF(", speed Iq limit=%.2fA, start preload=%.2fA\n", motor.speed_iq_limit,
               fminf(MOTOR_ENCODER_START_IQ_A, fabsf(motor.speed_iq_limit)));
    } else {
        PRINTF(", Iq=%.2fA\n", motor.ol_iq_set);
    }
    ol_print_high_speed_guard_notice(cmd.target_rpm);
    Bsp_Usart_Flush();

    cmd.enable = true;
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        PRINTF("Invalid start command\n");
        return -1;
    }
    return 0;
}

/**
 * @description: olstop 命令 — 停止开环运行并清零积分器。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表（无额外参数）
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlStop(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    DETECT_Abort();
    Motor_Hall_DetectAbort();
    (void)Motor_Position_Enable(false);
    if (Motor_RequestStop(&motor) != MOTOR_OK) return -1;
    PRINTF("Open-loop stopped\n");
    return 0;
}

/**
 * @description: olstatus 命令 — 手动打印一次开环与保护状态。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表（无额外参数）
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlStatus(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    ol_print_status();
    return 0;
}

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
 * @description: 输出低速观察所需的单行状态，不读取编码器或性能报表。
 * @return {int} 0=已发送，-1=格式化失败
 */
static int ol_print_rpm_brief(void)
{
    char reply[192];
    motor_sensorless_light_snapshot_t observer;
    float measured_rpm, target_rpm, iq;
    bool takeover, feedback_valid;
    mc_fault_code fault;
    int length;
    uint32_t key = Motor_Crit_Enter();
    Motor_Sensorless_GetLightSnapshot(&observer);
    measured_rpm = Motor_SpeedLoop_GetMeasuredRpm();
    target_rpm = motor.ol_target_rpm;
    iq = motor.Iq;
    takeover = motor.sensorless_takeover;
    feedback_valid = Motor_SpeedLoop_IsFeedbackValid();
    fault = Motor_Protect_GetFault();
    Motor_Crit_Exit(key);
    length = snprintf(reply, sizeof(reply),
                      "rpm obs=%.1f meas=%.1f tgt=%.0f iq=%.3f %s tk=%d fb=%s fault=%d\r\n",
                      observer.rpm, measured_rpm, target_rpm, iq, ol_sensorless_state_name(observer.state),
                      takeover ? 1 : 0, feedback_valid ? "valid" : "invalid", (int)fault);
    if (length < 0 || (unsigned)length >= sizeof(reply)) return -1;
    (void)Bsp_Usart_Write((const UCHAR *)reply, (ULONG)length);
    Bsp_Usart_Flush();
    return 0;
}

/**
 * @description: 输出完整转速诊断，brief参数只输出必要安全字段。
 * @param {int} argc 参数个数
 * @param {char *[]} argv 命令参数
 * @return {int} 0=成功，-1=失败
 */
int Cmd_OlRpm(int argc, char *argv[])
{
    char reply[640];
    int length;
    float profile_us[MOTOR_TIMING_STAGE_COUNT];
    uint32_t profile_samples;
    motor_sensorless_diagnostics_t diag;
    uint32_t primask;
    float measured_rpm;
    float target_rpm;
    float iq;
    bool takeover;
    bool feedback_valid;
    mc_fault_code fault;
    float encoder_angle, encoder_mech, encoder_rpm, encoder_age;
    bool encoder_valid, encoder_zero;

    if (argc == 2 && strcmp(argv[1], "brief") == 0) return ol_print_rpm_brief();

    /* 所有字段先捕获，串口格式化/发送期间不再读取运行状态。 */
    primask = Motor_Crit_Enter();
    Motor_Sensorless_GetDiagnostics(&diag);
    measured_rpm = Motor_SpeedLoop_GetMeasuredRpm();
    target_rpm = motor.ol_target_rpm;
    iq = motor.Iq;
    takeover = motor.sensorless_takeover;
    feedback_valid = Motor_SpeedLoop_IsFeedbackValid();
    fault = Motor_Protect_GetFault();
    encoder_angle = Motor_Encoder_GetElecAngle();
    encoder_mech = Motor_Encoder_GetMechAngle();
    encoder_rpm = Motor_Encoder_GetMechRpm();
    /* 角度来自控制侧消费的帧，不能配采集任务更新后的另一帧时间戳。 */
    encoder_age = Hal_Motor_TimeElapsedSec(Motor_Encoder_GetSampleTimeUs());
    encoder_valid = Motor_Encoder_IsSpeedValid() && !Motor_Encoder_IsSignalFault();
    encoder_zero = Motor_Encoder_IsZeroValid();
    Motor_Timing_ProfileGet(profile_us, &profile_samples);
    Motor_Crit_Exit(primask);
    length = snprintf(reply, sizeof(reply),
           "rpm obs=%.1f meas=%.1f tgt=%.0f iq=%.3f %s tk=%d fb=%s fault=%d\r\n"
           "chk F=%d S=%d D=%d P=%d lock=%.3f flux=%.5f ph=%.3f lockph=%.3f\r\n"
           "ang if=%.3f raw=%.3f pll=%.3f blend=%.2f\r\n"
           "enc mech=%.4f elec=%.4f rpm=%.1f valid=%d zero=%d age=%.6fs\r\n"
           "prof n=%lu sample=%.1f current=%.1f control=%.1f observer=%.1f output=%.1f us\r\n",
           diag.mechanical_rpm, measured_rpm, target_rpm, iq, ol_sensorless_state_name(diag.state), takeover ? 1 : 0,
           feedback_valid ? "valid" : "invalid", (int)fault, diag.flux_ok ? 1 : 0, diag.speed_ok ? 1 : 0,
           diag.direction_ok ? 1 : 0, diag.phase_ok ? 1 : 0, diag.lock_timer, diag.flux_magnitude, diag.phase_error,
           diag.lock_phase_error,
           diag.reference_angle, diag.observer_angle,
           diag.pll_angle, diag.blend,
           encoder_mech, encoder_angle, encoder_rpm, encoder_valid ? 1 : 0, encoder_zero ? 1 : 0, encoder_age,
           (unsigned long)profile_samples, profile_us[MOTOR_TIMING_SAMPLE], profile_us[MOTOR_TIMING_CURRENT],
           profile_us[MOTOR_TIMING_CONTROL], profile_us[MOTOR_TIMING_OBSERVER], profile_us[MOTOR_TIMING_OUTPUT]);
    if (length < 0 || (unsigned)length >= sizeof(reply)) return -1;
    (void)Bsp_Usart_Write((const UCHAR *)reply, (ULONG)length);
    Bsp_Usart_Flush();
    return 0;
}

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
    if (!ol_capture_config(&saved_cfg)) {
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

    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.sensorless_takeover = (enable != 0);
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    if (!motor.ol_enable) {
        Motor_Sensorless_Reset();
        Motor_SpeedLoop_Reset();
    }
    PRINTF("Sensorless takeover %s\n", motor.sensorless_takeover ? "armed" : "blocked");
    return 0;
}

/**
 * @description: olfault 命令 — 查询或清除锁存的电机保护故障。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 "clear" 时清除故障
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlFault(int argc, char *argv[])
{
    motor_protect_snapshot_t snapshot;

    /* 不带 clear 参数时只查询，避免误操作解除保护。 */
    if (argc < 2 || strcmp(argv[1], "clear") != 0) {
        PRINTF("Fault = %d %s (%s)\n", Motor_Protect_GetFault(), Motor_Protect_GetFaultName(Motor_Protect_GetFault()),
               Motor_Protect_IsFaultLatched() ? "LATCHED" : "CLEAR");
        if (Motor_Protect_IsFaultLatched()) {
            Motor_Protect_GetSnapshot(&snapshot);
            PRINTF("At fault: Ipeak=%.3fA Vbus=%.2fV target=%.0fRPM ramp=%.0fRPM\n", snapshot.current_peak,
                   snapshot.bus_voltage, snapshot.target_rpm, snapshot.ramp_rpm);
            if (snapshot.speed_valid) {
                PRINTF("At fault: measured=%.1fRPM (valid)\n", snapshot.measured_rpm);
            } else {
                PRINTF("At fault: measured speed unavailable (I/F open-loop)\n");
            }
            PRINTF("At fault: Id=%.3fA Iq=%.3fA IqCmd=%.3fA VdInt=%.2fV VqInt=%.2fV Vout=%.2fV\n", snapshot.id,
                   snapshot.iq, snapshot.iq_command, snapshot.vd_integrator, snapshot.vq_integrator,
                   snapshot.applied_voltage);
            PRINTF("At fault: SL=%s obs=%.1fRPM flux=%.6fWb phase=%.3frad lock=%.3fs blend=%.2f\n",
                   ol_sensorless_state_name((motor_sensorless_state_t)snapshot.sensorless_state), snapshot.observer_rpm,
                   snapshot.flux, snapshot.phase_error, snapshot.lock_timer, snapshot.blend);
            PRINTF("At fault: SL checks F=%d S=%d D=%d P=%d takeover=%d speedPI=%d\n", snapshot.flux_ok,
                   snapshot.observer_speed_ok, snapshot.direction_ok, snapshot.phase_ok, snapshot.sensorless_takeover,
                   snapshot.speed_loop_enabled);
            if (!motor.ol_enable) {
                motor_protect_sample_t sample;
                Motor_Protect_GetFaultSample(&sample);
                PRINTF("Fault ADC snapshot valid=%d; raw CURR1/2/3, logical A/C/B mapping\n", sample.valid);
                if (sample.valid) {
                    PRINTF("Fault ADC raw=%.0f/%.0f/%.0f offset_cfg=%.6f/%.6f/%.6f scale=%.9f A/count\n",
                           sample.adc[0], sample.adc[1], sample.adc[2], sample.configured_offset[0],
                           sample.configured_offset[1], sample.configured_offset[2], sample.scale_a_per_count);
                    PRINTF("Fault ADC abc=%.6f/%.6f/%.6f A dt=%.3fus OC_elapsed=%.3fus threshold=%.3fA delay=%.3fus\n",
                           sample.current_abc[0], sample.current_abc[1], sample.current_abc[2],
                           sample.trigger_dt_s * 1e6f, sample.over_current_elapsed_s * 1e6f,
                           sample.over_current_threshold_a, sample.over_current_delay_s * 1e6f);
                    PRINTF("Fault ADC entry_us=%lu callback CNT=%u ARR=%u CR1=%u; not trigger-time snapshot\n",
                           (unsigned long)sample.entry_us, (unsigned)sample.callback_count,
                           (unsigned)sample.callback_period, (unsigned)sample.callback_cr1);
                }
            }
        }
        if (!Motor_Protect_IsFaultLatched() && !motor.ol_enable) {
            motor_protect_sample_t sample;
            Motor_Protect_GetFaultSample(&sample);
            PRINTF("Fault ADC snapshot valid=%d (no latched fault)\n", sample.valid);
        }
        PRINTF("Usage: olfault clear\n");
        return 0;
    }

    /* 先撤销运行命令并清空 PI，防止解除锁存后立即带载重启。 */
    Motor_Hall_DetectAbort();
    if (Motor_RequestStop(&motor) != MOTOR_OK) return -1;
    Motor_Protect_ClearFault();
    PRINTF("Motor fault cleared; use olstart to restart\n");
    return 0;
}

/**
 * @description: olmode 命令 — 切换开环模式（V/F 或 I/F）。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 0(V/F) 或 1(I/F)
 * @return {int} 0 成功，-1 参数无效
 * @author: lkc
 */
int Cmd_OlMode(int argc, char *argv[])
{
    motor_command_t cmd;
    int mode;

    if (argc < 2) {
        PRINTF("Open-loop mode = %d (%s)\n", motor.ol_mode,
               motor.ol_mode == MOTOR_OL_MODE_IF ? "I/F current-loop" : "V/F voltage");
        PRINTF("Usage: olmode <0|1>\n");
        PRINTF("  0 = V/F voltage mode\n");
        PRINTF("  1 = I/F current closed-loop\n");
        return 0;
    }

    mode = atoi(argv[1]);
    if (mode != MOTOR_OL_MODE_VF && mode != MOTOR_OL_MODE_IF) {
        PRINTF("Invalid mode, use 0 or 1\n");
        return -1;
    }

    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return -1;
    if (mode != MOTOR_OL_MODE_IF) {
        (void)Motor_Position_Enable(false);
    }
    cmd.mode = mode;
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        PRINTF("Invalid mode command\n");
        return -1;
    }
    PRINTF("Open-loop mode set to %d (%s)\n", mode, mode == 1 ? "Current-loop" : "V/F voltage");
    return 0;
}

/**
 * @description: olfb 命令 — 切换转子反馈源（有感电流环电角来源）。
 * @param {int} argc
 * @param {char *} argv[] argv[1]= openloop|sensorless|encoder|hall 或 0..3
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlFeedback(int argc, char *argv[])
{
    motor_feedback_source_t src;
    const char *name;

    if (argc < 2) {
        src = Motor_GetFeedbackSource();
        switch (src) {
            case MOTOR_FEEDBACK_SRC_OPENLOOP:
                name = "openloop";
                break;
            case MOTOR_FEEDBACK_SRC_SENSORLESS:
                name = "sensorless";
                break;
            case MOTOR_FEEDBACK_SRC_ENCODER:
                name = "encoder";
                break;
            case MOTOR_FEEDBACK_SRC_HALL:
                name = "hall";
                break;
            default:
                name = "?";
                break;
        }
        PRINTF("feedback = %d (%s)\n", (int)src, name);
        PRINTF("Usage: olfb <openloop|sensorless|encoder|hall>\n");
        PRINTF("  encoder: need Encoder Detect (zero_valid)\n");
        PRINTF("  hall: need calibrated hall_table\n");
        return 0;
    }

    if (strcmp(argv[1], "openloop") == 0 || strcmp(argv[1], "0") == 0) {
        src = MOTOR_FEEDBACK_SRC_OPENLOOP;
    } else if (strcmp(argv[1], "sensorless") == 0 || strcmp(argv[1], "1") == 0) {
        src = MOTOR_FEEDBACK_SRC_SENSORLESS;
    } else if (strcmp(argv[1], "encoder") == 0 || strcmp(argv[1], "2") == 0) {
        src = MOTOR_FEEDBACK_SRC_ENCODER;
    } else if (strcmp(argv[1], "hall") == 0 || strcmp(argv[1], "3") == 0) {
        src = MOTOR_FEEDBACK_SRC_HALL;
    } else {
        PRINTF("Invalid feedback source\n");
        return -1;
    }

    if (Motor_SetFeedbackSource(src) != MOTOR_OK) {
        PRINTF("Set feedback failed\n");
        return -1;
    }
    if (src != MOTOR_FEEDBACK_SRC_ENCODER) {
        (void)Motor_Position_Enable(false);
    }
    PRINTF("feedback set to %d\n", (int)src);
    return 0;
}

/**
 * @description: oliq 命令 — 设置 I/F 模式 Iq 目标电流。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为目标电流 (A)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlIq(int argc, char *argv[])
{
    motor_command_t cmd;
    float iq;

    if (argc < 2) {
        PRINTF("Iq set = %.2f A  (usage: oliq <amps>)\n", motor.ol_iq_set);
        return 0;
    }
    iq = atof(argv[1]);
    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return -1;
    cmd.iq_set = iq;
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        PRINTF("Invalid Iq command\n");
        return -1;
    }
    PRINTF("Iq set to %.2f A\n", motor.ol_iq_set);
    return 0;
}

/**
 * @description: olid 命令 — 设置 I/F 模式 Id 目标电流。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为目标电流 (A)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlId(int argc, char *argv[])
{
    motor_command_t cmd;

    if (argc < 2) {
        PRINTF("Id set = %.2f A  (usage: olid <amps>)\n", motor.ol_id_set);
        return 0;
    }
    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return -1;
    cmd.id_set = atof(argv[1]);
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        PRINTF("Invalid Id command\n");
        return -1;
    }
    PRINTF("Id set to %.2f A\n", motor.ol_id_set);
    return 0;
}

/**
 * @description: fw 命令 — 弱磁 |Id| 上限、进入门槛与斜坡。
 * @param {int} argc 参数个数
 * @param {char *} argv[]  argv[1]=max_A，argv[2]=duty_start，argv[3]=ramp_A/s；无参查询；0 关闭
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_Fw(int argc, char *argv[])
{
    float max_a;
    float duty;
    float ramp;

    if (argc < 2) {
        PRINTF("FW: max=%.2f A  duty_start=%.2f  ramp=%.1f A/s  (supply %.1f A, budget %.1f A; 0=off)\n",
               motor.fw_current_max, motor.fw_duty_start, motor.fw_ramp_a_s, MOTOR_FW_SUPPLY_CURRENT,
               MOTOR_FW_I_BUDGET);
        return 0;
    }
    max_a = (float)atof(argv[1]);
    if (max_a < 0.0f) {
        max_a = 0.0f;
    }
    if (max_a > MOTOR_FW_I_BUDGET) {
        max_a = MOTOR_FW_I_BUDGET;
    }
    motor.fw_current_max = max_a;
    if (argc >= 3) {
        duty = (float)atof(argv[2]);
        if (duty < 0.50f) {
            duty = 0.50f;
        }
        if (duty > 0.99f) {
            duty = 0.99f;
        }
        motor.fw_duty_start = duty;
    }
    if (argc >= 4) {
        ramp = (float)atof(argv[3]);
        if (ramp < 0.0f) {
            ramp = 0.0f;
        }
        motor.fw_ramp_a_s = ramp;
    }
    PRINTF("FW set: max=%.2f A  duty_start=%.2f  ramp=%.1f A/s\n", motor.fw_current_max, motor.fw_duty_start,
           motor.fw_ramp_a_s);
    return 0;
}

/**
 * @description: olkp 命令 — 设置开环电流环比例增益 Kp。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 Kp 值
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlKp(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Current Kp = %.4f  (usage: olkp <value>)\n", motor.ol_current_kp);
        return 0;
    }
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.ol_current_kp = atof(argv[1]);
    if (cfg.ol_current_kp < 0.0f) {
        cfg.ol_current_kp = 0.0f;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    PRINTF("Current Kp set to %.4f\n", motor.ol_current_kp);
    return 0;
}

/**
 * @description: olki 命令 — 设置开环电流环积分增益 Ki。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 Ki 值
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlKi(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Current Ki = %.2f  (usage: olki <value>)\n", motor.ol_current_ki);
        return 0;
    }
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.ol_current_ki = atof(argv[1]);
    if (cfg.ol_current_ki < 0.0f) {
        cfg.ol_current_ki = 0.0f;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    PRINTF("Current Ki set to %.2f\n", motor.ol_current_ki);
    return 0;
}

/**
 * @description: olphase 命令 — 保留旧 I/F 启动相位参数，当前平滑启动流程不使用。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为相位补偿 (rad)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_OlPhase(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Legacy phase boost = %.2f rad (unused by smooth startup; usage: olphase <radians>)\n",
               motor.ol_start_phase_boost);
        return 0;
    }
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.ol_start_phase_boost = atof(argv[1]);
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    PRINTF("Legacy phase boost stored as %.2f rad; smooth startup does not use it\n",
           motor.ol_start_phase_boost);
    return 0;
}

/**
 * @description: olalign 命令 — 配置 I/F 启动前 D 轴转子预对齐参数。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表：<0|1> <Id_A> <time_s> [angle_rad]
 * @return {int} 0 成功，-1 运行中修改或参数无效
 * @author: lkc
 */
int Cmd_OlAlign(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;
    int enable;

    if (argc < 2) {
        PRINTF("Align: %s Id=%.2fA time=%.3fs angle=%.3frad\n", motor.ol_align_enable ? "ON" : "OFF",
               motor.ol_align_current, motor.ol_align_time, motor.ol_align_angle);
        PRINTF("Usage: olalign <0|1> <Id_A> <time_s> [angle_rad]\n");
        return 0;
    }

    enable = atoi(argv[1]);
    if (enable != 0 && enable != 1) {
        PRINTF("Invalid enable value, use 0 or 1\n");
        return -1;
    }
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.ol_align_enable = (enable != 0);
    if (argc >= 4) {
        cfg.ol_align_current = fabsf(atof(argv[2]));
        cfg.ol_align_time = fabsf(atof(argv[3]));
    } else if (enable != 0) {
        PRINTF("Usage: olalign <0|1> <Id_A> <time_s> [angle_rad]\n");
        return -1;
    }
    if (argc >= 5) {
        cfg.ol_align_angle = atof(argv[4]);
        utils_norm_angle_rad(&cfg.ol_align_angle);
    }

    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }

    Motor_Sensorless_Reset();
    Motor_SpeedLoop_Reset();
    PRINTF("Align set: %s Id=%.2fA time=%.3fs angle=%.3frad\n", motor.ol_align_enable ? "ON" : "OFF",
           motor.ol_align_current, motor.ol_align_time, motor.ol_align_angle);
    return 0;
}

/**
 * @description: hfistart 命令 — 配置 HFI 高频注入启动参数。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表：<0|1> <voltage_V> <cycles> [min_confidence]
 * @return {int} 0 成功，-1 运行中修改或参数无效
 * @author: lkc
 */
int Cmd_HfiStart(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;
    int enable;
    int cycles;

    if (argc < 4) {
        PRINTF("HFI start: %s V=%.2f cycles=%u min_conf=%.3f\n", motor.hfi_start_enable ? "ON" : "OFF",
               motor.hfi_start_voltage, motor.hfi_start_cycles, motor.hfi_start_min_confidence);
        PRINTF("Usage: hfistart <0|1> <voltage_V> <cycles_per_axis> [min_confidence]\n");
        return 0;
    }

    enable = atoi(argv[1]);
    cycles = atoi(argv[3]);
    if ((enable != 0 && enable != 1) || cycles < 2 || cycles > 32) {
        PRINTF("Invalid value: enable=0/1, cycles=2..32\n");
        return -1;
    }

    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.hfi_start_enable = (enable != 0);
    cfg.hfi_start_voltage = fabsf(atof(argv[2]));
    cfg.hfi_start_cycles = (uint16_t)cycles;
    if (argc >= 5) {
        cfg.hfi_start_min_confidence = atof(argv[4]);
        if (cfg.hfi_start_min_confidence < 0.0f) {
            cfg.hfi_start_min_confidence = 0.0f;
        }
        if (cfg.hfi_start_min_confidence > 1.0f) {
            cfg.hfi_start_min_confidence = 1.0f;
        }
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    PRINTF("HFI start set: %s V=%.2f cycles=%u min_conf=%.3f\n", motor.hfi_start_enable ? "ON" : "OFF",
           motor.hfi_start_voltage, motor.hfi_start_cycles, motor.hfi_start_min_confidence);
    return 0;
}

#define SPEED_TEST_DWELL_DEFAULT_MS 2000U
#define SPEED_TEST_DWELL_MIN_MS     500U
#define SPEED_TEST_DWELL_MAX_MS     10000U
#define SPEED_TEST_POLL_MS          50U
#define SPEED_FF_SETTLE_MS          2500U
#define SPEED_FF_SAMPLE_COUNT       20U
#define SPEED_FF_SAMPLE_MS          50U

static const float s_speed_test_rpm[] = {
    150.0f, 300.0f, 600.0f, 300.0f, 150.0f, 0.0f, -150.0f, -300.0f, -150.0f, 0.0f,
};

static bool speed_wait_or_key(uint32_t period_ms);
static bool speed_test_set_rpm(float rpm);

static bool speed_ff_set_enable(bool enable)
{
    motor_command_t cmd;
    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return false;
    cmd.enable = enable;
    return Motor_SetCommand(&motor, &cmd) == MOTOR_OK;
}

static bool speed_ff_measure_point(float target_rpm, motor_speed_ff_fit_t *fit)
{
    float rpm_sum = 0.0f;
    float vq_sum = 0.0f;
    float measured;
    float allowed_error;
    unsigned i;

    if (!speed_test_set_rpm(target_rpm)) return false;
    if (speed_wait_or_key(SPEED_FF_SETTLE_MS)) return false;
    for (i = 0U; i < SPEED_FF_SAMPLE_COUNT; i++) {
        if (Motor_Protect_IsFaultLatched() || !Motor_SpeedLoop_IsFeedbackValid()) return false;
        rpm_sum += Motor_SpeedLoop_GetMeasuredRpm();
        vq_sum += motor.ol_vq_int;
        if (speed_wait_or_key(SPEED_FF_SAMPLE_MS)) return false;
    }
    measured = rpm_sum / (float)SPEED_FF_SAMPLE_COUNT;
    vq_sum /= (float)SPEED_FF_SAMPLE_COUNT;
    allowed_error = fmaxf(20.0f, fabsf(target_rpm) * 0.05f);
    if (fabsf(target_rpm - measured) > allowed_error || fabsf(vq_sum) > MOTOR_ENCODER_SPEED_VOLT_LIMIT_V * 0.90f ||
        !Motor_SpeedFfFit_Add(fit, measured, vq_sum)) {
        PRINTF("  reject target=%.0f meas=%.1f vq=%.3f\n", target_rpm, measured, vq_sum);
        return false;
    }
    PRINTF("  target=%+.0f meas=%+.1f vq=%+.3f V\n", target_rpm, measured, vq_sum);
    return true;
}

static bool speed_ff_run_direction(float sign, float max_rpm, motor_speed_ff_fit_t *fit)
{
    unsigned i;
    for (i = 1U; i <= 4U; i++) {
        if (!speed_ff_measure_point(sign * max_rpm * (float)i / 4.0f, fit)) return false;
    }
    return true;
}

static int speed_ff_detect(float max_rpm)
{
    motor_speed_ff_fit_t fit;
    motor_speed_ff_result_t result;
    motor_command_t initial_cmd;
    bool initial_braking;
    bool ok = false;

    if (Motor_GetCommand(&motor, &initial_cmd) != MOTOR_OK) return -1;
    initial_braking = motor.speed_allow_braking;
    if (initial_cmd.enable) {
        PRINTF("speed ff detect requires stopped motor: olstop\n");
        return -1;
    }
    if (Motor_Protect_IsFaultLatched() || Motor_Feedback_GetSourceDefault() != MOTOR_FEEDBACK_SRC_ENCODER ||
        !Motor_Encoder_IsZeroValid()) {
        PRINTF("speed ff detect needs clear fault and calibrated encoder feedback\n");
        return -1;
    }
    if (max_rpm < 600.0f || max_rpm > motor.ol_max_rpm) {
        PRINTF("speed ff max RPM must be 600..%.0f\n", motor.ol_max_rpm);
        return -1;
    }
    Motor_SpeedFfFit_Init(&fit);
    (void)Motor_Position_Enable(false);
    (void)Motor_Joint_Enable(false);
    motor.speed_allow_braking = true;
    PRINTF("speed ff detect: REMOVE LOAD, %+.0f RPM, any key aborts\n", max_rpm);
    if (!speed_test_set_rpm(max_rpm / 4.0f) || !speed_ff_set_enable(true)) goto done;
    if (!speed_ff_run_direction(1.0f, max_rpm, &fit)) goto done;
    (void)Motor_RequestStop(&motor);
    osDelay(pdMS_TO_TICKS(500U));
    if (!speed_test_set_rpm(-max_rpm / 4.0f) || !speed_ff_set_enable(true)) goto done;
    if (!speed_ff_run_direction(-1.0f, max_rpm, &fit)) goto done;
    if (!Motor_SpeedFfFit_Solve(&fit, &result) || result.rms_error_v > 0.20f) {
        PRINTF("speed ff detect: fit rejected\n");
        goto done;
    }
    PRINTF("speed ff detect: PASS points=%u\n", result.points);
    PRINTF("  MOTOR_ENCODER_SPEED_FF_V_PER_RPM = %.7f V/RPM\n", result.slope_v_per_rpm);
    PRINTF("  MOTOR_ENCODER_SPEED_FF_BOOST_V   = %.4f V\n", result.boost_v);
    PRINTF("  fit RMS error                    = %.4f V\n", result.rms_error_v);
    PRINTF("Result is not saved; review it before changing the motor profile.\n");
    ok = true;
done:
    (void)Motor_RequestStop(&motor);
    initial_cmd.enable = false;
    (void)Motor_SetCommand(&motor, &initial_cmd);
    motor.speed_allow_braking = initial_braking;
    if (!ok) PRINTF("speed ff detect: FAIL, motor auto-stopped\n");
    return ok ? 0 : -1;
}
static CMD_QUEUE_ITEM s_speed_key;

/**
 * @description: 把已拼好的字节写入串口发送队列。
 * @param {const char *} buf  内容，可为空
 * @param {uint32_t} len  字节数
 * @return {*} 无
 */
static void speed_emit_buf(const char *buf, uint32_t len)
{
    COMMON_ASSERT(buf != NULL);
    if ((len == 0U)) return;
    (void)Bsp_Usart_Write((const UCHAR *)buf, (ULONG)len);
}

/**
 * @description: 发送以 0 结尾的字符串。
 * @param {const char *} text  内容，可为空
 * @return {*} 无
 */
static void speed_emit_str(const char *text)
{
    COMMON_ASSERT(text != NULL);
    speed_emit_buf(text, (uint32_t)strlen(text));
}

/**
 * @description: 测试过程中是否收到任意按键。
 * @detail: 命令任务正占用在转速序列里，主循环不会取队列。读到一包就停止并清掉后续输入。
 * @return {bool} true=要停止
 */
static bool speed_stop_key(void)
{
    if (queueCmdHandle == NULL) return false;
    if (osMessageQueueGet(queueCmdHandle, &s_speed_key, NULL, 0) != osOK) return false;
    while (osMessageQueueGet(queueCmdHandle, &s_speed_key, NULL, 0) == osOK) {
    }
    return true;
}

/**
 * @description: 等待转速保持时间，中途有按键则提前返回。
 * @param {uint32_t} period_ms  等待时间，单位 ms
 * @return {bool} true=用户按键
 */
static bool speed_wait_or_key(uint32_t period_ms)
{
    uint32_t waited_ms = 0U;

    while (waited_ms < period_ms) {
        uint32_t step_ms = period_ms - waited_ms;

        if (step_ms > SPEED_TEST_POLL_MS) {
            step_ms = SPEED_TEST_POLL_MS;
        }
        osDelay(pdMS_TO_TICKS(step_ms));
        waited_ms += step_ms;
        if (speed_stop_key()) return true;
    }
    return false;
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
        if (speed_stop_key()) { PRINTF("sltest: stopped by input\n"); return false; }
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
    if (!ol_capture_config(&saved)) { PRINTF("sltest: config unavailable\n"); return -1; }
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
 * @description: 按固定两位小数追加转速。
 * @param {textbuf_t *} tb 输出缓冲
 * @param {float} rpm 转速，单位 RPM
 * @return {*} 无
 */
static void speed_append_rpm(textbuf_t *tb, float rpm)
{
    float scaled = rpm * 100.0f;
    int32_t centi;

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }
    centi = (int32_t)scaled;
    (void)textbuf_append_fixed2(tb, centi);
}

/**
 * @description: 写入一个速度测试目标。
 * @detail: 关掉位置环并打开速度环。运行中禁止正负直接对穿，必须先经过 0。
 * @param {float} rpm  目标转速，单位 RPM
 * @return {bool} true=已写入
 */
static bool speed_test_set_rpm(float rpm)
{
    motor_command_t cmd;
    float ref_rpm;

    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return false;
    if (cmd.enable) {
        ref_rpm = (fabsf(motor.ol_current_rpm) > 1.0f) ? motor.ol_current_rpm : cmd.target_rpm;
        if ((rpm != 0.0f) && (ref_rpm != 0.0f) && ((rpm > 0.0f) != (ref_rpm > 0.0f))) {
            speed_emit_str("speed test: pass through 0 before reverse\r\n");
            return false;
        }
    }
    (void)Motor_Position_Enable(false);
    (void)Motor_Joint_Enable(false);
    cmd.mode = MOTOR_OL_MODE_IF;
    cmd.speed_enable = true;
    cmd.target_rpm = rpm;
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        speed_emit_str("speed test: invalid speed\r\n");
        return false;
    }
    return true;
}

/**
 * @description: 依次切到预设转速，任意键停止。
 * @detail: 每段之间 osDelay，不关调度器。正转到反转中间有 0 RPM。
 * @param {uint32_t} dwell_ms  每个转速的保持时间，单位 ms
 * @return {*} 无
 */
static void speed_test_run(uint32_t dwell_ms)
{
    char line[96];
    textbuf_t tb;
    unsigned int count = sizeof(s_speed_test_rpm) / sizeof(s_speed_test_rpm[0]);
    unsigned int i;

    textbuf_init(&tb, line, sizeof(line));
    (void)textbuf_append_str(&tb, "speed test ");
    (void)textbuf_append_u32(&tb, dwell_ms);
    (void)textbuf_append_str(&tb, " ms, any key stops\r\n");
    speed_emit_buf(line, textbuf_length(&tb));

    for (i = 0U; i < count; i++) {
        float measured_rpm;
        float error_rpm;

        if (!speed_test_set_rpm(s_speed_test_rpm[i])) return;
        textbuf_init(&tb, line, sizeof(line));
        (void)textbuf_append_u32(&tb, (uint32_t)(i + 1U));
        (void)textbuf_append_char(&tb, '/');
        (void)textbuf_append_u32(&tb, (uint32_t)count);
        (void)textbuf_append_str(&tb, " target=");
        (void)textbuf_append_i32(&tb, (int32_t)s_speed_test_rpm[i], false);
        (void)textbuf_append_str(&tb, " RPM\r\n");
        speed_emit_buf(line, textbuf_length(&tb));

        if (speed_wait_or_key(dwell_ms)) {
            speed_emit_str("speed test stopped\r\n");
            return;
        }

        measured_rpm = Motor_SpeedLoop_GetMeasuredRpm();
        error_rpm = s_speed_test_rpm[i] - measured_rpm;
        textbuf_init(&tb, line, sizeof(line));
        (void)textbuf_append_str(&tb, "  meas=");
        speed_append_rpm(&tb, measured_rpm);
        (void)textbuf_append_str(&tb, " err=");
        speed_append_rpm(&tb, error_rpm);
        (void)textbuf_append_str(&tb, " RPM\r\n");
        speed_emit_buf(line, textbuf_length(&tb));
    }
    speed_emit_str("speed test done\r\n");
}

/**
 * @description: speed 命令 — 查看速度环状态，或按预设转速切换。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 无参数时打印状态；argv[1] 为 test 时跑转速序列，argv[2] 为保持时间 (ms)
 * @return {int} 0 成功，-1 前置条件不满足
 * @author: lkc
 */
int Cmd_SpeedStatus(int argc, char *argv[])
{
    uint32_t dwell_ms = SPEED_TEST_DWELL_DEFAULT_MS;

    if (argc >= 3 && strcmp(argv[1], "ff") == 0 && strcmp(argv[2], "detect") == 0) {
        float max_rpm = argc >= 4 ? atof(argv[3]) : 1200.0f;
        return speed_ff_detect(max_rpm);
    }
    if ((argc < 2) || (strcmp(argv[1], "test") != 0)) {
        if (argc >= 2) {
            PRINTF("Usage: speed [test [ms] | ff detect [max_rpm]]\n");
            return -1;
        }
        ol_print_status();
        return 0;
    }

    if (argc >= 3) {
        dwell_ms = (uint32_t)atoi(argv[2]);
    }
    if (dwell_ms < SPEED_TEST_DWELL_MIN_MS) {
        dwell_ms = SPEED_TEST_DWELL_MIN_MS;
    }
    if (dwell_ms > SPEED_TEST_DWELL_MAX_MS) {
        dwell_ms = SPEED_TEST_DWELL_MAX_MS;
    }
    if (Motor_Protect_IsFaultLatched()) {
        PRINTF("Clear motor fault before speed test\n");
        return -1;
    }
    if (motor.ol_mode != MOTOR_OL_MODE_IF) {
        PRINTF("Speed test requires I/F mode; run: olmode 1\n");
        return -1;
    }
    if (!motor.sensorless_enable && !Motor_Feedback_IsSensoredSource(Motor_GetFeedbackSource())) {
        PRINTF("Need speed feedback: slenable 1, or olfb encoder|hall\n");
        return -1;
    }
    if (!speed_test_set_rpm(s_speed_test_rpm[0])) return -1;
    if (!motor.ol_enable) {
        PRINTF("Target RPM set to 150 (stopped, use olstart)\n");
        return 0;
    }
    speed_test_run(dwell_ms);
    return 0;
}

/**
 * @description: spenable 命令 — 启停速度 PI 闭环。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 0 或 1
 * @return {int} 0 成功，-1 参数错误或前置条件不满足
 * @author: lkc
 */
int Cmd_SpeedEnable(int argc, char *argv[])
{
    motor_command_t cmd;
    int enable;

    if (argc < 2) {
        PRINTF("Speed loop = %s  (usage: spenable <0|1>)\n", motor.speed_enable ? "ON" : "OFF");
        return 0;
    }

    enable = atoi(argv[1]);
    if (enable != 0 && enable != 1) {
        PRINTF("Invalid value, use 0 or 1\n");
        return -1;
    }
    if (!enable) {
        (void)Motor_Position_Enable(false);
    }
    if (enable && motor.ol_mode != MOTOR_OL_MODE_IF) {
        PRINTF("Speed loop requires I/F mode; run: olmode 1\n");
        return -1;
    }
    if (enable && Motor_Protect_IsFaultLatched()) {
        PRINTF("Clear motor fault before enabling speed loop\n");
        return -1;
    }
    if (enable && !motor.sensorless_enable && !Motor_Feedback_IsSensoredSource(Motor_GetFeedbackSource())) {
        PRINTF("Need speed feedback: slenable 1, or olfb encoder|hall\n");
        return -1;
    }

    (void)Motor_Joint_Enable(false);
    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return -1;
    cmd.speed_enable = (enable != 0);
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        PRINTF("Invalid speed-loop command\n");
        return -1;
    }
    PRINTF("Speed loop %s\n", motor.speed_enable ? "enabled" : "disabled");
    if (motor.speed_enable && !Motor_SpeedLoop_IsFeedbackValid()) {
        if (Motor_Feedback_IsSensoredSource(Motor_GetFeedbackSource())) {
            PRINTF("Waiting: encoder Detect / hall table, and speed_valid\n");
        } else {
            PRINTF("I/F startup Iq is used until the sensorless observer locks\n");
        }
    }
    return 0;
}

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

    if (!ol_capture_config(&cfg)) {
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
        ol_print_config_result(result);
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

    if (!ol_capture_config(&cfg)) {
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
        ol_print_config_result(result);
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

    if (!ol_capture_config(&cfg)) {
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
        ol_print_config_result(result);
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

    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.sensorless_motor_r = fabsf(atof(argv[1]));
    cfg.sensorless_motor_l = fabsf(atof(argv[2])) * 0.001f;
    cfg.sensorless_flux_linkage = fabsf(atof(argv[3])) * 0.001f;
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
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

    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.sensorless_observer_gain = fabsf(atof(argv[1]));
    cfg.sensorless_pll_kp = fabsf(atof(argv[2]));
    cfg.sensorless_pll_ki = fabsf(atof(argv[3]));
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
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
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.sensorless_phase_offset = atof(argv[1]);
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    Motor_Sensorless_Reset();
    PRINTF("Sensorless phase offset set to %.4f rad\n", motor.sensorless_phase_offset);
    return 0;
}

/**
 * @description: spkp 命令 — 设置速度环比例增益 Kp。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 Kp (A/RPM)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_SpeedKp(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Speed Kp = %.6f A/RPM  (usage: spkp <value>)\n", motor.speed_kp);
        return 0;
    }
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.speed_kp = atof(argv[1]);
    if (cfg.speed_kp < 0.0f) {
        cfg.speed_kp = 0.0f;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    Motor_SpeedLoop_Reset();
    PRINTF("Speed Kp set to %.6f A/RPM\n", motor.speed_kp);
    return 0;
}

/**
 * @description: spki 命令 — 设置速度环积分增益 Ki。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 Ki (A/(RPM·s))
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_SpeedKi(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Speed Ki = %.6f A/(RPM*s)  (usage: spki <value>)\n", motor.speed_ki);
        return 0;
    }
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.speed_ki = atof(argv[1]);
    if (cfg.speed_ki < 0.0f) {
        cfg.speed_ki = 0.0f;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    Motor_SpeedLoop_Reset();
    PRINTF("Speed Ki set to %.6f A/(RPM*s)\n", motor.speed_ki);
    return 0;
}

/**
 * @description: spiq 命令 — 设置速度 PI 可请求的最大 Iq 电流。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 Iq 上限 (A)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_SpeedIqLimit(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Speed Iq limit = %.3f A  (usage: spiq <amps>)\n", motor.speed_iq_limit);
        return 0;
    }
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.speed_iq_limit = fabsf(atof(argv[1]));
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    Motor_SpeedLoop_Reset();
    PRINTF("Speed Iq limit set to %.3f A\n", motor.speed_iq_limit);
    return 0;
}

/**
 * @description: spbrake 命令 — 允许或禁止速度 PI 输出反向制动电流。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 0 或 1
 * @return {int} 0 成功，-1 参数无效
 * @author: lkc
 */
int Cmd_SpeedBrake(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;
    int enable;

    if (argc < 2) {
        PRINTF("Speed braking = %s  (usage: spbrake <0|1>)\n", motor.speed_allow_braking ? "ON" : "OFF");
        return 0;
    }
    enable = atoi(argv[1]);
    if (enable != 0 && enable != 1) {
        PRINTF("Invalid value, use 0 or 1\n");
        return -1;
    }
    if (!ol_capture_config(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.speed_allow_braking = (enable != 0);
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        ol_print_config_result(result);
        return -1;
    }
    Motor_SpeedLoop_Reset();
    PRINTF("Speed braking %s\n", motor.speed_allow_braking ? "enabled" : "disabled");
    return 0;
}

/* 以下命令均注册在 command.c 命令表中 */

/**
 * @description: 后台开环状态周期性打印任务（兼容旧接口）。
 * @param {void *} argument 任务参数（未使用）
 * @return {void} 无
 * @author: lkc
 */
VOID Openloop_ControlTask(VOID *argument)
{
    (void)argument;

    PRINTF("\n");
    while (1) {
        ol_print_status();
        osDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @description: openloop 命令 — 创建后台开环状态打印任务。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 "create"
 * @return {int} 0 成功，-1 用法错误
 * @author: lkc
 */
int Cmd_Motor_OpenLoop(int argc, char *argv[])
{
    if (argc >= 2 && strcmp(argv[1], "create") == 0) {
        PRINTF("OpenLoop task create\r\n");
        taskPlotHandle = osThreadNew(Openloop_ControlTask, NULL, &taskDutyAttr);
        return 0;
    }
    PRINTF("usage: openloop create\r\n");
    return -1;
}

/* Cmd_Motor_OpenLoop 注册在 command.c 命令表中 */
#ifdef __cplusplus
}
#endif
