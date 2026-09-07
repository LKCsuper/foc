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

    if (cfg == NULL || !Motor_Settings_Capture(&motor, &settings)) {
        return false;
    }
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

    if (*used >= (OL_STATUS_BUFFER_SIZE - 1U)) {
        return;
    }

    remaining = OL_STATUS_BUFFER_SIZE - *used;
    va_start(arguments, format);
    written = vsnprintf(&sgcOlStatusBuffer[*used], remaining, format, arguments);
    va_end(arguments);

    if (written < 0) {
        return;
    }

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
                              fabsf(target_rpm) > motor.sensorless_openloop_max_rpm;

    if (!high_speed_request) {
        return;
    }

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
    float foc_load_percent;
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
    /* 平均耗时包含轻/重两种 ADC 回调，所以乘 30kHz ADC 频率才是总 ISR 占用。 */
    foc_load_percent = foc_average_us * adc_frequency_hz / 10000.0f;

    ol_status_append(&used, "\n===== Open-Loop Status [VF-LEGACY-R3] =====\n");
    ol_status_append(&used, "  Enable     : %s\n", status.enable ? "ON" : "OFF");
    ol_status_append(&used, "  Mode       : %d (%s)\n", status.mode,
                     status.mode == MOTOR_OL_MODE_IF ? "I/F current-loop" : "V/F voltage");
    ol_status_append(&used, "  Target RPM : %.0f\n", status.target_rpm);
    ol_status_append(&used, "  Max RPM    : %.0f\n", motor.ol_max_rpm);
    ol_status_append(&used, "  Current RPM: %.0f\n", status.current_rpm);
    ol_status_append(&used, "  Start RPM  : %.0f\n", motor.ol_min_rpm);
    ol_status_append(&used, "  Ramp       : %.0f RPM/s\n", motor.ol_ramp_rpm_s);
    ol_status_append(&used, "  Pole pairs : %.0f\n", motor.ol_pole_pairs);
    ol_status_append(&used, "  PWM map    : %s\n", Motor_PwmMap_GetSwapBc() ? "ACB (B/C swapped)" : "ABC");
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
    ol_status_append(&used, "  FOC loop   : %.0f Hz adc=%.0f div=%d avg=%.1fus max=%.1fus load=%.0f%%\n",
                     foc_frequency_hz, adc_frequency_hz, FOC_CONTROL_LOOP_FREQ_DIVIDER, foc_average_us, foc_maximum_us,
                     foc_load_percent);
    ol_status_append(&used, "  UART TX    : %s @ %lu baud\n", Bsp_Usart_TxDmaIsReady() ? "DMA" : "POLLING",
                     (unsigned long)DEBUG_USART_BAUD);

    if (status.mode == MOTOR_OL_MODE_IF) {
        /* 电流模式 */
        ol_status_append(&used, "  Iq set     : %.2f A\n", motor.ol_iq_set);
        ol_status_append(&used, "  Id set     : %.2f A\n", motor.ol_id_set);
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
        ol_status_append(&used, "  Id, Iq     : %.3f %.3f A\n", status.id, status.iq);
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
        ol_status_append(&used, "  Speed err  : %.1f RPM\n", Motor_SpeedLoop_GetError());
        ol_status_append(&used, "  Speed Iq   : %.3f A\n", status.speed_iq);
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

    Bsp_Usart_PutBuf((const UCHAR *)sgcOlStatusBuffer, (ULONG)used);
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
    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) {
        return -1;
    }
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
        PRINTF("Ramp = %.0f RPM/s  (usage: olramp <rpm/s>)\n", motor.ol_ramp_rpm_s);
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
    PRINTF("Ramp set to %.0f RPM/s\n", motor.ol_ramp_rpm_s);
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

    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) {
        return -1;
    }
    if (cmd.target_rpm == 0.0f) {
        cmd.target_rpm = motor.ol_min_rpm;
    }
    cmd.enable = true;
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        PRINTF("Invalid start command\n");
        return -1;
    }
    PRINTF("Open-loop started (%s), target %.0f RPM",
           (motor.ol_mode == MOTOR_OL_MODE_VF) ? "V/F" : "I/F", motor.ol_target_rpm);
    if (motor.ol_mode == MOTOR_OL_MODE_VF) {
        PRINTF(", boost=%.1fV\n", motor.ol_v_boost);
    } else {
        PRINTF(", Iq=%.2fA\n", motor.ol_iq_set);
    }
    ol_print_high_speed_guard_notice(motor.ol_target_rpm);
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

    Motor_Hall_DetectAbort();
    if (Motor_RequestStop(&motor) != MOTOR_OK) {
        return -1;
    }
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

/**
 * @description: 停机时选择最终三相 PWM 映射，用于无传感器相序实机判定。
 */
int Cmd_OlMap(int argc, char *argv[])
{
    if (argc < 2) {
        PRINTF("PWM map = %s  (usage: olmap abc|acb)\n",
               Motor_PwmMap_GetSwapBc() ? "acb" : "abc");
        return 0;
    }
    if (motor.ol_enable) {
        PRINTF("Stop motor before changing PWM map: olstop\n");
        return -1;
    }
    if (strcmp(argv[1], "abc") == 0) {
        Motor_PwmMap_SetSwapBc(false);
    } else if (strcmp(argv[1], "acb") == 0) {
        Motor_PwmMap_SetSwapBc(true);
    } else {
        PRINTF("Usage: olmap abc|acb\n");
        return -1;
    }
    Hal_Motor_PwmSetSafe();
    PRINTF("PWM map set to %s\n", Motor_PwmMap_GetSwapBc() ? "ACB (B/C swapped)" : "ABC");
    return 0;
}

/**
 * @description: sltakeover 命令 — 授权或禁止无感观测器接管电角度。
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
    Motor_Sensorless_Reset();
    Motor_SpeedLoop_Reset();
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
        }
        PRINTF("Usage: olfault clear\n");
        return 0;
    }

    /* 先撤销运行命令并清空 PI，防止解除锁存后立即带载重启。 */
    Motor_Hall_DetectAbort();
    if (Motor_RequestStop(&motor) != MOTOR_OK) {
        return -1;
    }
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

    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) {
        return -1;
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
    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) {
        return -1;
    }
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
    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) {
        return -1;
    }
    cmd.id_set = atof(argv[1]);
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        PRINTF("Invalid Id command\n");
        return -1;
    }
    PRINTF("Id set to %.2f A\n", motor.ol_id_set);
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

    if (argc < 4) {
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
    cfg.ol_align_current = fabsf(atof(argv[2]));
    cfg.ol_align_time = fabsf(atof(argv[3]));
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

/**
 * @description: speed 命令 — 查看速度环与开环综合状态。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表（无额外参数）
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_SpeedStatus(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    ol_print_status();
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

    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) {
        return -1;
    }
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

    /* 小于 0.10rad 很难在有负载时锁定；大于 1.50rad 已接近危险的 90°。 */
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
 * @description: slobs 命令 — 设置无感观测器增益与 PLL 参数。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表：<observer_gain> <pll_kp> <pll_ki>
 * @return {int} 0 成功
 * @author: lkc
 */
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
