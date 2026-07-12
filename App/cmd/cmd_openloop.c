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
typedef enum
{
    OPEN_LOOP_NONE,
    OPEN_LOOP_CREATE,
    OPEN_LOOP_DESTORY,
    OPEN_LOOP_SET,
    OPEN_LOOP_START,
    OPEN_LOOP_STOP,
    OPEN_LOOP_NUM,
}OPEN_LOOP_TYPE;
/* Private macro -------------------------------------------------------------*/
#define OL_STATUS_BUFFER_SIZE             (2048U)
/* Private variables ---------------------------------------------------------*/
STATIC TaskHandle_t taskPlotHandle;
/* olstatus 先在 RAM 中完成格式化，再一次性交给 UART DMA，避免逐行 printf。 */
STATIC char sgcOlStatusBuffer[OL_STATUS_BUFFER_SIZE];
/* 防止用户开启后台 openloop 状态任务后，与手动 olstatus 同时改写静态缓冲区。 */
STATIC volatile bool sgbOlStatusBusy = false;

STATIC const osThreadAttr_t taskDutyAttr =
{
    .name = "Open Loop",
    .priority = PRI_INIT,
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

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

static void ol_status_unlock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    sgbOlStatusBusy = false;
    if (0U == primask) {
        __enable_irq();
    }
}

/*
 * 向状态页尾部追加一段格式化文字。
 * vsnprintf 只写 RAM，不会像 printf 那样每生成一个字符就进入串口驱动。
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
 * @description: 打印开环当前状态
 */
static void ol_print_status(void)
{
    const char *sensorless_state;
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
    format_start = Bsp_Tim_DtNow();

    switch (Motor_Sensorless_GetState()) {
        case MOTOR_SENSORLESS_LOCKING:
            sensorless_state = "LOCKING";
            break;
        case MOTOR_SENSORLESS_CLOSED:
            sensorless_state = "CLOSED";
            break;
        case MOTOR_SENSORLESS_OPEN_LOOP:
        default:
            sensorless_state = "OPEN_LOOP";
            break;
    }

    fault = Motor_Protect_GetFault();
    adc_frequency_hz = Motor_DmaInt_GetAdcFrequencyHz();
    foc_frequency_hz = Motor_DmaInt_GetControlFrequencyHz();
    foc_average_us = Motor_DmaInt_GetAverageRuntimeUs();
    foc_maximum_us = Motor_DmaInt_GetMaximumRuntimeUs();
    /* 平均耗时包含轻/重两种 ADC 回调，所以乘 30kHz ADC 频率才是总 ISR 占用。 */
    foc_load_percent = foc_average_us * adc_frequency_hz / 10000.0f;

    ol_status_append(&used, "\n===== Open-Loop Status =====\n");
    ol_status_append(&used, "  Enable     : %s\n", motor.ol_enable ? "ON" : "OFF");
    ol_status_append(&used, "  Mode       : %d (%s)\n", motor.ol_mode,
                     motor.ol_mode == MOTOR_OL_MODE_IF ?
                     "I/F current-loop" : "V/F voltage");
    ol_status_append(&used, "  Target RPM : %.0f\n", motor.ol_target_rpm);
    ol_status_append(&used, "  Max RPM    : %.0f\n", motor.ol_max_rpm);
    ol_status_append(&used, "  Current RPM: %.0f\n", motor.ol_current_rpm);
    ol_status_append(&used, "  Min RPM    : %.0f\n", motor.ol_min_rpm);
    ol_status_append(&used, "  Ramp       : %.0f RPM/s\n", motor.ol_ramp_rpm_s);
    ol_status_append(&used, "  Pole pairs : %.0f\n", motor.ol_pole_pairs);
    ol_status_append(&used, "  V_bus      : %.1f V\n", motor.v_bus);
    ol_status_append(&used, "  Fault      : %d %s (%s)\n", fault,
                     Motor_Protect_GetFaultName(fault),
                     Motor_Protect_IsFaultLatched() ? "LATCHED" : "CLEAR");
    ol_status_append(&used,
                     "  FOC loop   : %.0f Hz adc=%.0f div=%d avg=%.1fus max=%.1fus load=%.0f%%\n",
                     foc_frequency_hz, adc_frequency_hz,
                     FOC_CONTROL_LOOP_FREQ_DIVIDER,
                     foc_average_us, foc_maximum_us, foc_load_percent);
    ol_status_append(&used, "  UART TX    : %s @ %lu baud\n",
                     Bsp_Usart_TxDmaIsReady() ? "DMA" : "POLLING",
                     (unsigned long)DEBUG_USART_BAUD);

    if (motor.ol_mode == MOTOR_OL_MODE_IF)
    {
        /* 电流模式 */
        ol_status_append(&used, "  Iq set     : %.2f A\n", motor.ol_iq_set);
        ol_status_append(&used, "  Id set     : %.2f A\n", motor.ol_id_set);
        ol_status_append(&used, "  Current Kp : %.4f\n", motor.ol_current_kp);
        ol_status_append(&used, "  Current Ki : %.2f\n", motor.ol_current_ki);
        ol_status_append(&used, "  Vq limit   : %.1f V\n", motor.ol_vq_limit);
        ol_status_append(&used, "  Start Phs  : %.2f rad\n", motor.ol_start_phase_boost);
        ol_status_append(&used,
                         "  Align cfg  : %s Id=%.2fA time=%.2fs angle=%.2frad\n",
                         motor.ol_align_enable ? "ON" : "OFF",
                         motor.ol_align_current, motor.ol_align_time,
                         motor.ol_align_angle);
        ol_status_append(&used, "  Align run  : %s elapsed=%.3fs\n",
                         Motor_OpenLoop_IsAligning() ? "ALIGNING" : "RAMP/IDLE",
                         Motor_OpenLoop_GetAlignElapsed());
        ol_status_append(&used, "  Ia,Ib,Ic   : %.3f %.3f %.3f A\n",
                         motor.ia, motor.ib, motor.ic);
        ol_status_append(&used, "  Id, Iq     : %.3f %.3f A\n", motor.Id, motor.Iq);
        ol_status_append(&used, "  Vd, Vq int : %.3f %.3f\n",
                         motor.ol_vd_int, motor.ol_vq_int);
        ol_status_append(&used, "  Speed loop : %s\n", motor.speed_enable ? "ON" : "OFF");
        ol_status_append(&used, "  Sensorless : %s (%s)\n", sensorless_state,
                         motor.sensorless_enable ? "ENABLED" : "DISABLED");
        ol_status_append(&used, "  SL takeover: %s\n",
                         motor.sensorless_takeover ? "ARMED" : "BLOCKED");
        ol_status_append(&used, "  Observer RPM: %.1f\n",
                         Motor_Sensorless_GetMechanicalRpm());
        ol_status_append(&used, "  Flux       : %.6f Wb\n",
                         Motor_Sensorless_GetFluxMagnitude());
        ol_status_append(&used, "  Phase err  : %.3f rad\n",
                         Motor_Sensorless_GetPhaseError());
        ol_status_append(&used, "  Meas RPM   : %.1f\n",
                         Motor_SpeedLoop_GetMeasuredRpm());
        ol_status_append(&used, "  Speed err  : %.1f RPM\n",
                         Motor_SpeedLoop_GetError());
        ol_status_append(&used, "  Speed Iq   : %.3f A\n",
                         Motor_SpeedLoop_GetIqTarget());
        ol_status_append(&used,
                         "  Speed PI   : Kp=%.5f Ki=%.5f Limit=%.2fA @ %.0fHz\n",
                         motor.speed_kp, motor.speed_ki,
                         motor.speed_iq_limit, motor.speed_loop_hz);
    }
    else
    {
        /* V/F 模式 */
        ol_status_append(&used, "  V_boost    : %.2f V\n", motor.ol_v_boost);
        ol_status_append(&used, "  Kv_f       : %.4f V/Hz\n", motor.ol_kv_f);
        ol_status_append(&used, "  Vq limit   : %.1f V\n", motor.ol_vq_limit);

        float f_elec = fabsf(motor.ol_current_rpm) / 60.0f * motor.ol_pole_pairs;
        float vq_est = motor.ol_v_boost + motor.ol_kv_f * f_elec;
        if (vq_est > motor.ol_vq_limit) vq_est = motor.ol_vq_limit;
        float mod = 0.0f;
        if (motor.v_bus > 1.0f) mod = vq_est / motor.v_bus * 100.0f;
        ol_status_append(&used, "  Est Vq     : %.2f V (%.0f%% mod)\n", vq_est, mod);
    }
    /* 统计从状态快照到 RAM 格式化完成的墙钟时间，其中也包含 FOC 中断抢占时间。 */
    format_elapsed_us = Bsp_Tim_DtNow() - format_start;
    ol_status_append(&used, "  Status fmt : %lu us (RAM)\n",
                     (unsigned long)format_elapsed_us);
    ol_status_append(&used, "=============================\n\n");

    Bsp_Usart_PutBuf((const UCHAR *)sgcOlStatusBuffer, (ULONG)used);
    ol_status_unlock();
}

/**
 * @description: olspeed 命令 — 设置/查看开环目标转速
 * @param argc 参数个数
 * @param argv 参数列表
 */
int Cmd_OlSpeed(int argc, char *argv[])
{
    if (argc < 2)
    {
        ol_print_status();
        PRINTF("Usage: olspeed <rpm>  — set target speed, negative for reverse\n");
        return 0;
    }

    float rpm = atof(argv[1]);

    if (motor.ol_max_rpm > 0.0f) {
        if (rpm > motor.ol_max_rpm) {
            rpm = motor.ol_max_rpm;
        } else if (rpm < -motor.ol_max_rpm) {
            rpm = -motor.ol_max_rpm;
        }
    }

    motor.ol_target_rpm = rpm;
    motor.ol_enable = (rpm != 0.0f);
    if (!motor.ol_enable) motor.ol_current_rpm = 0.0f;
    if (motor.ol_enable) Bsp_Tim_StartPwm(MOTOR1);
    PRINTF("Target RPM set to %.0f\n", rpm);
    return 0;
}

/**
 * @description: olboost 命令 — 设置 V/F 启动补偿电压
 */
int Cmd_OlBoost(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("V_boost = %.2f V  (usage: olboost <volts>)\n", motor.ol_v_boost);
        return 0;
    }
    motor.ol_v_boost = atof(argv[1]);
    if (motor.ol_v_boost < 0.0f) motor.ol_v_boost = 0.0f;
    PRINTF("V_boost set to %.2f V\n", motor.ol_v_boost);
    return 0;
}

/**
 * @description: olkvf 命令 — 设置 V/F 斜率
 */
int Cmd_OlKvf(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Kv_f = %.4f V/Hz  (usage: olkvf <V/Hz>)\n", motor.ol_kv_f);
        return 0;
    }
    motor.ol_kv_f = atof(argv[1]);
    if (motor.ol_kv_f < 0.0f) motor.ol_kv_f = 0.0f;
    PRINTF("Kv_f set to %.4f V/Hz\n", motor.ol_kv_f);
    return 0;
}

/**
 * @description: olramp 命令 — 设置软启动斜坡速率
 */
int Cmd_OlRamp(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Ramp = %.0f RPM/s  (usage: olramp <rpm/s>)\n", motor.ol_ramp_rpm_s);
        return 0;
    }
    motor.ol_ramp_rpm_s = atof(argv[1]);
    if (motor.ol_ramp_rpm_s < 0.0f) motor.ol_ramp_rpm_s = 0.0f;
    PRINTF("Ramp set to %.0f RPM/s\n", motor.ol_ramp_rpm_s);
    return 0;
}

/**
 * @description: olvlimit 命令 — 设置 Vq 上限
 */
int Cmd_OlVlimit(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Vq_limit = %.1f V  (usage: olvlimit <volts>)\n", motor.ol_vq_limit);
        return 0;
    }
    motor.ol_vq_limit = atof(argv[1]);
    if (motor.ol_vq_limit < 0.0f) motor.ol_vq_limit = 0.0f;
    PRINTF("Vq_limit set to %.1f V\n", motor.ol_vq_limit);
    return 0;
}

int Cmd_OlMaxSpeed(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Max RPM = %.0f  (usage: olmax <rpm>)\n", motor.ol_max_rpm);
        return 0;
    }

    motor.ol_max_rpm = atof(argv[1]);
    if (motor.ol_max_rpm < 1.0f) motor.ol_max_rpm = 1.0f;
    if (motor.ol_target_rpm > motor.ol_max_rpm) motor.ol_target_rpm = motor.ol_max_rpm;
    if (motor.ol_target_rpm < -motor.ol_max_rpm) motor.ol_target_rpm = -motor.ol_max_rpm;
    PRINTF("Max RPM set to %.0f\n", motor.ol_max_rpm);
    return 0;
}

int Cmd_OlMinSpeed(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Min RPM = %.0f  (usage: olmin <rpm>)\n", motor.ol_min_rpm);
        return 0;
    }

    motor.ol_min_rpm = atof(argv[1]);
    if (motor.ol_min_rpm < 0.0f) motor.ol_min_rpm = 0.0f;
    if (motor.ol_max_rpm > 0.0f && motor.ol_min_rpm > motor.ol_max_rpm)
    {
        motor.ol_min_rpm = motor.ol_max_rpm;
    }
    PRINTF("Min RPM set to %.0f\n", motor.ol_min_rpm);
    return 0;
}

int Cmd_OlStart(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    if (motor.ol_target_rpm == 0.0f)
    {
        motor.ol_target_rpm = motor.ol_min_rpm;
    }
    if (motor.ol_max_rpm > 0.0f && motor.ol_target_rpm > motor.ol_max_rpm)
    {
        motor.ol_target_rpm = motor.ol_max_rpm;
    }
    else if (motor.ol_max_rpm > 0.0f && motor.ol_target_rpm < -motor.ol_max_rpm)
    {
        motor.ol_target_rpm = -motor.ol_max_rpm;
    }

    motor.ol_enable = true;
    Bsp_Tim_StartPwm(MOTOR1);
    PRINTF("Open-loop started, target %.0f RPM\n", motor.ol_target_rpm);
    return 0;
}

int Cmd_OlStop(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    motor.ol_enable = false;
    motor.ol_target_rpm = 0.0f;
    motor.ol_current_rpm = 0.0f;
    motor.ol_vd_int = 0.0f;
    motor.ol_vq_int = 0.0f;
    Motor_SpeedLoop_Reset();
    TIMER_UPDATE_DUTY_M1(TIM1->ARR / 2, TIM1->ARR / 2, TIM1->ARR / 2);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    PRINTF("Open-loop stopped\n");
    return 0;
}

/*
 * olstatus：手动打印一次开环、电流环、无感和保护状态。
 * 原有 ol_print_status() 只被后台 openloop 任务调用；导出这个 Shell 包装
 * 函数后，调试时无需启动后台任务也能随时查询 Ia/Ib/Ic 与故障状态。
 */
int Cmd_OlStatus(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    ol_print_status();
    return 0;
}

/* sltakeover <0|1>：单独授权观测器接管角度，默认关闭便于安全观察。 */
int Cmd_SensorlessTakeover(int argc, char *argv[])
{
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

    motor.sensorless_takeover = (enable != 0);
    Motor_Sensorless_Reset();
    Motor_SpeedLoop_Reset();
    PRINTF("Sensorless takeover %s\n",
           motor.sensorless_takeover ? "armed" : "blocked");
    return 0;
}

int Cmd_OlFault(int argc, char *argv[])
{
    motor_protect_snapshot_t snapshot;

    /* 不带 clear 参数时只查询，避免误操作解除保护。 */
    if (argc < 2 || strcmp(argv[1], "clear") != 0)
    {
        PRINTF("Fault = %d %s (%s)\n", Motor_Protect_GetFault(),
               Motor_Protect_GetFaultName(Motor_Protect_GetFault()),
               Motor_Protect_IsFaultLatched() ? "LATCHED" : "CLEAR");
        if (Motor_Protect_IsFaultLatched()) {
            Motor_Protect_GetSnapshot(&snapshot);
            PRINTF("At fault: Ipeak=%.3fA Vbus=%.2fV target=%.0fRPM ramp=%.0fRPM\n",
                   snapshot.current_peak, snapshot.bus_voltage,
                   snapshot.target_rpm, snapshot.ramp_rpm);
            if (snapshot.speed_valid) {
                PRINTF("At fault: measured=%.1fRPM (valid)\n", snapshot.measured_rpm);
            } else {
                PRINTF("At fault: measured speed unavailable (I/F open-loop)\n");
            }
        }
        PRINTF("Usage: olfault clear\n");
        return 0;
    }

    /* 先撤销运行命令并清空 PI，防止解除锁存后立即带载重启。 */
    motor.ol_enable = false;
    motor.ol_target_rpm = 0.0f;
    motor.ol_current_rpm = 0.0f;
    motor.ol_vd_int = 0.0f;
    motor.ol_vq_int = 0.0f;
    Motor_SpeedLoop_Reset();
    TIMER_UPDATE_DUTY_M1(TIM1->ARR / 2, TIM1->ARR / 2, TIM1->ARR / 2);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    Motor_Protect_ClearFault();
    PRINTF("Motor fault cleared; use olstart to restart\n");
    return 0;
}

/**
 * @description: olmode 命令 — 切换开环模式
 * @usage: olmode <0|1>  (0=V/F, 1=I/F)
 */
int Cmd_OlMode(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Open-loop mode = %d (%s)\n", motor.ol_mode,
               motor.ol_mode == MOTOR_OL_MODE_IF ? "I/F current-loop" : "V/F voltage");
        PRINTF("Usage: olmode <0|1>\n");
        PRINTF("  0 = V/F voltage mode\n");
        PRINTF("  1 = I/F current closed-loop\n");
        return 0;
    }

    int mode = atoi(argv[1]);
    if (mode != MOTOR_OL_MODE_VF && mode != MOTOR_OL_MODE_IF)
    {
        PRINTF("Invalid mode, use 0 or 1\n");
        return -1;
    }

    motor.ol_mode = mode;
    motor.ol_vd_int = 0.0f;
    motor.ol_vq_int = 0.0f;
    Motor_SpeedLoop_Reset();
    PRINTF("Open-loop mode set to %d (%s)\n", mode,
           mode == 1 ? "Current-loop" : "V/F voltage");
    return 0;
}

/**
 * @description: oliq 命令 — 设置开环电流模式 Iq 目标
 */
int Cmd_OlIq(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Iq set = %.2f A  (usage: oliq <amps>)\n", motor.ol_iq_set);
        return 0;
    }
    motor.ol_iq_set = atof(argv[1]);
    if (motor.ol_iq_set < 0.0f) motor.ol_iq_set = 0.0f;
    PRINTF("Iq set to %.2f A\n", motor.ol_iq_set);
    return 0;
}

/**
 * @description: olid 命令 — 设置开环电流模式 Id 目标（通常为 0）
 */
int Cmd_OlId(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Id set = %.2f A  (usage: olid <amps>)\n", motor.ol_id_set);
        return 0;
    }
    motor.ol_id_set = atof(argv[1]);
    PRINTF("Id set to %.2f A\n", motor.ol_id_set);
    return 0;
}

/**
 * @description: olkp 命令 — 设置开环电流环 Kp
 */
int Cmd_OlKp(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Current Kp = %.4f  (usage: olkp <value>)\n", motor.ol_current_kp);
        return 0;
    }
    motor.ol_current_kp = atof(argv[1]);
    if (motor.ol_current_kp < 0.0f) motor.ol_current_kp = 0.0f;
    PRINTF("Current Kp set to %.4f\n", motor.ol_current_kp);
    return 0;
}

/**
 * @description: olki 命令 — 设置开环电流环 Ki
 */
int Cmd_OlKi(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Current Ki = %.2f  (usage: olki <value>)\n", motor.ol_current_ki);
        return 0;
    }
    motor.ol_current_ki = atof(argv[1]);
    if (motor.ol_current_ki < 0.0f) motor.ol_current_ki = 0.0f;
    PRINTF("Current Ki set to %.2f\n", motor.ol_current_ki);
    return 0;
}

/**
 * @description: olphase 命令 — 设置启动相位补偿
 */
int Cmd_OlPhase(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Start phase boost = %.2f rad  (usage: olphase <radians>)\n",
               motor.ol_start_phase_boost);
        return 0;
    }
    motor.ol_start_phase_boost = atof(argv[1]);
    PRINTF("Start phase boost set to %.2f rad\n", motor.ol_start_phase_boost);
    return 0;
}

/*
 * olalign <0|1> <Id_A> <time_s> [angle_rad]
 *
 * 设置 I/F 启动前的固定 D 轴预对齐。参数只能在停止状态修改，避免运行中
 * 突然把旋转磁场切换成固定磁场。
 */
int Cmd_OlAlign(int argc, char *argv[])
{
    int enable;

    if (argc < 4) {
        PRINTF("Align: %s Id=%.2fA time=%.3fs angle=%.3frad\n",
               motor.ol_align_enable ? "ON" : "OFF",
               motor.ol_align_current, motor.ol_align_time, motor.ol_align_angle);
        PRINTF("Usage: olalign <0|1> <Id_A> <time_s> [angle_rad]\n");
        return 0;
    }

    if (motor.ol_enable) {
        PRINTF("Stop motor before changing alignment parameters\n");
        return -1;
    }

    enable = atoi(argv[1]);
    if (enable != 0 && enable != 1) {
        PRINTF("Invalid enable value, use 0 or 1\n");
        return -1;
    }

    motor.ol_align_enable = (enable != 0);
    motor.ol_align_current = fabsf(atof(argv[2]));
    motor.ol_align_time = fabsf(atof(argv[3]));
    if (argc >= 5) {
        motor.ol_align_angle = atof(argv[4]);
        utils_norm_angle_rad(&motor.ol_align_angle);
    }

    Motor_Sensorless_Reset();
    Motor_SpeedLoop_Reset();
    PRINTF("Align set: %s Id=%.2fA time=%.3fs angle=%.3frad\n",
           motor.ol_align_enable ? "ON" : "OFF",
           motor.ol_align_current, motor.ol_align_time, motor.ol_align_angle);
    return 0;
}

/* speed 命令：查看速度反馈、速度误差、PI 参数和 Iq 输出。 */
int Cmd_SpeedStatus(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    ol_print_status();
    return 0;
}

/* spenable <0|1>：切换手动 Iq 和速度 PI，不自动修改目标转速。 */
int Cmd_SpeedEnable(int argc, char *argv[])
{
    int enable;

    if (argc < 2) {
        PRINTF("Speed loop = %s  (usage: spenable <0|1>)\n",
               motor.speed_enable ? "ON" : "OFF");
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
    if (enable && !motor.sensorless_enable) {
        PRINTF("Enable sensorless observer first: slenable 1\n");
        return -1;
    }

    Motor_SpeedLoop_Reset();
    motor.speed_enable = (enable != 0);
    PRINTF("Speed loop %s\n", motor.speed_enable ? "enabled" : "disabled");
    if (motor.speed_enable && !Motor_SpeedLoop_IsFeedbackValid()) {
        PRINTF("I/F startup Iq is used until the sensorless observer locks\n");
    }
    return 0;
}

/* slenable <0|1>：启停无感观测器；关闭时只保留原来的 I/F 开环。 */
int Cmd_SensorlessEnable(int argc, char *argv[])
{
    int enable;

    if (argc < 2) {
        PRINTF("Sensorless observer = %s  (usage: slenable <0|1>)\n",
               motor.sensorless_enable ? "ON" : "OFF");
        return 0;
    }
    enable = atoi(argv[1]);
    if (enable != 0 && enable != 1) {
        PRINTF("Invalid value, use 0 or 1\n");
        return -1;
    }

    motor.sensorless_enable = (enable != 0);
    if (!motor.sensorless_enable) {
        motor.speed_enable = false;
        motor.sensorless_takeover = false;
    }
    Motor_Sensorless_Reset();
    Motor_SpeedLoop_Reset();
    PRINTF("Sensorless observer %s\n",
           motor.sensorless_enable ? "enabled" : "disabled");
    return 0;
}

/* slrpm <transition> <fallback>：设置开环接管和失锁回退机械转速。 */
int Cmd_SensorlessRpm(int argc, char *argv[])
{
    if (argc < 3) {
        PRINTF("Sensorless transition/fallback = %.0f/%.0f RPM\n",
               motor.sensorless_transition_rpm, motor.sensorless_fallback_rpm);
        PRINTF("Usage: slrpm <transition_rpm> <fallback_rpm>\n");
        return 0;
    }

    motor.sensorless_transition_rpm = fabsf(atof(argv[1]));
    motor.sensorless_fallback_rpm = fabsf(atof(argv[2]));
    if (motor.sensorless_fallback_rpm > motor.sensorless_transition_rpm) {
        motor.sensorless_fallback_rpm = motor.sensorless_transition_rpm;
    }
    Motor_Sensorless_Reset();
    PRINTF("Sensorless transition/fallback set to %.0f/%.0f RPM\n",
           motor.sensorless_transition_rpm, motor.sensorless_fallback_rpm);
    return 0;
}

/* slmotor <R_ohm> <L_mH> <flux_mWb>：设置无感电机模型并换算到 SI 单位。 */
int Cmd_SensorlessMotor(int argc, char *argv[])
{
    if (argc < 4) {
        PRINTF("Sensorless motor: R=%.4f ohm L=%.4f mH Flux=%.4f mWb\n",
               motor.sensorless_motor_r,
               motor.sensorless_motor_l * 1000.0f,
               motor.sensorless_flux_linkage * 1000.0f);
        PRINTF("Usage: slmotor <R_ohm> <L_mH> <flux_mWb>\n");
        return 0;
    }

    motor.sensorless_motor_r = fabsf(atof(argv[1]));
    motor.sensorless_motor_l = fabsf(atof(argv[2])) * 0.001f;
    motor.sensorless_flux_linkage = fabsf(atof(argv[3])) * 0.001f;
    Motor_Sensorless_Reset();
    PRINTF("Sensorless motor set: R=%.4f ohm L=%.4f mH Flux=%.4f mWb\n",
           motor.sensorless_motor_r,
           motor.sensorless_motor_l * 1000.0f,
           motor.sensorless_flux_linkage * 1000.0f);
    return 0;
}

/* slobs <gain> <pll_kp> <pll_ki>：设置观测器收敛增益和 PLL 增益。 */
int Cmd_SensorlessObserver(int argc, char *argv[])
{
    if (argc < 4) {
        PRINTF("Observer gain=%.1f PLL Kp=%.1f Ki=%.1f\n",
               motor.sensorless_observer_gain,
               motor.sensorless_pll_kp,
               motor.sensorless_pll_ki);
        PRINTF("Usage: slobs <observer_gain> <pll_kp> <pll_ki>\n");
        return 0;
    }

    motor.sensorless_observer_gain = fabsf(atof(argv[1]));
    motor.sensorless_pll_kp = fabsf(atof(argv[2]));
    motor.sensorless_pll_ki = fabsf(atof(argv[3]));
    Motor_Sensorless_Reset();
    PRINTF("Observer gain=%.1f PLL Kp=%.1f Ki=%.1f\n",
           motor.sensorless_observer_gain,
           motor.sensorless_pll_kp,
           motor.sensorless_pll_ki);
    return 0;
}

/* slphase <rad>：补偿观测器固定电角度偏差，调试前保持 0。 */
int Cmd_SensorlessPhase(int argc, char *argv[])
{
    if (argc < 2) {
        PRINTF("Sensorless phase offset = %.4f rad  (usage: slphase <rad>)\n",
               motor.sensorless_phase_offset);
        return 0;
    }
    motor.sensorless_phase_offset = atof(argv[1]);
    Motor_Sensorless_Reset();
    PRINTF("Sensorless phase offset set to %.4f rad\n",
           motor.sensorless_phase_offset);
    return 0;
}

/* spkp <A/RPM>：设置速度环比例增益。 */
int Cmd_SpeedKp(int argc, char *argv[])
{
    if (argc < 2) {
        PRINTF("Speed Kp = %.6f A/RPM  (usage: spkp <value>)\n", motor.speed_kp);
        return 0;
    }
    motor.speed_kp = atof(argv[1]);
    if (motor.speed_kp < 0.0f) motor.speed_kp = 0.0f;
    Motor_SpeedLoop_Reset();
    PRINTF("Speed Kp set to %.6f A/RPM\n", motor.speed_kp);
    return 0;
}

/* spki <A/(RPM*s)>：设置速度环积分增益。 */
int Cmd_SpeedKi(int argc, char *argv[])
{
    if (argc < 2) {
        PRINTF("Speed Ki = %.6f A/(RPM*s)  (usage: spki <value>)\n", motor.speed_ki);
        return 0;
    }
    motor.speed_ki = atof(argv[1]);
    if (motor.speed_ki < 0.0f) motor.speed_ki = 0.0f;
    Motor_SpeedLoop_Reset();
    PRINTF("Speed Ki set to %.6f A/(RPM*s)\n", motor.speed_ki);
    return 0;
}

/* spiq <A>：设置速度 PI 可以请求的最大转矩电流。 */
int Cmd_SpeedIqLimit(int argc, char *argv[])
{
    if (argc < 2) {
        PRINTF("Speed Iq limit = %.3f A  (usage: spiq <amps>)\n",
               motor.speed_iq_limit);
        return 0;
    }
    motor.speed_iq_limit = fabsf(atof(argv[1]));
    Motor_SpeedLoop_Reset();
    PRINTF("Speed Iq limit set to %.3f A\n", motor.speed_iq_limit);
    return 0;
}

/* spbrake <0|1>：是否允许速度 PI 输出与目标方向相反的制动电流。 */
int Cmd_SpeedBrake(int argc, char *argv[])
{
    int enable;

    if (argc < 2) {
        PRINTF("Speed braking = %s  (usage: spbrake <0|1>)\n",
               motor.speed_allow_braking ? "ON" : "OFF");
        return 0;
    }
    enable = atoi(argv[1]);
    if (enable != 0 && enable != 1) {
        PRINTF("Invalid value, use 0 or 1\n");
        return -1;
    }
    motor.speed_allow_braking = (enable != 0);
    Motor_SpeedLoop_Reset();
    PRINTF("Speed braking %s\n", motor.speed_allow_braking ? "enabled" : "disabled");
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olspeed, Cmd_OlSpeed, openloop V/F speed set);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olstatus, Cmd_OlStatus, show openloop current and fault status);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olboost, Cmd_OlBoost, openloop V/F boost voltage);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olkvf, Cmd_OlKvf, openloop V/F slope);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olramp, Cmd_OlRamp, openloop V/F ramp rate);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olvlimit, Cmd_OlVlimit, openloop Vq limit);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olmax, Cmd_OlMaxSpeed, openloop max speed);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olmin, Cmd_OlMinSpeed, openloop min speed);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olstart, Cmd_OlStart, openloop start);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olstop, Cmd_OlStop, openloop stop);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olfault, Cmd_OlFault, show or clear latched motor fault);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olmode, Cmd_OlMode, openloop mode 0=VF 1=current);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                oliq, Cmd_OlIq, openloop current Iq target);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olid, Cmd_OlId, openloop current Id target);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olkp, Cmd_OlKp, openloop current Kp);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olki, Cmd_OlKi, openloop current Ki);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olphase, Cmd_OlPhase, openloop start phase boost);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olalign, Cmd_OlAlign, startup rotor alignment configuration);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                speed, Cmd_SpeedStatus, show speed loop status);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                spenable, Cmd_SpeedEnable, enable speed PI loop);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                spkp, Cmd_SpeedKp, set speed PI Kp);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                spki, Cmd_SpeedKi, set speed PI Ki);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                spiq, Cmd_SpeedIqLimit, set speed PI Iq limit);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                spbrake, Cmd_SpeedBrake, allow speed loop braking);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                slenable, Cmd_SensorlessEnable, enable sensorless observer);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                sltakeover, Cmd_SensorlessTakeover, allow observer angle takeover);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                slrpm, Cmd_SensorlessRpm, set sensorless transition rpm);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                slmotor, Cmd_SensorlessMotor, set sensorless motor model);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                slobs, Cmd_SensorlessObserver, set observer and PLL gains);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                slphase, Cmd_SensorlessPhase, set observer phase offset);

/**
 * @description: 原 openloop 任务（保留兼容）
 */
VOID Openloop_ControlTask(VOID *argument)
{
    (void)argument;

    PRINTF("\n");
    while (1)
    {
        ol_print_status();
        osDelay(pdMS_TO_TICKS(1000));
    }
}

VOID Cmd_Motor_OpenLoop(int cTaskStat, int cPrecentage)
{
    switch (cTaskStat)
    {
        case OPEN_LOOP_CREATE:
            PRINTF("OpenLoop task create\n");
            taskPlotHandle = osThreadNew(Openloop_ControlTask, NULL, &taskDutyAttr);
            break;
        default:
            break;
    }
    return;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                openloop, Cmd_Motor_OpenLoop, open loop task);
#ifdef __cplusplus
}
#endif
