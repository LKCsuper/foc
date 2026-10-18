/*
 * @Description: 电机保护：过流、欠压、堵转、失速；核心只依赖注入状态/辅助快照
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "common_assert.h"
#include <string.h>
#include <stdbool.h>

#include "motor_param.h"
#include "motor_protect.h"
#include "motor_speed.h"
#include "motor_sensorless.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* 默认单实例包装，步骤 2 过渡期保留；学习/测试请优先用 *State API。
 * 配置字段内联常量初始化（ARMCC 不能引用另一 static const 对象）。 */
static motor_protect_state_t s_protect = {
    .config =
        {
            .over_current = 10.0f,
            .under_voltage = 8.0f,
            .stall_target_rpm = 100.0f,
            .stall_speed_rpm = 30.0f,
            .stall_current = 0.8f,
            .speed_loss_ratio = 0.2f,
            .over_current_time = 0.001f,
            .under_voltage_time = 0.1f,
            .stall_time = 1.0f,
            .speed_loss_time = 0.3f,
        },
    .fault_latched = false,
    .fault = FAULT_CODE_NONE,
};
/* 项目兼容入口：采样快照映射到独立保护核心。 */
static motor_protect_sample_t s_fault_sample;
static bool s_fault_sample_pending;
/* Private function prototypes -----------------------------------------------*/
static void Motor_Protect_MapInput(motor_protect_input_t *input, const MOTOR_PARAM_S *motor);
static void Motor_Protect_FillAuxDefault(motor_protect_aux_t *aux, const MOTOR_PARAM_S *motor);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 把电机实时量映射成保护核心输入。
 * @detail: 只拷贝电流、母线、斜坡和开环标志，不读无感全局、不改保护状态。
 *          调用者保证 input 与 motor 非空。电流 A，母线 V，转速为机械 RPM。
 * @param {motor_protect_input_t *} input 输出，不可为空
 * @param {const MOTOR_PARAM_S *} motor   采样与命令来源，不可为空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Protect_MapInput(motor_protect_input_t *input, const MOTOR_PARAM_S *motor)
{
    input->ia = motor->ia;
    input->ib = motor->ib;
    input->ic = motor->ic;
    input->v_bus = motor->v_bus;
    input->ol_target_rpm = motor->ol_target_rpm;
    input->ol_current_rpm = motor->ol_current_rpm;
    input->Id = motor->Id;
    input->Iq = motor->Iq;
    input->ol_vd_int = motor->ol_vd_int;
    input->ol_vq_int = motor->ol_vq_int;
    input->ol_iq_set = motor->ol_iq_set;
    input->ol_enable = motor->ol_enable;
    input->sensorless_takeover = motor->sensorless_takeover;
    input->speed_enable = motor->speed_enable;
}


/**
 * @description: 过流去抖检测（三相电流峰值）。
 * @detail: 只推进过流计时并返回是否满窗，不锁存故障、不停 PWM。
 *          state 或 motor 为空返回 false。软件去抖不能代替硬件过流关断。
 * @param {motor_protect_state_t *} state 状态对象；空则忽略
 * @param {const MOTOR_PARAM_S *} motor   采样电流 (A)
 * @param {float} dt                      本拍时间 (s)，通常为 ADC 周期
 * @return {bool} true=条件已持续超过 over_current_time
 * @author: lkc
 */
bool Motor_Protect_CheckOverCurrentState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt)
{
    motor_protect_input_t input;
    if (state == NULL || motor == NULL) { return false; }
    Motor_Protect_MapInput(&input, motor);
    return Motor_Protect_CheckOverCurrentInputState(state, &input, dt);
}


/**
 * @description: 欠压去抖检测。
 * @detail: 仅在使能且目标转速非零时累计，只返回是否满窗，不锁存、不停 PWM。
 *          state 或 motor 为空返回 false。
 * @param {motor_protect_state_t *} state 状态对象；空则忽略
 * @param {const MOTOR_PARAM_S *} motor   母线电压 (V) 与使能来源
 * @param {float} dt                      本拍时间 (s)
 * @return {bool} true=欠压已持续超过 under_voltage_time
 * @author: lkc
 */
bool Motor_Protect_CheckUnderVoltageState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt)
{
    motor_protect_input_t input;
    if (state == NULL || motor == NULL) { return false; }
    Motor_Protect_MapInput(&input, motor);
    return Motor_Protect_CheckUnderVoltageInputState(state, &input, dt);
}


/**
 * @description: 堵转去抖检测。
 * @detail: 高目标、近零实测速、大电流，且速度反馈有效时才累计。
 *          只返回是否满窗，不锁存、不停 PWM。state 或 motor 为空返回 false。
 * @param {motor_protect_state_t *} state 状态对象；空则忽略
 * @param {const MOTOR_PARAM_S *} motor   目标转速 (RPM) 与电流 (A)
 * @param {float} dt                      本拍时间 (s)
 * @return {bool} true=堵转条件已持续超过 stall_time
 * @author: lkc
 */
bool Motor_Protect_CheckStallState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt)
{
    motor_protect_input_t input;
    if (state == NULL || motor == NULL) { return false; }
    Motor_Protect_MapInput(&input, motor);
    return Motor_Protect_CheckStallInputState(state, &input, dt);
}


/**
 * @description: 失速去抖检测。
 * @detail: 斜坡跟上目标后，曾达约 50% 再跌破 speed_loss_ratio 才累计。
 *          斜坡未到位不算失速。只返回是否满窗，不锁存、不停 PWM。
 *          state 或 motor 为空返回 false。
 * @param {motor_protect_state_t *} state 状态对象；空则忽略
 * @param {const MOTOR_PARAM_S *} motor   目标与斜坡转速 (RPM)
 * @param {float} dt                      本拍时间 (s)
 * @return {bool} true=失速条件已持续超过 speed_loss_time
 * @author: lkc
 */
bool Motor_Protect_CheckSpeedLossState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt)
{
    motor_protect_input_t input;
    if (state == NULL || motor == NULL) { return false; }
    Motor_Protect_MapInput(&input, motor);
    return Motor_Protect_CheckSpeedLossInputState(state, &input, dt);
}


/**
 * @description: 按优先级跑完保护并在首次触发时锁存故障与快照。
 * @detail: 优先级：过流 > 欠压 > 堵转 > 失速。已锁存则直接返回 true，不覆盖首因。
 *          state 或 motor 为空返回 false。aux 可为 NULL。不关 PWM。
 * @param {motor_protect_state_t *} state 状态对象；空则忽略
 * @param {const MOTOR_PARAM_S *} motor   电机实时量
 * @param {float} dt                      本拍时间 (s)
 * @param {const motor_protect_aux_t *} aux 锁存用辅助快照；可为 NULL
 * @return {bool} true=当前已锁存故障（含本拍新锁存）
 * @author: lkc
 */
bool Motor_Protect_RunState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt,
                            const motor_protect_aux_t *aux)
{
    motor_protect_input_t input;
    if (state == NULL || motor == NULL) { return false; }
    Motor_Protect_MapInput(&input, motor);
    return Motor_Protect_RunInputState(state, &input, dt, aux);
}


/**
 * @description: 由外部模块直接锁存故障。
 * @detail: 不覆盖已有首因，不关 PWM。state 与 motor 不可为空，违反约定触发 COMMON_ASSERT。
 *          fault 为 FAULT_CODE_NONE 时忽略。aux 可为 NULL。
 * @param {motor_protect_state_t *} state 状态对象，不可为空
 * @param {mc_fault_code} fault           故障码；FAULT_CODE_NONE 忽略
 * @param {const MOTOR_PARAM_S *} motor   用于填充快照，不可为空
 * @param {const motor_protect_aux_t *} aux 辅助快照；可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_TriggerFaultState(motor_protect_state_t *state, mc_fault_code fault,
                                     const MOTOR_PARAM_S *motor, const motor_protect_aux_t *aux)
{
    motor_protect_input_t input;
    COMMON_ASSERT(state != NULL && motor != NULL);
    Motor_Protect_MapInput(&input, motor);
    Motor_Protect_TriggerFaultInputState(state, fault, &input, aux);
}

/**
 * @description: 从无感与速度环采集默认实例的锁存辅助信息。
 * @detail: 过渡期耦合点。State API 不要走这里，由调用者显式填 aux。
 *          aux 与 motor 不可为空，违反约定触发 COMMON_ASSERT。不关 PWM。
 * @param {motor_protect_aux_t *} aux   输出辅助快照，不可为空
 * @param {const MOTOR_PARAM_S *} motor 电机实时量，不可为空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Protect_FillAuxDefault(motor_protect_aux_t *aux, const MOTOR_PARAM_S *motor)
{
    motor_sensorless_diagnostics_t sensorless_diagnostics;

    COMMON_ASSERT(aux != NULL && motor != NULL);

    memset(aux, 0, sizeof(*aux));
    Motor_Sensorless_GetDiagnostics(&sensorless_diagnostics);
    aux->observer_rpm = Motor_Sensorless_GetMechanicalRpm();
    aux->flux = Motor_Sensorless_GetFluxMagnitude();
    aux->phase_error = Motor_Sensorless_GetPhaseError();
    aux->applied_voltage = sensorless_diagnostics.applied_voltage;
    aux->lock_timer = sensorless_diagnostics.lock_timer;
    aux->blend = sensorless_diagnostics.blend;
    aux->sensorless_state = (int)Motor_Sensorless_GetState();
    aux->flux_ok = sensorless_diagnostics.flux_ok;
    aux->observer_speed_ok = sensorless_diagnostics.speed_ok;
    aux->direction_ok = sensorless_diagnostics.direction_ok;
    aux->phase_ok = sensorless_diagnostics.phase_ok;
    aux->iq_command =
        motor->speed_enable && Motor_SpeedLoop_IsFeedbackValid() ? Motor_SpeedLoop_GetIqTarget() : motor->ol_iq_set;
}

/**
 * @description: 初始化默认单实例保护模块。
 * @detail: 清故障锁存、计时和首发采样证据。config 为 NULL 时用代码默认阈值。
 *          不访问硬件，可在上电初始化调用。
 * @param {const motor_protect_config_t *} config 自定义参数；NULL 用默认
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_Init(const motor_protect_config_t *config)
{
    Motor_Protect_InitState(&s_protect, config);
    memset(&s_fault_sample, 0, sizeof(s_fault_sample));
    s_fault_sample_pending = false;
}

/**
 * @description: 更新默认实例的速度反馈。
 * @detail: 只写默认单实例。valid 为 false 时清堵转/失速计时及曾达速标志。
 * @param {float} rpm  机械转速 (RPM)
 * @param {bool} valid 反馈是否可信
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_SetSpeedFeedback(float rpm, bool valid)
{
    Motor_Protect_SetSpeedFeedbackState(&s_protect, rpm, valid);
}

/**
 * @description: 默认实例完整保护步进，仅首次故障同拍采集无感/速度辅助快照。
 * @detail: 先判定再锁存，已锁存时不再读无感全局。motor 为空返回 false。
 *          首发锁存后置采样待采集，由 FOC ISR 调 CaptureFaultSample。不关 PWM。
 * @param {const MOTOR_PARAM_S *} motor 电机实时参数；空则忽略
 * @param {float} dt                    本拍时间 (s)，ADC 路径约 1/30 kHz
 * @return {bool} true=已锁存故障
 * @author: lkc
 */
bool Motor_Protect_Run(const MOTOR_PARAM_S *motor, float dt)
{
    motor_protect_aux_t aux;
    motor_protect_input_t input;
    mc_fault_code fault;

    if (motor == NULL) return false;
    Motor_Protect_MapInput(&input, motor);
    fault = Motor_Protect_EvaluateInputState(&s_protect, &input, dt);
    if (fault == FAULT_CODE_NONE) return false;
    if (!s_protect.fault_latched) {
        Motor_Protect_FillAuxDefault(&aux, motor);
        Motor_Protect_TriggerFaultInputState(&s_protect, fault, &input, &aux);
        s_fault_sample.trigger_dt_s = dt;
        s_fault_sample.over_current_elapsed_s = s_protect.over_current_timer;
        s_fault_sample.over_current_threshold_a = s_protect.config.over_current;
        s_fault_sample.over_current_delay_s = s_protect.config.over_current_time;
        s_fault_sample_pending = true;
    }
    return s_protect.fault_latched;
}

/**
 * @description: 默认实例外部触发故障锁存。
 * @detail: 同拍采集无感/速度辅助快照，不覆盖已有首因，不关 PWM。
 *          motor 不可为空，违反约定触发 COMMON_ASSERT。FAULT_CODE_NONE 忽略。
 *          不武装首发 ADC 采样采集。
 * @param {mc_fault_code} fault         故障码；FAULT_CODE_NONE 忽略
 * @param {const MOTOR_PARAM_S *} motor 快照来源，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_TriggerFault(mc_fault_code fault, const MOTOR_PARAM_S *motor)
{
    motor_protect_aux_t aux;

    COMMON_ASSERT(motor != NULL);
    if (fault == FAULT_CODE_NONE) return;
    Motor_Protect_FillAuxDefault(&aux, motor);
    Motor_Protect_TriggerFaultState(&s_protect, fault, motor, &aux);
}

/**
 * @description: 清除默认实例故障锁存、计时和首发采样证据。
 * @detail: 保留配置。不访问硬件，不停 PWM。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_ClearFault(void)
{
    Motor_Protect_ClearFaultState(&s_protect);
    memset(&s_fault_sample, 0, sizeof(s_fault_sample));
    s_fault_sample_pending = false;
}

/**
 * @description: 首发默认保护故障后复制一帧已有采样证据。
 * @detail: 由 FOC ISR 在待采集且已锁存时调用一次。只复制 motor 里已有 ADC 与电流，
 *          不重新访问 ADC，不改保护和输出。motor 为空、未待采集或未锁存则返回。
 *          adc 与 configured_offset 按 CURR1/2/3，current_abc 按逻辑 A/B/C。
 *          valid 只表示已复制，不证明 DMA 帧一致或 ADC 物理准确。
 *          entry_us 是控制入口时间；count、period、cr1 是回调期顺序读数，
 *          不是采样孔径或锁存证明。外部触发故障不武装此采集。
 * @param {const MOTOR_PARAM_S *} motor 已换算的电流 (A) 与 ADC 帧；空则忽略
 * @param {float} scale_a_per_count     电流比例 (A/count)
 * @param {uint32_t} entry_us           控制入口时间 (us)
 * @param {uint16_t} count              回调期定时器计数值
 * @param {uint16_t} period             回调期定时器周期
 * @param {uint16_t} cr1                回调期定时器 CR1
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_CaptureFaultSample(const MOTOR_PARAM_S *motor, float scale_a_per_count, uint32_t entry_us,
                                      uint16_t count, uint16_t period, uint16_t cr1)
{
    unsigned phase;
    if (motor == NULL || !s_fault_sample_pending || !s_protect.fault_latched) return;
    s_fault_sample_pending = false;
    for (phase = 0; phase < 3; ++phase) {
        s_fault_sample.adc[phase] = motor->currentAdc[phase];
        s_fault_sample.configured_offset[phase] = motor->currentOffset[phase];
    }
    s_fault_sample.current_abc[0] = motor->ia;
    s_fault_sample.current_abc[1] = motor->ib;
    s_fault_sample.current_abc[2] = motor->ic;
    s_fault_sample.scale_a_per_count = scale_a_per_count;
    s_fault_sample.entry_us = entry_us;
    s_fault_sample.callback_count = count;
    s_fault_sample.callback_period = period;
    s_fault_sample.callback_cr1 = cr1;
    s_fault_sample.valid = true;
}

/**
 * @description: 读取首发故障采样快照。
 * @detail: 停机后的应用上下文调用，不阻塞。sample 为空则忽略。无数据时 valid 为 false。
 * @param {motor_protect_sample_t *} sample 输出缓冲；可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_GetFaultSample(motor_protect_sample_t *sample)
{
    if (sample != NULL) *sample = s_fault_sample;
}

/**
 * @description: 查询首发故障采样是否仍待采集。
 * @detail: FOC ISR 仅在返回 true 时读取回调元数据。首发采集完成后返回 false，
 *          故障保持期间不再重复取时序。
 * @return {bool} true=待采集
 * @author: lkc
 */
bool Motor_Protect_FaultSampleIsPending(void)
{
    return s_fault_sample_pending;
}

/**
 * @description: 查询默认实例是否已锁存故障。
 * @detail: 锁存且故障码不是 FAULT_CODE_NONE 才返回 true。
 * @return {bool} true=已锁存有效故障
 * @author: lkc
 */
bool Motor_Protect_IsFaultLatched(void)
{
    return Motor_Protect_IsFaultLatchedState(&s_protect);
}

/**
 * @description: 读取默认实例当前故障码。
 * @detail: 未锁存时为 FAULT_CODE_NONE。不访问硬件。
 * @return {mc_fault_code} 故障码
 * @author: lkc
 */
mc_fault_code Motor_Protect_GetFault(void)
{
    return Motor_Protect_GetFaultState(&s_protect);
}

/**
 * @description: 故障码转可读短名，供 Shell 与日志使用。
 * @detail: 返回静态字符串，生命周期与程序相同。未知码返回 "UNKNOWN"。
 * @param {mc_fault_code} fault 故障码
 * @return {const char *} 静态短名，如 "MOTOR_STALL"
 * @author: lkc
 */
const char *Motor_Protect_GetFaultName(mc_fault_code fault)
{
    switch (fault) {
        case FAULT_CODE_NONE:
            return "NONE";
        case FAULT_CODE_ABS_OVER_CURRENT:
            return "ABS_OVER_CURRENT";
        case FAULT_CODE_UNDER_VOLTAGE:
            return "UNDER_VOLTAGE";
        case FAULT_CODE_MOTOR_STALL:
            return "MOTOR_STALL";
        case FAULT_CODE_MOTOR_SPEED_LOSS:
            return "MOTOR_SPEED_LOSS";
        case FAULT_CODE_CONTROL_TIMING:
            return "CONTROL_TIMING";
        case FAULT_CODE_ENCODER_SPI:
            return "ENCODER_SPI";
        default:
            return "UNKNOWN";
    }
}

/**
 * @description: 拷贝默认实例故障快照。
 * @detail: 停机后仍保留至 ClearFault。snapshot 为空则忽略，不阻塞。
 * @param {motor_protect_snapshot_t *} snapshot 输出缓冲；可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_GetSnapshot(motor_protect_snapshot_t *snapshot)
{
    Motor_Protect_GetSnapshotState(&s_protect, snapshot);
}

#ifdef __cplusplus
}
#endif
