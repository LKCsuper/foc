/*
 * @Description: 电机保护：过流、欠压、堵转、失速；核心只依赖注入状态/辅助快照
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
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
static const motor_protect_config_t s_default_config = {
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
};

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
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 取三相电流绝对值峰值，用作过流/堵转判据
 * @param {const MOTOR_PARAM_S} *motor  采样电流来源，不可为空
 * @return {float} 三相 |ia/ib/ic| 峰值 (A)
 * @author: lkc
 */
static float Motor_Protect_CurrentPeak(const MOTOR_PARAM_S *motor)
{
    float peak = fabsf(motor->ia);

    if (fabsf(motor->ib) > peak) {
        peak = fabsf(motor->ib);
    }
    if (fabsf(motor->ic) > peak) {
        peak = fabsf(motor->ic);
    }
    return peak;
}

/**
 * @description: 条件去抖：成立则累加 dt，断开则清零；满 delay 返回 true
 * @detail: 用于滤掉采样毛刺；不能代替硬件过流关断
 * @param {bool} condition  本拍保护条件是否成立
 * @param {float} dt        本拍时间 (s)
 * @param {float} delay     去抖门限时间 (s)
 * @param {float} *timer    去抖累计时间 (s)，由调用者持有
 * @return {bool} true=条件已持续满 delay
 * @author: lkc
 */
static bool Motor_Protect_Debounce(bool condition, float dt, float delay, float *timer)
{
    if (!condition) {
        *timer = 0.0f;
        return false;
    }
    *timer += dt;
    return *timer >= delay;
}

/**
 * @description: 首次锁存故障：写快照、置 fault_latched；已锁存则忽略（保留首因）
 * @param {motor_protect_state_t} *state  状态对象
 * @param {mc_fault_code} fault           待锁存故障码
 * @param {const MOTOR_PARAM_S} *motor    快照中电机实时量来源
 * @param {const motor_protect_aux_t} *aux  辅助快照；可为 NULL
 * @return {*}
 * @author: lkc
 */
static void Motor_Protect_LatchFaultState(motor_protect_state_t *state, mc_fault_code fault, const MOTOR_PARAM_S *motor,
                                          const motor_protect_aux_t *aux)
{
    if (state == NULL || motor == NULL || state->fault_latched) {
        return;
    }

    /* 电机本体量：电流/母线/斜坡/积分，与保护侧速度反馈。 */
    state->snapshot.current_peak = Motor_Protect_CurrentPeak(motor);
    state->snapshot.bus_voltage = motor->v_bus;
    state->snapshot.target_rpm = motor->ol_target_rpm;
    state->snapshot.ramp_rpm = motor->ol_current_rpm;
    state->snapshot.measured_rpm = state->speed_rpm;
    state->snapshot.id = motor->Id;
    state->snapshot.iq = motor->Iq;
    state->snapshot.vd_integrator = motor->ol_vd_int;
    state->snapshot.vq_integrator = motor->ol_vq_int;
    state->snapshot.speed_valid = state->speed_valid;
    state->snapshot.sensorless_takeover = motor->sensorless_takeover;
    state->snapshot.speed_loop_enabled = motor->speed_enable;

    if (aux != NULL) {
        /* 调用者注入的无感/Iq 命令快照。 */
        state->snapshot.observer_rpm = aux->observer_rpm;
        state->snapshot.flux = aux->flux;
        state->snapshot.phase_error = aux->phase_error;
        state->snapshot.applied_voltage = aux->applied_voltage;
        state->snapshot.lock_timer = aux->lock_timer;
        state->snapshot.blend = aux->blend;
        state->snapshot.sensorless_state = aux->sensorless_state;
        state->snapshot.iq_command = aux->iq_command;
        state->snapshot.flux_ok = aux->flux_ok;
        state->snapshot.observer_speed_ok = aux->observer_speed_ok;
        state->snapshot.direction_ok = aux->direction_ok;
        state->snapshot.phase_ok = aux->phase_ok;
    } else {
        state->snapshot.observer_rpm = 0.0f;
        state->snapshot.flux = 0.0f;
        state->snapshot.phase_error = 0.0f;
        state->snapshot.applied_voltage = 0.0f;
        state->snapshot.lock_timer = 0.0f;
        state->snapshot.blend = 0.0f;
        state->snapshot.sensorless_state = 0;
        state->snapshot.iq_command = motor->ol_iq_set;
        state->snapshot.flux_ok = false;
        state->snapshot.observer_speed_ok = false;
        state->snapshot.direction_ok = false;
        state->snapshot.phase_ok = false;
    }

    state->fault = fault;
    state->fault_latched = true;
}

/**
 * @description: 默认包装：从无感/速度环采集锁存辅助信息（过渡期耦合点）
 * @detail: State API 路径不要走这里；由 ISR/调用者显式填 aux
 * @param {motor_protect_aux_t} *aux      输出辅助快照，不可为空
 * @param {const MOTOR_PARAM_S} *motor    电机实时量，不可为空
 * @return {*}
 * @author: lkc
 */
static void Motor_Protect_FillAuxDefault(motor_protect_aux_t *aux, const MOTOR_PARAM_S *motor)
{
    motor_sensorless_diagnostics_t sensorless_diagnostics;

    if (aux == NULL || motor == NULL) {
        return;
    }

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
 * @description: 初始化指定保护状态对象，写入阈值并清零计时与锁存
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const motor_protect_config_t} *config  NULL 用默认阈值
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_InitState(motor_protect_state_t *state, const motor_protect_config_t *config)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->config = (config != NULL) ? *config : s_default_config;
    state->fault = FAULT_CODE_NONE;
}

/**
 * @description: 清除指定状态的故障锁存与计时（保留配置）
 * @param {motor_protect_state_t} *state  状态对象
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_ClearFaultState(motor_protect_state_t *state)
{
    motor_protect_config_t cfg;

    if (state == NULL) {
        return;
    }
    cfg = state->config;
    memset(state, 0, sizeof(*state));
    state->config = cfg;
    state->fault = FAULT_CODE_NONE;
    state->fault_latched = false;
}

/**
 * @description: 向指定状态注入速度反馈；无效时禁止堵转/失速计时继续累加
 * @param {motor_protect_state_t} *state  状态对象
 * @param {float} rpm   机械转速 (RPM)
 * @param {bool} valid  反馈可信标志
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_SetSpeedFeedbackState(motor_protect_state_t *state, float rpm, bool valid)
{
    if (state == NULL) {
        return;
    }
    state->speed_rpm = rpm;
    state->speed_valid = valid;
    if (!valid) {
        state->speed_was_reached = false;
        state->stall_timer = 0.0f;
        state->speed_loss_timer = 0.0f;
    }
}

/**
 * @description: 过流去抖：|ia/ib/ic| 峰值 ≥ over_current 持续 over_current_time
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const MOTOR_PARAM_S} *motor    三相电流
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=去抖完成，应锁存过流
 * @author: lkc
 */
bool Motor_Protect_CheckOverCurrentState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt)
{
    bool over_current;

    if (state == NULL || motor == NULL) {
        return false;
    }
    over_current = Motor_Protect_CurrentPeak(motor) >= state->config.over_current;
    return Motor_Protect_Debounce(over_current, dt, state->config.over_current_time, &state->over_current_timer);
}

/**
 * @description: 欠压去抖：仅在使能且目标转速非零时，v_bus 过低持续 under_voltage_time
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const MOTOR_PARAM_S} *motor    母线与命令
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=去抖完成，应锁存欠压
 * @author: lkc
 */
bool Motor_Protect_CheckUnderVoltageState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt)
{
    bool running;
    bool under_voltage;

    if (state == NULL || motor == NULL) {
        return false;
    }
    running = motor->ol_enable && fabsf(motor->ol_target_rpm) > 0.0f;
    under_voltage = running && motor->v_bus < state->config.under_voltage;
    return Motor_Protect_Debounce(under_voltage, dt, state->config.under_voltage_time, &state->under_voltage_timer);
}

/**
 * @description: 堵转去抖：高目标 + 近零实测速 + 大电流（需速度反馈有效）
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const MOTOR_PARAM_S} *motor    目标与电流
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=去抖完成，应锁存堵转
 * @author: lkc
 */
bool Motor_Protect_CheckStallState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt)
{
    bool commanded;
    bool stopped;
    bool loaded;

    if (state == NULL || motor == NULL) {
        return false;
    }
    if (!state->speed_valid) {
        state->stall_timer = 0.0f;
        return false;
    }

    commanded = fabsf(motor->ol_target_rpm) >= state->config.stall_target_rpm;
    stopped = fabsf(state->speed_rpm) <= state->config.stall_speed_rpm;
    loaded = Motor_Protect_CurrentPeak(motor) >= state->config.stall_current;
    return Motor_Protect_Debounce(commanded && stopped && loaded, dt, state->config.stall_time, &state->stall_timer);
}

/**
 * @description: 失速去抖：曾达目标约一半后，实测跌破目标×speed_loss_ratio
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const MOTOR_PARAM_S} *motor    目标转速
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=去抖完成，应锁存失速
 * @author: lkc
 */
bool Motor_Protect_CheckSpeedLossState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt)
{
    float target_abs;
    float speed_abs;
    bool speed_lost;

    if (state == NULL || motor == NULL) {
        return false;
    }
    if (!state->speed_valid) {
        state->speed_was_reached = false;
        state->speed_loss_timer = 0.0f;
        return false;
    }

    target_abs = fabsf(motor->ol_target_rpm);
    speed_abs = fabsf(state->speed_rpm);
    if (target_abs < state->config.stall_target_rpm) {
        state->speed_was_reached = false;
        state->speed_loss_timer = 0.0f;
        return false;
    }

    if (speed_abs >= target_abs * 0.5f) {
        state->speed_was_reached = true;
    }

    speed_lost = state->speed_was_reached && speed_abs < target_abs * state->config.speed_loss_ratio;
    return Motor_Protect_Debounce(speed_lost, dt, state->config.speed_loss_time, &state->speed_loss_timer);
}

/**
 * @description: 按优先级跑保护并在首次触发时锁存（过流>欠压>堵转>失速）
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const MOTOR_PARAM_S} *motor    电机实时量
 * @param {float} dt  本拍时间 (s)
 * @param {const motor_protect_aux_t} *aux  锁存辅助；可为 NULL
 * @return {bool} true=已锁存故障
 * @author: lkc
 */
bool Motor_Protect_RunState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt,
                            const motor_protect_aux_t *aux)
{
    if (state == NULL || motor == NULL) {
        return false;
    }
    /* 自愈：只有锁存标志、无故障码时清掉脏状态，避免误拦校准/启动 */
    if (state->fault_latched && state->fault == FAULT_CODE_NONE) {
        state->fault_latched = false;
    }
    if (state->fault_latched) {
        return true;
    }

    if (Motor_Protect_CheckOverCurrentState(state, motor, dt)) {
        Motor_Protect_LatchFaultState(state, FAULT_CODE_ABS_OVER_CURRENT, motor, aux);
    } else if (Motor_Protect_CheckUnderVoltageState(state, motor, dt)) {
        Motor_Protect_LatchFaultState(state, FAULT_CODE_UNDER_VOLTAGE, motor, aux);
    } else if (Motor_Protect_CheckStallState(state, motor, dt)) {
        Motor_Protect_LatchFaultState(state, FAULT_CODE_MOTOR_STALL, motor, aux);
    } else if (Motor_Protect_CheckSpeedLossState(state, motor, dt)) {
        Motor_Protect_LatchFaultState(state, FAULT_CODE_MOTOR_SPEED_LOSS, motor, aux);
    }

    return state->fault_latched;
}

/**
 * @description: 外部直接锁存故障（如无感失锁），不覆盖已有首因
 * @param {motor_protect_state_t} *state  状态对象
 * @param {mc_fault_code} fault  故障码
 * @param {const MOTOR_PARAM_S} *motor    快照来源
 * @param {const motor_protect_aux_t} *aux  辅助快照
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_TriggerFaultState(motor_protect_state_t *state, mc_fault_code fault, const MOTOR_PARAM_S *motor,
                                     const motor_protect_aux_t *aux)
{
    if (fault == FAULT_CODE_NONE || state == NULL || motor == NULL) {
        return;
    }
    Motor_Protect_LatchFaultState(state, fault, motor, aux);
}

/**
 * @description: 查询指定状态是否已锁存故障
 * @param {const motor_protect_state_t} *state  状态对象
 * @return {bool} true=已锁存
 * @author: lkc
 */
bool Motor_Protect_IsFaultLatchedState(const motor_protect_state_t *state)
{
    if (state == NULL) {
        return false;
    }
    /* 仅当锁存且有有效故障码才算故障；latched+NONE 视为脏状态（未锁存） */
    return state->fault_latched && state->fault != FAULT_CODE_NONE;
}

/**
 * @description: 读取指定状态故障码
 * @param {const motor_protect_state_t} *state  状态对象
 * @return {mc_fault_code} 故障码；NULL 返回 NONE
 * @author: lkc
 */
mc_fault_code Motor_Protect_GetFaultState(const motor_protect_state_t *state)
{
    return state == NULL ? FAULT_CODE_NONE : state->fault;
}

/**
 * @description: 拷贝指定状态故障快照
 * @param {const motor_protect_state_t} *state  状态对象
 * @param {motor_protect_snapshot_t} *snapshot  输出
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_GetSnapshotState(const motor_protect_state_t *state, motor_protect_snapshot_t *snapshot)
{
    if (state != NULL && snapshot != NULL) {
        *snapshot = state->snapshot;
    }
}

/**
 * @description: 初始化默认单实例保护模块
 * @param {const motor_protect_config_t} *config  NULL 用默认
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_Init(const motor_protect_config_t *config)
{
    Motor_Protect_InitState(&s_protect, config);
}

/**
 * @description: 更新默认实例速度反馈
 * @param {float} rpm   机械转速 (RPM)
 * @param {bool} valid  反馈可信
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_SetSpeedFeedback(float rpm, bool valid)
{
    Motor_Protect_SetSpeedFeedbackState(&s_protect, rpm, valid);
}

/**
 * @description: 默认实例过流去抖检测
 * @param {const MOTOR_PARAM_S} *motor  电机实时参数
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=过流去抖完成
 * @author: lkc
 */
bool Motor_Protect_CheckOverCurrent(const MOTOR_PARAM_S *motor, float dt)
{
    return Motor_Protect_CheckOverCurrentState(&s_protect, motor, dt);
}

/**
 * @description: 默认实例欠压去抖检测
 * @param {const MOTOR_PARAM_S} *motor  电机实时参数
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=欠压去抖完成
 * @author: lkc
 */
bool Motor_Protect_CheckUnderVoltage(const MOTOR_PARAM_S *motor, float dt)
{
    return Motor_Protect_CheckUnderVoltageState(&s_protect, motor, dt);
}

/**
 * @description: 默认实例堵转去抖检测
 * @param {const MOTOR_PARAM_S} *motor  电机实时参数
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=堵转去抖完成
 * @author: lkc
 */
bool Motor_Protect_CheckStall(const MOTOR_PARAM_S *motor, float dt)
{
    return Motor_Protect_CheckStallState(&s_protect, motor, dt);
}

/**
 * @description: 默认实例失速去抖检测
 * @param {const MOTOR_PARAM_S} *motor  电机实时参数
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=失速去抖完成
 * @author: lkc
 */
bool Motor_Protect_CheckSpeedLoss(const MOTOR_PARAM_S *motor, float dt)
{
    return Motor_Protect_CheckSpeedLossState(&s_protect, motor, dt);
}

/**
 * @description: 默认实例完整保护步进（内部 FillAuxDefault）
 * @param {const MOTOR_PARAM_S} *motor  电机实时参数
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=已锁存故障
 * @author: lkc
 */
bool Motor_Protect_Run(const MOTOR_PARAM_S *motor, float dt)
{
    motor_protect_aux_t aux;

    Motor_Protect_FillAuxDefault(&aux, motor);
    return Motor_Protect_RunState(&s_protect, motor, dt, &aux);
}

/**
 * @description: 默认实例外部触发故障锁存
 * @param {mc_fault_code} fault  故障码
 * @param {const MOTOR_PARAM_S} *motor  快照来源
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_TriggerFault(mc_fault_code fault, const MOTOR_PARAM_S *motor)
{
    motor_protect_aux_t aux;

    if (fault == FAULT_CODE_NONE || motor == NULL) {
        return;
    }
    Motor_Protect_FillAuxDefault(&aux, motor);
    Motor_Protect_TriggerFaultState(&s_protect, fault, motor, &aux);
}

/**
 * @description: 清除默认实例故障锁存与计时
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_ClearFault(void)
{
    Motor_Protect_ClearFaultState(&s_protect);
}

/**
 * @description: 查询默认实例是否已锁存故障
 * @return {bool} true=已锁存
 * @author: lkc
 */
bool Motor_Protect_IsFaultLatched(void)
{
    return Motor_Protect_IsFaultLatchedState(&s_protect);
}

/**
 * @description: 读取默认实例故障码
 * @return {mc_fault_code} 故障码
 * @author: lkc
 */
mc_fault_code Motor_Protect_GetFault(void)
{
    return Motor_Protect_GetFaultState(&s_protect);
}

/**
 * @description: 故障码转可读短名
 * @param {mc_fault_code} fault  故障码
 * @return {const char} * 静态字符串
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
        default:
            return "UNKNOWN";
    }
}

/**
 * @description: 拷贝默认实例故障快照
 * @param {motor_protect_snapshot_t} *snapshot  输出
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_GetSnapshot(motor_protect_snapshot_t *snapshot)
{
    Motor_Protect_GetSnapshotState(&s_protect, snapshot);
}

#ifdef __cplusplus
}
#endif
