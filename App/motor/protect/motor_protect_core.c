/* 独立保护判据与故障锁存，无硬件或全局控制依赖。 */
#include <math.h>
#include "common_assert.h"
#include <string.h>
#include "motor_protect_core.h"

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

/**
 * @description: 取三相电流绝对值峰值，用作过流/堵转判据
 * @param {const motor_protect_input_t} *motor  采样电流来源，不可为空
 * @return {float} 三相 |ia/ib/ic| 峰值 (A)
 * @author: lkc
 */
static float Motor_Protect_CurrentPeak(const motor_protect_input_t *motor)
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

static bool Motor_Protect_CheckOverCurrentPeak(motor_protect_state_t *state, float peak, float dt)
{
    return Motor_Protect_Debounce(peak >= state->config.over_current, dt,
                                  state->config.over_current_time, &state->over_current_timer);
}

/**
 * @description: 首次锁存故障：写快照、置 fault_latched；已锁存则忽略（保留首因）
 * @param {motor_protect_state_t} *state  状态对象
 * @param {mc_fault_code} fault           待锁存故障码
 * @param {const motor_protect_input_t} *motor    快照中电机实时量来源
 * @param {const motor_protect_aux_t} *aux  辅助快照；可为 NULL
 * @return {*}
 * @author: lkc
 */
static void Motor_Protect_LatchFaultInputState(motor_protect_state_t *state, mc_fault_code fault, const motor_protect_input_t *motor,
                                          const motor_protect_aux_t *aux)
{
    COMMON_ASSERT(state != NULL && motor != NULL);
    if (state->fault_latched) return;

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
 * @description: 初始化指定保护状态对象，写入阈值并清零计时与锁存
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const motor_protect_config_t} *config  NULL 用默认阈值
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_InitState(motor_protect_state_t *state, const motor_protect_config_t *config)
{
    COMMON_ASSERT(state != NULL);
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

    COMMON_ASSERT(state != NULL);
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
    COMMON_ASSERT(state != NULL);
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
 * @param {const motor_protect_input_t} *motor    三相电流
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=去抖完成，应锁存过流
 * @author: lkc
 */
bool Motor_Protect_CheckOverCurrentInputState(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt)
{
    if (state == NULL || motor == NULL) return false;
    return Motor_Protect_CheckOverCurrentPeak(state, Motor_Protect_CurrentPeak(motor), dt);
}

/**
 * @description: 欠压去抖：仅在使能且目标转速非零时，v_bus 过低持续 under_voltage_time
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const motor_protect_input_t} *motor    母线与命令
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=去抖完成，应锁存欠压
 * @author: lkc
 */
bool Motor_Protect_CheckUnderVoltageInputState(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt)
{
    bool running;
    bool under_voltage;

    if (state == NULL || motor == NULL) return false;
    running = motor->ol_enable && fabsf(motor->ol_target_rpm) > 0.0f;
    under_voltage = running && motor->v_bus < state->config.under_voltage;
    return Motor_Protect_Debounce(under_voltage, dt, state->config.under_voltage_time, &state->under_voltage_timer);
}

/**
 * @description: 堵转去抖：高目标 + 近零实测速 + 大电流（需速度反馈有效）
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const motor_protect_input_t} *motor    目标与电流
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=去抖完成，应锁存堵转
 * @author: lkc
 */
static bool Motor_Protect_CheckStallPeak(motor_protect_state_t *state, const motor_protect_input_t *motor,
                                         float dt, float peak)
{
    bool commanded;
    bool stopped;
    bool loaded;

    if (state == NULL || motor == NULL) return false;
    if (!state->speed_valid) {
        state->stall_timer = 0.0f;
        return false;
    }

    commanded = fabsf(motor->ol_target_rpm) >= state->config.stall_target_rpm;
    stopped = fabsf(state->speed_rpm) <= state->config.stall_speed_rpm;
    loaded = peak >= state->config.stall_current;
    return Motor_Protect_Debounce(commanded && stopped && loaded, dt, state->config.stall_time, &state->stall_timer);
}

bool Motor_Protect_CheckStallInputState(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt)
{
    if (state == NULL || motor == NULL) return false;
    return Motor_Protect_CheckStallPeak(state, motor, dt,
                                        state->speed_valid ? Motor_Protect_CurrentPeak(motor) : 0.0f);
}

/**
 * @description: 失速去抖：斜坡已跟上目标后，曾达约 50% 再跌破目标×speed_loss_ratio
 * @detail: 斜坡未到位时不算失速。速度环按 VESC 给定爬升时，实测落后斜坡是加速，不是丢速。
 *          150 运转中 olspeed 3000，斜坡未到 3000 前不得锁 MOTOR_SPEED_LOSS。
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const motor_protect_input_t} *motor    目标与斜坡转速
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=去抖完成，应锁存失速
 * @author: lkc
 */
bool Motor_Protect_CheckSpeedLossInputState(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt)
{
    float target_abs;
    float speed_abs;
    bool speed_lost;

    if (state == NULL || motor == NULL) return false;
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
    /* 给定还在爬：新轨迹，清“曾跟上”以免 150→3000 把旧转速当成已经到达高速。 */
    if (fabsf(motor->ol_current_rpm - motor->ol_target_rpm) > 1.0f) {
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
 * @description: 按过流>欠压>堵转>失速推进判定，交由调用者在同拍锁存快照。
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const motor_protect_input_t} *motor    电机实时量
 * @param {float} dt  本拍时间 (s)
 * @return {mc_fault_code} 待锁存故障或已有首因，无故障为 NONE
 * @author: lkc
 */
mc_fault_code Motor_Protect_EvaluateInputState(motor_protect_state_t *state, const motor_protect_input_t *motor,
                                              float dt)
{
    float peak;
    if (state == NULL || motor == NULL) return FAULT_CODE_NONE;
    /* 自愈：只有锁存标志、无故障码时清掉脏状态，避免误拦校准/启动 */
    if (state->fault_latched && state->fault == FAULT_CODE_NONE) {
        state->fault_latched = false;
    }
    if (state->fault_latched) return state->fault;

    peak = Motor_Protect_CurrentPeak(motor);
    if (Motor_Protect_CheckOverCurrentPeak(state, peak, dt)) {
        return FAULT_CODE_ABS_OVER_CURRENT;
    } else if (Motor_Protect_CheckUnderVoltageInputState(state, motor, dt)) {
        return FAULT_CODE_UNDER_VOLTAGE;
    } else if (Motor_Protect_CheckStallPeak(state, motor, dt, peak)) {
        return FAULT_CODE_MOTOR_STALL;
    } else if (Motor_Protect_CheckSpeedLossInputState(state, motor, dt)) {
        return FAULT_CODE_MOTOR_SPEED_LOSS;
    }
    return FAULT_CODE_NONE;
}

bool Motor_Protect_RunInputState(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt,
                            const motor_protect_aux_t *aux)
{
    mc_fault_code fault;
    if (state == NULL || motor == NULL) return false;
    fault = Motor_Protect_EvaluateInputState(state, motor, dt);
    if (fault != FAULT_CODE_NONE) Motor_Protect_LatchFaultInputState(state, fault, motor, aux);
    return state->fault_latched;
}

/**
 * @description: 外部直接锁存故障（如无感失锁），不覆盖已有首因
 * @param {motor_protect_state_t} *state  状态对象
 * @param {mc_fault_code} fault  故障码
 * @param {const motor_protect_input_t} *motor    快照来源
 * @param {const motor_protect_aux_t} *aux  辅助快照
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_TriggerFaultInputState(motor_protect_state_t *state, mc_fault_code fault, const motor_protect_input_t *motor,
                                     const motor_protect_aux_t *aux)
{
    COMMON_ASSERT(state != NULL && motor != NULL);
    if (fault == FAULT_CODE_NONE) return;
    Motor_Protect_LatchFaultInputState(state, fault, motor, aux);
}

/**
 * @description: 查询指定状态是否已锁存故障
 * @param {const motor_protect_state_t} *state  状态对象
 * @return {bool} true=已锁存
 * @author: lkc
 */
bool Motor_Protect_IsFaultLatchedState(const motor_protect_state_t *state)
{
    if (state == NULL) return false;
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
