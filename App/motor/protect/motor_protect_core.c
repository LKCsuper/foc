/*
 * @Description: 过流、欠压、堵转、失速的判据和故障锁存
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "common_assert.h"
#include <string.h>
#include "motor_protect_core.h"
#if defined(__CC_ARM)
/* 位于 ADC 中断控制路径，与其他实时模块一致局部 O2；不启用快速浮点，公式不变。 */
#pragma O2
#endif
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* config 为空时使用。不进 Flash。 */
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
/* Private function prototypes -----------------------------------------------*/
static float Motor_Protect_CurrentPeak(const motor_protect_input_t *motor);
static bool Motor_Protect_Debounce(bool condition, float dt, float delay, float *timer);
static bool Motor_Protect_CheckOverCurrentPeak(motor_protect_state_t *state, float peak, float dt);
static void Motor_Protect_LatchFaultInputState(motor_protect_state_t *state, mc_fault_code fault,
                                              const motor_protect_input_t *motor, const motor_protect_aux_t *aux);
static bool Motor_Protect_CheckStallPeak(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt,
                                         float peak);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 取三相电流绝对值里最大的一相。
 * @detail: 过流和堵转都用这个峰值，不用 Id/Iq。调用者保证 motor 非空。
 * @param {const motor_protect_input_t *} motor 本拍相电流 (A)
 * @return {float} max(|ia|, |ib|, |ic|) (A)
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
 * @description: 条件成立就累加本拍时间，不成立就清零。
 * @detail: 累加到 delay 返回 true。只负责去抖，不锁存故障，也不能代替硬件过流关断。
 *          delay 为 0 时，条件成立的第一拍就返回 true。
 * @param {bool} condition 本拍条件是否成立
 * @param {float} dt       本拍时间 (s)
 * @param {float} delay    需要持续的时间 (s)
 * @param {float *} timer  累计时间 (s)，由调用者持有
 * @return {bool} true=已经持续满 delay
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
 * @description: 过流去抖。相电流峰值达到阈值并持续 over_current_time 才返回 true。
 * @detail: 不看是否使能、目标转速是不是 0。未到时间返回 false，计时继续留在 state 里。
 * @param {motor_protect_state_t *} state 保护状态
 * @param {float} peak                    三相电流峰值 (A)
 * @param {float} dt                      本拍时间 (s)
 * @return {bool} true=去抖完成，调用者可以锁存过流
 * @author: lkc
 */
static bool Motor_Protect_CheckOverCurrentPeak(motor_protect_state_t *state, float peak, float dt)
{
    return Motor_Protect_Debounce(peak >= state->config.over_current, dt,
                                  state->config.over_current_time, &state->over_current_timer);
}

/**
 * @description: 第一次命中时写下快照并锁存；已经锁存则什么都不改。
 * @detail: 电流、母线、斜坡、Id/Iq 和积分来自本拍输入。实测转速用保护侧刚注入的速度反馈。
 *          aux 为空时，无感辅助量写 0，Iq 命令改用输入里的 ol_iq_set。不改 PWM。
 * @param {motor_protect_state_t *} state       保护状态，不可为空
 * @param {mc_fault_code} fault                 这次要锁的故障码
 * @param {const motor_protect_input_t *} motor 本拍电流和母线，不可为空
 * @param {const motor_protect_aux_t *} aux     无感辅助量；可为空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Protect_LatchFaultInputState(motor_protect_state_t *state, mc_fault_code fault, const motor_protect_input_t *motor,
                                          const motor_protect_aux_t *aux)
{
    COMMON_ASSERT(state != NULL && motor != NULL);
    if (state->fault_latched) return;

    /* 电机本体量：电流、母线、斜坡、积分，以及保护侧的速度反馈。 */
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
        /* 调用者另外传入的无感和 Iq 命令。 */
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
 * @description: 清零一块保护状态，并写入阈值。
 * @detail: config 为空时用本文件的默认阈值。不读硬件。state 为空会触发断言。
 * @param {motor_protect_state_t *} state         保护状态，不可为空
 * @param {const motor_protect_config_t *} config 阈值；空则用默认
 * @return {void} 无
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
 * @description: 清掉故障锁存、快照、速度反馈和全部计时。
 * @detail: 配置保留。不清 PWM，也不碰别的保护实例。
 * @param {motor_protect_state_t *} state 保护状态，不可为空
 * @return {void} 无
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
 * @description: 写入本拍机械转速，供堵转和失速使用。
 * @detail: 过流和欠压不看这个速度。valid 为 false 时清掉堵转计时、失速计时和“曾经跟上”。
 * @param {motor_protect_state_t *} state 保护状态，不可为空
 * @param {float} rpm                     机械转速 (RPM)
 * @param {bool} valid                    true=这拍速度可以拿来判断
 * @return {void} 无
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
 * @description: 用三相电流做一次过流去抖。
 * @detail: 只推进过流计时，不锁存。state 或 motor 为空返回 false。
 * @param {motor_protect_state_t *} state       保护状态
 * @param {const motor_protect_input_t *} motor 三相电流 (A)
 * @param {float} dt                            本拍时间 (s)
 * @return {bool} true=过流已经持续够时间
 * @author: lkc
 */
bool Motor_Protect_CheckOverCurrentInputState(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt)
{
    if (state == NULL || motor == NULL) return false;
    return Motor_Protect_CheckOverCurrentPeak(state, Motor_Protect_CurrentPeak(motor), dt);
}

/**
 * @description: 运行中的母线过低，并持续 under_voltage_time，才算欠压。
 * @detail: 未使能，或者目标转速为 0，本拍不算欠压，计时清零。state 或 motor 为空返回 false。
 *          这里的欠压要等去抖；母线低于 1 V 时控制直接停，那条不经过本函数。
 * @param {motor_protect_state_t *} state       保护状态
 * @param {const motor_protect_input_t *} motor 母线 (V)、使能和目标转速 (RPM)
 * @param {float} dt                            本拍时间 (s)
 * @return {bool} true=欠压已经持续够时间
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
 * @description: 高目标、实测几乎停住、电流又不小，持续 stall_time 才算堵转。
 * @detail: 速度反馈无效时清掉堵转计时并返回 false。三个条件要同时成立。
 * @param {motor_protect_state_t *} state       保护状态
 * @param {const motor_protect_input_t *} motor 目标转速 (RPM)
 * @param {float} dt                            本拍时间 (s)
 * @param {float} peak                          三相电流峰值 (A)
 * @return {bool} true=堵转已经持续够时间
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

/**
 * @description: 用三相电流峰值做一次堵转去抖。
 * @detail: 只推进堵转计时，不锁存。速度反馈无效时不读电流。state 或 motor 为空返回 false。
 * @param {motor_protect_state_t *} state       保护状态
 * @param {const motor_protect_input_t *} motor 目标转速和三相电流
 * @param {float} dt                            本拍时间 (s)
 * @return {bool} true=堵转已经持续够时间
 * @author: lkc
 */
bool Motor_Protect_CheckStallInputState(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt)
{
    if (state == NULL || motor == NULL) return false;
    return Motor_Protect_CheckStallPeak(state, motor, dt,
                                        state->speed_valid ? Motor_Protect_CurrentPeak(motor) : 0.0f);
}

/**
 * @description: 斜坡已经跟上目标后，转速曾经到过目标的一半，再跌破目标乘 speed_loss_ratio，才算失速。
 * @detail: 速度反馈无效、目标还不到堵转门槛、或者斜坡和目标还差 1 RPM 以上，都清掉“曾经跟上”和失速计时。
 *          斜坡还在爬时，实测落后是加速，不是丢速。例如 150 RPM 运转中把目标改成 3000，斜坡未到 3000 之前不锁失速。
 *          一半这个比例写死在本函数里，不在配置中。state 或 motor 为空返回 false。
 * @param {motor_protect_state_t *} state       保护状态
 * @param {const motor_protect_input_t *} motor 目标转速和斜坡转速 (RPM)
 * @param {float} dt                            本拍时间 (s)
 * @return {bool} true=失速已经持续够时间
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
    /* 斜坡还没贴上目标：给定仍在爬。清掉旧的“曾经跟上”，避免 150 直接被当成已经到了 3000。 */
    if (fabsf(motor->ol_current_rpm - motor->ol_target_rpm) > 1.0f) {
        state->speed_was_reached = false;
        state->speed_loss_timer = 0.0f;
        return false;
    }

    /* 50% 写死在这里。到过这一半才允许后面的失速去抖开始计时。 */
    if (speed_abs >= target_abs * 0.5f) {
        state->speed_was_reached = true;
    }

    speed_lost = state->speed_was_reached && speed_abs < target_abs * state->config.speed_loss_ratio;
    return Motor_Protect_Debounce(speed_lost, dt, state->config.speed_loss_time, &state->speed_loss_timer);
}

/**
 * @description: 推进一拍判定，返回该锁的故障码，自己不写快照。
 * @detail: 已锁存时不再给四项累加时间，直接返回第一次的故障码。
 *          未锁存时按过流、欠压、堵转、失速检查；前面一项本拍已经去抖完成，就不再查后面的。
 *          只有锁存标志、故障码却是 NONE 时，先清掉这个脏状态，避免挡住后面的启动。
 *          调用者要在同一拍把非 NONE 交给 TriggerFaultInputState 锁存。不要同一拍再跑 RunInputState，否则去抖会被推进两次。
 *          state 或 motor 为空返回 NONE。
 * @param {motor_protect_state_t *} state       保护状态
 * @param {const motor_protect_input_t *} motor 本拍电流、母线和转速
 * @param {float} dt                            本拍时间 (s)
 * @return {mc_fault_code} 本拍应锁存的故障；没有则为 NONE；已锁存则是第一次的故障码
 * @author: lkc
 */
mc_fault_code Motor_Protect_EvaluateInputState(motor_protect_state_t *state, const motor_protect_input_t *motor,
                                              float dt)
{
    float peak;
    if (state == NULL || motor == NULL) return FAULT_CODE_NONE;
    /* 只有锁存标志、没有故障码，是脏状态。清掉，避免误拦启动。 */
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

/**
 * @description: 判定本拍，并在第一次命中时锁存快照。
 * @detail: 已锁存时不再累加去抖，也不改快照。aux 为空时，无感辅助量写 0，Iq 命令用输入里的 ol_iq_set。
 *          返回值只表示现在有没有锁存。不改 PWM。state 或 motor 为空返回 false。
 * @param {motor_protect_state_t *} state       保护状态
 * @param {const motor_protect_input_t *} motor 本拍电流、母线和转速
 * @param {float} dt                            本拍时间 (s)
 * @param {const motor_protect_aux_t *} aux     无感辅助量；可为空
 * @return {bool} true=已经锁存故障
 * @author: lkc
 */
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
 * @description: 外部直接锁存一个故障，例如无感失锁。
 * @detail: 不覆盖已经锁存的第一个原因。fault 为 NONE 时直接返回。不改 PWM。
 *          state 或 motor 为空会触发断言。
 * @param {motor_protect_state_t *} state       保护状态，不可为空
 * @param {mc_fault_code} fault                 故障码；NONE 忽略
 * @param {const motor_protect_input_t *} motor 快照用的电流和母线，不可为空
 * @param {const motor_protect_aux_t *} aux     无感辅助量；可为空
 * @return {void} 无
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
 * @description: 查询这块状态是否已经锁存了有效故障。
 * @detail: 只有锁存标志、故障码却是 NONE 时返回 false。Evaluate 会把这种脏状态清掉。state 为空返回 false。
 * @param {const motor_protect_state_t *} state 保护状态；可为空
 * @return {bool} true=已锁存，且故障码不是 NONE
 * @author: lkc
 */
bool Motor_Protect_IsFaultLatchedState(const motor_protect_state_t *state)
{
    if (state == NULL) return false;
    /* 锁存标志和故障码都在，才算已经锁住。 */
    return state->fault_latched && state->fault != FAULT_CODE_NONE;
}

/**
 * @description: 读取这块状态里的故障码。
 * @detail: 不区分有没有锁存。未写入过时是 NONE。state 为空返回 NONE。
 * @param {const motor_protect_state_t *} state 保护状态；可为空
 * @return {mc_fault_code} 故障码
 * @author: lkc
 */
mc_fault_code Motor_Protect_GetFaultState(const motor_protect_state_t *state)
{
    return state == NULL ? FAULT_CODE_NONE : state->fault;
}

/**
 * @description: 拷贝这块状态里的故障快照。
 * @detail: 停机后仍保留，直到 ClearFaultState。state 或 snapshot 为空则不写。
 * @param {const motor_protect_state_t *} state 保护状态；可为空
 * @param {motor_protect_snapshot_t *} snapshot 输出；可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_GetSnapshotState(const motor_protect_state_t *state, motor_protect_snapshot_t *snapshot)
{
    if (state != NULL && snapshot != NULL) {
        *snapshot = state->snapshot;
    }
}
#ifdef __cplusplus
}
#endif
