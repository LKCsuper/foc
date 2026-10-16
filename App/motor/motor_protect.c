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


bool Motor_Protect_CheckOverCurrentState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt)
{
    motor_protect_input_t input;
    if (state == NULL || motor == NULL) { return false; }
    Motor_Protect_MapInput(&input, motor);
    return Motor_Protect_CheckOverCurrentInputState(state, &input, dt);
}


bool Motor_Protect_CheckUnderVoltageState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt)
{
    motor_protect_input_t input;
    if (state == NULL || motor == NULL) { return false; }
    Motor_Protect_MapInput(&input, motor);
    return Motor_Protect_CheckUnderVoltageInputState(state, &input, dt);
}


bool Motor_Protect_CheckStallState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt)
{
    motor_protect_input_t input;
    if (state == NULL || motor == NULL) { return false; }
    Motor_Protect_MapInput(&input, motor);
    return Motor_Protect_CheckStallInputState(state, &input, dt);
}


bool Motor_Protect_CheckSpeedLossState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt)
{
    motor_protect_input_t input;
    if (state == NULL || motor == NULL) { return false; }
    Motor_Protect_MapInput(&input, motor);
    return Motor_Protect_CheckSpeedLossInputState(state, &input, dt);
}


bool Motor_Protect_RunState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt,
                            const motor_protect_aux_t *aux)
{
    motor_protect_input_t input;
    if (state == NULL || motor == NULL) { return false; }
    Motor_Protect_MapInput(&input, motor);
    return Motor_Protect_RunInputState(state, &input, dt, aux);
}


void Motor_Protect_TriggerFaultState(motor_protect_state_t *state, mc_fault_code fault,
                                     const MOTOR_PARAM_S *motor, const motor_protect_aux_t *aux)
{
    motor_protect_input_t input;
    if (state == NULL || motor == NULL) { return; }
    Motor_Protect_MapInput(&input, motor);
    Motor_Protect_TriggerFaultInputState(state, fault, &input, aux);
}

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

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
 * @description: 默认实例完整保护步进，仅在新故障时采集辅助快照。
 * @param {const MOTOR_PARAM_S} *motor  电机实时参数
 * @param {float} dt  本拍时间 (s)
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
    }
    return s_protect.fault_latched;
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
        case FAULT_CODE_CONTROL_TIMING:
            return "CONTROL_TIMING";
        case FAULT_CODE_ENCODER_SPI:
            return "ENCODER_SPI";
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
