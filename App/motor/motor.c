/*
 * @Description: 电机公共 API（命令提交、状态快照、调试打印）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motor.h"
#include "motor_param.h"
#include "motor_settings.h"
#include "motor_protect.h"
#include "motor_speed.h"
#include "motor_bldc.h"
#include "hal_motor.h"
#include "motor_feedback.h"
#include "motor_crit.h"
#include "motor_anticogging.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static bool motor_is_finite(float value);
static bool motor_command_valid(const MOTOR_PARAM_S *motor, const motor_command_t *cmd, motor_command_t *normalized);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 判断浮点是否为有限值（非 NaN/Inf）。
 * @param {float} value 待检查值
 * @return {bool} true=有限
 * @author: lkc
 */
static bool motor_is_finite(float value)
{
    return isfinite(value) != 0;
}

/**
 * @description: 校验命令并输出限幅后的规范化副本；失败不改 normalized。
 * @param {const MOTOR_PARAM_S *} motor 配置来源，不可为空
 * @param {const motor_command_t *} cmd 原始命令，不可为空
 * @param {motor_command_t *} normalized 输出规范化命令，不可为空；失败不修改
 * @return {bool} true=命令合法
 * @author: lkc
 */
static bool motor_command_valid(const MOTOR_PARAM_S *motor, const motor_command_t *cmd, motor_command_t *normalized)
{
    motor_command_t out;

    if (motor == NULL || cmd == NULL || normalized == NULL) {
        return false;
    }

    out = *cmd;
    if (out.mode != MOTOR_OL_MODE_VF && out.mode != MOTOR_OL_MODE_IF) {
        return false;
    }
    if (!motor_is_finite(out.target_rpm) || !motor_is_finite(out.iq_set) || !motor_is_finite(out.id_set)) {
        return false;
    }
    if (motor->ol_max_rpm > 0.0f && motor_is_finite(motor->ol_max_rpm)) {
        if (out.target_rpm > motor->ol_max_rpm) {
            out.target_rpm = motor->ol_max_rpm;
        } else if (out.target_rpm < -motor->ol_max_rpm) {
            out.target_rpm = -motor->ol_max_rpm;
        }
    }

    *normalized = out;
    return true;
}

/**
 * @description: 从电机对象捕获当前命令字段。
 * @detail: 短临界区拷贝；不修改电机状态。可在任务上下文调用。
 * @param {const MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {motor_command_t *} cmd 输出命令，不可为空
 * @return {motor_result_t} MOTOR_OK 或 MOTOR_ERR_NULL
 * @author: lkc
 */
motor_result_t Motor_GetCommand(const MOTOR_PARAM_S *motor, motor_command_t *cmd)
{
    uint32_t primask;

    if (motor == NULL || cmd == NULL) {
        return MOTOR_ERR_NULL;
    }

    primask = Motor_Crit_Enter();
    cmd->enable = motor->ol_enable;
    cmd->mode = motor->ol_mode;
    cmd->target_rpm = motor->ol_target_rpm;
    cmd->iq_set = motor->ol_iq_set;
    cmd->id_set = motor->ol_id_set;
    cmd->speed_enable = motor->speed_enable;
    Motor_Crit_Exit(primask);
    return MOTOR_OK;
}

/**
 * @description: 校验并整体提交命令到电机对象。
 * @detail: 拒绝 NaN/Inf 与非法模式；Iq 符号表示转矩方向；按 ol_max_rpm 限幅目标转速。
 *          短临界区写入 enable/mode/目标电流/速度环开关。模式变化或停机时清零
 *          电流积分与斜坡转速，并复位速度环状态。不写 Flash。
 *          成功后按 enable 同步功率桥：使能则 PwmStart，停机则 PwmStop
 *         （Shell 不应再直接写 PWM）。
 * @param {MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {const motor_command_t *} cmd 完整命令，不可为空
 * @return {motor_result_t} MOTOR_OK / MOTOR_ERR_NULL / MOTOR_ERR_INVALID
 * @author: lkc
 */
motor_result_t Motor_SetCommand(MOTOR_PARAM_S *motor, const motor_command_t *cmd)
{
    motor_command_t normalized;
    uint32_t primask;
    bool mode_changed;
    bool stopping;
    bool speed_changed;

    if (motor == NULL || cmd == NULL) {
        return MOTOR_ERR_NULL;
    }
    if (!motor_command_valid(motor, cmd, &normalized)) {
        return MOTOR_ERR_INVALID;
    }
    if (normalized.enable && Motor_Bldc_IsSelected()) {
        return MOTOR_ERR_BUSY;
    }

    primask = Motor_Crit_Enter();
    mode_changed = (motor->ol_mode != normalized.mode);
    stopping = !normalized.enable;
    speed_changed = (motor->speed_enable != normalized.speed_enable);
    motor->ol_enable = normalized.enable;
    motor->ol_mode = normalized.mode;
    motor->ol_target_rpm = normalized.target_rpm;
    motor->ol_iq_set = normalized.iq_set;
    motor->ol_id_set = normalized.id_set;
    motor->speed_enable = normalized.speed_enable;
    Motor_Crit_Exit(primask);

    if (mode_changed || stopping) {
        (void)Motor_ClearRuntimeControl(motor);
    }

    if (mode_changed || stopping || speed_changed) {
        Motor_SpeedLoop_Reset();
    }

    /* 应用层命令提交后同步桥臂；占空比仍由 ISR/控制步进决定。 */
    if (normalized.enable) {
        Hal_Motor_PwmStart();
    } else {
        /* 旋转中三相 50% 同步 PWM 会形成主动短路制动；停机命令应关断桥臂。 */
        Hal_Motor_PwmStop();
    }
    return MOTOR_OK;
}

/**
 * @description: 提交停机命令并清除斜坡/电流积分（不清故障锁存）。
 * @detail: 仅令 enable=false，保留 target_rpm/mode/iq/id/speed_enable，便于再次启动沿用目标。
 * @param {MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @return {motor_result_t} MOTOR_OK 或错误码
 * @author: lkc
 */
motor_result_t Motor_RequestStop(MOTOR_PARAM_S *motor)
{
    motor_command_t cmd;
    motor_result_t result;

    result = Motor_GetCommand(motor, &cmd);
    if (result != MOTOR_OK) {
        return result;
    }
    cmd.enable = false;
    Motor_Anticogging_Stop();
    return Motor_SetCommand(motor, &cmd);
}

/**
 * @description: 从电机对象拷贝运行时快照（样本/斜坡/积分等）。
 * @detail: 短临界区；不改电机状态；不包含配置与命令目标。
 * @param {const MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {motor_runtime_t *} runtime 输出快照，不可为空
 * @return {motor_result_t} MOTOR_OK 或 MOTOR_ERR_NULL
 * @author: lkc
 */
motor_result_t Motor_GetRuntime(const MOTOR_PARAM_S *motor, motor_runtime_t *runtime)
{
    uint32_t primask;

    if (motor == NULL || runtime == NULL) {
        return MOTOR_ERR_NULL;
    }

    primask = Motor_Crit_Enter();
    memcpy(runtime->currentAdc, motor->currentAdc, sizeof(runtime->currentAdc));
    memcpy(runtime->voltAdc, motor->voltAdc, sizeof(runtime->voltAdc));
    runtime->ia = motor->ia;
    runtime->ib = motor->ib;
    runtime->ic = motor->ic;
    runtime->va = motor->va;
    runtime->vb = motor->vb;
    runtime->vc = motor->vc;
    runtime->v_bus = motor->v_bus;
    runtime->Ialpha = motor->Ialpha;
    runtime->Ibeta = motor->Ibeta;
    runtime->Id = motor->Id;
    runtime->Iq = motor->Iq;
    runtime->theta = motor->theta;
    runtime->e_err_ki_sum = motor->e_err_ki_sum;
    runtime->e_iq_out = motor->e_iq_out;
    runtime->e_id_out = motor->e_id_out;
    runtime->modD = motor->modD;
    runtime->modQ = motor->modQ;
    runtime->modAlpha = motor->modAlpha;
    runtime->modBeta = motor->modBeta;
    runtime->duty1 = motor->duty1;
    runtime->duty2 = motor->duty2;
    runtime->duty3 = motor->duty3;
    runtime->curHallABC = motor->curHallABC;
    runtime->lastHallABC = motor->lastHallABC;
    runtime->ol_current_rpm = motor->ol_current_rpm;
    runtime->ol_theta = motor->ol_theta;
    runtime->ol_vd_int = motor->ol_vd_int;
    runtime->ol_vq_int = motor->ol_vq_int;
    Motor_Crit_Exit(primask);
    return MOTOR_OK;
}

/**
 * @description: 清除控制用运行时状态（斜坡转速与 d/q 积分），不改命令与配置。
 * @detail: 短临界区；不清除采样值与校准。
 * @param {MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @return {motor_result_t} MOTOR_OK 或 MOTOR_ERR_NULL
 * @author: lkc
 */
motor_result_t Motor_ClearRuntimeControl(MOTOR_PARAM_S *motor)
{
    uint32_t primask;

    if (motor == NULL) {
        return MOTOR_ERR_NULL;
    }

    primask = Motor_Crit_Enter();
    motor->ol_current_rpm = 0.0f;
    motor->ol_vd_int = 0.0f;
    motor->ol_vq_int = 0.0f;
    Motor_Crit_Exit(primask);
    return MOTOR_OK;
}

/**
 * @description: 校验并应用配置（不改校准与运行命令目标）。
 * @detail: 委托 Motor_Settings_ApplyConfig；停机字段在运行中返回 MOTOR_ERR_BUSY。
 *          成功后按新 max_rpm 重新限幅当前命令目标。不写 Flash。
 * @param {MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {const motor_config_t *} config 完整配置快照，不可为空
 * @return {motor_result_t} MOTOR_OK / NULL / INVALID / BUSY
 * @author: lkc
 */
motor_result_t Motor_SetConfig(MOTOR_PARAM_S *motor, const motor_config_t *config)
{
    motor_settings_t current;
    motor_command_t cmd;

    if (motor == NULL || config == NULL) {
        return MOTOR_ERR_NULL;
    }
    if (!Motor_Settings_Capture(motor, &current)) {
        return MOTOR_ERR_INVALID;
    }
    if (motor->ol_enable && Motor_Settings_ConfigRequiresStop(&current.config, config)) {
        return MOTOR_ERR_BUSY;
    }
    if (!Motor_Settings_ApplyConfig(motor, config)) {
        return MOTOR_ERR_INVALID;
    }

    /* 配置限幅变化后，用 SetCommand 再次整体提交以夹紧目标转速。 */
    if (Motor_GetCommand(motor, &cmd) == MOTOR_OK) {
        (void)Motor_SetCommand(motor, &cmd);
    }
    return MOTOR_OK;
}

/**
 * @description: 设置转子反馈来源。
 * @detail: 委托 Motor_Feedback_SetSourceDefault；I/F 下编码器/霍尔在角有效时走有感电流环。
 * @param {motor_feedback_source_t} source 反馈源
 * @return {motor_result_t} MOTOR_OK；非法枚举返回 MOTOR_ERR_INVALID
 * @author: lkc
 */
motor_result_t Motor_SetFeedbackSource(motor_feedback_source_t source)
{
    if (source != MOTOR_FEEDBACK_SRC_OPENLOOP && source != MOTOR_FEEDBACK_SRC_SENSORLESS &&
        source != MOTOR_FEEDBACK_SRC_ENCODER && source != MOTOR_FEEDBACK_SRC_HALL) {
        return MOTOR_ERR_INVALID;
    }
    Motor_Feedback_SetSourceDefault(source);
    return MOTOR_OK;
}

/**
 * @description: 读取当前请求的反馈来源。
 * @return {motor_feedback_source_t} 当前源
 * @author: lkc
 */
motor_feedback_source_t Motor_GetFeedbackSource(void)
{
    return Motor_Feedback_GetSourceDefault();
}

/**
 * @description: 显示电机参数
 * @detail:
 * @return {*}
 * @author: lkc
 */
void MOTOR_ShowParam(void)
{
    MOTOR_PARAM_S *stMotorNow = &motor;

    printf("Current Offset [%f] [%f] [%f]\r\n", stMotorNow->currentOffset[0], stMotorNow->currentOffset[1],
           stMotorNow->currentOffset[2]);
    printf("Voltage Offset [%f] [%f] [%f]\r\n", stMotorNow->voltOffset[0], stMotorNow->voltOffset[1],
           stMotorNow->voltOffset[2]);
    printf("Volt Un Offset [%f] [%f] [%f]\r\n", stMotorNow->voltundrivenOffset[0], stMotorNow->voltundrivenOffset[1],
           stMotorNow->voltundrivenOffset[2]);
}
#ifdef __cplusplus
}
#endif
