/*
 * @Description: 电流环：Clarke/Park、目标选择、d/q PI 与矢量限幅
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>

#include "motor_current.h"
#include "motor_foc.h"
#include "motor_math.h"
#include "motor_pid.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 三相电流 → αβ → d/q，写回 motor 反馈字段
 * @param {MOTOR_PARAM_S} *motor  含 ia/ib/ic
 * @param {float} theta           Park 用电角度 (rad)
 * @return {*}
 * @author: lkc
 */
void Motor_Current_FeedbackUpdateDq(MOTOR_PARAM_S *motor, float theta)
{
    if (motor == NULL) {
        return;
    }
    clarke_transform(motor->ia, motor->ib, motor->ic, &motor->Ialpha, &motor->Ibeta);
    park_transform(motor->Ialpha, motor->Ibeta, theta, &motor->Id, &motor->Iq);
}

/**
 * @description: 按工况选择 Id/Iq 目标。
 * @param {const MOTOR_PARAM_S} * motor  命令与对齐电流
 * @param {const motor_current_target_ctx_t} * ctx  注入上下文
 * @param {float} * id_target  输出 Id (A)
 * @param {float} * iq_target  输出 Iq (A)
 * @return {*}
 * @author: lkc
 */
void Motor_Current_GetTargets(const MOTOR_PARAM_S *motor, const motor_current_target_ctx_t *ctx,
                              float *id_target, float *iq_target)
{
    motor_current_target_ctx_t local;

    if (motor == NULL || id_target == NULL || iq_target == NULL) {
        return;
    }

    if (ctx == NULL) {
        local.alignment_running = false;
        local.if_iq_scale = 1.0f;
        local.speed_loop_active = false;
        local.speed_iq_target = 0.0f;
        ctx = &local;
    }

    if (ctx->alignment_running) {
        /*
         * 对齐必须使用 D 轴电流而不是 Q 轴电流。Q 轴电流负责转矩，会让
         * 转子继续摆动；D 轴磁场固定后，转子会自行吸到已知电角位置。
         */
        *id_target = motor->ol_align_current;
        *iq_target = 0.0f;
        return;
    }

    *id_target = motor->ol_id_set;
    if (ctx->speed_loop_active) {
        /* 无感闭环建立后，速度 PI 的输出作为电流内环 Iq 目标。 */
        *iq_target = ctx->speed_iq_target;
    } else {
        /* 观测器锁定前必须保留 I/F 启动电流，否则无感电机无法从零速起转。 */
        *iq_target = motor->ol_iq_set * ctx->if_iq_scale;
    }
}

/**
 * @description: vd/vq 合成矢量限幅 + 积分抗饱和。
 * @param {MOTOR_PARAM_S} * motor  增益与积分
 * @param {motor_dq_voltage_t} * v  电压指令
 * @param {float} id_err  Id 误差 (A)
 * @param {float} iq_err  Iq 误差 (A)
 * @param {float} v_limit  矢量模长上限 (V)
 * @return {*}
 * @author: lkc
 */
void Motor_Current_LimitVoltage(MOTOR_PARAM_S *motor, motor_dq_voltage_t *v, float id_err, float iq_err,
                                float v_limit)
{
    if (motor == NULL || v == NULL) {
        return;
    }

    /*
     * 单独限制 vd/vq 不够，合成矢量可能超限。
     * 这里按矢量幅值整体缩放，并回写积分项，避免 windup。
     */
    if (utils_saturate_vector_2d(&v->vd, &v->vq, v_limit)) {
        motor->ol_vd_int = v->vd - id_err * motor->ol_current_kp;
        motor->ol_vq_int = v->vq - iq_err * motor->ol_current_kp;
    }
}

/**
 * @description: d/q 双轴电流 PI + 矢量限幅。
 * @param {MOTOR_PARAM_S} * motor  反馈与积分
 * @param {float} id_target  Id 目标 (A)
 * @param {float} iq_target  Iq 目标 (A)
 * @param {float} dt  控制周期 (s)
 * @param {float} v_limit  限幅 (V)
 * @return {motor_dq_voltage_t} vd/vq (V)
 * @author: lkc
 */
motor_dq_voltage_t Motor_Current_RunPi(MOTOR_PARAM_S *motor, float id_target, float iq_target, float dt,
                                       float v_limit)
{
    motor_dq_voltage_t v = {0.0f, 0.0f};

    if (motor == NULL) {
        return v;
    }

    v.vd = Motor_Current_PI_Run(id_target, motor->Id, &motor->ol_vd_int, motor->ol_current_kp,
                                motor->ol_current_ki, dt, v_limit);
    v.vq = Motor_Current_PI_Run(iq_target, motor->Iq, &motor->ol_vq_int, motor->ol_current_kp,
                                motor->ol_current_ki, dt, v_limit);

    Motor_Current_LimitVoltage(motor, &v, id_target - motor->Id, iq_target - motor->Iq, v_limit);
    return v;
}

/**
 * @description: 电流环一步：反馈 + 目标 + PI（不含 PWM）。
 * @param {MOTOR_PARAM_S} * motor  电机对象
 * @param {float} theta  电角度 (rad)
 * @param {const motor_current_target_ctx_t} * ctx  目标上下文
 * @param {float} dt  控制周期 (s)
 * @param {float} v_limit  电压限幅 (V)
 * @param {float} * id_target_out  可选输出 Id 目标
 * @param {float} * iq_target_out  可选输出 Iq 目标
 * @return {motor_dq_voltage_t} vd/vq (V)
 * @author: lkc
 */
motor_dq_voltage_t Motor_Current_Step(MOTOR_PARAM_S *motor, float theta, const motor_current_target_ctx_t *ctx,
                                      float dt, float v_limit, float *id_target_out, float *iq_target_out)
{
    float id_target = 0.0f;
    float iq_target = 0.0f;
    motor_dq_voltage_t voltage = {0.0f, 0.0f};

    if (motor == NULL) {
        return voltage;
    }

    Motor_Current_FeedbackUpdateDq(motor, theta);
    Motor_Current_GetTargets(motor, ctx, &id_target, &iq_target);
    if (id_target_out != NULL) {
        *id_target_out = id_target;
    }
    if (iq_target_out != NULL) {
        *iq_target_out = iq_target;
    }
    return Motor_Current_RunPi(motor, id_target, iq_target, dt, v_limit);
}

#ifdef __cplusplus
}
#endif
