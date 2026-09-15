/*
 * @Description: 电流环：Clarke/Park 反馈、目标选择、d/q PI 与矢量限幅
 * @Author: lkc
 *
 * 学习要点：
 *   1) 电流环是内环：输出 vd/vq (V)，不是 PWM；PWM 由上层反 Park + SVPWM。
 *   2) 对齐用 Id、不用 Iq——D 轴磁场定位，Q 轴会产生转矩让转子晃。
 *   3) 矢量限幅后要回退积分，否则饱和退饱和会超调（anti-windup）。
 *   4) 本模块不访问 HAL/无感全局；角度与速度 Iq 由调用者经 context 注入。
 */
#ifndef MOTOR_CURRENT_H
#define MOTOR_CURRENT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: d/q 轴电压指令中间量
 */
typedef struct {
    float vd; /* D 轴电压 (V)，励磁 */
    float vq; /* Q 轴电压 (V)，转矩 */
} motor_dq_voltage_t;

/**
 * @description: 电流目标选择时由调用者注入的上下文（对齐/I/F/速度环）
 */
typedef struct {
    bool alignment_running;        /* true=预对齐：Id=align_current, Iq=0 */
    float alignment_current_scale; /* Id 定角电流比例；拖转交接时由 1 降到 0 */
    float if_iq_scale;             /* I/F 低速 Iq 缩放；有感反转接管前可为 -1..0 */
    float soft_current_scale;      /* 运行电流比例；拖转交接时由 0 升到 1 */
    bool speed_loop_active;        /* true=使用速度环 Iq 替代 ol_iq_set */
    float speed_iq_target;         /* 速度环给出的 Iq (A) */
} motor_current_target_ctx_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 三相电流 → αβ → d/q，写回 motor->Ialpha/Ibeta/Id/Iq
 * @param {MOTOR_PARAM_S} *motor  含 ia/ib/ic；输出 Id/Iq (A)
 * @param {float} theta           Park 用电角度 (rad)
 * @return {*}
 * @author: lkc
 */
void Motor_Current_FeedbackUpdateDq(MOTOR_PARAM_S *motor, float theta);

/**
 * @description: 按工况选择 Id/Iq 目标
 * @detail: 对齐优先；否则 Id=ol_id_set；Iq 来自速度环或 ol_iq_set×if_iq_scale
 * @param {const MOTOR_PARAM_S} *motor              命令与对齐电流
 * @param {const motor_current_target_ctx_t} *ctx   注入上下文；NULL 视为非对齐、缩放1、不用速度环
 * @param {float} *id_target                        输出 Id 目标 (A)
 * @param {float} *iq_target                        输出 Iq 目标 (A)
 * @return {*}
 * @author: lkc
 */
void Motor_Current_GetTargets(const MOTOR_PARAM_S *motor, const motor_current_target_ctx_t *ctx, float *id_target,
                              float *iq_target);

/**
 * @description: 对 vd/vq 做合成矢量限幅，并回写 PI 积分抗饱和
 * @param {MOTOR_PARAM_S} *motor      使用 ol_current_kp 与 ol_vd_int/ol_vq_int
 * @param {motor_dq_voltage_t} *v     输入/输出电压指令 (V)
 * @param {float} id_err              Id 误差 (A)，用于反算 P 项
 * @param {float} iq_err              Iq 误差 (A)
 * @param {float} v_limit             电压矢量模长上限 (V)
 * @return {*}
 * @author: lkc
 */
void Motor_Current_LimitVoltage(MOTOR_PARAM_S *motor, motor_dq_voltage_t *v, float id_err, float iq_err, float v_limit);

/**
 * @description: 运行 d/q 双轴电流 PI，并做矢量限幅
 * @param {MOTOR_PARAM_S} *motor  含 Id/Iq 反馈与积分、增益
 * @param {float} id_target       Id 目标 (A)
 * @param {float} iq_target       Iq 目标 (A)
 * @param {float} dt              控制周期 (s)
 * @param {float} v_limit         单轴与矢量限幅参考 (V)
 * @return {motor_dq_voltage_t} 限幅后的 vd/vq (V)
 * @author: lkc
 */
motor_dq_voltage_t Motor_Current_RunPi(MOTOR_PARAM_S *motor, float id_target, float iq_target, float dt, float v_limit);

/**
 * @description: 电流环一步：反馈变换 + 取目标 + PI（不含 PWM）
 * @param {MOTOR_PARAM_S} *motor                    电机采样与积分
 * @param {float} theta                             Park/反 Park 用电角度 (rad)
 * @param {const motor_current_target_ctx_t} *ctx   目标上下文
 * @param {float} dt                                控制周期 (s)
 * @param {float} v_limit                           电压限幅 (V)
 * @param {float} *id_target_out                    可选，回写本拍 Id 目标；可为 NULL
 * @param {float} *iq_target_out                    可选，回写本拍 Iq 目标；可为 NULL
 * @return {motor_dq_voltage_t} vd/vq (V)
 * @author: lkc
 */
motor_dq_voltage_t Motor_Current_Step(MOTOR_PARAM_S *motor, float theta, const motor_current_target_ctx_t *ctx,
                                      float dt, float v_limit, float *id_target_out, float *iq_target_out);

/**
 * @description: 按电角速度叠加 PMSM 交叉解耦与反电势前馈，再做矢量限幅。
 * @detail: 教科书 vd -= ωe·L·Iq*、vq += ωe·L·Id* + ωe·ψf。只用于转子 dq（编码器/霍尔）。
 *          I/F 开环角不得调用：反电势不在该坐标系。不改 PI 积分。
 * @param {motor_dq_voltage_t *} v     输入/输出电压 (V)，不可为空
 * @param {float} omega_e              电角速度 (rad/s)
 * @param {float} inductance           相电感 (H)
 * @param {float} flux                 永磁磁链 (Wb)
 * @param {float} id_ref               Id 参考 (A)
 * @param {float} iq_ref               Iq 参考 (A)
 * @param {float} v_limit              电压矢量模长上限 (V)；≤0 则不限幅
 * @return {*}
 */
void Motor_Current_ApplySpeedFeedforward(motor_dq_voltage_t *v, float omega_e, float inductance, float flux,
                                         float id_ref, float iq_ref, float v_limit);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
