/*
 * @Description: 电流环：Clarke/Park、目标选择、d/q PI 与矢量限幅
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "motor_current.h"
#include "motor_foc.h"
#include "motor_math.h"
#include "motor_pid.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t s_recon_last;     /* 0=未重构，1=A，2=B，3=C */
static uint32_t s_recon_samples; /* 本档调用次数 */
static uint32_t s_recon_hits;    /* 本档重构次数 */
static uint32_t s_recon_hit_a;
static uint32_t s_recon_hit_b;
static uint32_t s_recon_hit_c;
/* Private function prototypes -----------------------------------------------*/
static void motor_current_account_recon(uint8_t phase);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 累计一拍最长零矢量重构结果
 * @param {uint8_t} phase  0=未重构，1=A，2=B，3=C
 * @return {*}
 * @author: lkc
 */
static void motor_current_account_recon(uint8_t phase)
{
    s_recon_last = phase;
    if (s_recon_samples < UINT32_MAX) {
        s_recon_samples++;
    }
    if (phase == 0U) {
        return;
    }
    if (s_recon_hits < UINT32_MAX) {
        s_recon_hits++;
    }
    if (phase == 1U && s_recon_hit_a < UINT32_MAX) {
        s_recon_hit_a++;
    } else if (phase == 2U && s_recon_hit_b < UINT32_MAX) {
        s_recon_hit_b++;
    } else if (phase == 3U && s_recon_hit_c < UINT32_MAX) {
        s_recon_hit_c++;
    }
}

/**
 * @description: 清零最长零矢量重构诊断
 * @return {*}
 * @author: lkc
 */
void Motor_Current_ResetReconstructDiag(void)
{
    s_recon_last = 0U;
    s_recon_samples = 0U;
    s_recon_hits = 0U;
    s_recon_hit_a = 0U;
    s_recon_hit_b = 0U;
    s_recon_hit_c = 0U;
}

/**
 * @description: 读取上一拍重构相及本档累计次数
 * @param {int} *last_phase     0=未重构，1=A，2=B，3=C；可为 NULL
 * @param {uint32_t} *samples   本档调用次数；可为 NULL
 * @param {uint32_t} *hits      本档重构次数；可为 NULL
 * @param {uint32_t} *hit_a     A 相重构次数；可为 NULL
 * @param {uint32_t} *hit_b     B 相重构次数；可为 NULL
 * @param {uint32_t} *hit_c     C 相重构次数；可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Current_GetReconstructDiag(int *last_phase, uint32_t *samples, uint32_t *hits, uint32_t *hit_a,
                                      uint32_t *hit_b, uint32_t *hit_c)
{
    if (last_phase != NULL) {
        *last_phase = (int)s_recon_last;
    }
    if (samples != NULL) {
        *samples = s_recon_samples;
    }
    if (hits != NULL) {
        *hits = s_recon_hits;
    }
    if (hit_a != NULL) {
        *hit_a = s_recon_hit_a;
    }
    if (hit_b != NULL) {
        *hit_b = s_recon_hit_b;
    }
    if (hit_c != NULL) {
        *hit_c = s_recon_hit_c;
    }
}

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
 * @description: 高占空比时重构采样窗口最短的那一相电流。
 * @detail: 对齐 VESC LONGEST_ZERO。任一相比 ARR 高过门槛时，把唯一最大 duty 的相写成
 *          −(另外两相)。duty 为软件 A/B/C 计数，须与 ia/ib/ic 同相序。pwm_period 不大于
 *          门槛、或三相窗口都够、或两相并列最大时不改。不访问 HAL。调用方在过流判定之后、
 *          Clarke 之前使用。
 * @param {float} *ia            A 相电流 (A)，可被改写
 * @param {float} *ib            B 相电流 (A)，可被改写
 * @param {float} *ic            C 相电流 (A)，可被改写
 * @param {uint32_t} duty_a      A 相 PWM 比较值（计数）
 * @param {uint32_t} duty_b      B 相 PWM 比较值（计数）
 * @param {uint32_t} duty_c      C 相 PWM 比较值（计数）
 * @param {uint32_t} pwm_period  PWM 周期 ARR（计数）
 * @return {bool} true=已改写其中一相
 * @author: lkc
 */
bool Motor_Current_ReconstructLongestZero(float *ia, float *ib, float *ic, uint32_t duty_a, uint32_t duty_b,
                                          uint32_t duty_c, uint32_t pwm_period)
{
    uint32_t limit;
    uint8_t phase = 0U;

    if (ia == NULL || ib == NULL || ic == NULL) {
        return false;
    }
    if (pwm_period <= MOTOR_CURRENT_SHUNT_PICK_COUNTS) {
        motor_current_account_recon(0U);
        return false;
    }

    limit = pwm_period - MOTOR_CURRENT_SHUNT_PICK_COUNTS;
    if (duty_a <= limit && duty_b <= limit && duty_c <= limit) {
        motor_current_account_recon(0U);
        return false;
    }

    /* 与 VESC 相同：只在唯一最大 CCR 时重构，两相并列最大则保持原样。 */
    if (duty_a > duty_b && duty_a > duty_c) {
        *ia = -(*ib + *ic);
        phase = 1U;
    } else if (duty_b > duty_a && duty_b > duty_c) {
        *ib = -(*ia + *ic);
        phase = 2U;
    } else if (duty_c > duty_a && duty_c > duty_b) {
        *ic = -(*ia + *ib);
        phase = 3U;
    }
    motor_current_account_recon(phase);
    return phase != 0U;
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
void Motor_Current_GetTargets(const MOTOR_PARAM_S *motor, const motor_current_target_ctx_t *ctx, float *id_target,
                              float *iq_target)
{
    motor_current_target_ctx_t local;

    if (motor == NULL || id_target == NULL || iq_target == NULL) {
        return;
    }

    if (ctx == NULL) {
        memset(&local, 0, sizeof(local));
        local.if_iq_scale = 1.0f;
        local.soft_current_scale = 1.0f;
        ctx = &local;
    }

    if (ctx->alignment_running) {
        /*
         * 对齐 / 软起定角必须用 D 轴电流。Q 轴会产转矩并在定角上把电压积分顶满。
         * align_current≤0 时回退到 |ol_iq_set|，保证软起仍有吸合力。
         */
        float id_mag = motor->ol_align_current;
        if (id_mag <= 0.0f) {
            id_mag = fabsf(motor->ol_iq_set);
        }
        *id_target = id_mag * ctx->soft_current_scale;
        *iq_target = 0.0f;
        return;
    }

    *id_target = motor->ol_align_current * ctx->alignment_current_scale + motor->ol_id_set * ctx->soft_current_scale;
    *id_target += ctx->id_fw;
    if (ctx->speed_loop_active) {
        /* 无感闭环建立后，速度 PI 的输出作为电流内环 Iq 目标。 */
        *iq_target = ctx->speed_iq_target * ctx->soft_current_scale;
    } else {
        /* 观测器锁定前必须保留 I/F 启动电流，否则无感电机无法从零速起转。 */
        *iq_target = motor->ol_iq_set * ctx->if_iq_scale * ctx->soft_current_scale;
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
void Motor_Current_LimitVoltage(MOTOR_PARAM_S *motor, motor_dq_voltage_t *v, float id_err, float iq_err, float v_limit)
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
motor_dq_voltage_t Motor_Current_RunPi(MOTOR_PARAM_S *motor, float id_target, float iq_target, float dt, float v_limit)
{
    motor_dq_voltage_t v = {0.0f, 0.0f};

    if (motor == NULL) {
        return v;
    }

    v.vd = Motor_Current_PI_Run(id_target, motor->Id, &motor->ol_vd_int, motor->ol_current_kp, motor->ol_current_ki, dt,
                                v_limit);
    v.vq = Motor_Current_PI_Run(iq_target, motor->Iq, &motor->ol_vq_int, motor->ol_current_kp, motor->ol_current_ki, dt,
                                v_limit);

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

/**
 * @description: 按电角速度叠加 PMSM 交叉解耦与反电势前馈，再做矢量限幅。
 * @detail: 教科书符号；I/F 开环路径不调用。
 * @param {motor_dq_voltage_t *} v     输入/输出电压 (V)
 * @param {float} omega_e              电角速度 (rad/s)
 * @param {float} inductance           相电感 (H)
 * @param {float} flux                 永磁磁链 (Wb)
 * @param {float} id_ref               Id 参考 (A)
 * @param {float} iq_ref               Iq 参考 (A)
 * @param {float} v_limit              电压矢量模长上限 (V)
 * @return {*}
 */
void Motor_Current_ApplySpeedFeedforward(motor_dq_voltage_t *v, float omega_e, float inductance, float flux,
                                         float id_ref, float iq_ref, float v_limit)
{
    if (v == NULL) {
        return;
    }
    if (!isfinite(omega_e) || !isfinite(inductance) || !isfinite(flux) || !isfinite(id_ref) || !isfinite(iq_ref)) {
        return;
    }
    if (inductance < 0.0f) {
        inductance = 0.0f;
    }
    if (flux < 0.0f) {
        flux = 0.0f;
    }

    /* vd 去耦 −ωe L Iq；vq 去耦 +ωe L Id，并补偿 ωe ψf（反电势）。仅转子对齐的 dq 可调用。 */
    v->vd -= omega_e * inductance * iq_ref;
    v->vq += omega_e * inductance * id_ref + omega_e * flux;

    if (v_limit > 0.0f) {
        (void)utils_saturate_vector_2d(&v->vd, &v->vq, v_limit);
    }
}

#ifdef __cplusplus
}
#endif
