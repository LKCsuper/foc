/*
 * @Description: 电流环：Clarke/Park、目标选择、d/q PI 与矢量限幅
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "common_assert.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "motor_current.h"
#include "motor_foc.h"
#include "motor_math.h"
#include "motor_pid.h"
#include "motor_crit.h"
#include "motor_timing.h"
#if defined(__CC_ARM)
/* 仅提高电流控制计算优化等级；保留浮点规则、PI公式和采样基线。 */
#pragma O2
#endif
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
static uint32_t s_pi_samples;
static uint32_t s_pi_clamp_samples;
static float s_pi_iq_err_sum;
static float s_pi_iq_err_abs_sum;
static float s_pi_vq_int_delta;
static float s_pi_ki;
static float s_pi_dt;
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
    if (phase == 0U) return;
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
    COMMON_ASSERT(motor != NULL);
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

    if (ia == NULL || ib == NULL || ic == NULL) return false;
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

    COMMON_ASSERT(motor != NULL && id_target != NULL && iq_target != NULL);

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
    COMMON_ASSERT(motor != NULL && v != NULL);

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
 * @detail: Iq 误差在 PI 前取样，限幅抗饱和和钳位统计共用这一拍。PI 不改 Id/Iq。
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

    float iq_err;
    float vq_int_before;
    float vq_step;
    bool profile;
    uint32_t stage_start;

    if (motor == NULL) return v;

    iq_err = iq_target - motor->Iq;
    vq_int_before = motor->ol_vq_int;
    profile = Motor_Timing_ProfileDetailedIsActive();
    stage_start = profile ? Motor_Timing_ProfileTimeUs() : 0U;
    v.vd = Motor_Current_PI_Run(id_target, motor->Id, &motor->ol_vd_int, motor->ol_current_kp, motor->ol_current_ki, dt,
                                v_limit);
    v.vq = Motor_Current_PI_Run(iq_target, motor->Iq, &motor->ol_vq_int, motor->ol_current_kp, motor->ol_current_ki, dt,
                                v_limit);
    if (profile) Motor_Timing_ProfileRecord(MOTOR_TIMING_CURRENT_PI, stage_start);

    stage_start = profile ? Motor_Timing_ProfileTimeUs() : 0U;
    Motor_Current_LimitVoltage(motor, &v, id_target - motor->Id, iq_err, v_limit);
    if (profile) Motor_Timing_ProfileRecord(MOTOR_TIMING_CURRENT_LIMIT, stage_start);
    vq_step = motor->ol_vq_int - vq_int_before;
    if (s_pi_samples < UINT32_MAX) {
        s_pi_samples++;
    }
    s_pi_iq_err_sum += iq_err;
    s_pi_iq_err_abs_sum += fabsf(iq_err);
    s_pi_vq_int_delta += vq_step;
    s_pi_ki = motor->ol_current_ki;
    s_pi_dt = dt;
    if (fabsf(vq_step - iq_err * motor->ol_current_ki * dt) > 1.0e-4f) {
        if (s_pi_clamp_samples < UINT32_MAX) {
            s_pi_clamp_samples++;
        }
    }
    return v;
}

/**
 * @description: 读出并清零 q 轴电流 PI 统计
 * @detail: 关中断后复制，避免和 FOC 中断同时改累加器。samples 为 0 时平均值记 0。
 * @param {motor_current_pi_diag_t *} diag  输出统计，不可为空
 * @return {*}
 * @author: lkc
 */
void Motor_Current_TakePiDiag(motor_current_pi_diag_t *diag)
{
    uint32_t primask;

    COMMON_ASSERT(diag != NULL);
    primask = Motor_Crit_Enter();
    diag->samples = s_pi_samples;
    diag->clamp_samples = s_pi_clamp_samples;
    diag->vq_int_delta = s_pi_vq_int_delta;
    diag->ki = s_pi_ki;
    diag->dt = s_pi_dt;
    diag->iq_err_avg = (s_pi_samples > 0U) ? (s_pi_iq_err_sum / (float)s_pi_samples) : 0.0f;
    diag->iq_err_abs_avg = (s_pi_samples > 0U) ? (s_pi_iq_err_abs_sum / (float)s_pi_samples) : 0.0f;
    s_pi_samples = 0U;
    s_pi_clamp_samples = 0U;
    s_pi_iq_err_sum = 0.0f;
    s_pi_iq_err_abs_sum = 0.0f;
    s_pi_vq_int_delta = 0.0f;
    Motor_Crit_Exit(primask);
}

/**
 * @description: 坐标交接时旋转电流 PI 积分，并补偿目标变化的 P 项。
 * @detail: 只用于无感渐变等坐标交接。delta 为新电角减旧电角 (rad)，不得传入正常转速积分角。
 *          由电流环在矢量限幅前调用，不改 PWM。motor 为空或 delta 非有限值时直接返回。
 * @param {MOTOR_PARAM_S *} motor 电机对象，读写 ol_vd_int/ol_vq_int (V)
 * @param {float} delta           交接附加角 (rad)
 * @param {float} old_id          旧坐标 Id 目标 (A)
 * @param {float} old_iq          旧坐标 Iq 目标 (A)
 * @param {float} new_id          新坐标 Id 目标 (A)
 * @param {float} new_iq          新坐标 Iq 目标 (A)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Current_TransportPi(MOTOR_PARAM_S *motor, float delta, float old_id, float old_iq,
                               float new_id, float new_iq)
{
    float c, s, vd, vq;
    COMMON_ASSERT(motor != NULL);
    if (!isfinite(delta)) return;
    c = cos_f32(delta);
    s = sin_f32(delta);
    vd = motor->ol_vd_int;
    vq = motor->ol_vq_int;
    /* R(-delta): 同一静止电压矢量在新dq轴上的坐标。反馈也按同一旋转变化，
     * 因此只需补偿旧/新目标P项，不能清零积分或重新注入启动电流。 */
    motor->ol_vd_int = c * vd + s * vq + motor->ol_current_kp * (c * old_id + s * old_iq - new_id);
    motor->ol_vq_int = -s * vd + c * vq + motor->ol_current_kp * (-s * old_id + c * old_iq - new_iq);
}

/**
 * @description: 电流环一步：反馈变换、取目标、可选坐标交接、PI 及矢量限幅。
 * @detail: 不含 PWM。ctx->frame_correction 非 0 时先旋转积分，再跑 PI 并限幅。
 * @param {MOTOR_PARAM_S *} motor                   电机采样与积分
 * @param {float} theta                             Park 用电角度 (rad)
 * @param {const motor_current_target_ctx_t *} ctx  目标上下文
 * @param {float} dt                                控制周期 (s)
 * @param {float} v_limit                           电压限幅 (V)
 * @param {float *} id_target_out                   可选，回写本拍 Id 目标 (A)；可为 NULL
 * @param {float *} iq_target_out                   可选，回写本拍 Iq 目标 (A)；可为 NULL
 * @return {motor_dq_voltage_t} 限幅后的 vd/vq (V)
 * @author: lkc
 */
motor_dq_voltage_t Motor_Current_Step(MOTOR_PARAM_S *motor, float theta, const motor_current_target_ctx_t *ctx,
                                      float dt, float v_limit, float *id_target_out, float *iq_target_out)
{
    float id_target = 0.0f;
    float iq_target = 0.0f;
    motor_dq_voltage_t voltage = {0.0f, 0.0f};
    bool profile;
    bool capture;
    uint32_t stage_start;

    if (motor == NULL) return voltage;

    profile = Motor_Timing_ProfileDetailedIsActive();
    stage_start = profile ? Motor_Timing_ProfileTimeUs() : 0U;
    Motor_Current_FeedbackUpdateDq(motor, theta);
    if (profile) Motor_Timing_ProfileRecord(MOTOR_TIMING_CURRENT_PARK, stage_start);
    stage_start = profile ? Motor_Timing_ProfileTimeUs() : 0U;
    Motor_Current_GetTargets(motor, ctx, &id_target, &iq_target);
    capture = Motor_Timing_HandoffDue();
    if (capture) Motor_Timing_HandoffBefore(motor, theta, ctx != NULL ? ctx->frame_correction : 0.0f,
                                           id_target, iq_target, dt);
    if (profile) Motor_Timing_ProfileRecord(MOTOR_TIMING_CURRENT_TARGET, stage_start);
    if (ctx != NULL && ctx->frame_correction != 0.0f) {
        bool profile = Motor_Timing_ProfileDetailedIsActive();
        uint32_t start = profile ? Motor_Timing_ProfileTimeUs() : 0U;
        Motor_Current_TransportPi(motor, ctx->frame_correction, motor->id_target, motor->iq_target,
                                  id_target, iq_target);
        if (profile) Motor_Timing_ProfileRecord(MOTOR_TIMING_TRANSPORT, start);
    }
    if (id_target_out != NULL) {
        *id_target_out = id_target;
    }
    if (iq_target_out != NULL) {
        *iq_target_out = iq_target;
    }
    if (capture) Motor_Timing_HandoffTransport(motor);
    voltage = Motor_Current_RunPi(motor, id_target, iq_target, dt, v_limit);
    if (capture) Motor_Timing_HandoffAfter(motor, voltage.vd, voltage.vq);
    return voltage;
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
    COMMON_ASSERT(v != NULL);
    if (!isfinite(omega_e) || !isfinite(inductance) || !isfinite(flux) || !isfinite(id_ref) || !isfinite(iq_ref)) return;
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
