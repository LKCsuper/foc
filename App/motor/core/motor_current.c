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
#include <float.h>

#include "motor_current.h"
#include "motor_fw.h"
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
 * @description: 使用浮点范围比较检查有限数，避免目标限流路径调用通用分类函数。
 * @param {float} value 待检查的电流或限幅
 * @return {bool} 有限数为 true，NaN 或无穷大为 false
 */
static inline bool Motor_Current_IsFinite(float value)
{
    return value <= FLT_MAX && value >= -FLT_MAX;
}


/**
 * @description: 逐控制拍累计电流跟踪超差，仅写诊断。
 * @detail: 接口和范围见motor_current.h；不修改控制输出或保护。
 * @param {motor_current_tracking_diag_t *} diag 调用者持有的观察状态，不可为空
 * @param {bool} eligible 当前采样点工况适用
 * @param {float} id 实测d轴电流
 * @param {float} iq 实测q轴电流
 * @param {float} id_cmd d轴目标
 * @param {float} iq_cmd q轴目标
 * @param {float} dt 当前控制拍周期
 * @param {uint32_t} entry_us 当前控制入口时间
 * @return {void} 无
 */
void Motor_Current_TrackingObserve(motor_current_tracking_diag_t *diag, bool eligible,
                                  float id, float iq, float id_cmd, float iq_cmd, float dt, uint32_t entry_us)
{
    float error_d, error_q, squared;
    COMMON_ASSERT(diag != NULL);
    diag->active = eligible && dt > 0.0f && dt <= 0.0002f;
    if (!diag->active) {
        diag->over_time_s = 0.0f;
        diag->over = false;
        return;
    }
    error_d = id_cmd - id;
    error_q = iq_cmd - iq;
    squared = error_d * error_d + error_q * error_q;
    if (!isfinite(squared)) {
        diag->active = false;
        diag->over_time_s = 0.0f;
        diag->over = false;
        return;
    }
    if (diag->eligible_samples < UINT32_MAX) diag->eligible_samples++;
    if (squared > diag->max_error_squared) diag->max_error_squared = squared;
    if (squared < 4.0f) {
        diag->over_time_s = 0.0f;
        diag->over = false;
        return;
    }
    if (diag->over) return;
    diag->over_time_s += dt;
    if (diag->over_time_s < 0.002f) return;
    diag->over_time_s = 0.002f;
    diag->over = true;
    if (diag->events < UINT32_MAX) diag->events++;
    if (!diag->event_latched) {
        diag->event_latched = true;
        diag->first_entry_us = entry_us;
        diag->first_id = id; diag->first_iq = iq;
        diag->first_id_cmd = id_cmd; diag->first_iq_cmd = iq_cmd;
    }
}

/**
 * @description: 累计一拍最长零矢量重构结果。
 * @detail: 只写本档诊断计数，不改电流。计数到 UINT32_MAX 后不再增加。
 * @param {uint8_t} phase 0=未重构，1=A，2=B，3=C
 * @return {void} 无
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
 * @description: 清零最长零矢量重构诊断。
 * @detail: 目标转速变化时由 ISR 调用。只清本档计数，不改电流或 PWM。
 * @return {void} 无
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
 * @description: 读取上一拍重构相及本档累计次数。
 * @detail: 各输出指针可为空，空则跳过该项。不改计数，不访问硬件。
 * @param {int *} last_phase   0=未重构，1=A，2=B，3=C；可为 NULL
 * @param {uint32_t *} samples 本档调用次数；可为 NULL
 * @param {uint32_t *} hits    本档重构次数；可为 NULL
 * @param {uint32_t *} hit_a   A 相重构次数；可为 NULL
 * @param {uint32_t *} hit_b   B 相重构次数；可为 NULL
 * @param {uint32_t *} hit_c   C 相重构次数；可为 NULL
 * @return {void} 无
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
 * @description: 三相电流变换到 αβ，再变换到 d/q，写回反馈字段。
 * @detail: 读 ia/ib/ic (A)，写 Ialpha/Ibeta/Id/Iq (A)。不改相电流，不输出 PWM。
 *          同时把 theta 及其正余弦记入 park_*，供同一实例的反 Park 复用；
 *          theta 原码与上次相同时不重算三角函数。
 *          motor 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor 电机对象，含 ia/ib/ic (A)
 * @param {float} theta           Park 用电角度 (rad)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Current_FeedbackUpdateDq(MOTOR_PARAM_S *motor, float theta)
{
    COMMON_ASSERT(motor != NULL);
    clarke_transform(motor->ia, motor->ib, motor->ic, &motor->Ialpha, &motor->Ibeta);
    if (!motor->park_sincos_valid || !utils_float_same_bits(motor->park_theta, theta)) {
        sincos_f32(theta, &motor->park_sin, &motor->park_cos);
        motor->park_theta = theta;
        motor->park_sincos_valid = true;
    }
    park_transform_sincos(motor->Ialpha, motor->Ibeta, motor->park_sin, motor->park_cos, &motor->Id, &motor->Iq);
}

/**
 * @description: 高占空比时重构采样窗口最短的那一相电流。
 * @detail: 对齐 VESC LONGEST_ZERO。任一相比 ARR 高过门槛时，把唯一最大 duty 的相写成
 *          −(另外两相)。duty 为软件 A/B/C 计数，须与 ia/ib/ic 同相序。pwm_period 不大于
 *          门槛、三相窗口都够、或两相并列最大时不改。任一电流指针为空返回 false。
 *          不访问 HAL。调用方在过流判定之后、Clarke 之前使用。
 * @param {float *} ia           A 相电流 (A)，可被改写
 * @param {float *} ib           B 相电流 (A)，可被改写
 * @param {float *} ic           C 相电流 (A)，可被改写
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
 * @description: 按目标电流圆限幅，先限制 Id，再用剩余幅值限制 Iq，保留符号。
 * @detail: 纯计算，由控制上下文调用。不限制实测瞬态电流，不替代故障保护。
 *          非有限输入或非正上限时两个目标清零。输出指针不可为空，违反约定触发 COMMON_ASSERT。
 * @param {float *} id_target   输入/输出 Id 目标 (A)，不可为空
 * @param {float *} iq_target   输入/输出 Iq 目标 (A)，不可为空
 * @param {float} current_limit 目标矢量模长上限 (A)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Current_LimitTargets(float *id_target, float *iq_target, float current_limit)
{
    float iq_room_squared;
    COMMON_ASSERT(id_target != NULL && iq_target != NULL);
    if (!Motor_Current_IsFinite(*id_target) || !Motor_Current_IsFinite(*iq_target) ||
        !Motor_Current_IsFinite(current_limit) || current_limit <= 0.0f) {
        *id_target = 0.0f;
        *iq_target = 0.0f;
        return;
    }
    if (*id_target > current_limit) *id_target = current_limit;
    if (*id_target < -current_limit) *id_target = -current_limit;
    iq_room_squared = current_limit * current_limit - *id_target * *id_target;
    if (*iq_target * *iq_target > iq_room_squared) {
        float iq_room = sqrtf(iq_room_squared > 0.0f ? iq_room_squared : 0.0f);
        *iq_target = *iq_target < 0.0f ? -iq_room : iq_room;
    }
}

/**
 * @description: 按工况选择 Id/Iq 目标，再按 MOTOR_FW_I_BUDGET 限幅。
 * @detail: 对齐优先，用 D 轴电流、Iq 置 0。否则叠加定角比例、Id 给定和弱磁，
 *          再选择速度环或 I/F 的 Iq。ctx 为空视为非对齐、缩放 1、不用速度环。
 *          motor 与两个输出不可为空，违反约定触发 COMMON_ASSERT。不输出 PWM。
 * @param {const MOTOR_PARAM_S *} motor             命令与对齐电流 (A)，不可为空
 * @param {const motor_current_target_ctx_t *} ctx  注入上下文；可为 NULL
 * @param {float *} id_target                       输出 Id 目标 (A)，不可为空
 * @param {float *} iq_target                       输出 Iq 目标 (A)，不可为空
 * @return {void} 无
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
        Motor_Current_LimitTargets(id_target, iq_target, MOTOR_FW_I_BUDGET);
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
    Motor_Current_LimitTargets(id_target, iq_target, MOTOR_FW_I_BUDGET);
}

/**
 * @description: 对 vd/vq 做合成矢量限幅，饱和时回写 PI 积分。
 * @detail: 未饱和则不改积分。用 Id/Iq 误差和 ol_current_kp 反算 P 项后回写积分，避免 windup。
 *          不输出 PWM。motor 与 v 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor  使用 ol_current_kp 与 ol_vd_int/ol_vq_int (V)
 * @param {motor_dq_voltage_t *} v 输入/输出电压指令 (V)
 * @param {float} id_err           Id 误差 (A)
 * @param {float} iq_err           Iq 误差 (A)
 * @param {float} v_limit          电压矢量模长上限 (V)
 * @return {void} 无
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
 * @description: 运行 d/q 双轴电流 PI，并做矢量限幅。
 * @detail: Iq 误差在 PI 前取样，限幅抗饱和和钳位统计共用这一拍。不改 Id/Iq，不输出 PWM。
 *          motor 为空返回零电压。
 * @param {MOTOR_PARAM_S *} motor 含 Id/Iq 反馈 (A) 与积分、增益
 * @param {float} id_target       Id 目标 (A)
 * @param {float} iq_target       Iq 目标 (A)
 * @param {float} dt              控制周期 (s)
 * @param {float} v_limit         单轴与矢量限幅 (V)
 * @return {motor_dq_voltage_t} 限幅后的 vd/vq (V)；motor 为空时为 0
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
    MOTOR_PROFILE_BEGIN(profile, stage_start);
    v.vd = Motor_Current_PI_Run(id_target, motor->Id, &motor->ol_vd_int, motor->ol_current_kp, motor->ol_current_ki, dt,
                                v_limit);
    v.vq = Motor_Current_PI_Run(iq_target, motor->Iq, &motor->ol_vq_int, motor->ol_current_kp, motor->ol_current_ki, dt,
                                v_limit);
    MOTOR_PROFILE_END(profile, MOTOR_TIMING_CURRENT_PI, stage_start);

    MOTOR_PROFILE_BEGIN(profile, stage_start);
    Motor_Current_LimitVoltage(motor, &v, id_target - motor->Id, iq_err, v_limit);
    MOTOR_PROFILE_END(profile, MOTOR_TIMING_CURRENT_LIMIT, stage_start);
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
 * @description: 读出并清零 q 轴电流 PI 统计。
 * @detail: 进入临界区后复制，避免和 FOC 中断同时改累加器。samples 为 0 时平均值记 0。
 *          diag 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_current_pi_diag_t *} diag 输出统计，不可为空
 * @return {void} 无
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
 *          由电流环在矢量限幅前调用，不改 PWM。motor 不可为空，违反约定触发 COMMON_ASSERT。
 *          delta 非有限值时直接返回。
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
    sincos_f32(delta, &s, &c);
    vd = motor->ol_vd_int;
    vq = motor->ol_vq_int;
    /* R(-delta): 同一静止电压矢量在新dq轴上的坐标。反馈也按同一旋转变化，
     * 因此只需补偿旧/新目标P项，不能清零积分或重新注入启动电流。 */
    motor->ol_vd_int = c * vd + s * vq + motor->ol_current_kp * (c * old_id + s * old_iq - new_id);
    motor->ol_vq_int = -s * vd + c * vq + motor->ol_current_kp * (-s * old_id + c * old_iq - new_iq);
}

/**
 * @description: 电流环一步：反馈变换、取目标、可选坐标交接、PI 及矢量限幅。
 * @detail: 不含 PWM。ctx 为空时按非对齐、缩放 1、不用速度环取目标。
 *          ctx->frame_correction 非 0 时先旋转积分，再跑 PI 并限幅。
 *          motor 为空返回零电压。两个目标输出可为 NULL。
 * @param {MOTOR_PARAM_S *} motor                  电机采样与积分；空则忽略
 * @param {float} theta                            Park 用电角度 (rad)
 * @param {const motor_current_target_ctx_t *} ctx 目标上下文；可为 NULL
 * @param {float} dt                               控制周期 (s)
 * @param {float} v_limit                          电压限幅 (V)
 * @param {float *} id_target_out                  回写本拍 Id 目标 (A)；可为 NULL
 * @param {float *} iq_target_out                  回写本拍 Iq 目标 (A)；可为 NULL
 * @return {motor_dq_voltage_t} 限幅后的 vd/vq (V)；motor 为空时为 0
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
    MOTOR_PROFILE_BEGIN(profile, stage_start);
    Motor_Current_FeedbackUpdateDq(motor, theta);
    MOTOR_PROFILE_END(profile, MOTOR_TIMING_CURRENT_PARK, stage_start);
    MOTOR_PROFILE_BEGIN(profile, stage_start);
    Motor_Current_GetTargets(motor, ctx, &id_target, &iq_target);
    capture = Motor_Timing_HandoffDue();
    if (capture) Motor_Timing_HandoffBefore(motor, theta, ctx != NULL ? ctx->frame_correction : 0.0f,
                                           id_target, iq_target, dt);
    MOTOR_PROFILE_END(profile, MOTOR_TIMING_CURRENT_TARGET, stage_start);
    if (ctx != NULL && ctx->frame_correction != 0.0f) {
        uint32_t start;
        MOTOR_PROFILE_BEGIN(profile, start);
        Motor_Current_TransportPi(motor, ctx->frame_correction, motor->id_target, motor->iq_target,
                                  id_target, iq_target);
        MOTOR_PROFILE_END(profile, MOTOR_TIMING_TRANSPORT, start);
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
 * @detail: 教科书 vd -= ωe·L·Iq、vq += ωe·L·Id + ωe·ψf。只用于转子对齐的 dq。
 *          I/F 开环角不得调用。不改 PI 积分，不输出 PWM。
 *          v 不可为空，违反约定触发 COMMON_ASSERT。任一浮点非有限则不改电压。
 *          电感或磁链为负时按 0 计。v_limit 不大于 0 则不限幅。
 * @param {motor_dq_voltage_t *} v 输入/输出电压 (V)，不可为空
 * @param {float} omega_e          电角速度 (rad/s)
 * @param {float} inductance       相电感 (H)
 * @param {float} flux             永磁磁链 (Wb)
 * @param {float} id_ref           Id 参考 (A)
 * @param {float} iq_ref           Iq 参考 (A)
 * @param {float} v_limit          电压矢量模长上限 (V)；不大于 0 则不限幅
 * @return {void} 无
 * @author: lkc
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
