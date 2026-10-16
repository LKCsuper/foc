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
#include <stdint.h>
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/*
 * 三电阻最长零矢量：某相 CCR > ARR−本值时，低边窗口过短，用另外两相重构。
 * 用锁定 vesc_foc.c 高调制路径的 500，不用 mcpwm_foc SHUNT_PICK_THR=900。
 * 空载 3000 峰值 CCR≈4902：900 会提前改写并把三相和打成 0，500 仍留给 ADC rank2。
 */
#define MOTOR_CURRENT_SHUNT_PICK_COUNTS 500U
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: d/q 轴电压指令中间量
 */
typedef struct {
    float vd; /* D 轴电压 (V)，励磁 */
    float vq; /* Q 轴电压 (V)，转矩 */
} motor_dq_voltage_t;

/**
 * @description: 两次读取之间的 q 轴电流 PI 统计
 * @detail: 由 Motor_Current_RunPi 累加，Motor_Current_TakePiDiag 读出后清零
 */
typedef struct {
    uint32_t samples;      /* 电流 PI 拍数 */
    uint32_t clamp_samples; /* 积分增量被限幅或抗饱和改写的拍数 */
    float iq_err_avg;      /* Iq 误差平均值 (A) */
    float iq_err_abs_avg;  /* Iq 误差绝对值平均 (A) */
    float vq_int_delta;    /* q 轴积分实际变化量 (V) */
    float ki;              /* 最近一拍 Ki (V/(A·s)) */
    float dt;              /* 最近一拍周期 (s) */
} motor_current_pi_diag_t;

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
    float id_fw;                   /* 弱磁叠加 Id，≤0 (A)；对齐阶段不使用 */
    float frame_correction;        /* 仅坐标交接附加角(rad)，普通旋转必须为0 */
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
 * @description: 坐标交接时旋转电流 PI 积分，并补偿目标变化的 P 项。
 * @detail: 只用于无感渐变等坐标交接。delta 为新电角减旧电角 (rad)，不得传入正常转速积分角。
 *          由电流环在矢量限幅前调用，不改 PWM。motor 不可为空并由 COMMON_ASSERT 检查；delta 非有限值时直接返回。
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
                               float new_id, float new_iq);

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
                                          uint32_t duty_c, uint32_t pwm_period);

/**
 * @description: 清零最长零矢量重构诊断（目标转速变化时由 ISR 调用）
 * @return {*}
 * @author: lkc
 */
void Motor_Current_ResetReconstructDiag(void);

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
                                      uint32_t *hit_b, uint32_t *hit_c);

/**
 * @description: 按工况选择 Id/Iq 目标
 * @detail: 对齐优先；否则 Id=ol_id_set+id_fw；Iq 来自速度环或 ol_iq_set×if_iq_scale
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
 * @description: 读出并清零 q 轴电流 PI 统计
 * @detail: 关中断后复制，避免和 FOC 中断同时改累加器。samples 为 0 时平均值记 0。
 * @param {motor_current_pi_diag_t *} diag  输出统计，不可为空
 * @return {*}
 * @author: lkc
 */
void Motor_Current_TakePiDiag(motor_current_pi_diag_t *diag);

/**
 * @description: 电流环一步：反馈变换、取目标、可选坐标交接、PI 及矢量限幅。
 * @detail: 不含 PWM。ctx->frame_correction 非 0 时先旋转积分，再跑 PI 并限幅。
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
