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
 * @description: 电流跟踪误差观察，只标记，不停机。
 * @detail: 由 Motor_Current_TrackingObserve 写入。调用者持有并负责清零。
 */
typedef struct {
    float over_time_s;       /* 当前超差累计 (s) */
    float max_error_squared; /* 本实例最大误差平方和 (A²) */
    float first_id;          /* 首次标记时 Id (A) */
    float first_iq;          /* 首次标记时 Iq (A) */
    float first_id_cmd;      /* 首次标记时 Id 目标 (A) */
    float first_iq_cmd;      /* 首次标记时 Iq 目标 (A) */
    uint32_t eligible_samples; /* 允许观察的控制拍数 */
    uint32_t events;         /* 标记次数 */
    uint32_t first_entry_us; /* 首次标记的控制入口时间 (us) */
    bool active;             /* 本拍输入可用于累计 */
    bool over;               /* 本拍已满 2 ms 超差 */
    bool event_latched;      /* 已记下首次事件 */
} motor_current_tracking_diag_t;

/**
 * @description: 只观察电流跟踪误差，平方和达到 4 A² 并累计 2 ms 后标记。
 * @detail: 单一控制写者调用，实例由调用者持有。不停机、不改电流或保护。
 *          不适用、非有限输入或 dt 不在 (0, 200 us] 时中断本次累计。
 *          恢复后清当前累计，保留首次事件与次数。首次时间是检测入口 (us)，
 *          不是 ADC 触发锁存值。diag 不可为空，违反约定触发 COMMON_ASSERT；初始须清零。
 * @param {motor_current_tracking_diag_t *} diag 诊断实例，不可为空
 * @param {bool} eligible                        当前工况允许观察
 * @param {float} id                             实测 Id (A)
 * @param {float} iq                             实测 Iq (A)
 * @param {float} id_cmd                         本拍 Id 目标 (A)
 * @param {float} iq_cmd                         本拍 Iq 目标 (A)
 * @param {float} dt                             本控制拍周期 (s)
 * @param {uint32_t} entry_us                    本控制入口时间 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Current_TrackingObserve(motor_current_tracking_diag_t *diag, bool eligible,
                                  float id, float iq, float id_cmd, float iq_cmd, float dt, uint32_t entry_us);

/**
 * @description: 电流目标选择时由调用者注入的上下文。
 * @detail: 覆盖对齐、I/F 和速度环。本模块不读无感或速度环全局。
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
 * @description: 三相电流变换到 αβ，再变换到 d/q，写回反馈字段。
 * @detail: 读 ia/ib/ic (A)，写 Ialpha/Ibeta/Id/Iq (A)。不改相电流，不输出 PWM。
 *          motor 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor 电机对象，含 ia/ib/ic (A)
 * @param {float} theta           Park 用电角度 (rad)
 * @return {void} 无
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
                                          uint32_t duty_c, uint32_t pwm_period);

/**
 * @description: 清零最长零矢量重构诊断。
 * @detail: 目标转速变化时由 ISR 调用。只清本档计数，不改电流或 PWM。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Current_ResetReconstructDiag(void);

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
                                      uint32_t *hit_b, uint32_t *hit_c);

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
                              float *iq_target);

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
void Motor_Current_LimitTargets(float *id_target, float *iq_target, float current_limit);

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
void Motor_Current_LimitVoltage(MOTOR_PARAM_S *motor, motor_dq_voltage_t *v, float id_err, float iq_err, float v_limit);

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
motor_dq_voltage_t Motor_Current_RunPi(MOTOR_PARAM_S *motor, float id_target, float iq_target, float dt, float v_limit);

/**
 * @description: 读出并清零 q 轴电流 PI 统计。
 * @detail: 进入临界区后复制，避免和 FOC 中断同时改累加器。samples 为 0 时平均值记 0。
 *          diag 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_current_pi_diag_t *} diag 输出统计，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Current_TakePiDiag(motor_current_pi_diag_t *diag);

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
                                      float dt, float v_limit, float *id_target_out, float *iq_target_out);

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
                                         float id_ref, float iq_ref, float v_limit);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
