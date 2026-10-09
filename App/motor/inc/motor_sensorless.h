/*
 * @Description: 无感 Ortega 磁链观测器与开环切闭环接口（支持状态实例）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 静止/极低速时反电动势弱，观测角不可信 → 必须先 I/F 起转再接管。
 *   2) OPEN_LOOP/LOCKING 仍用强制角；CLOSED 才渐变使用 PLL 角。
 *   3) “观测器在算”≠“已接管”；takeover 是最后一道人工授权。
 *   4) State API 通过 effects 报告“要发速度反馈/预置 Iq/锁存失速”，
 *      不在核心里直接调速度环/保护（默认包装负责落地）。
 */
#ifndef _MOTOR_SENSORLESS_H
#define _MOTOR_SENSORLESS_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/*
 * 无感控制状态。注意“观测器正在计算”不等于“观测器已经接管”：
 *
 *   OPEN_LOOP
 *     电流环完全使用 I/F 强制角度；观测器只在后台计算。
 *     磁链、转速、方向或相位至少有一项不满足接管条件。
 *
 *   LOCKING
 *     所有接管条件当前均满足，但还要连续保持 sensorless_lock_time。
 *     此状态仍然使用 I/F 角度，takeover=false 时会一直停留在这里观察。
 *
 *   CLOSED
 *     已获得 sensorless_takeover 授权并完成锁定，观测角通过渐变参与
 *     Park 变换；PLL 机械 RPM 同时提供给速度 PI 和速度类保护。
 */
typedef enum {
    MOTOR_SENSORLESS_OPEN_LOOP = 0,
    MOTOR_SENSORLESS_LOCKING,
    MOTOR_SENSORLESS_CLOSED,
} motor_sensorless_state_t;

/*
 * 无感接管条件快照。
 * olstatus/olfault 使用它直接显示究竟是磁链、速度、方向还是相位阻止接管，
 * 避免只看到 OPEN_LOOP 却不知道失败在哪一项。
 */
typedef struct {
    bool flux_ok;             /* 估算磁链是否在额定磁链允许范围内 */
    bool speed_ok;            /* 观测转速是否达到并跟随 I/F 转速 */
    bool direction_ok;        /* 观测方向是否与目标方向一致 */
    bool phase_ok;            /* 观测角与 I/F 角的误差是否在容差内 */
    float lock_timer;         /* 四项条件连续成立的时间，单位 s */
    float blend;              /* 角度接管比例，0=I/F，1=观测角 */
    float applied_voltage;    /* 实际下发的 α/β 电压矢量模长，单位 V */
    float mechanical_rpm;     /* 同拍低通机械 RPM */
    float flux_magnitude;     /* 同拍转子磁链 Wb */
    float phase_error;        /* 与 phase_ok 同拍的最短角差 rad */
    float reference_angle;    /* 本拍判定相位时的 I/F 角，CLOSED 回写前，rad */
    float observer_angle;     /* 含固定补偿的 atan2 观测角 rad */
    float pll_angle;          /* 同拍 PLL 电角 rad */
    motor_sensorless_state_t state;
} motor_sensorless_diagnostics_t;

/**
 * @description: 无感观测器运行时状态（不持久化）。
 * @detail: 角度均为电角度 (rad)；时间单位 s；转速为机械 RPM。
 */
typedef struct {
    float x_alpha;              /* α 轴估算定子磁链 Wb */
    float x_beta;               /* β 轴估算定子磁链 Wb */
    float applied_v_alpha;      /* 上一周期命令 α 电压 V */
    float applied_v_beta;       /* 上一周期命令 β 电压 V */
    float observer_angle;       /* atan2 原始观测电角 rad */
    float pll_angle;            /* PLL 平滑电角 rad */
    float pll_speed_electrical; /* PLL 电角速度 electrical rad/s */
    float mechanical_rpm;       /* 未滤波机械 RPM */
    float filtered_rpm;         /* 低通后机械 RPM */
    float flux_magnitude;       /* 转子磁链模长 Wb */
    float phase_error;          /* PLL 角相对 I/F 角差 rad */
    float reference_angle;      /* 本拍相位判定用 I/F 电角 rad */
    float lock_timer;           /* 锁定候选累计 s */
    float loss_timer;           /* 失锁累计 s */
    float blend;                /* 0=开环角 … 1=观测角 */
    float control_angle;        /* 送入 Park 的电角 rad */
    bool flux_ok;
    bool speed_ok;
    bool direction_ok;
    bool phase_ok;
    bool initialized;           /* 磁链积分器是否已 Seed */
    motor_sensorless_state_t state;
} motor_sensorless_runtime_t;

/**
 * @description: UpdateState 产生的副作用请求（由包装落地到速度环/保护）。
 * @detail: 核心不直接调用 Motor_Speed* / Motor_Protect*，便于主机双实例测试。
 */
typedef struct {
    bool request_speed_loss_fault; /* 高速失锁：应锁存 MOTOR_SPEED_LOSS */
    bool feedback_valid;           /* 是否向速度环发布有效转速 */
    float feedback_rpm;            /* 发布的机械 RPM */
    bool prime_speed_iq;           /* 切入 CLOSED 时预置速度 PI */
    float prime_iq;                /* 预置 Iq (A) */
} motor_sensorless_effects_t;

/**
 * @description: 初始化指定观测器运行时状态。
 * @detail: 清空历史量并回到 OPEN_LOOP；不改速度环（由包装决定）。
 * @param {motor_sensorless_runtime_t *} rt  运行时对象，不可为空
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_InitState(motor_sensorless_runtime_t *rt);

/**
 * @description: 复位指定观测器运行时状态。
 * @param {motor_sensorless_runtime_t *} rt  运行时对象
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_ResetState(motor_sensorless_runtime_t *rt);

/**
 * @description: 记录指定实例本周期实际下发的 α/β 电压。
 * @param {motor_sensorless_runtime_t *} rt  运行时对象
 * @param {float} v_alpha  α 轴电压 (V)
 * @param {float} v_beta   β 轴电压 (V)
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_SetAppliedVoltageState(motor_sensorless_runtime_t *rt, float v_alpha, float v_beta);

/**
 * @description: 推进指定实例的 Ortega/PLL/接管状态机一拍。
 * @detail: 需要时写回 motor->ol_theta（CLOSED 同步）；副作用写入 effects（可为 NULL）。
 * @param {motor_sensorless_runtime_t *} rt  运行时对象
 * @param {MOTOR_PARAM_S *} motor  电机参数与采样；R/L/flux 用 Ω、H、Wb
 * @param {float} dt  FOC 控制周期 (s)
 * @param {motor_sensorless_effects_t *} effects  副作用输出；NULL 则忽略
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_UpdateState(motor_sensorless_runtime_t *rt, MOTOR_PARAM_S *motor, float dt,
                                  motor_sensorless_effects_t *effects);

/**
 * @description: 获取指定实例本周期 Park 应用电角度。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @param {float} open_loop_angle  I/F 强制电角度 (rad)
 * @return {float} 控制用电角度 (rad)；未 CLOSED 时返回 open_loop_angle
 * @author: lkc
 */
float Motor_Sensorless_GetControlAngleState(const motor_sensorless_runtime_t *rt, float open_loop_angle);

/**
 * @description: 读取指定实例状态机。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象；NULL 返回 OPEN_LOOP
 * @return {motor_sensorless_state_t} OPEN_LOOP / LOCKING / CLOSED
 * @author: lkc
 */
motor_sensorless_state_t Motor_Sensorless_GetStateState(const motor_sensorless_runtime_t *rt);

/**
 * @description: 读取指定实例 PLL 平滑观测电角。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {float} 观测电角度 (rad)
 * @author: lkc
 */
float Motor_Sensorless_GetObserverAngleState(const motor_sensorless_runtime_t *rt);

/**
 * @description: 读取指定实例转子磁链模长。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {float} 磁链模长 (Wb)
 * @author: lkc
 */
float Motor_Sensorless_GetFluxMagnitudeState(const motor_sensorless_runtime_t *rt);

/**
 * @description: 读取指定实例低通机械转速。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {float} 机械转速 (RPM)，反转为负
 * @author: lkc
 */
float Motor_Sensorless_GetMechanicalRpmState(const motor_sensorless_runtime_t *rt);

/**
 * @description: 读取指定实例相位误差。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {float} 相位误差 (rad)，约 ±π
 * @author: lkc
 */
float Motor_Sensorless_GetPhaseErrorState(const motor_sensorless_runtime_t *rt);

/**
 * @description: 拷贝指定实例接管诊断。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @param {motor_sensorless_diagnostics_t *} diagnostics  输出；NULL 忽略
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_GetDiagnosticsState(const motor_sensorless_runtime_t *rt,
                                          motor_sensorless_diagnostics_t *diagnostics);

/**
 * @description: 查询指定实例是否已 CLOSED（观测角参与 Park）。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {bool} true=已闭环接管
 * @author: lkc
 */
bool Motor_Sensorless_IsClosedLoopState(const motor_sensorless_runtime_t *rt);

/**
 * @description: 查询指定实例角度渐变是否完成（CLOSED 且 blend≥1）。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {bool} true=完全使用观测角
 * @author: lkc
 */
bool Motor_Sensorless_IsTakeoverCompleteState(const motor_sensorless_runtime_t *rt);

/**
 * @description: 上电初始化默认单实例观测器。
 * @detail: MOTOR_Param_Init() 完成参数赋值后调用一次；并声明速度反馈无效。
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_Init(void);

/**
 * @description: 复位默认实例观测器，并声明速度反馈无效。
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_Reset(void);

/**
 * @description: 记录默认实例本周期实际下发的 α/β 电压。
 * @param {float} v_alpha  α 轴电压 (V)
 * @param {float} v_beta   β 轴电压 (V)
 * @detail: 下一 FOC 周期与新采样电流一起送入磁链模型。
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_SetAppliedVoltage(float v_alpha, float v_beta);

/**
 * @description: 默认实例每个 FOC 周期更新 Ortega/PLL/接管，并落地速度环/保护副作用。
 * @param {MOTOR_PARAM_S *} motor  电机实时参数；R/L/flux 分别使用 Ω、H、Wb
 * @param {float} dt  控制周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_Update(MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 获取默认实例本周期 Park 变换应使用的电角度。
 * @param {float} open_loop_angle  调用者提供的 I/F 强制电角度 (rad)
 * @return {float} 控制用电角度 (rad)；未闭环时返回 open_loop_angle
 * @author: lkc
 */
float Motor_Sensorless_GetControlAngle(float open_loop_angle);

/**
 * @description: 获取默认实例观测器状态机当前状态。
 * @return {motor_sensorless_state_t} OPEN_LOOP、LOCKING 或 CLOSED
 * @author: lkc
 */
motor_sensorless_state_t Motor_Sensorless_GetState(void);

/**
 * @description: 获取默认实例 PLL 平滑后的观测电角度。
 * @return {float} 观测电角度 (rad)
 * @author: lkc
 */
float Motor_Sensorless_GetObserverAngle(void);

/**
 * @description: 获取默认实例估算转子磁链模长。
 * @return {float} 磁链模长 (Wb)
 * @author: lkc
 */
float Motor_Sensorless_GetFluxMagnitude(void);

/**
 * @description: 获取默认实例低通后的机械转速。
 * @return {float} 机械转速 (RPM)，反转时为负
 * @author: lkc
 */
float Motor_Sensorless_GetMechanicalRpm(void);

/**
 * @description: 获取默认实例 PLL 观测角与 I/F 强制角的最短相位差。
 * @return {float} 相位误差 (rad)，范围为 ±π
 * @author: lkc
 */
float Motor_Sensorless_GetPhaseError(void);

/**
 * @description: 拷贝默认实例观测器的接管诊断数据。
 * @detail: 短临界区复制运行时后计算诊断，返回同一拍角度、相位和判定；不打印。
 * @param {motor_sensorless_diagnostics_t *} diagnostics  输出指针
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_GetDiagnostics(motor_sensorless_diagnostics_t *diagnostics);

/**
 * @description: 判断默认实例观测角是否已接管电流环。
 * @return {bool} true=已进入 CLOSED 且观测角参与 Park 变换
 * @author: lkc
 */
bool Motor_Sensorless_IsClosedLoop(void);

/**
 * @description: 判断默认实例开环角到观测角的接管是否全部完成。
 * @detail: 需要已进入 CLOSED 且 blend 达到 1.0；供 I/F 最高转速保护使用。
 * @return {bool} true=接管渐变已完成
 * @author: lkc
 */
bool Motor_Sensorless_IsTakeoverComplete(void);

#ifdef __cplusplus
}
#endif

#endif
