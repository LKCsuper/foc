/*
 * @Description: 无感 Ortega 磁链观测器与开环切闭环接口
 * @Author: lkc
 */
#ifndef _MOTOR_SENSORLESS_H
#define _MOTOR_SENSORLESS_H

#ifdef __cplusplus
extern "C" {
#endif

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
} motor_sensorless_diagnostics_t;

/**
 * @description: 上电初始化观测器运行时状态。
 * @detail: MOTOR_Param_Init() 完成参数赋值后调用一次。
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_Init(void);

/**
 * @description: 复位观测器。
 * @detail: 清空磁链、PLL、锁定计时和接管比例，并把速度反馈标记为无效。
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_Reset(void);

/**
 * @description: 记录本周期实际下发给逆变器的 α/β 电压。
 * @param {float} v_alpha  α 轴电压 (V)
 * @param {float} v_beta   β 轴电压 (V)
 * @detail: 下一 FOC 周期会把它与新采样电流一起送入磁链模型。
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_SetAppliedVoltage(float v_alpha, float v_beta);

/**
 * @description: 每个 FOC 周期更新 Ortega 磁链、PLL、锁定/失锁状态和角度渐变。
 * @param {MOTOR_PARAM_S} *motor  电机实时参数；R/L/flux 分别使用 Ω、H、Wb
 * @param {float} dt              控制周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_Update(MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 获取本周期 Park 变换应使用的电角度。
 * @param {float} open_loop_angle  调用者提供的 I/F 强制电角度 (rad)
 * @return {float} 控制用电角度 (rad)；未闭环时返回 open_loop_angle
 * @author: lkc
 */
float Motor_Sensorless_GetControlAngle(float open_loop_angle);

/**
 * @description: 获取观测器状态机当前状态。
 * @return {motor_sensorless_state_t} OPEN_LOOP、LOCKING 或 CLOSED
 * @author: lkc
 */
motor_sensorless_state_t Motor_Sensorless_GetState(void);

/**
 * @description: 获取 PLL 平滑后的观测电角度。
 * @return {float} 观测电角度 (rad)
 * @author: lkc
 */
float Motor_Sensorless_GetObserverAngle(void);

/**
 * @description: 获取估算转子磁链模长。
 * @return {float} 磁链模长 (Wb)
 * @author: lkc
 */
float Motor_Sensorless_GetFluxMagnitude(void);

/**
 * @description: 获取低通后的机械转速。
 * @return {float} 机械转速 (RPM)，反转时为负
 * @author: lkc
 */
float Motor_Sensorless_GetMechanicalRpm(void);

/**
 * @description: 获取 PLL 观测角与 I/F 强制角的最短相位差。
 * @return {float} 相位误差 (rad)，范围为 ±π
 * @author: lkc
 */
float Motor_Sensorless_GetPhaseError(void);

/**
 * @description: 拷贝观测器的接管诊断数据。
 * @param {motor_sensorless_diagnostics_t} *diagnostics  输出指针，填充四项接管条件、锁定计时、接管比例和实际输出电压
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_GetDiagnostics(motor_sensorless_diagnostics_t *diagnostics);

/**
 * @description: 判断观测角是否已接管电流环。
 * @return {bool} true=已进入 CLOSED 且观测角参与 Park 变换
 * @author: lkc
 */
bool Motor_Sensorless_IsClosedLoop(void);

/**
 * @description: 判断开环角到观测角的接管是否全部完成。
 * @detail: 需要已进入 CLOSED 且角度渐变比例 blend 达到 1.0；供 I/F 最高转速保护使用。
 * @return {bool} true=接管渐变已完成
 * @author: lkc
 */
bool Motor_Sensorless_IsTakeoverComplete(void);

#ifdef __cplusplus
}
#endif

#endif
