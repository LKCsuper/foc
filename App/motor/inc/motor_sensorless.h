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

/* 上电初始化运行时状态；MOTOR_Param_Init() 完成参数赋值后调用一次。 */
void Motor_Sensorless_Init(void);

/*
 * 清空磁链、PLL、锁定计时和接管比例，并把速度反馈标记为无效。
 * 停机、模式切换、关闭无感或检测到 NaN/Inf 时都会调用。
 */
void Motor_Sensorless_Reset(void);

/*
 * 保存本周期反 Park 后实际下发的 α/β 电压，单位 V。
 * 下一 FOC 周期会把它与新采样电流一起送入磁链模型。
 */
void Motor_Sensorless_SetAppliedVoltage(float v_alpha, float v_beta);

/*
 * 每个 FOC 周期更新 Ortega 磁链、PLL、锁定/失锁状态和角度渐变。
 * motor 中的 R/L/flux 必须分别使用 Ω、H、Wb，dt 的单位必须是 s。
 */
void Motor_Sensorless_Update(MOTOR_PARAM_S *motor, float dt);

/*
 * 返回本周期 Park 变换应使用的电角度，单位 rad。
 * 未闭环时返回 open_loop_angle；闭环后返回渐变后的观测控制角。
 */
float Motor_Sensorless_GetControlAngle(float open_loop_angle);

/* 查询当前状态，主要用于状态显示和调试。 */
motor_sensorless_state_t Motor_Sensorless_GetState(void);

/* 返回 PLL 平滑后的观测电角度，单位 rad。 */
float Motor_Sensorless_GetObserverAngle(void);

/* 返回估算转子磁链模长，单位 Wb。 */
float Motor_Sensorless_GetFluxMagnitude(void);

/* 返回低通后的机械转速，单位 RPM，反转时为负。 */
float Motor_Sensorless_GetMechanicalRpm(void);

/* 返回 PLL 角与 I/F 强制角的最短角度差，单位 rad。 */
float Motor_Sensorless_GetPhaseError(void);

/* true 表示观测角已经接管，而不只是观测器处于启用状态。 */
bool Motor_Sensorless_IsClosedLoop(void);

#ifdef __cplusplus
}
#endif

#endif
