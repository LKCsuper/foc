#ifndef _MOTOR_SPEED_H
#define _MOTOR_SPEED_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 通用速度 PI 外环：
 *
 *   真实机械转速 -> 速度 PI -> Iq 目标 -> 电流 PI -> SVPWM
 *
 * 本模块不关心速度来自霍尔、编码器还是无感观测器。反馈源只需要调用
 * Motor_SpeedFeedback_Set() 提供机械 RPM 和有效标志即可。
 */

/* 清除速度反馈、PI 积分和输出。 */
void Motor_Speed_Init(void);

/*
 * 更新真实机械转速。
 * @param rpm   机械转速，单位 RPM，反转为负数。
 * @param valid true=反馈可信；false=当前不能用于速度闭环。
 */
void Motor_SpeedFeedback_Set(float rpm, bool valid);

/* 运行速度 PI；函数内部按照 motor->speed_loop_hz 自动分频。 */
void Motor_SpeedLoop_Run(MOTOR_PARAM_S *motor, float dt);

/* 只清除 PI 状态，不清除反馈源最近一次提供的速度。 */
void Motor_SpeedLoop_Reset(void);

/* 无感开环切闭环时，用当前等效 Iq 预置积分，减小接管瞬间的电流阶跃。 */
void Motor_SpeedLoop_PrimeIq(float iq);

float Motor_SpeedLoop_GetIqTarget(void);
float Motor_SpeedLoop_GetMeasuredRpm(void);
float Motor_SpeedLoop_GetError(void);
bool Motor_SpeedLoop_IsFeedbackValid(void);

#ifdef __cplusplus
}
#endif

#endif
