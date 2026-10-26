/*
 * @Description: 通用单轴电流 PI 控制器
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_pid.h"
#if defined(__CC_ARM)
/* 位于 ADC 中断控制路径，与其他实时模块一致局部 O2；不启用快速浮点，公式不变。 */
#pragma O2
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 单轴电流 PI 控制器（d/q 轴通用）
 * @detail: 参考 VESC mcpwm_foc.c 的电流环实现。PI 并行形式:
 *          v_out = v_int + Ierr * kp；v_int += Ierr * ki * dt。
 *          电机绕组是一阶系统 (tf = 1/(Ls+R))，不需要 D 项。
 * @param {float} target 目标电流 (A)
 * @param {float} measured 实测电流 (A)
 * @param {float *} v_int PI 积分项指针，不可为空
 * @param {float} kp 比例增益
 * @param {float} ki 积分增益
 * @param {float} dt 控制周期 (s)
 * @param {float} v_limit 输出电压限幅绝对值 (V)
 * @return {float} 输出电压 (V)，已限幅
 * @author: lkc
 */
float Motor_Current_PI_Run(float target, float measured, float *v_int,
                           float kp, float ki, float dt, float v_limit)
{
    return Motor_Current_PI_RunError(target - measured, v_int, kp, ki, dt, v_limit);
}
#ifdef __cplusplus
}
#endif
