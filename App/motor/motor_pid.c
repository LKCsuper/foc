/*
 * @Description: 电流 PI 控制器
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2026-09-03 00:14:06
 * @FilePath: \foc\App\motor\utils.c
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 单轴电流 PI 控制器（d/q 轴通用）
 * @detail:   参考 VESC mcpwm_foc.c 的电流环实现。
 *            PI 并行形式: v_out = v_int + Ierr * kp
 *                        v_int += Ierr * ki * dt
 *            电机绕组是一阶系统 (tf = 1/(Ls+R))，不需要 D 项。
 * @param target:    目标电流 (A)
 * @param measured:  实测电流 (A)
 * @param v_int:     PI 积分项指针
 * @param kp:        比例增益
 * @param ki:        积分增益
 * @param dt:        控制周期 (s)
 * @param v_limit:   输出电压限幅绝对值 (V)
 * @return:          输出电压 (V)，已限幅
 */
float Motor_Current_PI_Run(float target, float measured, float *v_int,
                           float kp, float ki, float dt, float v_limit)
{
    // 电流误差 = 目标电流 - 实测电流
    float Ierr = target - measured;

    /* 积分累加 */
    *v_int += Ierr * ki * dt;

    /* PI 输出 = 积分 + 比例 */
    float v_out = *v_int + Ierr * kp;

    /* 输出限幅 + 积分抗饱和（clamping） */
    if (v_out > v_limit)
    {
        v_out = v_limit;
        /* 限幅时回退积分，防止 windup */
        *v_int = v_limit - Ierr * kp;
    }
    else if (v_out < -v_limit)
    {
        v_out = -v_limit;
        *v_int = -v_limit - Ierr * kp;
    }

    return v_out;
}
#ifdef __cplusplus
}
#endif
