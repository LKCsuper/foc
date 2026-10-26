/*
 * @Description: 电流环 PI 控制器接口
 * @Author: lkc
 */
#ifndef _MOTOR_PID_H
#define _MOTOR_PID_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 单轴电流 PI 控制器（d/q 轴通用，VESC 风格）
 * @param {float} target    目标电流 (A)
 * @param {float} measured  实测电流 (A)
 * @param {float} *v_int    PI 积分项指针，调用者维护
 * @param {float} kp        比例增益
 * @param {float} ki        积分增益
 * @param {float} dt        控制周期 (s)
 * @param {float} v_limit   输出电压限幅绝对值 (V)
 * @return {float} 输出电压 (V)，已限幅在 [-v_limit, +v_limit]
 * @author: lkc
 */
float Motor_Current_PI_Run(float target, float measured, float *v_int,
                           float kp, float ki, float dt, float v_limit);

/**
 * @description: 误差形式的单轴电流 PI（d/q 通用）：out = v_int + error*kp，饱和时回写积分防 windup。
 * @param {float} error 目标与实测电流之差 (A)
 * @param {float *} v_int 输入输出 PI 积分电压 (V)
 * @param {float} kp 电流比例增益 (V/A)
 * @param {float} ki 电流积分增益 (V/(A·s))
 * @param {float} dt 本次推进时间间隔 (s)
 * @param {float} v_limit 输出电压幅值上限 (V)
 * @return {float} 当前值 output
 */
static inline float Motor_Current_PI_RunError(float error, float *v_int,
                                              float kp, float ki, float dt, float v_limit)
{
    float proportional = error * kp;
    float output;
    *v_int += error * ki * dt;
    output = *v_int + proportional;
    if (output > v_limit) {
        output = v_limit;
        *v_int = v_limit - proportional;
    } else if (output < -v_limit) {
        output = -v_limit;
        *v_int = -v_limit - proportional;
    }
    return output;
}

#ifdef __cplusplus
}
#endif

#endif
