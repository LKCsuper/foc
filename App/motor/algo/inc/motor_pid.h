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

#ifdef __cplusplus
}
#endif

#endif
